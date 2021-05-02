// ------------------------------ //
//            INCLUDES            //
// ------------------------------ //

#include "Motor.h"
#include "Pixy2.h"
#include "ThisThread.h"
#include "VL53L1X.h"
#include "mbed.h"

// ------------------------------ //
//           DEFINITIONS          //
// ------------------------------ //

// Pin definitions for I2C
#define I2C_SDA D4
#define I2C_SCL D5

// Update interval of the Time-of-Flight(Vl53L1X) sensor. Time is in
// milliseconds
#define DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS 500

// The offset of the distance sensor. Change this variable to let the code
// account for a difference in distance sensor mount position. Mounting the
// sensor towards the front of the robot is the - direction. And mounting the
// sensor towards the back of the robot is the + direction.
// Example if the sensor is mounted 80mm deep in the robot, set it to 80, if the
// sensor is placed 20mm in front of the robot, set it to -20. This variable
// makes sure that the robot can stop at the right distance.
#define DISTANCE_SENSOR_OFFSET_IN_MM 0

// At what distance should the robot stop? The VL53L1X Time of Flight sensor
// provides the distance. The distance is in millimeters.
#define STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM 100

// Limit the top speed of the robot drive motors. This value is in percent
// ranging from 0% to 100%. When the left or right drive motor exedes the
// maximum speed it will be limited to this value.
#define ROBOT_MAX_SPEED_IN_PERCENT 100

// Should the Pixy2 camera use the x coorninate in the distance(true) or
// closeby(false). By using closeby(false) the x0 location returned by the Pixy2
// line vector will be used, this is on the lower part of the camera, or closer
// to the robot. This is a safe way of following ta line without 'corner
// cutting'. By using distance(true) the x1 location returned by the Pixy2 line
// vector will be used, this is on the upper part of the camera, or further away
// from robot. This will allow the robot to cut corners instead of following it
// strictly.
#define LINE_DETECTION_LOOK_AHEAD true

// When true print information over serial USB to the connected PC. Use a serial
// terminal with a baudrate of 115200 to watch the output. Set this to false
// when this information is not required to safe resources.
#define DEBUG true

// ------------------------------ //
//             OBJECTS            //
// ------------------------------ //

// Distance tracking sensor to measure the distance in front of the robot, uses
// the I2C protocol
VL53L1X distance_sensor(I2C_SDA, I2C_SCL); // SDA(D4) SCL(D5)

// The line tracking camera (Pixy2). Uses the SPI protocol (MOSI, MISO, SCK) for
// communication. MOSI(D11) MISO(D12) SCK(D13)
Pixy2 pixy_camera;

// Motor on the left side of the motor (PWM FWD BCK)
Motor motor_left(A6, D4, D5);

// Motor on the right side of the motor (PWM FWD BCK)
Motor motor_right(D3, D6, D7);

// Set-up a default timer that runs since the robot has initialized
Timer main_timer;

// ------------------------------ //
//            VARIABLES           //
// ------------------------------ //

// Automatically updated, the last time the distance sensor has updated is
// stored in this variable in an unsigned 32 bit number. Time stored here is in
// milliseconds and corresponds with DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS.
uint32_t distance_sensor_read_last_milliseconds = 0;

// The last measured distance is stored in this variable. Measured distance is
// in millimeters. This variable is updated with an interval of
// DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS milliseconds in the loop() function.
int16_t distance_measured_in_mm = 0;

// This is the raw location of the line as detected by the Pixy2 camera.
// Depending of LINE_DETECTION_LOOK_AHEAD this can either be x0 or x1 from the
// line vector.
int16_t pixy2_line_vector_location = 0;

// This takes the value from 'pixy2_line_vector_location' and calculates the
// location percentage on the screen using the Pixy2's frame width variable.
// Ranging from 0%(left) to 100%(right). 50% is the center and thus the target
// position for the line following.
int16_t pixy2_line_vector_location_percentage = 0;

// ------------------------------ //
//            FUNCTIONS           //
// ------------------------------ //

// Dummy functions to be populated below, needed here because it needs to be
// defined before 'int main() {'
// setup is executed once when the code starts up
void setup();
// loop is executed continuesly after setup has finished
void loop();

// Executed when the microcontroller started, needs to have a while loop that
// runs forever to prevent a dead lock that requires a power-cycle. This
// function executes setup() once to initialize things like the pixy camera,
// motor controllers, distance sensor, etc... and after that executes loop()
// continously to let the robot function as it should. Like line following,
// package delivery, distance tracking, etc...
int main() {
  setup();

  while (true) {
    loop();
  }
}

// initialize things like the pixy camera,  motor controllers, distance sensor,
// etc...
void setup() {
  // ---------- //
  //  GENERAL   //
  // ---------- //

  // Wait a second before running the set-up code to make sure the Pixy2
  // camera module has started and can be initialized. This prevents unnecessary
  // timing issues between different microcontrollers (STM32 <=SPI=> Pixy2)
  ThisThread::sleep_for(1000ms);

  // Start the main timer to run as long as the robot is online.
  main_timer.start();

  // ---------- //
  //   PIXY2    //
  // ---------- //

  // Initialize the Pixy2 camera module. This allows the code to communicate to
  // the Pixy2. Just a slap in the face to wake the Pixy2 module up and set-up
  // the SPI interface to it.
  pixy_camera.init();

  // Change the pixy to run in line-tracking mode. This is an internal feature
  // in the Pixy2 camera and gives the vector(s) back that can be used to keep
  // track of the line.
  pixy_camera.changeProg("line_tracking");

  // Turn all light on the Pixy2 module on. setLamp(top, bottom), top are the
  // two bright white LEDs to illuminate the road in front of it. bottom is the
  // RGB LED on the bottom set as full white to help illuminate the road.
  pixy_camera.setLamp(1, 1);

  // Set the line features to onlt send the line vector(s) instead of line
  // vector(s), barcodes and intersections. This saves up some time when
  // communicating over SPI. To get all features instead of only the vector(s)
  // set it to: (LINE_VECTOR | LINE_INTERSECTION | LINE_BARCODE).
  pixy_camera.line.setMode(LINE_VECTOR);

  // ---------- //
  //  VL53L1X   //
  // ---------- //

  // Initialize the VL53L1X time-of-flight sensor and wake it up. Initializes
  // the I2C communication to the sensor as well.
  distance_sensor.begin();

  // Set the sense modus. 2: Long range
  distance_sensor.setDistanceMode(2);

  // ---------- //
  //   MOTORS   //
  // ---------- //

  // Turn the motors off on startup to make sure it doesn't drive when it
  // shouldn't. Just a safety measure.
  motor_left.speed(0.0);
  motor_right.speed(0.0);
}

// Executed continously to let the robot function as it should. Like line
// following, package delivery, distance tracking, etc...
void loop() {
  // ---------- //
  //  GENERAL   //
  // ---------- //

  // Initialize the two drive motor speed variables. By default this is 0(off)
  // and will be updated in this loop() function.
  float motor_left_power = 0.0;
  float motor_right_power = 0.0;

  // ---------- //
  //   PIXY2    //
  // ---------- //

  // For unknown reasons it might occur that the resolution is not retreived
  // correctly. By polling it in this loop() function it will correct itself.
  if (pixy_camera.frameWidth == 0 || pixy_camera.frameHeight == 0) {
    pixy_camera.getResolution();
  }

  // Get a single vector from the Pixy2 camera.
  pixy_camera.line.getMainFeatures(LINE_VECTOR);

  // Update the variable with the correct x coordinate as retrieved from the
  // Pixy2
  pixy2_line_vector_location = LINE_DETECTION_LOOK_AHEAD
                                   ? pixy_camera.line.vectors[0].m_x1
                                   : pixy_camera.line.vectors[0].m_x0;

  pixy2_line_vector_location_percentage =
      (pixy2_line_vector_location * 100) / pixy_camera.frameWidth;

  // Print the line location over serial USB if definition DEBUG is
  // enabled(true).
  // Example output when debug_line_position_width is 12: |  ^ ||    |.
  // W = wall, L = detected line position C = center     W^ L^C^^   W^.
  // The size (and thus resolution) can be adjusted by changing the varialbe
  // 'debug_line_position_width'.
  uint8_t debug_line_position_width = 12;
  for (int i = 0; i < debug_line_position_width; i++) {
    if (i == 0 || i == debug_line_position_width - 1) {
      printf("|");
    } else if (i ==
               (int)(pixy2_line_vector_location * debug_line_position_width) /
                   100) {
      printf("^");
    } else if (debug_line_position_width % 2 == 0
                   ? (i == (int)(debug_line_position_width / 2) ||
                      i == (int)(debug_line_position_width / 2) - 1)
                   : (i == (int)(debug_line_position_width / 2))) {
      printf("|");
    } else {
      printf(" ");
    }
  }

  // Line: |   ||   |

  // Read the milliseconds since the startup of the microcontroller and store it
  // in the local variable 'current_ms' accessable in the loop function. Stored
  // in an unsigned 32 bit variable to prevent overflow. 32 bit unsigned integer
  // has a max size of 4,294,967,295 which means that the microprocessor will
  // overflow after about 49.7 days which is most plenty time to execute the
  // required parcours. With an unsigned 16 bit number, which has a maximum size
  // of 65535, it will overflow after about 10.9 minutes which would be enough
  // but for safety an unsiged 32 bit number is used instead.
  uint32_t current_ms =
      chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
          .count();

  // ---------- //
  //  VL53L1X   //
  // ---------- //

  // Update the distance_measured_in_mm variable every
  // DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS milliseconds since microcontroller
  // startup. This is a non-block proces that uses the Mbed Timer module to
  // update its variable. The timing is calculated by doing: current
  // time(current_ms) - last time(distance_sensor_read_last_milliseconds) > max
  // time(DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS). When that is true the last
  // time(distance_sensor_read_last_milliseconds) variable is set to the current
  // time(current_ms) and the cycle starts over.
  if (current_ms - distance_sensor_read_last_milliseconds >
      DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS) {
    // Update the last milliseconds to the current milliseconds
    distance_sensor_read_last_milliseconds = current_ms;
    // Has the measurement completed and is the data ready to be read
    if (distance_sensor.newDataReady()) {
      // Update the distance variable for the robot to use including the
      // predefined offset(DISTANCE_SENSOR_OFFSET_IN_MM)
      distance_measured_in_mm =
          distance_sensor.getDistance() + DISTANCE_SENSOR_OFFSET_IN_MM;
    } else {
      // Tell the VL53L1X sensor to start a measurement.
      distance_sensor.startMeasurement();
    }
  }

  // Is the distance in front of the robot smaller than the predefined target
  // distance. Stop the motors.
  if (distance_measured_in_mm < STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM) {
  }

  // Limit the drive motors maximum speed. This works with a single line if else
  // statement in the format: (variable = condition ? true : false).
  motor_left_power = motor_left_power > ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                         ? ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                         : motor_left_power;
  motor_right_power = motor_right_power > ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                          ? ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                          : motor_right_power;

  // When DEBUG is enabled(true) print a new line character.
  if (DEBUG)
    printf("\n");
}

// <<EOF>>