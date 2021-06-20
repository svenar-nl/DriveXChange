// ------------------------------ //
//            INCLUDES            //
// ------------------------------ //

// #define DISTANCE_SENSOR_TOF
#include <chrono>
#define DISTANCE_SENSOR_ULTRASONIC

#include "Motor.h"
#include "PIDLoop.h"
#include "Pixy2.h"
#include "PixySettings.h"
#include "ThisThread.h"
#include "mbed.h"
#include <cstdint>

#if !defined(DISTANCE_SENSOR_TOF) && !defined(DISTANCE_SENSOR_ULTRASONIC)
#error "No distance sensor has been defined!"
#endif

#ifdef DISTANCE_SENSOR_TOF
#include "VL53L1X.h"
#endif

#ifdef DISTANCE_SENSOR_ULTRASONIC
#include "ultrasonic.h"
#endif

// ------------------------------ //
//           DEFINITIONS          //
// ------------------------------ //

// Pin definitions for I2C
#define I2C_SDA D4
#define I2C_SCL D5

// Update interval of the Time-of-Flight(Vl53L1X) sensor. Time is in
// milliseconds
#define DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS 50

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

// Limit the minimal and top speed of the robot drive motors. This value is in
// percent ranging from 0% to 100%. When the left or right drive motor exedes
// the maximum speed it will be limited to this value. And below the minimal
// value , the motors will be stopped.
#define ROBOT_MIN_SPEED_IN_PERCENT 30
#define ROBOT_MAX_SPEED_IN_PERCENT 100

// Should the Pixy2 camera use the x coorninate in the distance(true) or
// closeby(false). By using closeby(false) the x0 location returned by the Pixy2
// line vector will be used, this is on the lower part of the camera, or closer
// to the robot. This is a safe way of following ta line without 'corner
// cutting'. By using distance(true) the x1 location returned by the Pixy2 line
// vector will be used, this is on the upper part of the camera, or further away
// from robot. This will allow the robot to cut corners instead of following it
// strictly.
#define LINE_DETECTION_LOOK_AHEAD false

// When true print information over serial USB to the connected PC. Use a serial
// terminal with a baudrate of 115200 to watch the output. Set this to false
// when this information is not required to safe resources.
#define DEBUG true

// How many milliseconds should be between each debug message, serial
// communication is slow, limiting its print interval helps the general code
// speed to prevent unforseen issues. Uses the 'main_timer' for timing.
#define DEBUG_PRINT_INTERVAL 500

// defined constrain() as a macro (rather than a function)
// Constrains a number to be within a range.
#define constrain(amt, low, high)                                              \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// The wheel diameter of the robot divided by 1000
#define WHEEL_DIAMETER 150 / 1000

#define NUMBER_OF_MAGNETS_ON_WHEEL 1

// Target distance to deliver the packet at
#define PACKET_TARGET_DISTANCE_IN_MM 10 * 1000

// Rough PI
#define PI 3.1415

// ------------------------------ //
//             OBJECTS            //
// ------------------------------ //

#ifdef DISTANCE_SENSOR_TOF
// Distance tracking sensor to measure the distance in front of the robot, uses
// the I2C protocol
VL53L1X distance_sensor(I2C_SDA, I2C_SCL); // SDA(D4) SCL(D5)
#endif

#ifdef DISTANCE_SENSOR_ULTRASONIC
// Use same pins as the ToF distance sensor for ease of use
// I2C_SCL = TRIG
// I2C_SDA = ECHO
ULTRASONIC distance_sensor(I2C_SCL, I2C_SDA);
#endif

// The line tracking camera (Pixy2). Uses the SPI protocol (MOSI, MISO, SCK) for
// communication. MOSI(D11) MISO(D12) SCK(D13)
Pixy2 pixy_camera;

// Motor on the left side of the motor (PWM FWD BCK)
Motor motor_left(A6, D8, D9);

// Motor on the right side of the motor (PWM FWD BCK)
Motor motor_right(D3, D6, D7);

// Hefmotor
Motor hef_motor(A3, A0, A1);

// Set-up a default timer that runs since the robot has initialized
Timer main_timer;

// Hall-effect sensor to measure the rotations of then weel
DigitalIn halleffect(D2, PullDown);

// Low = off
// High = on
DigitalIn start_button(A2, PullDown);

// PID for pixy
PIDLoop headingLoop(5000, 0, 0, false);

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
uint32_t distance_measured_in_mm = 0;

// This is the raw location of the line as detected by the Pixy2 camera.
// Depending of LINE_DETECTION_LOOK_AHEAD this can either be x0 or x1 from the
// line vector.
int16_t pixy2_line_vector_location = 0;

// This takes the value from 'pixy2_line_vector_location' and calculates the
// location percentage on the screen using the Pixy2's frame width variable.
// Ranging from 0%(left) to 100%(right). 50% is the center and thus the target
// position for the line following.
int16_t pixy2_line_vector_location_percentage = 0;

// Automatically updated, the last time a debug message has printed is
// stored in this variable in an unsigned 32 bit number. Time stored here is in
// milliseconds and corresponds with DEBUG_PRINT_INTERVAL.
uint32_t last_debug_message_time = 0;

//
//
//
uint32_t start_time_hef = 0;

// How far has the robot traveled in meters? Calculated using the halleffect
// sensor on one of the robots wheels
float robot_distance_traveled = 0;

// Has a pulse been detected on the halleffect sensor? Prevents continuous
// execution while the pulse is active
bool halleffect_pulse_detected = false;

// True if the package has been delivered and the robot can drive to the end
bool has_delivered_package = false;

uint32_t last_not_started_millis = 0;
uint8_t last_not_started_toggle = 0;

enum HEF_STATE {
  READY = 0,
  MOVE_UP = 1,
  WAIT_UP = 2,
  MOVE_DOWN = 3,
  WAIT_DOWN = 4,
  DONE = 5
};

HEF_STATE current_hef_state = READY;

// ------------------------------ //
//            FUNCTIONS           //
// ------------------------------ //

// Dummy functions to be populated below, needed here because it needs to be
// defined before 'int main() {'

// setup is executed once when the code starts up
void setup();
// loop is executed continuesly after setup has finished
void loop();

// Map a specific value to a different range
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Play tones on the motors
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void tone(uint16_t period, uint16_t delay) {
  uint16_t note = 2200 - period;
  if (period > 0) {
    motor_left.period_us(note);
    motor_right.period_us(note);
    motor_left.speed(0.1f);
    motor_right.speed(0.1f);
  }

  ThisThread::sleep_for(delay);
  //   ThisThread::sleep_for(chrono::duration<chrono::milliseconds>(delay).count());
  motor_left.period(0.001);
  motor_right.period(0.001);
  motor_left.speed(0.0f);
  motor_right.speed(0.0f);
}
#pragma GCC diagnostic pop

/**
    @return true to end loop
*/
bool hefmotor() {
  // programmeer hefmotor
  // Begin time for Hef
  uint32_t current_ms_hef =
      chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
          .count();

  uint32_t delta_time = current_ms_hef - start_time_hef;

  switch (current_hef_state) {
  case READY:
    tone(1500, 100);
    tone(1100, 100);
    tone(1500, 100);
    tone(1100, 100);
    start_time_hef =
        chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
            .count();
    current_hef_state = MOVE_UP;
    break;
  case MOVE_UP:
    hef_motor.speed(map(delta_time, 0, 2000, 0, 100) / 100);

    if (delta_time > 5000) {
      start_time_hef =
          chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
              .count();
      current_hef_state = WAIT_UP;
    }
    break;
  case WAIT_UP:
    if (delta_time > 1000) {
      start_time_hef =
          chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
              .count();
      current_hef_state = MOVE_DOWN;
    }
    break;
  case MOVE_DOWN:
    hef_motor.speed(map(delta_time, 0, 2000, 0, 100) / 100);

    if (delta_time > 5000) {
      start_time_hef =
          chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
              .count();
      current_hef_state = WAIT_DOWN;
    }
    break;
  case WAIT_DOWN:
    if (delta_time > 1000) {
      start_time_hef =
          chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
              .count();
      current_hef_state = DONE;
    }
    break;
  case DONE:
    tone(1100, 100);
    tone(1500, 100);
    tone(1100, 100);
    tone(1500, 100);
    return true;
    break;
  }

  // uint32_t delta_time = current_ms_hef - start_time_hef;

  // if(delta_time <= 10000) {
  //     hef_motor.speed(100);
  //     return false;
  // } else{
  //     return true;
  // }
  return false;
}

void handle_packet_delivery() {
  if (has_delivered_package)
    return;

  if (robot_distance_traveled >= PACKET_TARGET_DISTANCE_IN_MM) {
    motor_left.speed(0);
    motor_right.speed(0);

    // Begin timer to compare with current_ms
    start_time_hef =
        chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
            .count();
    while (!hefmotor())
      ;

    has_delivered_package = true;
  }
}

// Executed when the microcontroller started, needs to have a while loop that
// runs forever to prevent a dead lock that requires a power-cycle. This
// function executes setup() once to initialize things like the pixy camera,
// motor controllers, distance sensor, etc... and after that executes loop()
// continously to let the robot function as it should. Like line following,
// package delivery, distance tracking, etc...
int main() {
  setup();

  while (true) {
    if (start_button) {
      loop();
    } else {

      uint32_t current_ms =
          chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
              .count();
      if (current_ms - last_not_started_millis > 250) {
        last_not_started_millis = current_ms;
        last_not_started_toggle = last_not_started_toggle == 0 ? 1 : 0;
        pixy_camera.setLamp(last_not_started_toggle, last_not_started_toggle);
      }

      motor_left.speed(0);
      motor_right.speed(0);
    }
  }
}

// initialize things like the pixy camera,  motor controllers, distance sensor,
// etc...
void setup() {
  // ---------- //
  //  GENERAL   //
  // ---------- //

  printf("Starting...\n");

  // Wait a few seconds before running the set-up code to make sure the Pixy2
  // camera module has started and can be initialized. This prevents unnecessary
  // timing issues between different microcontrollers (STM32 <=SPI=> Pixy2)
  ThisThread::sleep_for(2000ms);

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

#ifdef DISTANCE_SENSOR_TOF
  // Initialize the VL53L1X time-of-flight sensor and wake it up. Initializes
  // the I2C communication to the sensor as well.
  distance_sensor.begin();

  // Set the sense modus. 2: Long range
  distance_sensor.setDistanceMode(2);
#endif

  // ---------- //
  // ULTRASONIC //
  // ---------- //

#ifdef DISTANCE_SENSOR_ULTRASONIC
  distance_sensor.set_measure_speed(100);
  distance_sensor.start();
#endif

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

  // When true, a debug message will be printed to serial USB
  bool do_print_debug = false;
  if (DEBUG) {
    if (current_ms - last_debug_message_time > DEBUG_PRINT_INTERVAL) {
      last_debug_message_time = current_ms;
      do_print_debug = true;
    }
  }

  // ---------- //
  // HALLEFFECT //
  // ---------- //

  if (!halleffect) {
    if (!halleffect_pulse_detected) {
      robot_distance_traveled += (float)((float)WHEEL_DIAMETER * PI) /
                                 (float)NUMBER_OF_MAGNETS_ON_WHEEL;

      // Deze toggle functie zorgt dat dit gedeelte van de functie niet meerdere
      // keren per rondslag afstand optelt.
      halleffect_pulse_detected = true;
    }
  } else {
    halleffect_pulse_detected = false;
  }

  if (do_print_debug) {
    printf("[traveled %.2fm] ", robot_distance_traveled);
  }

  // ---------- //
  //  PACKAGE   //
  //  HANDLING  //
  // ---------- //

  handle_packet_delivery();

  //   printf("b");

  // ---------- //
  //   PIXY2    //
  // ---------- //

  // For unknown reasons it might occur that the resolution is not retreived
  // correctly. By polling it in this loop() function it will correct itself.
  if (pixy_camera.frameWidth == 0 || pixy_camera.frameHeight == 0) {
    pixy_camera.getResolution();
    pixy_camera.setLamp(1, 1);
  }

  // Get a single vector from the Pixy2 camera.
  pixy_camera.line.getMainFeatures(LINE_VECTOR);

  // Only update the location when a vector is visible, otherwise retain the
  // last known line location.
  if (pixy_camera.line.numVectors > 0) {
    // Update the variable with the correct x coordinate as retrieved from the
    // Pixy2
    pixy2_line_vector_location = LINE_DETECTION_LOOK_AHEAD
                                     ? pixy_camera.line.vectors[0].m_x1
                                     : pixy_camera.line.vectors[0].m_x0;

    // Calculate the percentage of the vector's location on the camera frame
    pixy2_line_vector_location_percentage =
        (pixy2_line_vector_location * 100) / pixy_camera.frameWidth;
  }

  // Print the robot's uptime over serial USB when DEBUG is enabled(true).
  if (do_print_debug) {
    long milli = current_ms;
    int hr = milli / 3600000;
    milli = milli - 3600000 * hr;
    int min = milli / 60000;
    milli = milli - 60000 * min;
    int sec = milli / 1000;
    milli = milli - 1000 * sec;

    // Format XX:XX ex. 11:04 only minutes and seconds are displayed, the robot
    // won't run for more than a single hour for the demonstration.
    printf("%s%d:%s%d ", min < 10 ? "0" : "", min, sec < 10 ? "0" : "", sec);
  }

  // Print the line location over serial USB if definition DEBUG is
  // enabled(true).
  // Example output when debug_line_position_width is 12: |  ^ ||    |.
  // W = wall, L = detected line position C = center     W^ L^C^^   W^.
  // The size (and thus resolution) can be adjusted by changing the varialbe
  // 'debug_line_position_width'.
  if (do_print_debug) {
    uint8_t debug_line_position_width = 12;
    for (int i = 0; i < debug_line_position_width; i++) {
      if (i == 0 || i == debug_line_position_width - 1) {
        printf("|");
      } else if (i == (int)(pixy2_line_vector_location_percentage *
                            debug_line_position_width) /
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

    printf(" \"%d\" ", pixy2_line_vector_location_percentage);
  }

  ////////////////////////////////////////////

  uint32_t error = (int32_t)pixy2_line_vector_location_percentage - (int32_t)50;

  headingLoop.update(error);

  // 0 - 1000???
  int left = headingLoop.m_command;
  int right = -headingLoop.m_command;

  printf(" :%d, %d: ", left, right);

  // old pixy code

  //   if (pixy2_line_vector_location_percentage < 50 - PIXY2_CENTER_THRESHOLD
  //   ||
  //       pixy2_line_vector_location_percentage > 50 + PIXY2_CENTER_THRESHOLD)
  //       {
  //     if (pixy2_line_vector_location_percentage < 50) { // Line on the left
  //     side
  //       //   motor_right_power = 0;
  //       motor_right_power =
  //           map(pixy2_line_vector_location_percentage, 0, 50, 30, 100) /
  //           100.0;
  //     } else { // Line on the right side
  //       //   motor_left_power = 0;
  //       motor_left_power =
  //           map(pixy2_line_vector_location_percentage, 100, 50, 30, 100) /
  //           100.0;
  //     }
  //   } else {
  //     motor_left_power = 1.0;
  //     motor_right_power = 1.0;
  //   }

  // old pixy code

  //   if (pixy2_line_vector_location_percentage < 50 - PIXY2_CENTER_THRESHOLD)
  //   {
  //     motor_left_power =
  //         map(pixy2_line_vector_location_percentage, 50, 0, 0, 100) / 100.0;
  //     // motor_left_power = 0;
  //   } else {
  //     motor_left_power = 1.0;
  //   }

  //   if (pixy2_line_vector_location_percentage > 50 + PIXY2_CENTER_THRESHOLD)
  //   {
  //     motor_right_power =
  //         map(pixy2_line_vector_location_percentage, 50, 100, 0, 100) /
  //         100.0;
  //     // motor_right_power = 0;
  //   } else {
  //     motor_right_power = 1.0;
  //   }

  ////////////////////////////////////////////

  // ---------- //
  //  VL53L1X   //
  // ---------- //

#ifdef DISTANCE_SENSOR_TOF
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

#endif

  // ---------- //
  // ULTRASONIC //
  // ---------- //

#ifdef DISTANCE_SENSOR_ULTRASONIC
  //   distance_sensor.update();

  if (current_ms - distance_sensor_read_last_milliseconds >
      DISTANCE_SENSOR_UPDATE_INTERVAL_IN_MS) {
    // Update the last milliseconds to the current milliseconds
    distance_sensor_read_last_milliseconds = current_ms;
    distance_measured_in_mm =
        distance_sensor.get_distance() + DISTANCE_SENSOR_OFFSET_IN_MM;
  }
#endif

  // Is the distance in front of the robot smaller than the predefined target
  // distance. Stop the motors.
  if (distance_measured_in_mm < STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM) {
    tone(1500, 100);
    tone(1300, 100);
    tone(1100, 100);

    motor_left_power = 0;
    motor_right_power = 0;
    // return;
  }

  ///////////////////////////////////////
  if (distance_measured_in_mm < STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM + 500) {
    motor_left_power = .5;
    motor_right_power = .5;
  }

  // Drive back when too close
  if (distance_measured_in_mm < STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM - 10) {
    motor_left_power = -.3;
    motor_right_power = -.3;
  }

  float a = constrain(
      (distance_measured_in_mm - STOP_ROBOT_AT_DISTANCE_IN_FRONT_IN_MM) / 100.0,
      -1, 1);

  if (distance_measured_in_mm > 0) {
    motor_left_power *= a;
    motor_right_power *= a;
  }
  ////////////////////////////////////////

  // Limit the drive motors maximum speed. This works with a single line if else
  // statement in the format: (variable = condition ? true : false).
  motor_left_power = motor_left_power > ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                         ? ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                         : motor_left_power;
  motor_right_power = motor_right_power > ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                          ? ROBOT_MAX_SPEED_IN_PERCENT / 100.0
                          : motor_right_power;

  // Limit the drive motors minimal speed. This works with a single line if else
  // statement in the format: (variable = condition ? true : false).
  motor_left_power =
      (abs(motor_left_power) < ROBOT_MIN_SPEED_IN_PERCENT / 100.0)
          ? 0
          : motor_left_power;

  motor_right_power =
      (abs(motor_right_power) < ROBOT_MIN_SPEED_IN_PERCENT / 100.0)
          ? 0
          : motor_right_power;

  motor_left.speed(motor_left_power);
  motor_right.speed(motor_right_power);

  // When DEBUG is enabled(true) print a new line character.
  if (do_print_debug) {
    printf(" [%4d] :%.2f: {%.2f, %.2f}", distance_measured_in_mm, a,
           motor_left_power, motor_right_power);
    printf("\n");
  }
}

// <<EOF>>