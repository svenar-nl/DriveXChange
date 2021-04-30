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

  // Start the main timer
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
      // Update the distance variable for the robot to use
      distance_measured_in_mm = distance_sensor.getDistance();
    } else {
      // Tell the VL53L1X sensor to start a measurement.
      distance_sensor.startMeasurement();
    }
  }
}

// <<EOF>>