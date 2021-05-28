#include "AnalogIn.h"
#include "DigitalIn.h"
#include "Motor.h"
#include "PinNames.h"
#include "Pixy2.h"
#include "PwmOut.h"
#include "VL53L1X.h"
#include "mbed.h"
#include "src/robot.h"

// 8 bit - 256
// 16 bit - 65535
// 32 bit - 4294967295

// =====-----------------------------------------------=====
//
//                      PIN DEFINITION
//
// =====-----------------------------------------------=====
DigitalIn HW_SWITCH1(D2); // user defined switch on the PCB
DigitalOut HW_LED1(LED1); // Led on PCB

// PinName HW_MRS1(D6); // Motor 1 reset pin // D6
// PinName HW_MRS1(D2); // Motor 1 reset pin // D6
// PinName HW_MRS2(D7); // Motor 2 reset pin
// PinName HW_MRS3(D8); // Motor 3 reset pin (forklift)

// PinName HW_M1IN1(D11); // Motor 1 control pin 1 // D11
// PinName HW_M1IN2(D12); // Motor 1 control pin 2 // D12
// PinName HW_M1IN1(D4); // Motor 1 control pin 1 // D11
// PinName HW_M1IN2(D5); // Motor 1 control pin 2 // D12

// PinName HW_M2IN1(D9);  // Motor 2 control pin 1
// PinName HW_M2IN2(D10); // Motor 2 control pin 2

// PinName HW_M3IN1(A6); // Motor 3 control pin 1 (forklift)
// PinName HW_M3IN2(D3); // Motor 3 control pin 2 (forklift)

// PinName HW_LINE_DAT1(A0); // Line detection sensor 1
// PinName HW_LINE_DAT2(A1); // Line detection sensor 2

// PinName HW_CURRENTDETOUT1(A2); // Motor 1 current detection
// PinName HW_CURRENTDETOUT2(A3); // Motor 2 current detection

// PinName HW_HEFSW1(A4); // Forklift limit switch 1
// PinName HW_HEFSW2(A5); // Forklift limit switch 2

// =====-----------------------------------------------=====
//
//                      PIN DEFINITION
//
// =====-----------------------------------------------=====

// =====-----------------------------------------------=====
//
//                   LIBRARY DEFINITION
//
// =====-----------------------------------------------=====
ROBOT robot;
Timer main_timer;
Timer signal_timer;
VL53L1X distance_sensor(I2C_SDA, I2C_SCL);
Pixy2 pixy;
Motor motor_left(A6, D7, D8);   // pwm, fwd, rev
Motor motor_right(D3, D9, D10); // pwm, fwd, rev
// =====-----------------------------------------------=====
//
//                   LIBRARY DEFINITION
//
// =====-----------------------------------------------=====

// =====-----------------------------------------------=====
//
//                   FUNCTION DEFINITION
//
// =====-----------------------------------------------=====
void setup();
void loop();
// =====-----------------------------------------------=====
//
//                   FUNCTION DEFINITION
//
// =====-----------------------------------------------=====

uint16_t current_distance_travelled = 0;
    	
void compare_distance()
{
    

}

int main() {
  setup();

  while (true) {
    loop();
  }
}

// Executed once on startup
void setup() {
  // Start timer and pass it to the robot logic
  main_timer.start();
  signal_timer.start();

  // Pass the timer instance to the robot code
  robot.setup_timer_instance(&main_timer);

  // Initialize the distance sensor
  robot.setup_distance_sensor(500); // read distance every 500ms

  // Change the distance sensor offset
  robot.set_distance_sensor_offset(85);

  // Initialize the line detection sensor pins
  // robot.setup_pins_linedetection(HW_LINE_DAT1, HW_LINE_DAT2);

  // // Initialize the drive motors pins
  // robot.setup_pins_motor_left(HW_M1IN1, HW_M1IN2);
  // robot.setup_pins_motor_right(HW_M2IN1, HW_M2IN2);

  // // initialize the drive motors current sensor pins
  // robot.setup_pins_motor_left_current_measurement(HW_CURRENTDETOUT1);
  // robot.setup_pins_motor_right_current_measurement(HW_CURRENTDETOUT2);

  // // Initialize the forklift motor & sensor pins
  // robot.setup_pins_motor_forklift(HW_M3IN1, HW_M3IN2);
  // robot.setup_pins_forklift_limit_switches(HW_HEFSW1, HW_HEFSW2); //
  // limit_switch_up, limit_switch_down
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

// Executed continuously
void loop() {
  // Blink onboard LED when the switch is off, otherwise run the robot code
  if (HW_SWITCH1 == 0) {
    // HW_LED1 = !HW_LED1;
    // ThisThread::sleep_for(500ms);
    if (signal_timer.read_ms() % 500 == 0) {
      HW_LED1 = !HW_LED1;
    }
  } else {
    robot.run();
    if (signal_timer.read_ms() % (HW_LED1 ? 25 : 250) == 0) {
      HW_LED1 = !HW_LED1;
    }
  }
}

#pragma GCC diagnostic pop