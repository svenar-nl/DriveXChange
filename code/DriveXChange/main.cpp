#include "AnalogIn.h"
#include "DigitalIn.h"
#include "PinNames.h"
#include "PwmOut.h"
#include "mbed.h"

#include "src/robot.h"
// =====-----------------------------------------------=====
//
//                      PIN DEFINITION
//
// =====-----------------------------------------------=====
DigitalIn HW_SWITCH1(D2); // user defined switch on the PCB
DigitalOut HW_LED1(LED1); // Led on PCB

PinName HW_MRS1(D6); // Motor 1 reset pin
PinName HW_MRS2(D7); // Motor 2 reset pin
PinName HW_MRS3(D8); // Motor 3 reset pin (forklift)

PinName HW_M1IN1(D11); // Motor 1 control pin 1
PinName HW_M1IN2(D12); // Motor 1 control pin 2

PinName HW_M2IN1(D9); // Motor 2 control pin 1
PinName HW_M2IN2(D10); // Motor 2 control pin 2

PinName HW_M3IN1(A6); // Motor 3 control pin 1 (forklift)
PinName HW_M3IN2(A7); // Motor 3 control pin 2 (forklift)

PinName HW_LINE_DAT1(A0); // Line detection sensor 1
PinName HW_LINE_DAT2(A1); // Line detection sensor 2

PinName HW_CURRENTDETOUT1(A2); // Motor 1 current detection
PinName HW_CURRENTDETOUT2(A3); // Motor 2 current detection

PinName HW_HEFSW1(A4); // Forklift limit switch 1
PinName HW_HEFSW2(A5); // Forklift limit switch 2

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

int main() {
    setup();

    while (true) {
        loop();
    }
}

// Executed once on startup
void setup() {
    // Initialize the line detection sensor pins
    robot.setup_pins_linedetection(HW_LINE_DAT1, HW_LINE_DAT2);

    // Initialize the drive motors pins
    robot.setup_pins_motor_left(HW_M1IN1, HW_M1IN2, HW_MRS1);
    robot.setup_pins_motor_right(HW_M2IN1, HW_M2IN2, HW_MRS2);

    // initialize the drive motors current sensor pins
    robot.setup_pins_motor_left_current_measurement(HW_CURRENTDETOUT1);
    robot.setup_pins_motor_right_current_measurement(HW_CURRENTDETOUT2);

    // Initialize the forklift motor & sensor pins
    robot.setup_pins_motor_forklift(HW_M3IN1, HW_M3IN2, HW_MRS3);
    robot.setup_pins_forklift_limit_switches(HW_HEFSW1, HW_HEFSW2); // limit_switch_up, limit_switch_down
}

// Executed continuously
void loop() {
    // Blink onboard LED when the switch is off, otherwise run the robot code
    if (HW_SWITCH1 == 0) {
        HW_LED1 = !HW_LED1;
        ThisThread::sleep_for(500ms);
    } else {
        robot.run();
    }
}