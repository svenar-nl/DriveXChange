#include "robot.h"
#include "AnalogIn.h"
#include "DigitalOut.h"
#include "PwmOut.h"

ROBOT::ROBOT() :
_line_sensor_left(A0), _line_sensor_right(A0),
_motor_left_pwm1(D0), _motor_left_pwm2(D0), _motor_left_reset(D0), _motor_left_current_sensor(A0),
_motor_right_pwm1(D0), _motor_right_pwm2(D0), _motor_right_reset(D0), _motor_right_current_sensor(A0),
_motor_forklift_pwm1(D0), _motor_forklift_pwm2(D0), _motor_forklift_reset(D0),
_limit_switch_up(D0),_limit_switch_down(D0) {

}

void ROBOT::setup_pins_linedetection(PinName line_sensor_left, PinName line_sensor_right) {
    _line_sensor_left = mbed::AnalogIn(line_sensor_left);
    _line_sensor_right = mbed::AnalogIn(line_sensor_right);
}

void ROBOT::setup_pins_motor_left(PinName pwm1, PinName pwm2, PinName rst) {
    _motor_left_pwm1 = mbed::PwmOut(pwm1);
    _motor_left_pwm2 = mbed::PwmOut(pwm2);
    _motor_left_reset = mbed::DigitalOut(rst);
}

void ROBOT::setup_pins_motor_right(PinName pwm1, PinName pwm2, PinName rst) {
    _motor_right_pwm1 = mbed::PwmOut(pwm1);
    _motor_right_pwm2 = mbed::PwmOut(pwm2);
    _motor_right_reset = mbed::DigitalOut(rst);
}

void ROBOT::setup_pins_motor_left_current_measurement(PinName current_sensor) {
    _motor_left_current_sensor = mbed::AnalogIn(current_sensor);
}

void ROBOT::setup_pins_motor_right_current_measurement(PinName current_sensor) {
    _motor_right_current_sensor = mbed::AnalogIn(current_sensor);
}

void ROBOT::setup_pins_motor_forklift(PinName pwm1, PinName pwm2, PinName rst) {
    _motor_forklift_pwm1 = mbed::PwmOut(pwm1);
    _motor_forklift_pwm2 = mbed::PwmOut(pwm2);
    _motor_forklift_reset = mbed::DigitalOut(rst);
}

void ROBOT::setup_pins_forklift_limit_switches(PinName limit_switch_up, PinName limit_switch_down) {
    _limit_switch_up = mbed::DigitalIn(limit_switch_up);
    _limit_switch_down = mbed::DigitalIn(limit_switch_down);
}

void ROBOT::run() {

}