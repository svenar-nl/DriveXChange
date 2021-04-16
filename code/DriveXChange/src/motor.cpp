#include "motor.h"

Motor::Motor(PinName pwm1, PinName pwm2) : _motor_in1(D0), _motor_in2(D0), _motor_in3(D0), _motor_in4(D0) {
    _motor_in1 = mbed::PwmOut(pwm1);
    _motor_in2 = mbed::PwmOut(pwm2);
}

Motor::Motor(PinName pwm1, PinName pwm2, PinName pwm3, PinName pwm4) : _motor_in1(D0), _motor_in2(D0), _motor_in3(D0), _motor_in4(D0) {
    _motor_in1 = mbed::PwmOut(pwm1);
    _motor_in2 = mbed::PwmOut(pwm2);
    _motor_in3 = mbed::PwmOut(pwm3);
    _motor_in4 = mbed::PwmOut(pwm4);
}

void Motor::stop() {

}

void Motor::forward(int speed_percent) {

}

void Motor::back(int speed_percent) {

}