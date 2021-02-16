#pragma once
#include "DigitalIn.h"
#include "DigitalOut.h"
#include "I2C.h"
#include "PwmOut.h"
#ifndef ROBOT_H
#define ROBOT_H

#include "AnalogIn.h"
#include "PinNames.h"

class ROBOT {
    public:
        ROBOT(); // Constructor
        void setup_pins_linedetection(PinName line_sensor_left, PinName line_sensor_right);
        void setup_pins_motor_left(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_motor_right(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_motor_left_current_measurement(PinName current_sensor);
        void setup_pins_motor_right_current_measurement(PinName current_sensor);
        void setup_pins_motor_forklift(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_forklift_limit_switches(PinName limit_switch_up, PinName limit_switch_down);
        void run();
    private:
        mbed::AnalogIn _line_sensor_left;
        mbed::AnalogIn _line_sensor_right;

        mbed::PwmOut _motor_left_pwm1;
        mbed::PwmOut _motor_left_pwm2;
        mbed::DigitalOut _motor_left_reset;
        mbed::AnalogIn _motor_left_current_sensor;

        mbed::PwmOut _motor_right_pwm1;
        mbed::PwmOut _motor_right_pwm2;
        mbed::DigitalOut _motor_right_reset;
        mbed::AnalogIn _motor_right_current_sensor;

        mbed::PwmOut _motor_forklift_pwm1;
        mbed::PwmOut _motor_forklift_pwm2;
        mbed::DigitalOut _motor_forklift_reset;

        mbed::DigitalIn _limit_switch_up;
        mbed::DigitalIn _limit_switch_down;

        mbed::I2C HW_I2C(PinName(I2C_SDA), PinName(I2C_SCL)); // sda(PB_7), scl(PB_6)
};

#endif