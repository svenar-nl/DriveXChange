#pragma once
#include "BufferedSerial.h"
#include <cstdint>
#ifndef ROBOT_H
#define ROBOT_H

#include "DigitalIn.h"
#include "DigitalOut.h"
#include "AnalogIn.h"
#include "PinNames.h"
#include "I2C.h"
#include "PwmOut.h"
#include "Timer.h"
#include "VL53L1X.h"
#include "Motor.h"

extern uint32_t distance_read_last_milliseconds;

class ROBOT {
    public:
        ROBOT(); // Constructor
        void run();
        void setup_timer_instance(mbed::Timer* timer);
        void setup_pins_linedetection(PinName line_sensor_left, PinName line_sensor_right);
        void setup_pins_motor_left(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_motor_right(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_motor_left_current_measurement(PinName current_sensor);
        void setup_pins_motor_right_current_measurement(PinName current_sensor);
        void setup_pins_motor_forklift(PinName pwm1, PinName pwm2, PinName rst);
        void setup_pins_forklift_limit_switches(PinName limit_switch_up, PinName limit_switch_down);
        void setup_distance_sensor(int read_interval);
        void set_distance_sensor_offset(int offset);
    private:
        void update_distance_sensor();

        //

        int distance_update_interval_in_ms = 500;
        uint16_t distance_in_front_of_robot_mm = 0;
        uint16_t robot_distance_offset = 0; // What offset for the distance sensor to process? (EX.sensor detects 300mm but the sensor is placed back 100mm in the robot, that means the distance in front of the robot is actually 200mm)

        // 
        
        mbed::Timer* _main_timer;

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

        VL53L1X _distance_sensor;

        // Motor motor_left;  // TODO C++ why u so bitch
        // Motor motor_right; // TODO C++ why u so bitch
};

#endif