#pragma once
#include "BufferedSerial.h"
#include <cstdint>
#ifndef MOTOR_H
#define MOTOR_H

#include "PinNames.h"
#include "PwmOut.h"

extern uint32_t distance_read_last_milliseconds;

class Motor {
    public:
        Motor(PinName pwm1, PinName pwm2); // Constructor
        Motor(PinName pwm1, PinName pwm2, PinName pwm3, PinName pwm4); // Constructor
        void stop();
        void forward(int speed_percent);
        void back(int speed_percent);
        
    private:

        //

        int wheel_diameter_in_mm = 150;
        int distance_traveled_in_mm = 0;

        // 
        
        mbed::PwmOut _motor_in1;
        mbed::PwmOut _motor_in2;
        mbed::PwmOut _motor_in3;
        mbed::PwmOut _motor_in4;

};

#endif