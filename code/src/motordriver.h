#pragma once

class MotorDriver {
    public:
        void setup(int pwm_pin1, int pwm_pin2);
        void move_forward(int speed); // Speed in percent 0%-100%
        void move_back(int speed); // Speed in percent 0%-100%
    private:
        int pwm_pin1;
        int pwm_pin2;
}