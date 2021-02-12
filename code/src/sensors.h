#pragma once

class CNY70 {
    public:
        void setup(int pin_analog);
        bool :line_visible();
    private:
        int _data_pin;
}

class VL53L1X {
    public:
        void setup(int pin_sda, int pin_scl);
    private:
        int _sda;
        int _scl;
}