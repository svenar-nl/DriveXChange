#include "sensors.h"

void CNY70::setup(int pin_analog) {

}

bool CNY70::line_visible() {
    return false;
}

void VL53L1X::setup(int pin_sda, int pin_scl) {

}

int VL53L1X::distance_cm() {
    return 0;
}