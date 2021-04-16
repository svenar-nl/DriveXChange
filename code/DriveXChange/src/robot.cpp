#include "robot.h"
#include "AnalogIn.h"
#include "DigitalOut.h"
#include "PinNames.h"
#include "PwmOut.h"
#include "VL53L1X.h"
#include <cstdio>

uint32_t distance_read_last_milliseconds = 0;

ROBOT::ROBOT() :
_line_sensor_left(A0), _line_sensor_right(A0),
_motor_left_pwm1(D0), _motor_left_pwm2(D0), _motor_left_reset(D0), _motor_left_current_sensor(A0),
_motor_right_pwm1(D0), _motor_right_pwm2(D0), _motor_right_reset(D0), _motor_right_current_sensor(A0),
_motor_forklift_pwm1(D0), _motor_forklift_pwm2(D0), _motor_forklift_reset(D0),
_limit_switch_up(D0),_limit_switch_down(D0),
_distance_sensor(I2C_SDA, I2C_SCL), motor_left(D0, D0), motor_right(D0, D0) {
    
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void ROBOT::run() {
    uint32_t current_ms = _main_timer->read_ms();
    if (current_ms - distance_read_last_milliseconds > distance_update_interval_in_ms) {
        distance_read_last_milliseconds = current_ms;
        update_distance_sensor();
    }

    printf("Distance: %dmm\n", distance_in_front_of_robot_mm);
}
#pragma GCC diagnostic pop

void ROBOT::setup_timer_instance(mbed::Timer* timer) {
    _main_timer = timer;
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

void ROBOT::setup_distance_sensor(int read_interval) {
    // Setup distance sensor and set I2C to 2.8V mode. In this mode 3.3V I2C is allowed.
    _distance_sensor.begin();
    _distance_sensor.setDistanceMode(2); // 2: Long range

    // Set distance sensor read interval
    distance_update_interval_in_ms = read_interval;
}

void ROBOT::update_distance_sensor() {

    if (_distance_sensor.newDataReady()) {
        distance_in_front_of_robot_mm = _distance_sensor.getDistance() + robot_distance_offset;
    } else {
        _distance_sensor.startMeasurement();
    }
}

void ROBOT::set_distance_sensor_offset(int offset) {
    robot_distance_offset = offset;
}