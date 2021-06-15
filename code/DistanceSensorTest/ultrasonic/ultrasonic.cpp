#include "ultrasonic.h"
#include "Callback.h"
#include "mbed_wait_api.h"
#include <chrono>
#include <cstdint>

ULTRASONIC::ULTRASONIC(PinName trig_pin, PinName echo_pin) : trigger(trig_pin), echo(echo_pin) {
    trigger = 0;

    measure_interval = 500;

    is_measuring = false;
    detected_pulse = false;

    begin_time = 0;
}

void ULTRASONIC::set_measure_speed(int delay_ms) {
    measure_interval = delay_ms;
}

void ULTRASONIC::update(void) {
    uint32_t current_time = chrono::duration_cast<chrono::microseconds>(_timer.elapsed_time()).count();

    if (is_measuring) {
        if (!detected_pulse && echo) {
            detected_pulse = true;
            begin_time = current_time;
            
        }

        if (detected_pulse && !echo) {
            is_measuring = false;
            distance_measured = (float)(current_time - begin_time) * 1000.0 / 58.0;

        }
    }
}

void ULTRASONIC::start_measurement(void) {
    if (is_measuring) {
        return;
    }

    trigger = 1;
    wait_us(5);
    trigger = 0;
    is_measuring = true;
    detected_pulse = false;

    begin_time = 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void ULTRASONIC::start(void) {
    _timer.stop();
    _ticker.detach();

    _timer.start();
    _ticker.attach(callback(this, &ULTRASONIC::start_measurement), (float)measure_interval / 1000.0);
}
#pragma GCC diagnostic pop

void ULTRASONIC::stop(void) {
    _ticker.detach();
    _timer.stop();
}

int ULTRASONIC::get_distance(void) {
    return distance_measured;
}
