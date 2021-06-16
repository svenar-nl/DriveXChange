#include "ultrasonic.h"
#include "Callback.h"
#include "mbed_wait_api.h"
#include <chrono>
#include <cstdint>

ULTRASONIC::ULTRASONIC(PinName trig_pin, PinName echo_pin) : trigger(trig_pin), echo(echo_pin) {
    trigger = 0;

    measure_interval = 500;
    max_distance = 4000; // 4m

    is_measuring = false;
    detected_pulse = false;
    echo_state_changed = false;

    distance_measured = max_distance;

    begin_time = 0;
}

void ULTRASONIC::set_measure_speed(int delay_ms) {
    measure_interval = delay_ms;
}

void ULTRASONIC::update(void) {
    uint32_t current_time = chrono::duration_cast<chrono::microseconds>(_timer.elapsed_time()).count();

    echo_state_changed = false;
    if (echo.read() != last_echo_state) {
        echo_state_changed = true;
    }
    last_echo_state = echo.read();

    if (echo_state_changed) {
        // printf("\necho: %d\n", last_echo_state);
        if (last_echo_state == 1) {
            begin_time = current_time;
        }

        if (last_echo_state == 0) {
            distance_measured = (float)(current_time - begin_time) * 10.0 / 58.0;
            distance_measured = constrain(distance_measured, 0, max_distance);
            
            // if (distance_measured == max_distance) {
            //     distance_measured = 0;
            // }

            // if (distance_measured == 0) {
            //     zero_count++;
            // } else {
            //     zero_count = 0;
            // }

            // last_distance_measured = distance_measured;

            // if (distance_measured == 0 && last_distance_measured > 3000) {
            //     distance_measured = last_distance_measured;
            // }
            // distance_measured = current_time - begin_time;
            is_measuring = false;
        }
        echo_state_changed = false;
    }

    if (is_measuring) {
        // Timeout after 10ms
        if (current_time - begin_time > 10 * 1000) {
            is_measuring = false;
        }

    }

    if (current_time / 1000 - last_time_measurement > measure_interval) {
        last_time_measurement = current_time / 1000;
        start_measurement();
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

    // begin_time = 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void ULTRASONIC::start(void) {
    _timer.stop();
    _ticker.detach();

    _timer.start();
    // _ticker.attach(callback(this, &ULTRASONIC::start_measurement), (float)measure_interval / 1000.0);
    _ticker.attach(callback(this, &ULTRASONIC::update), 50us);
}
#pragma GCC diagnostic pop

void ULTRASONIC::stop(void) {
    _ticker.detach();
    _timer.stop();
}

uint32_t ULTRASONIC::get_distance(void) {
    return distance_measured;
}