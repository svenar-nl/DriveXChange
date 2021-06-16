#include "ultrasonic.h"

ULTRASONIC::ULTRASONIC(PinName trigger_pin, PinName echo_pin)
    : trigger(trigger_pin), echo(echo_pin) {
  trigger = 0;
  distance_mm = 0;
}

void ULTRASONIC::setup() {
  //
}

void ULTRASONIC::update(uint32_t elapsed) {
  if (elapsed - last_update_ms > update_interval) {
      if (!is_measuring) {
        last_update_ms = elapsed;
        trigger = 1;
        wait_us(10);
        trigger = 0;
        start_time = elapsed;
        is_measuring = true;
      }
  }

  if (is_measuring) {
    if (echo) {
    //   trigger = 0;
      distance_mm = elapsed - start_time;
      is_measuring = false;
    }
  }
}

int ULTRASONIC::get_distance() { return distance_mm * 1000 / 58; }