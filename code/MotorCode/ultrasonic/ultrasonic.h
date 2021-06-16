#include "PinNames.h"
#include "mbed.h"

#define constrain(amt, low, high)                                              \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class ULTRASONIC {
public:
  ULTRASONIC(PinName trig_pin, PinName echo_pin);
  void set_measure_speed(int delay_ms);
  void update(void);
  void start_measurement(void);
  void start(void);
  void stop(void);
  uint32_t get_distance(void);

private:
  int measure_interval;
  int distance_measured;
  int max_distance;

  uint32_t begin_time;
  uint32_t last_time_measurement;

  bool is_measuring;
  bool detected_pulse;

  bool echo_state_changed;
  int last_echo_state;

  DigitalOut trigger;
  DigitalIn echo;

  Ticker _ticker;
  Timer _timer;
};