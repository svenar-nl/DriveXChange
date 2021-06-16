#include "PinNames.h"
#include "mbed.h"

class ULTRASONIC {
public:
  ULTRASONIC(PinName trig_pin, PinName echo_pin);
  void set_measure_speed(int delay_ms);
  void update(void);
  void start_measurement(void);
  void start(void);
  void stop(void);
  int get_distance(void);

private:
  int measure_interval;
  int distance_measured;

  uint32_t begin_time;

  bool is_measuring;
  bool detected_pulse;

  DigitalOut trigger;
  DigitalIn echo;

  Ticker _ticker;
  Timer _timer;
};