#include "Motor.h"
#include "mbed.h"

Motor motor_left(A6, D8, D9);
Motor motor_right(D3, D6, D7);

Timer main_timer;

unsigned long last_ms = 0l;

int melody[] = {

    932,  660,  1047, 710,  1047, 360,  1047, 660,  1109, 660,  1397,
    130,  1245, 130,  1109, 130,  1047, 130,  932,  760,  1047, 760,
    831,  1560, 831,  130,  831,  130,  932,  130,  1109, 260,  831,
    130,  932,  780,  1047, 780,  1047, 520,  1047, 780,  1109, 780,
    1397, 130,  1245, 130,  1109, 130,  1047, 130,  932,  780,  1047,
    780,  698,  1560, 1109, 130,  1109, 130,  0,    130,  1109, 130,
    1109, 130,  1109, 130,  1109, 130,  0,    520,  932,  260,  1047,
    260,  1109, 260,  1109, 260,  1245, 260,  1047, 520,  932,  130,
    831,  1820, 0,    260,  932,  260,  932,  260,  1047, 260,  1109,
    260,  831,  260,  0,    260,  831,  260,  1400, 260,  0,    260,
    1400, 260,  1245, 1300, 0,    260,  932,  260,  932,  260,  1047,
    260,  1109, 260,  932,  260,  1109, 260,  1245, 260,  0,    260,
    1047, 260,  932,  260,  831,  1300, 0,    260,  932,  260,  932,
    260,  1047, 260,  1109, 260,  932,  260,  831,  260,  0,    260,
    1245, 260,  1245, 260,  1245, 260,  1397, 260,  1245, 1040, 1109,
    1300, 1245, 260,  1397, 260,  1109, 260,  1245, 260,  1245, 260,
    1245, 260,  1397, 260,  1245, 520,  831,  520,  0,    520,  1047,
    65,   880,  65,   698,  65

};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;

void tone(int period, int beat);

int main() {
  main_timer.start();

  while (true) {
    uint32_t current_ms =
        chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
            .count();

    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

      int noteDuration = melody[thisNote + 1];

      printf("%d\t%d\t%.2f\n", thisNote / 2, melody[thisNote], noteDuration);

      int note = melody[thisNote];
      tone(note, noteDuration);

      ThisThread::sleep_for(50ms);
    }

    ThisThread::sleep_for(5000ms);
  }
}

void tone(int period, float delay) {
  int note = 2200 - period;
  if (period > 0) {
    motor_left.period_us(note);
    motor_right.period_us(note);
    motor_left.speed(0.05f);
    motor_right.speed(0.05f);
  }

  ThisThread::sleep_for(delay);
  motor_left.period(0.001);
  motor_right.period(0.001);
  motor_left.speed(0.0f);
  motor_right.speed(0.0f);
}