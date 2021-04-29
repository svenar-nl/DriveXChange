#include "mbed.h"

Timer delayMicrosecondsTimer;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

void delayMicroseconds(unsigned int delay) {
  float delay_s = delay / 1000000;
  delayMicrosecondsTimer.start();
  while (delayMicrosecondsTimer.read() < delay_s) {
    ;
  }
  // while(delayMicrosecondsTimer<float>{elapsed_time()}.count() < delay_s){;}

  delayMicrosecondsTimer.stop();
}

#pragma GCC diagnostic pop