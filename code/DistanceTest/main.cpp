#include "mbed.h"
#include "ultrasonic.h"

ULTRASONIC sen(D5, D4);
Timer main_timer;

uint32_t aaa = 0;

int main() {
  printf("Starting...\n");

    sen.setup();
    main_timer.start();

  while (true) {
    // printf("loop\n");
    uint32_t current_ms =
        chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
            .count();
    sen.update(current_ms);

    if (current_ms - aaa > 500) {
        aaa = current_ms;
        printf("distance: %dmm\n", sen.get_distance());
    }
    
  }
}