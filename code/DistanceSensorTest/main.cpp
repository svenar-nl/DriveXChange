#include "Callback.h"
#include "ThisThread.h"
#include "mbed.h"
#include "ultrasonic.h"

int distance_mm = 0;

ULTRASONIC sensor(A5, A4);
Timer timer;

uint32_t last_ms = 0;
uint32_t current_ms = 0;

int main() {
    printf("\nStarting...\n");

    timer.start();

    sensor.set_measure_speed(500);
    sensor.start();

    while (true) {
        current_ms = chrono::duration_cast<chrono::milliseconds>(timer.elapsed_time()).count();

        sensor.update();
    
        if (current_ms - last_ms > 500) {
            last_ms = current_ms;
            
            distance_mm = sensor.get_distance();
            printf("distance: %dmm\t\trun time: %dms\n", distance_mm, current_ms);
        }
    }
}

