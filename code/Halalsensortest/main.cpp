#include "mbed.h"
// main() runs in its own thread in the OS

DigitalIn halalIn(A2, PullDown);

Timer main_timer;

unsigned long pulseMillis = 0;
int rpm = 0;
bool pulseDetected = false;
float distanceTraveled = 0;

float WHEEL_DIAMETER = 0.15;

void calculateRPM();

int main() {
    // halalIn.rise(&printRpm);
    while (true) {
        uint32_t current_ms = chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time()).count();

        if (!halalIn) {
            if (!pulseDetected) {
                pulseDetected = true;
                calculateRPM();
                pulseMillis = current_ms;
            }
        } else {
            pulseDetected = false;
        }
    }

}

void calculateRPM() {
    float distance = (float) WHEEL_DIAMETER * 3.1415;

    distanceTraveled += distance;
    printf("Distance: %.2fm", distanceTraveled);
}
