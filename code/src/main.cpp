#include "mbed.h"
#include "sensors.h"
#include "motordriver.h"

void setup();
void loop();

/**
LEFT |||| RIGHT
*/
CNY70 line_sensor_left();
CNY70 line_sensor_right();

VL53L1X distance_sensor();

MotorDriver motor_left();
MotorDriver motor_right();

int main() {
    setup();
    while (true) {
        loop();
    }
}

// Setup, runs once on startup
// TODO: Check pins
void setup() {
    line_sensor_left.setup(A0); // Sensor on the left-side of the line
    line_sensor_right.setup(A1); // Sensor on the right-side of the line

    distance_sensor.setup(D1, D2); // Time of Flight sensor setup

    motor_left.setup(D3, D4);
    motor_right.setup(D5, D6);
}

void loop() {
    if (distance_sensor.distance_cm() > 10) {
        if (line_sensor_left.line_visible() && line_sensor_right.line_visible()) {
            motor_left.move_forward(100);
            motor_right.move_forward(100);
        } else {
            if (line_sensor_left.line_visible() && !line_sensor_right.line_visible()) {
                motor_left.move_forward(50);
                motor_right.move_forward(100);
            }

            if (!line_sensor_left.line_visible() && line_sensor_right.line_visible()) {
                motor_left.move_forward(100);
                motor_right.move_forward(50);
            }
        }
    }
}