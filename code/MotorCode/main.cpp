#include "mbed.h"
#include "VL53L1X.h"
#include "Pixy2.h"
// #include "I2C.h"

#define I2C_SDA D4
#define I2C_SCL D5

VL53L1X distance_sensor(I2C_SDA, I2C_SCL);
Pixy2 line_camera;

int main() {
  while (true) {
  }
}
