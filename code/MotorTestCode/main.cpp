#include "DigitalOut.h"
#include "Motor.h"
#include "Pixy2.h"
#include "PwmOut.h"
#include "mbed.h"

Pixy2 pixy;
Timer main_timer;
bool timer_running = false;

uint8_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;

// Motor on the side (PWM FWD BCK)
Motor motor_left(A6, D8, D9);
Motor motor_right(D3, D6, D7);

#define TOP_SPEED 0.3

// PwmOut test_a6(A6);
// DigitalOut test_d4(D4);
// DigitalOut test_d5(D5);

// PwmOut test_d3(D3);
// DigitalOut test_d6(D6);
// DigitalOut test_d7(D7);

void setup();
void loop();

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
  setup();

  while (true) {
    loop();
  }

  return 0;
}

void setup() {
  pixy.init();
  pixy.changeProg("line_tracking");
  pixy.getResolution();
  pixy.setLamp(1, 1);
  pixy.line.setMode(LINE_VECTOR);

  motor_left.speed(0.0);
  motor_right.speed(0.0);
}

void loop() {

  // test_a6 = 0;
  // test_d4

  // test_d3 = 0;
  //   return;

  pixy.line.getMainFeatures();
  //   pixy.line.getAllFeatures();

  if (pixy.frameWidth == 0 || pixy.frameHeight == 0) {
    pixy.changeProg("line_tracking");
    pixy.getResolution();
    pixy.setLamp(1, 0);
  }

  if (pixy.line.numVectors) {
    x0 = pixy.line.vectors[0].m_x0;
    y0 = pixy.line.vectors[0].m_y0;
    x1 = pixy.line.vectors[0].m_x1;
    y1 = pixy.line.vectors[0].m_y1;
    // printf("%d: \n", pixy.line.numVectors);
    // pixy.line.vectors->print();
    //   } else {
    // printf("0\n");
    if (timer_running) {
      main_timer.stop();
      main_timer.reset();
      timer_running = false;
    }
  } else {
    if (!timer_running) {
      main_timer.start();
      timer_running = true;
    }
  }

  //   printf("(%d, %d)\n", x0, y0);

  int used_x_variable = x1;

  int location_percentage = map(used_x_variable, 0, pixy.frameWidth, 0,
                                100); // 0 = left 50 = center 100 = right

  for (int i = 0; i < 20; i++) {
    if (i == 0 || i == 20 - 1) {
      printf("|");
    } else if (i == (int)(used_x_variable * 20) / pixy.frameWidth) {
      printf("^");
    } else if (i == 9 || i == 10) {
      printf("|");
    } else {
      printf(" ");
    }
  }

  int thresholdlow = 5;
  float motor_min_speed = 0.2;

  bool doDrive =
      !timer_running || (timer_running && main_timer.read_ms() < 1500);

  //   motor_right.speed(location_percentage < 50 - thresholdlow ? 1 : 0);
  //   motor_left.speed(location_percentage > 50 + thresholdlow ? 1 : 0);

  ///////////////////////////////////////////////////////////

  //   motor_left.speed(doDrive
  //                        ? (location_percentage < 50 - thresholdlow ?
  //                        (location_percentage < 50 - thresholdhigh ? 0 : 0.5)
  //                        : 1) : 0);
  //   motor_right.speed(doDrive
  //                         ? (location_percentage > 50 + thresholdhigh ?
  //                         (location_percentage > 50 + thresholdhigh ? 0 :
  //                         0.5) : 1) : 0);

  ///////////////////////////////////////////////////////////

  // if (doDrive) {
  //     if (location_percentage < 50 - thresholdlow) {
  //         if (location_percentage < 50 - thresholdhigh) {
  //             motor_left.speed(0);
  //         } else {
  //             motor_left.speed(0.5);
  //         }
  //     } else {
  //         motor_left.speed(1);
  //     }

  //     if (location_percentage > 50 + thresholdlow) {
  //         if (location_percentage > 50 + thresholdhigh) {
  //             motor_right.speed(0);
  //         } else {
  //             motor_right.speed(0.5);
  //         }
  //     } else {
  //         motor_right.speed(1);
  //     }
  // } else {
  //     motor_left.speed(0);
  //     motor_right.speed(0);
  // }

  ///////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////

  float power_left = 0.0, power_right = 0.0;

  if (location_percentage < 50 - thresholdlow) {
    power_left = map(location_percentage, 0, 50, 0, 100) / 100.0;
  } else {
    power_left = 1.0;
  }

  if (location_percentage > 50 + thresholdlow) {
    power_right = map(location_percentage, 100, 50, 0, 100) / 100.0;
  } else {
    power_right = 1.0;
  }

  power_left = power_left < motor_min_speed ? 0 : power_left;
  power_right = power_right < motor_min_speed ? 0 : power_right;

  if (!doDrive) {
    power_left = 0.0;
    power_right = 0.0;
  }

  power_left = power_left > TOP_SPEED ? TOP_SPEED : power_left;
  power_right = power_right > TOP_SPEED ? TOP_SPEED : power_right;

  motor_left.speed(power_left);
  motor_right.speed(power_right);

  printf("\t %d\t%d\t(%.2f, %.2f)", location_percentage, location_percentage,
         power_left, power_right);
  printf("\n");

  ///////////////////////////////////////////////////////////
}