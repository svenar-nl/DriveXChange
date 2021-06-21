#include "DigitalOut.h"
#include "Motor.h"
#include "PIDLoop.h"
#include "Pixy2.h"
#include "PwmOut.h"
#include "ThisThread.h"
#include "mbed.h"


Pixy2 pixy;

uint8_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;

// Motor on the side (PWM FWD BCK)
Motor motor_left(A6, D8, D9);
Motor motor_right(D3, D6, D7);

DigitalIn start_button(A2, PullDown);

PIDLoop headingLoop(5000, 0, 0, false);

#define TOP_SPEED 40

bool pixy_ready = false;

Timer main_timer;

// PwmOut test_a6(A6);
// DigitalOut test_d4(D4);
// DigitalOut test_d5(D5);

// PwmOut test_d3(D3);
// DigitalOut test_d6(D6);
// DigitalOut test_d7(D7);

void setup();
void loop();

// Map a specific value to a different range
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
  setup();

  while (true) {
    if (start_button) {
      loop();
    }
  }

  return 0;
}

void setup() {
  ThisThread::sleep_for(3s);

  pixy.init();
  pixy.changeProg("line_tracking");
  pixy.getResolution();
  pixy.setLamp(1, 1);
  pixy.line.setMode(LINE_VECTOR);

  main_timer.start();

  //   motor_left.speed(0.5);
  //   motor_right.speed(0.5);

  //   ThisThread::sleep_for(250ms);

  motor_left.speed(0.0);
  motor_right.speed(0.0);
}

void loop() {

  uint32_t current_ms =
      chrono::duration_cast<chrono::milliseconds>(main_timer.elapsed_time())
          .count();

  bool do_print = current_ms % 50 == 0;

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
  }

  //   printf("(%d, %d)\n", x0, y0);

  int used_x_variable = x0;

  int location_percentage = map(used_x_variable, 0, pixy.frameWidth, 0,
                                100); // 0 = left 50 = center 100 = right

  for (int i = 0; i < 20; i++) {
    if (i == 0 || i == 20 - 1) {
      if (do_print)
        printf("|");
    } else if (i == (int)(used_x_variable * 20) / pixy.frameWidth) {
      if (do_print)
        printf("^");
    } else if (i == 9 || i == 10) {
      if (do_print)
        printf("|");
    } else {
      if (do_print)
        printf(" ");
    }
  }

  if (location_percentage > 0) {
    pixy_ready = true;
  } else {
      if (!pixy_ready) {
        pixy.init();
        pixy.changeProg("line_tracking");
        pixy.getResolution();
        pixy.setLamp(0, 0);
        pixy.line.setMode(LINE_VECTOR);

        ThisThread::sleep_for(500ms);

        pixy.setLamp(1, 1);

        ThisThread::sleep_for(500ms);
      }
  }

  ///////////////////////////////////////////


  float power_left = 0.0;
  float power_right = 0.0;

  int error = location_percentage - 50;
  int error2 = used_x_variable - pixy.frameWidth / 2;

  if (do_print)
    printf(" :%d, %d: ", error, error2);

  headingLoop.update(error2);

  int left = headingLoop.m_command;
  int right = -headingLoop.m_command;

  if (location_percentage < 45 || location_percentage > 55) {
    // power_left = (float) map(abs(left), 0, 400, 0, 100) / 100.0;
    // power_right = (float) map(abs(right), 0, 400, 0, 100) / 100.0;

    power_left = (float) abs(left) / 400.0;
    power_right = (float) abs(right) / 400.0;

    power_left = left < 0 ? -power_left : power_left;
    power_right = right < 0 ? -power_right : power_right;
  } else {
      power_left = 1.0;
      power_right = 1.0;
    }

  if (do_print)
    printf(" <%d, %d> ", left, right);

  int thresholdlow = 5;
  float motor_min_speed = 0.2;

  power_left = (float)map(power_left * 100, -100, 100, -75, 75) / 100.0;
  power_right = (float)map(power_right * 100, -100, 100, -75, 75) / 100.0;

  if (power_left > 0 && power_left < motor_min_speed) {
      power_left = 0;
  }

  if (power_right > 0 && power_right < motor_min_speed) {
      power_right = 0;
  }

  if (power_left < 0 && power_left > -motor_min_speed) {
      power_left = 0;
  }

  if (power_right < 0 && power_right > -motor_min_speed) {
      power_right = 0;
  }

  if (pixy_ready) {
    motor_left.speed(power_left);
    motor_right.speed(power_right);
  }

  if (do_print)
    printf("\t %d\t%d\t(%.2f, %.2f)", location_percentage, pixy_ready ? 1 : 0,
           power_left, power_right);
  if (do_print)
    printf("\n");

  ///////////////////////////////////////////////////////////
}