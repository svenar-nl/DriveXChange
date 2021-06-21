#include "mbed.h"
 
DigitalOut trigger(D5);
DigitalIn  echo(D4);

// DigitalOut LedTrigger(D3); //monitor trigger
// DigitalOut LedEcho(D2); //monitor echo

int distance_measured = 0;
int correction = 0;
Timer sonar;
 
int main()
{
    sonar.reset();
// measure actual software polling timer delays
// delay used later in time correction
// start timer
    sonar.start();
// min software polling delay to read echo pin
    while (echo==2) {};
    // LedEcho = 0;
// stop timer
    sonar.stop();
// read timer
    correction = sonar.read_us();
    printf("Approximate software overhead timer delay is %d uS\n\r",correction);
 
//Loop to read Sonar distance values, scale, and print
    while(1) {
// trigger sonar to send a ping
        trigger = 1;
        // LedTrigger = 1;
        // LedEcho = 0;
        sonar.reset();
        wait_us(10.0);
        trigger = 0;
        // LedTrigger = 0;
//wait for echo high
        while (echo==0) {};
        // LedEcho=echo;
//echo high, so start timer
        sonar.start();
//wait for echo low
        while (echo==1) {};
//stop timer and read value
        sonar.stop();
//subtract software overhead timer delay and scale to cm
        distance_measured = (sonar.read_us()-correction)/58.0;
        // LedEcho = 0;
        printf(" %d cm \n\r",distance_measured);
//wait so that any echo(s) return before sending another ping
        wait_us(1000000);
    }
}