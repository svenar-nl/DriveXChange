#include "mbed.h"

// main() runs in its own thread in the OS

InterruptIn halalIn(A2, PullDown);

void printRpm();

int main()
{
    halalIn.rise(&printRpm);
    while (true) {
        
    }

}

void printRpm(){
    printf("Toer");
}
