#include "mbed.h"
#include "RateLimiter.h"

/** A class for driving a DC motor using a full-bridge driver (H-bridge). 
 *
 * The class has an option to drive all 4 transistors gates independently by using 4 PwmOut objects,
 * or to drive the H-bridge with complementary driven transistors using only 2 PwmOut objects.
 *
 * Example of use:
 *
 * @code
 * #include "mbed.h"
 * #include "HBridgeDCMotor.h"
 *
 * HBridgeDCMotor motor(p21, p22);
 *
 * int main() {
 *     float sampleTime = 50e-3, switchingFrequency = 25e3, rampTime = 3;
 *     motor.configure(sampleTime, switchingFrequency, rampTime, rampTime);
 *     while(true) {
 *         motor.setDutyCycle(1);
 *         wait(10);
 *         motor.setDutyCycle(-1);
 *         wait(10);
 *     }
 * }
 * @endcode
 */
class HBridgeDCMotor {
    public:
        /** Constructor for independently driven transistors.
         * H stands for high side and L for low side transistor.
         * A and B designate the individual half-bridges.
         * @param GH_A PWM signal for driving the high side transistor of the half-bridge A
         * @param GL_A PWM signal for driving the low side transistor of the half-bridge A
         * @param GH_B PWM signal for driving the high side transistor of the half-bridge B
         * @param GL_B PWM signal for driving the low side transistor of the half-bridge B
         */
        HBridgeDCMotor(PinName GH_A, PinName GL_A, PinName GH_B, PinName GL_B);
        
        /** Constructor for complementary driven transistors.
         * The high side transistors are driven directly, while the low side transistors are
         * complemented by the driver itself using logic circuits.
         * A and B designate the individual half-bridges.
         * @param GH_A PWM signal for driving the high side transistor of the half-bridge A
         * @param GH_B PWM signal for driving the high side transistor of the half-bridge B
         */
        HBridgeDCMotor(PinName GH_A, PinName GH_B);
        
        /** Configure the parameters.
         * @param sampleTime sample time in seconds
         * @param switchingFrequency switching frequency in Hz
         * @param rampUpTime set the ramp up time (in seconds) from 0 to maximum speed
         * @param rampDownTime set the ramp down time (in seconds) from maximum speed to 0
         */
        void configure(float sampleTime, float switchingFrequency, float rampUpTime, float rampDownTime);
        
        /** Set the motor speed by changing a duty cycle value.
         * @param dutyCycle Duty cycle value in a range of (-1, 1). Negative value means opposite direction. */
        void setDutyCycle(float dutyCycle);
        
        /** Set the drive in a coast mode. */
        void coast();
        
        /** Get the current duty cycle.
         * @returns the current value of the duty cycle.
         */
        float getDutyCycle();
        
    private:
        PwmOut *GH_A, *GL_A, *GH_B, *GL_B; // pointers to PwmOut objects
        RateLimiter rl;
        float switchingPeriod, dutyCycle, tempDutyCycle, sampleTime;
        bool independentGates;
        Ticker ticker;
        void adjustDutyCycle();
        void init();
};