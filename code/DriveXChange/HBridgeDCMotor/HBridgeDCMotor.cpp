#include "HBridgeDCMotor.h"

HBridgeDCMotor::HBridgeDCMotor(PinName gh_a, PinName gl_a, PinName gh_b, PinName gl_b) {
    GH_A = new PwmOut(gh_a);
    GL_A = new PwmOut(gl_a);
    GH_B = new PwmOut(gh_b);
    GL_B = new PwmOut(gl_b);
    independentGates = true;
    configure(1e-3, 20e3, 0.1, -0.1);
    init();
}

HBridgeDCMotor::HBridgeDCMotor(PinName gh_a, PinName gh_b) {
    GH_A = new PwmOut(gh_a);
    GL_A = NULL;
    GH_B = new PwmOut(gh_b);
    GL_B = NULL;
    independentGates = false;
    configure(1e-3, 20e3, 0.1, -0.1);
    init();
}

void HBridgeDCMotor::init() {
    dutyCycle = tempDutyCycle = 0;
}

void HBridgeDCMotor::configure(float sampleTime, float switchingFrequency, float rampUpTime, float rampDownTime) {
    if (sampleTime < 1e-6)
        sampleTime = 1e-3;
    if (switchingFrequency < 100)
        switchingFrequency = 20e3;
    if (rampUpTime < 1)
        rampUpTime = 5;
    if (rampDownTime < 1)
        rampDownTime = 5;
    this->sampleTime = sampleTime;
    switchingPeriod = 1.0 / switchingFrequency;
    GH_A->period(switchingPeriod);
    rl.setLimits(1.0/rampUpTime, -1.0/rampDownTime, 0, sampleTime);
}

void HBridgeDCMotor::adjustDutyCycle() {
    dutyCycle = rl.out(tempDutyCycle);
    if (dutyCycle >= 0 && dutyCycle <= 1) {
        GH_B->write(0);
        if(independentGates) {
            GL_B->write(1);
            GL_A->write(0);
        }
        GH_A->write(dutyCycle);
    } else if (dutyCycle >= -1 && dutyCycle < 0) { // opposite direction
        GH_A->write(0);
        if(independentGates) {
            GL_A->write(1);
            GL_B->write(0);
        }
        GH_B->write(-dutyCycle);
    } else {
        coast();
    }
}

void HBridgeDCMotor::setDutyCycle(float dc) {
    if (dc >= -1 && dc <= 1) {
        ticker.attach(this, &HBridgeDCMotor::adjustDutyCycle, sampleTime);
        tempDutyCycle = dc;
    } else {
        coast();
    }
}

void HBridgeDCMotor::coast() {
    GH_A->write(0);
    GH_B->write(0);
    if(independentGates) {
        GL_A->write(0);
        GL_B->write(0);
    }
    dutyCycle = tempDutyCycle = 0;
    rl.reset();
    ticker.detach();
}

float HBridgeDCMotor::getDutyCycle() {
    return dutyCycle;
}