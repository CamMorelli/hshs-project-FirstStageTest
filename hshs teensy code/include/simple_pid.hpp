#ifndef __simple_pid_hpp__
#define __simple_pid_hpp__

#include <Arduino.h>
#include "usb_serial.h"

class SimplePID {
  private:
    double lastError;
    double lastInput;
//    double errorSum;
    double setPoint;
    double kp, ki, kd;
    double differentiatorTimeConstant;
    double error;
    double deltaError;
    double newOutput;
    double samplingPeriod;
    double proportional;
    double integrator;
    double differentiator;
//    double maxIntegrator;
    bool shouldCompute;
    bool gainsAreSet;
    bool setPointIsSet;
    bool samplingPeriodIsSet;
    bool differentiatorTimeConstantIsSet;

  public:
    SimplePID();
    void setGains(double kp, double ki, double kd);
    void setOutputSetPoint(double);
    void setSamplingPeriod(double);
    void setDifferentiatorTimeConstant(double);
    void reset();
    void stop();
    bool start();
    double compute(double in);
    void printState();
};

#endif
