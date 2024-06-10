#include "simple_pid.hpp"
#include "logger.hpp"
#include "constants.hpp"
#include "serial_comm.hpp"

SimplePID::SimplePID():
    lastError(0),
    lastInput(0),
    setPoint(0),
    kp(0),
    ki(0),
    kd(0),
    differentiatorTimeConstant(1),
    samplingPeriod(1),
    integrator(0),
    differentiator(0),
    shouldCompute(false),
    gainsAreSet(false),
    setPointIsSet(false),
    samplingPeriodIsSet(false),
    differentiatorTimeConstantIsSet(false) {
}

void SimplePID::setSamplingPeriod(double v) {
    samplingPeriod = v;
    samplingPeriodIsSet = true;
}

void SimplePID::setOutputSetPoint(double v) {
    setPoint = v;
    setPointIsSet = true;
}

void SimplePID::setGains(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    gainsAreSet = true;
}

void SimplePID::setDifferentiatorTimeConstant(double _differentiatorTimeConstant) {
    differentiatorTimeConstant = _differentiatorTimeConstant;
    differentiatorTimeConstantIsSet = true;
}

void SimplePID::reset() {
    lastError = 0;
    lastInput = 0;
    integrator = 0;
    differentiator = 0;
//    errorSum = 0;
}

bool SimplePID::start() {
    if ( gainsAreSet && setPointIsSet && samplingPeriodIsSet && differentiatorTimeConstantIsSet) {
        shouldCompute = true;
        return true;
    } 
    else {
        return false;
    }
}

void SimplePID::stop() {
    shouldCompute = false;
}

double SimplePID::compute(double input) {

    if ( !shouldCompute ) {
        return 0;
    }

    if (isnan(input)) {
       Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Compute failed: input: [%f]", input);
//        PIDsProcess[0].printState(serial);
//        PIDsProcess[0].printState(serial);
        return 0;
    }

//    differentiatorTimeConstant = 0.5;
    error = setPoint - input;
    proportional = kp * error;
//    deltaError = (error - lastError) / samplingPeriod;
//    errorSum += error * samplingPeriod;

    integrator = integrator + ki * samplingPeriod * 0.5 * (error + lastError);
    if (integrator > constants::ANALOG_OUT_RESOLUTION_BITS) {     //   Implement anti-windup using static integrator clamping
        integrator = constants::ANALOG_OUT_RESOLUTION_BITS;
    } else if (integrator < 0) {
        integrator = 0;
    }

    differentiator = (2.0 * kd * (-input + lastInput)
                        + (2.0 * differentiatorTimeConstant - samplingPeriod) * differentiator)   
                        / (2.0 * differentiatorTimeConstant + samplingPeriod);	//  formula for band-limited differentiator, minus sign for derivative on input

    newOutput = proportional + integrator + differentiator;

    if (newOutput > constants::ANALOG_OUT_RESOLUTION_BITS) {       //      Clamp output
        newOutput = constants::ANALOG_OUT_RESOLUTION_BITS;
    } else if (newOutput < 0) {
        newOutput = 0;
    }

    lastError = error;
    lastInput = input;

    return newOutput;
}

const char* printBool( bool in ) {
    return in ? "true" : "false";
}

void SimplePID::printState() {
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "SimplePID.printState -- BEGIN");
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  shouldCompute:%s", printBool(shouldCompute));
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  gainsAreSet:%s", printBool(gainsAreSet));
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  setPointIsSet:%s", printBool(setPointIsSet));
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  samplingPeriodIsSet:%s", printBool(samplingPeriodIsSet));
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  differentiatorTimeConstantIsSet:%s", printBool(differentiatorTimeConstantIsSet));
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  setPoint:%.4f", setPoint);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  kp:%.4f", kp);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  ki:%.4f", ki);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  kd:%.4f", kd);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  samplingPeriod:%.4f", samplingPeriod);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  lastError:%.4f", lastError);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  lastInput:%.4f", lastInput);

    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  newOutput:%.4f", newOutput);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  proportional:%.4f", proportional);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  integrator:%.4f", integrator);
    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  differentiator:%.4f", differentiator);

    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "SimplePID.printState -- END");
}


