#include "state_control.hpp"
#include <Arduino.h>
#include "constants.hpp"
#include "serial_comm.hpp"
#include <core_pins.h>

StateControl::StateControl():
    state(Data_Off),
    stateLast(Data_Off),
    stateStageCounter(0),

    dataOn(true), dataOnLast(true),
    photodiodeDifferenceMeasure(0),
    crispLEDTargetValue(0),
    crispLEDActualValue(0),
    crispLEDSaveValue(0),
    pd1Value(0),
    pd2Value(0),
    pd1Raw(0),
    pd2Raw(0),
    servoTargetValue(0.2),
    servoActualValue(0.1),
    laserCurrentVoltsRaw(0), laserCurrentVoltsZero(0), tecCurrentVoltsRaw(0), tecCurrentVoltsZero(0), 
    kProp(0),
    kInteg(0),
    kDeriv(0),
    servoTimeConst(0.1),
    photodiodeDifferenceSetPointTarget(0.3),
    photodiodeDifferenceSetPoint(0.2),
    scanTime(0),
    scanRange(0),
    scanStart(0),
    scanStop(0),
    reportInterval(0.25),
    errorSumCount(0),
    zeroSumCount(0),
    zeroSum1(0),
    zeroSum2(0),
    pd1Zero(0),
    pd2Zero(0),
    errorSumBuffer(0),
    errorSumBufferLast(0),
    averageError(0),
    pd1SumBuffer(0),averagePD1(0),pd2SumBuffer(0),averagePD2(0),pdDiffSumBuffer(0),averagePDDiff(0),
    averageLaserCurrent(0), averageTECCurrent(0),
    ledScanTime(2), ledScanLow(0), ledScanHigh(100),
    ditherState(true),
    hopValue(0),
    ditherError(0),
    analogReadCount(10),
//    zeroCycleCount(100),
    ledFlashCounter(0),
    ledFlashCounterLast(0),
    lastErrorValue(0),
    lastInput(0),
    differentiatorTimeConstant(0),
    errorValue(0),
    samplingPeriod(0),
    proportional(0),
    integrator(0),
    differentiator(0),
    PIDDirectionSign(1),

    debuggingPointCountInterval(200),
//    waitForFrameTime(0),
//    readFrameTime(0),
//    calculateT0Time(0),
    writeDataTime(0),
    updateOutputsTime(0),
    innerLoopTime(0),
    handleSerialTime(0),
    temperature_sum_count(0),

    globalCounter(0),
    stateCounter(0),
    resetStateTime(false),
    stateStartTime_ms(millis()),
    globalStartTime_ms(millis()),
    stateTime_ms(0),
    globalTime_ms(0),
    resetGlobalTime(false),
    fittingInProcess{false},

    fitNumPoints(0),
    fitStartCounter(0),
    sumx{0}, sumy{0}, sumxy{0}, sumx2{0},
    slopeVals{0},
    debugData{0},

    stateTimeOutSec(1),
    numFailedFrames(0), maxNumFailedFrames(5),
    currentChannelZeroCounts(0) {
};

bool StateControl::updateOutputs () {
    int32_t pd1Sum = 0;
    int32_t pd2Sum = 0;
    int32_t thermSum[constants::NUM_CHANNELS];
    int32_t laserCurrentSum = 0;
    int32_t tecCurrentSum = 0;
    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
        thermSum[ii] = 0;
    }
//    uint32_t preReadTime = micros();
    for (uint32_t ii = 0; ii < constants::SAMPLES_TO_READ; ii++) {

        for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
            thermSum[ii] += analogRead(constants::THERMISTER_AIN_PIN_LIST[ii]);
        }
    laserCurrentSum += analogRead(constants::LASER_CURRENT_AIN_PIN);
    tecCurrentSum += analogRead(constants::TEC_CURRENT_AIN_PIN);
//    if (( debuggingPointCountInterval > 0 ) && ( globalCounter % debuggingPointCountInterval == 0 )) { //  For tuning and debugging software circuit breaker
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, tecCurrentSum: %.3f", globalTime_ms*0.001, analogRead(constants::TEC_CURRENT_AIN_PIN));
//    }
//        pd1Sum += analogRead(constants::PIN_IN_PD_1);
//        pd2Sum += analogRead(constants::PIN_IN_PD_2);
//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, ii: %d, pd1Sum: %d, pd2Sum: %d", globalTime_ms*0.001, ii, pd1Sum, pd2Sum);
    }
//    u_int32_t readTime = micros() - preReadTime;
//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, readTime: %d, or %f microseconds", globalTime_ms*0.001, readTime, readTime/1e6);
//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, pd1Sum: %d, pd2Sum: %d, photodiodeDifferenceMeasure: %f, pd1Value: %f, pd2value: %f", globalTime_ms*0.001, pd1Sum, pd2Sum, photodiodeDifferenceMeasure, pd1Value, pd2Value);


    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
////        lastTempMeasurements[temperature_channel] = computeTempFromAnalogRead(analogRead(constants::PIN_IN_THERM_0));
        temperaturesLast[ii] = temperaturesNew[ii];
        temperaturesNew[ii] = computeTempFromAnalogRead(1.0*thermSum[ii]/constants::SAMPLES_TO_READ);
//        if (ii == 0) {
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, thermSum[ii]: %d, termValue[ii]: %f, analogRead(constants::THERMISTER_AIN_PIN_LIST[ii]): %d, constants::SAMPLES_TO_READ: %d", globalTime_ms*0.001, thermSum[0], temperaturesNew[0], analogRead(constants::THERMISTER_AIN_PIN_LIST[0]), constants::SAMPLES_TO_READ);
//        }

////        thermSum[ii] += analogRead(constants::THERMISTER_AIN_PIN_LIST[ii]);
////        temperaturesNew[ii] = 1.0*thermSum[ii]/constants::SAMPLES_TO_READ/constants::ANALOG_IN_MAX_COUNT;
    }

    laserCurrentVoltsRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE * laserCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
    tecCurrentVoltsRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE * tecCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    laserCurrentRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000* laserCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    tecCurrentRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000 * tecCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    laserCurrentRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000* laserCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    tecCurrentRaw = 1.0 * constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000 * tecCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    laserCurrentValue = constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000* laserCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
//    tecCurrentValue = constants::ANALOG_IN_REFERENCE_VOLTAGE / constants::REFERENCE_RESISTANCE * constants::NUM_CHANNELS * 1000 * tecCurrentSum / constants::SAMPLES_TO_READ / constants::ANALOG_IN_MAX_COUNT;
    laserCurrentValue = ( laserCurrentVoltsRaw - laserCurrentVoltsZero ) / constants::HALL_SENSOR_VOLTS_PER_AMP;
    tecCurrentValue = ( tecCurrentVoltsRaw - tecCurrentVoltsZero )  / constants::HALL_SENSOR_VOLTS_PER_AMP;

    pd1Raw = 1.0*pd1Sum/constants::SAMPLES_TO_READ/constants::ANALOG_IN_MAX_COUNT;
    pd2Raw = 1.0*pd2Sum/constants::SAMPLES_TO_READ/constants::ANALOG_IN_MAX_COUNT;
    pd1Value = pd1Raw - pd1Zero;
    pd2Value = pd2Raw - pd2Zero;
//    photodiodeDifferenceMeasure = (1.0 * ( 1.0*pd1Sum - pd2Sum )) / ( 1.0*pd1Sum + pd2Sum ) ;    //  Keep maximum resolution for photodiodeDifferenceMeasure
    photodiodeDifferenceMeasure = (1000.0 * ( pd1Sum - pd2Sum )) / ( pd1Sum + pd2Sum ) ;    //  Keep maximum resolution for photodiodeDifferenceMeasure
//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, pd1Sum: %d, pd2Sum: %d, photodiodeDifferenceMeasure: %f, pd1Value: %f, pd2value: %f", globalTime_ms*0.001, pd1Sum, pd2Sum, photodiodeDifferenceMeasure, pd1Value, pd2Value);

    if ((state == Scan_On) ) {
        appendPoints(globalCounter, servoActualValue, photodiodeDifferenceMeasure);
/*
    } else if ((state == LED_Scan_On) ) {
        appendPoints(globalCounter, crispLEDActualValue, photodiodeDifferenceMeasure);
    } else if ((state == Zero_Cal) ) {
        appendPoints(globalCounter, stateTime_ms, photodiodeDifferenceMeasure);
*/
    }
    updateState( ); //  switch(state); switchstate: fits
//    updateCRISPLED();
//    updateServo();
//    updatePID();
    updateClocks();         //  Update globalCounter, clocks, and LED flash state; Do this once per cycle only for consistent timing
    if( (state != Data_Off) && (state != Shutdown) ) {
        updateLasers(); //  Update laser currents from command
        updateFan();    //  Update fan from command
        updateTECs();   //  Update TEC from command or from PID
    }
    return true;
}

bool StateControl::writeData(StateControl& _sc ) {
//    if (true) {
    if ((state != Data_Off) || ((state == Data_Off) && (stateCounter < 2*constants::CYCLES_PER_DATA_POINT)))  { //  Send Data_Off status twice to compensate for qwirk in data queue where termination code means queue is always one element behind; codes are at the beginning of packet, not the end
//        if( (constants::CYCLES_PER_DATA_POINT > 0) && (globalCounter % constants::CYCLES_PER_DATA_POINT) == 0) {//  Allow write on first data point with Data_Off
//    if ((state != Data_Off) || ((state == Data_Off) && (stateCounter == 0))
//        || ((state == Data_Off) && (stateCounter == 1))) {    //  Send Data_Off status twice to compensate for qwirk in data queue where termination code means queue is always one element behind; codes are at the beginning of packet, not the end
        if( (constants::CYCLES_PER_DATA_POINT > 0) && (globalCounter % constants::CYCLES_PER_DATA_POINT) == 0) {
            errorSumCount += 1;
            averageError = (errorSumBuffer + errorValue) / errorSumCount;
            averagePD1 = (pd1SumBuffer + pd1Value) / errorSumCount;
            averagePD2 = (pd2SumBuffer + pd2Value) / errorSumCount;
            averagePDDiff = (pdDiffSumBuffer + photodiodeDifferenceMeasure) / errorSumCount;

            for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
                averageTherm[ii] = (thermBuffer[ii] + temperaturesNew[ii]) / errorSumCount;
            }
            averageLaserCurrent = (laserCurrentBuffer + laserCurrentValue) / errorSumCount;
            averageTECCurrent = (tecCurrentBuffer + tecCurrentValue) / errorSumCount;

/*
            if (state == Dither_On) {
                if (ditherState) {
                    ditherError = (errorSumBuffer-errorSumBufferLast)/errorSumCount;
//                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, True, ditherError: %f, errorSumBuffer: %f, errorSumBufferLast: %f, errorSumCount: %d, hopValue: %f", 
//                    globalTime_ms*0.001, ditherError, errorSumBuffer, errorSumBufferLast, errorSumCount, hopValue);
                } else {
                    ditherError = -(errorSumBuffer-errorSumBufferLast)/errorSumCount;
//                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, False, ditherError: %f, errorSumBuffer: %f, errorSumBufferLast: %f, errorSumCount: %d, hopValue: %f", 
//                    globalTime_ms*0.001, ditherError, errorSumBuffer, errorSumBufferLast, errorSumCount, hopValue);
                }
            }
*/            
            Serial_IO::writeHSHSDataNew( _sc );
            errorSumBufferLast = errorSumBuffer;
            errorSumCount = 0;
            errorSumBuffer = 0;
            pd1SumBuffer = 0;
            pd2SumBuffer = 0;
            pdDiffSumBuffer = 0;
            for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
                thermBuffer[ii] = 0;
            }
            laserCurrentBuffer = 0;
            tecCurrentBuffer = 0;

        } else {
            errorSumCount += 1;
            errorSumBuffer += errorValue;
            pd1SumBuffer += pd1Value;
            pd2SumBuffer += pd2Value;
            pdDiffSumBuffer += photodiodeDifferenceMeasure;

            for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
                thermBuffer[ii] +=  temperaturesNew[ii];
            }
            laserCurrentBuffer += laserCurrentValue;
            tecCurrentBuffer += tecCurrentValue;
        }
    }
    if (( debuggingPointCountInterval > 0 ) && ( globalCounter % debuggingPointCountInterval == 0 )) { //  For tuning and debugging software circuit breaker
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f errorValue: %f, proportional: %f, integrator: %f, differentiator: %f, servoTargetValue: %f, servoActualValue: %f", globalTime_ms*0.001,  errorValue, proportional, integrator, differentiator, servoTargetValue, servoActualValue);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f temperatureSetPoint[0]: %f,  temperaturesNew[0]: %f, errorsNew[0]: %f, proportionalValues[0]: %f, integratorValues[0]: %f, differentiatorValues[0]: %f, tecNew[0]: %f", 
            globalTime_ms*0.001, temperatureSetPoint[0], temperaturesNew[0], errorsNew[0], proportionalValues[0], integratorValues[0], differentiatorValues[0], tecNew[0] );        
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, tecCurrentSum: %d", globalTime_ms*0.001, analogRead(constants::TEC_CURRENT_AIN_PIN));
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Teensy Temperature: %8.2f, Motor Temperature: %8.2f, LED 0 Temperature: %8.2f, Total Current: %8.2f", globalTime_ms*0.001, tempmonGetTemp(), lastTempMeasurements[0], lastTempMeasurements[1], lastCurrentMeasurement);
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Times for waitForFrameTime: %d, readFrameTime: %d, calculateT0: %d, writeData: %d, updateOutputs: %d, innerLoop: %d, handleSerial: %d", globalTime_ms*0.001, waitForFrameTime, readFrameTime, calculateT0Time, writeDataTime, updateOutputsTime, innerLoopTime, handleSerialTime);
    }
    return true;
}

bool StateControl::endFits(stateType stateNew){
    //  End fits when state changes
    //  These steps are for old state that is ending:
//    if ( (state == Scan_On) || (state == LED_Scan_On) || (state == Zero_Cal)) {  
    if ( (state == Scan_On) ) {  
        calcFits();
        if ((stateNew != Reset) && (stateNew != Data_Off)) {    //  Don't write transition/fit data for reset state

//            if ( (state == Scan_On) || (state == LED_Scan_On)) {
            if ( (state == Scan_On) ) {
                Serial_IO::writeTransitionData(state, fitStartCounter, fitNumPoints, slopeVals, interceptVals, debugData, 0 );
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f state: %d, fitStartCounter: %d, fitNumPoints: %d, slopeVals: %f, interceptVals: %f, debugData: %f, 0: %d", globalTime_ms*0.001, state, fitStartCounter, fitNumPoints, slopeVals, interceptVals, debugData, 0 );
//            } else if (state == Zero_Cal) {
//                Serial_IO::writeTransitionData(state, fitStartCounter, fitNumPoints, pd1Zero, pd2Zero, debugData, 0 );
            }
        }
    }
    return true;
}

errorType StateControl::switchState(stateType stateNew, uint8_t stateStageCounterNew ) {

//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f SwitchState: state: %d, stateNew: %d", globalTime_ms*0.001, state, stateNew);

//  These steps are for old state that is ending:
    if ((state == Scan_On) ) {
//    if ((state == Scan_On) || (state == LED_Scan_On) || (state == Zero_Cal)) {
        endFits(stateNew);
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Before resetFitParameters, fitNumPoints = %d", globalTime_ms*0.001, fitNumPoints);
        resetFitParameters();   //  Only reset after values are written above
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f After resetFitParameters, fitNumPoints = %d", globalTime_ms*0.001, fitNumPoints);
    }

//  Now set values for new state
    stateLast = state;
    state = stateNew;
    stateStageCounter = stateStageCounterNew;
    resetStateTime = true;  //  This resets stateTime and stateCounter
    return NO_ERROR;
}

void StateControl::resetFitParameters () {
    fitNumPoints = 0;
    sumx = 0; sumy = 0; sumxy = 0; sumx2 = 0;
}

void StateControl::appendPoints (uint32_t globalCounter, float time_sec, float temperatures)  {   //  Add points for states that have fits
//  This is called before state updates and tracks prior cycle values; exception is for state change from Serial_IO::handleSerialLine (TODO: review this approach)
    float xVal, yVal;
//  This includes all points in state for jump-based/end-point slopes

    if ((state == Scan_On) )  {
//    if ((state == Scan_On) || (state == LED_Scan_On)  || (state == Zero_Cal))  {
        if (fitNumPoints == 0) {
            fitStartCounter = globalCounter;
        }
        fitNumPoints += 1;
            if (state == Scan_On) {   //  Use delayed heat contributions as x values for Cycle_Bump_On and Cycle_Ramp_2
                xVal = servoActualValue;
//            }
//            else if (state == LED_Scan_On) {   //  Use delayed heat contributions as x values for Cycle_Bump_On and Cycle_Ramp_2
//                xVal = crispLEDActualValue;
            } else {      //  Use time for other fits; for Zero_Cal, we are only tracking the first point and fitNumPoints
                xVal = time_sec;
            }
                yVal = photodiodeDifferenceMeasure;
            sumx += xVal;
            sumx2 += xVal*xVal;
            sumy += yVal;
            sumxy += xVal * yVal;
    }
}

void StateControl::calcFits ()  {
    float deltaVals;
    deltaVals = fitNumPoints * sumx2 - sumx * sumx;
    interceptVals = (sumx2 * sumy - sumx * sumxy) / deltaVals;
    slopeVals = (fitNumPoints * sumxy - sumx * sumy) / deltaVals;    //  Least squares linear fit for y = ax + b
//    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f state: %d, deltaVals: %0.3f, interceptVals: %0.3f, slopeVals %0.3f", globalTime_ms*0.001, state, deltaVals, interceptVals, slopeVals);
    switch (state) {
        default:
        break;
    }
}

bool StateControl::updateClocks() {
    uint32_t now = millis();
//    deltaTime_ms = (uint32_t)(now - globalStartTime_ms) - globalTime_ms;  //  This expression corrects for millis() rollover
    stateCounter++;    //  Update counters and clocks
    globalCounter++;    //  Update globalCounter and clocks
    if (resetGlobalTime)  {
        globalStartTime_ms = now;
        globalTime_ms = 0;
        resetGlobalTime = false;
        globalCounter = 0;
        ledFlashCounterLast = -1;
        }
    if (resetStateTime)  {
        stateStartTime_ms = now;
        stateTime_ms = 0;
        resetStateTime = false;
        stateCounter = 0;
        ledFlashCounterLast = -1;
    }
        globalTime_ms = (uint32_t)(now - globalStartTime_ms);  //  This expression corrects for millis() rollover
        stateTime_ms = (uint32_t)(now - stateStartTime_ms);  //  This expression corrects for millis() rollover
        ledFlashCounter = stateTime_ms / 50; //  One every 50 ms

    if (dataOn) {
//    if (true) {
//        ledFlashCounter = stateTime_ms / 50; //  One count every 50 ms or 20 counts/second
        if (ledFlashCounter > ledFlashCounterLast) {
            ledFlashCounterLast = ledFlashCounter;
            uint16_t ledFlashCounterMod = ledFlashCounter % 20; //  20 counts / second, reseting to zero every second
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f , ledFlashCounterLast: %d, ledFlashCounter: %d", globalTime_ms*0.001, ledFlashCounterLast, ledFlashCounter);
            switch (state) {                          //  Flash LED to indicate group state timing
                case Ready:                             //  Steady on: data off, half second on: ready, one flash: servo or LED scan, two flashes Lock On
                    if (ledFlashCounterMod == 0) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                    if (ledFlashCounterMod == 10) analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED
                break;
                case Scan_On:
//                case Scan_On: case LED_Scan_On:
                    if (ledFlashCounterMod == 0) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                    if (ledFlashCounterMod == 1) analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED
                    if (ledFlashCounterMod == 2) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                    break;
                case Standby:
                    if (ledFlashCounterMod == 0) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                    if (ledFlashCounterMod == 1) analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED
                    if (ledFlashCounterMod == 2) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                    if (ledFlashCounterMod == 3) analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED
                    if (ledFlashCounterMod == 4) analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
                break;
                case Data_Off:
                        analogWrite(constants::PIN_STATUS_LED, int((0.5 + 0.5*sin(2*3.1415629*(ledFlashCounter % 60)/60)) * constants::ANALOG_OUT_MAX_COUNT));     //  pulse LED every 60/20 = 3 seconds
//                    analogWrite(constants::PIN_STATUS_LED, sin(2*3.1415629*(ledFlashCounter % 60)/60) * constants::ANALOG_OUT_MAX_COUNT);     //  pulse LED
                break;
                default:
                break;
            }
        }
    }
    if (dataOn == false) {
//        ledFlashCounter = stateTime_ms / 50; //  One every 50 ms

        analogWrite(constants::PIN_STATUS_LED, int((0.5 + 0.5*sin(2*3.1415629*(ledFlashCounter % 60)/60)) * constants::ANALOG_OUT_MAX_COUNT));     //  pulse LED every 60/20 = 3 seconds
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f , stateTime_ms: %d, ledFlashCounter: %d, ledFlashCounterMod: %d, mod value 1: %d, mod value 2: %f, sine value1: %f, sine value2: %d", globalTime_ms*0.001, stateTime_ms, ledFlashCounter, (ledFlashCounter % 20), (ledFlashCounter % 60), (ledFlashCounter % 60)/60, 0.5+0.5*sin(2*3.1415629*(ledFlashCounter % 60)/60), int((0.5+0.5*sin(2*3.1415629*(ledFlashCounter % 60)/60)) * constants::ANALOG_OUT_MAX_COUNT));

    }
    return true;
}

void StateControl::updateState( ) {
    //  Basic flow: Based on current state, append points for fit, schedule new outputs or state switch, update clocks, process end-point calculations
//    float maxTemp;

    errorValue = photodiodeDifferenceSetPoint - photodiodeDifferenceMeasure;

    switch (state) {

/*
        case Lock_Init: {
            integrator = servoActualValue;  //  Start lock from where we already are
                //  Write servo pin high?; this is not implemented currently
            switchState(Lock_On, 0 );
        } break;

        case Lock_On: {

//            if ( !shouldCompute ) {
//                return 0;
//            }

//            if (isnan(input)) {
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Compute failed: input: [%f]", input);
        //        PIDsProcess[0].printState(serial);
        //        PIDsProcess[0].printState(serial);
//                return 0;
//            }

        //    differentiatorTimeConstant = 0.5;
//            errorValue = photodiodeDifferenceSetPoint - photodiodeDifferenceMeasure;
            proportional = PIDDirectionSign*kProp * errorValue;
        //    deltaError = (errorValue - lastErrorValue) / samplingPeriod;
        //    errorSum += errorValue * samplingPeriod;

            integrator = integrator + PIDDirectionSign*kInteg * constants::DATA_UPDATE_INTERVAL_SEC * 0.5 * (errorValue + lastErrorValue);
            if (integrator > constants::SERVO_MAX_VALUE) {     //   Implement anti-windup using static integrator clamping
                integrator = constants::SERVO_MAX_VALUE;
            } else if (integrator < 0) {
                integrator = 0;
            }

            differentiator = (2.0 * PIDDirectionSign*kDeriv * (-photodiodeDifferenceMeasure + lastInput)
                                + (2.0 * differentiatorTimeConstant - constants::DATA_UPDATE_INTERVAL_SEC) * differentiator)   
                                / (2.0 * differentiatorTimeConstant + constants::DATA_UPDATE_INTERVAL_SEC);	//  formula for band-limited differentiator, minus sign for derivative on input

            servoTargetValue = proportional + integrator + differentiator;

            if (servoTargetValue > constants::SERVO_MAX_VALUE ) {       //      Clamp output
                servoTargetValue = constants::SERVO_MAX_VALUE;
            } else if (servoTargetValue < 0) {
                servoTargetValue = 0;
            }

            lastErrorValue = errorValue;
            lastInput = photodiodeDifferenceMeasure;

//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Lock_On state; errorValue: %f, proportional: %f, integrator: %f, differentiator: %f, servoTargetValue: %f, servoActualValue: %f", globalTime_ms*0.001,  errorValue, proportional, integrator, differentiator, servoTargetValue, servoActualValue);            
        } break;
*/

        case Scan_Init: {
            servoValueInitial = servoActualValue;    //  Need this to reset clock before Scan_On
            switchState(Scan_On, 0 );
        } break;

        case Scan_On: {
            if (stateTime_ms < scanTime * 1000) {
                servoTargetValue = servoValueInitial - scanRange/2 + stateTime_ms*0.001 / scanTime * scanRange;
            } else {
                servoTargetValue = servoValueInitial;
                switchState(Ready, 0 );
            }
        } break;

/*
        case LED_Scan_Init: {
            ledScanInitial = crispLEDActualValue;    //  Need this to reset clock before Scan_On?
            switchState(LED_Scan_On, 0 );
        } break;

        case Dither_Init: {
            servoValueInitial = servoActualValue;    //  Need this to reset clock before Scan_On
            switchState(Dither_On, 0 );
        } break;

        case Dither_On: {       //  Current order: updateOutputs (read; updateState; updateServo) writeData (writeHSHSData, reset errorSumCount)
            if (errorSumCount == 0) {   //  Change servo only after writeData has already been called
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Dither_On state; errorSumCount: %d, ditherState: %d, servoTargetValue: %f", 
//                    globalTime_ms*0.001,  errorSumCount, ditherState, servoTargetValue);
                servoTargetValue = (ditherState == true) ? servoValueInitial+hopValue : servoValueInitial-hopValue;
                ditherState = !ditherState;
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Dither_On state; errorSumCount: %d, ditherState: %d, servoTargetValue: %f", 
//                    globalTime_ms*0.001,  errorSumCount, ditherState, servoTargetValue);
//            } else {
            }       //  Dither_On continues until reset
        } break;

        case LED_Scan_On: {
            if (stateTime_ms < ledScanTime * 1000) {
                crispLEDTargetValue = ledScanLow + stateTime_ms*0.001 / ledScanTime * (ledScanHigh-ledScanLow);
            } else {
                crispLEDTargetValue = ledScanInitial;
                switchState(Ready, 0 );
            }
        } break;
*/

        case Reset: {
            switchState(Ready, 0 );
        } break;

/*
        case Zero_Init: {   //  Turn off CRISP LED and wait briefly
            if (stateCounter >= constants::CYCLES_FOR_ZERO_INIT) {
                switchState(Zero_Cal, 0 );
            }
        } break;

        case Zero_Cal: {    //  Average values of PD1 and PD2 with CRISP LED off
            if (stateCounter < constants::CYCLES_FOR_ZERO_CAL) {
                zeroSumCount += 1;
                zeroSum1 += pd1Raw;
                zeroSum2 += pd2Raw;
                //  Sum PD1 and PD2
            } else {
                crispLEDTargetValue = crispLEDSaveValue;    //  Restore LED value
                pd1Zero = zeroSum1/zeroSumCount;
                pd2Zero = zeroSum2/zeroSumCount;
                //  Calculate PD1 and PD2 average console
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f pd1Zero: %f pd2Zero: %f", globalTime_ms*0.001, pd1Zero, pd2Zero);
                switchState(Ready, 0 );
            }
        } break;
*/
        case Zero_Init: {
        for ( uint8_t ii=0; ii<constants::NUM_CHANNELS; ii++ ) {        // Save laser and TEC values
            laserSave[ii] = laserLast[ii];
            tecSave[ii] = tecLast[ii];
            laserNew[ii] = 0;       //  Turn off all lasers and TECs
            tecNew[ii] = 0;
        }
        zeroSumCount = 0;
        zeroSum1 = 0;
        zeroSum2 = 0;
        switchState(Zero_Cal, 0 );  
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d, imagingActive: %d, shutdownFlag: %d", globalTime_ms*0.001, stateCounter, imagingActive, shutdownFlag);
//            if ((stateCounter > 0) && shutdownFlag)  {
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d", globalTime_ms*0.001, stateCounter);
//                shutdownFlag = false;
//            }
        } break;

        case Zero_Cal: {

            if (stateCounter < constants::CYCLES_FOR_ZERO_CAL) {
                zeroSumCount += 1;
                zeroSum1 += laserCurrentVoltsRaw;
                zeroSum2 += tecCurrentVoltsRaw;
                //  Sum PD1 and PD2
            } else {
                laserCurrentVoltsZero = zeroSum1/zeroSumCount;
                tecCurrentVoltsZero = zeroSum2/zeroSumCount;
                //  Calculate PD1 and PD2 average console
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Zero Cal: laserCurrentVoltsZero: %f tecCurrentVoltsZero: %f", globalTime_ms*0.001, laserCurrentVoltsZero, tecCurrentVoltsZero);
            for ( uint8_t ii=0; ii<constants::NUM_CHANNELS; ii++ ) {        // Save laser and TEC values
                laserNew[ii] = laserSave[ii];   //  Restore laser and tec settings
                tecNew[ii] = tecSave[ii];
            }
                switchState(Ready, 0 );
            }

//            if ( stateStageCounter > zeroCycleCount) {
//                currentChannelZeroCounts = analogRead(constants::PIN_IN_CURRENT);
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "new currentChannelZeroCounts = %d", currentChannelZeroCounts);

//                switchState ( Ready, 0);
//            }
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d, imagingActive: %d, shutdownFlag: %d", globalTime_ms*0.001, stateCounter, imagingActive, shutdownFlag);
//            if ((stateCounter > 0) && shutdownFlag)  {
//                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d", globalTime_ms*0.001, stateCounter);
//                shutdownFlag = false;
//            }
        } break;


        case Shutdown_Init: {
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d, imagingActive: %d, shutdownFlag: %d", globalTime_ms*0.001, stateCounter, imagingActive, shutdownFlag);
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Shutdown_Init with stateCounter: %d", globalTime_ms*0.001, stateCounter);
            for ( uint8_t ii=0; ii<constants::NUM_CHANNELS; ii++ ) {        // Updates for laser PWM
                laserNew[ii] = 0;
                tecNew[ii] = 0;
            }
            fanPWM = 0;
            if (stateCounter == 1)  {   //  Allow one cycle to turn off outputs
//            if ((stateCounter > 0) && shutdownFlag)  {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d", globalTime_ms*0.001, stateCounter);
                shutdownFlag = false;
                switchState(Shutdown, 0 );
            }
        } break;

        case Shutdown: {
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d, imagingActive: %d, shutdownFlag: %d", globalTime_ms*0.001, stateCounter, imagingActive, shutdownFlag);
            if ((stateCounter > 0) && shutdownFlag)  {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f MCU shutdown complete with stateCounter: %d", globalTime_ms*0.001, stateCounter);
                shutdownFlag = false;
            }
        } break;

        default: {  //  No action for Ready and Datapid states; these stay until changed by handle serial commands
        } break;

    }   //  End of switch over states

};

/*
bool StateControl::updateCRISPLED() {
    if(abs(crispLEDTargetValue - crispLEDActualValue) > 0.001) {  //  Update CRISP LED output
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating CRISP LED; old value: %f, new valueii: %f, writing value: %d, port: %u",
//            globalTime_ms*0.001, crispLEDTargetValue, crispLEDActualValue, int(crispLEDTargetValue*0.01*constants::ANALOG_OUT_MAX_COUNT), constants::PIN_OUT_LED_LEVEL);
        analogWrite(constants::PIN_OUT_LED_LEVEL, int(crispLEDTargetValue*0.01*constants::ANALOG_OUT_MAX_COUNT));
        crispLEDActualValue = crispLEDTargetValue;
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating CRISP LED; old value: %.2f, new valueii: %.2f, writing value: %i, port: %u",
//            globalTime_ms*0.001, crispLEDTargetValue, crispLEDActualValue, int(crispLEDTargetValue*0.01*constants::ANALOG_OUT_MAX_COUNT), constants::PIN_OUT_LED_LEVEL);
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating CRISP LED; old value: %.2f, new valueii: %.2f, writing value: %i, port: %u",
//            globalTime_ms*0.001, 1.0, 2.0, 3, 4);
    }
    return true;
}
*/

/*
bool StateControl::updateServo() {
    if(abs(servoTargetValue - servoActualValue) > 0.001) {  //  Update servo output
        analogWrite(constants::PIN_OUT_SERVO_LEVEL, int(servoTargetValue/constants::SERVO_MAX_VALUE*constants::ANALOG_OUT_MAX_COUNT));
        servoActualValue = servoTargetValue;
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating servo; old value: %f, new valueii: %f, writing value: %d, port: %d",
//             globalTime_ms*0.001, servoTargetValue, servoActualValue, int(servoActualValue*0.01*constants::ANALOG_OUT_MAX_COUNT), constants::PIN_OUT_SERVO_LEVEL);
    }
    return true;
}
*/
/*
bool StateControl::updatePID() {
    if(abs(photodiodeDifferenceSetPointTarget - photodiodeDifferenceSetPoint) > 0.001) {  //  Update servo output
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating error offset; old value: %f, new value: %f, error value: %f",
//            globalTime_ms*0.001, photodiodeDifferenceSetPoint, photodiodeDifferenceSetPointTarget, photodiodeDifferenceMeasure);
        photodiodeDifferenceSetPoint = photodiodeDifferenceSetPointTarget;
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating error offset; old value: %f, new value: %f, error value: %f",
//            globalTime_ms*0.001, photodiodeDifferenceSetPoint, photodiodeDifferenceSetPointTarget, photodiodeDifferenceMeasure);
    }
    return true;
}
*/

bool StateControl::updateLasers() {
//    bool killSwitchStatus = digitalRead( constants::PIN_22 );
    bool pwmChanged = false;
    for ( uint8_t ii=0; ii<constants::NUM_CHANNELS; ii++ ) {        // Updates for laser PWM
        if (temperaturesNew[ii] > constants::TEMPERATURE_CUTOUT) {  //  Turn off laser is over temperature
            if (laserNew[ii] > 0.0) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Warning : Channel %d has temperature %.2f, which is over cutoff temperature %.2f; turning off laser", 
                    globalTime_ms*0.001, ii, temperaturesNew[ii], constants::TEMPERATURE_CUTOUT);    
                laserNew[ii] = 0;
            }
        }
        if( laserNew[ii] < 0.0) laserNew[ii] = 0.0;
        if( laserNew[ii] > 100.0) laserNew[ii] = 100.0;
        if(abs(laserLast[ii] - laserNew[ii]) > 0.001) {  //  Update PWM output, which should be the same as pwmArrayLast
            analogWrite(constants::LASER_OUT_PIN_LIST[ii], laserNew[ii]*0.01*constants::ANALOG_OUT_MAX_COUNT);   //  Write PWM if changed
            laserLast[ii]=laserNew[ii];
            pwmChanged = true;
        }
    }
    return pwmChanged;
}

bool StateControl::updateFan() {
    bool pwmChanged = false;
    if( fanPWM < 0.0) fanPWM = 0.0;     //  Updates for fans PWM
    if( fanPWM > 100.0) fanPWM = 100.0;
    if(abs(fanPWMLast - fanPWM) > 0.001) {  //  Update PWM output, which should be the same as pwmArrayLast
        analogWrite(constants::FAN_OUT_PIN, fanPWM*0.01*constants::ANALOG_OUT_MAX_COUNT);   //  Write PWM if changed
        fanPWMLast=fanPWM;
        pwmChanged = true;
    }
    return pwmChanged;
}

errorType StateControl::reset_clock () {    //  Interrupt current process and reset parameters
    resetGlobalTime = true;
    resetStateTime = true;
    return NO_ERROR;
}

bool StateControl::mcuSetup() {


    analogWriteResolution(constants::ANALOG_OUT_RESOLUTION_BITS);
    analogReadResolution(constants::ANALOG_IN_RESOLUTION_BITS);

    for (uint8_t ii = 0; ii < constants::NUM_CHANNELS; ii++) { //  Set up laser, TEC, and thermistor pins
        pinMode(constants::LASER_OUT_PIN_LIST[ii], OUTPUT);  // Laser power control pinsth optional software PWM
        analogWriteFrequency(constants::LASER_OUT_PIN_LIST[ii], constants::LASER_OUT_FREQUENCY); 
        analogWrite(constants::LASER_OUT_PIN_LIST[ii], 0);   //  Initialize low; these pins are digital out wi
        pinMode(constants::TEC_OUT_PIN_LIST[ii], OUTPUT);  // TEC power control pins
        analogWriteFrequency(constants::TEC_OUT_PIN_LIST[ii], constants::TEC_OUT_FREQUENCY); 
        analogWrite(constants::TEC_OUT_PIN_LIST[ii], 0);   //  Initialize off
    //  Use INPUT_DISABLE on analog inputs with Teensy 4.0 to make sure digital input is disabled and avoid odd behavior
    //  https://forum.pjrc.com/index.php?threads/analog-input-impedance-and-pull-up.34319/
        pinMode(constants::THERMISTER_AIN_PIN_LIST[ii], INPUT_DISABLE);  // Thermistor analog input pins
    }
    pinMode(constants::FAN_OUT_PIN, OUTPUT);  // TEC power control pins
    analogWriteFrequency(constants::FAN_OUT_PIN, constants::FAN_OUT_FREQUENCY); 
    analogWrite(constants::FAN_OUT_PIN, 0);   //  Initialize off
//    for (uint8_t ii = 0; ii < constants::NUM_CHANNELS; ii++) { //  Set up laser, TEC, and thermistor pins
//        analogWrite(constants::TEC_OUT_PIN_LIST[ii], 0);   //  Initialize off
//    }

//  Input pins
    pinMode(constants::CLOCK_DIN_PIN, OUTPUT);  // TEC power control pins
    pinMode(constants::CAMERA_A_DIN_PIN, INPUT);  // TEC power control pins
    pinMode(constants::CAMERA_B_DIN_PIN, INPUT);  // TEC power control pins
    pinMode(constants::CAMERA_C_DIN_PIN, INPUT);  // TEC power control pins
    pinMode(constants::LASER_CURRENT_AIN_PIN, INPUT_DISABLE);  // TEC power control pins, 
    pinMode(constants::TEC_CURRENT_AIN_PIN, INPUT_DISABLE);  // TEC power control pins, 
    pinMode(constants::TTL_DIN_PIN, INPUT);  // TEC power control pins

//  Spare pin, wired to DA-15 connector currently, can be output; digital and analog input
//    pinMode(constants::SPARE_PIN, OUTPUT);  // TEC power control pins

    pinMode(constants::PIN_STATUS_LED, OUTPUT); // internal LED pin

//    pinMode(constants::PIN_IN_PD_1, INPUT_DISABLE);     //  Pins for two photodiode signal levels
//    pinMode(constants::PIN_IN_PD_2, INPUT_DISABLE);
//    pinMode(constants::PIN_OUT_LED_LEVEL, OUTPUT);      //  Pin for LED level control
//    pinMode(constants::PIN_OUT_SERVO_LEVEL, OUTPUT);    //  Pin for servo output level control
//    pinMode(constants::BLUE_LED_PIN, OUTPUT);
//    pinMode(constants::GREEN_LED_PIN, OUTPUT);
//    pinMode(constants::YELLOW_LED_PIN, OUTPUT);
//    pinMode(constants::RED_LED_PIN, OUTPUT);

    analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED

//    analogWriteResolution(constants::SERVO_ANALOG_OUT_RESOLUTION_BITS);
//    analogWriteFrequency(constants::PIN_OUT_LED_LEVEL, constants::CRISP_LED_PWM_FREQUENCY); 
//    analogWriteFrequency(constants::PIN_OUT_SERVO_LEVEL, constants::SERVO_PWM_FREQUENCY);
//    analogWriteResolution(constants::PIN_OUT_SERVO_LEVEL, constants::SERVO_ANALOG_OUT_RESOLUTION_BITS);
//    analogWriteResolution(constants::PIN_OUT_LED_LEVEL, constants::CRISP_LED_ANALOG_OUT_RESOLUTION_BITS);

    return true;
}

    float StateControl::computeTempFromAnalogRead( int vcount ) {

        const float vm = vcount;
        const int vs = constants::ANALOG_IN_MAX_COUNT;
//        static const uint16_t ANALOG_IN_MAX_COUNT = (1 << ANALOG_IN_RESOLUTION_BITS) - 1;
//        const int vs = ( 1 << constants::ANALOG_OUT_RESOLUTION_BITS ) - 1;
        
        const float vsMinusVm = vs - vm;

//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "vcount = %d vs = %d", vcount, vs);
        if ( vsMinusVm  < 1.0 ) {       //  Avoid NaN for divide by zero and logarithm of negative number
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Temperature measurement failed: vm and vs are too close to each other, vm:%.2f, vs:%d", vm, vs);
            return 0.0;
        }

        if ( vm  == 0 ) {       // Avoid NaN for logarithm of zero
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Temperature measurement failed: vm = 0");
            return 0.0;
        }

        const float rRatio = vm / vsMinusVm;    //  Thermistor resistance divided by reference resistance

        float c3;
        float c4;
        if ( rRatio > 0.36036 ) {
            c3 = 2.0829210e-06;
            c4 = 7.3003206e-08;
        }
        else {
            c3 = 1.9621987e-06;
            c4 = 4.6045930e-08;
        }
    
        const float logR = log( rRatio );
        const float logR2 = logR * logR;
        const float logR3 = logR2 * logR;
        const float c1 = 3.3539264e-03;
        const float c2 = 2.5609446e-04;
        const float tKelvin= 1.0 / (c1 + c2*logR + c3*logR2 + c4*logR3);
        const float tCelsius =  tKelvin - 273.15;

        if ( isnan(tCelsius)) {       //  Flag nan
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Temperature measurement failed tCelsius = nan, rRatio = [%f], vm = [%d]", rRatio, vm);
            return 0.0;
        }
        return tCelsius;
    }

    void StateControl::updateTECs() {
        for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
            if (newTECRequested[ii]) {  //  
                tecNew[ii] = tecFromGUI[ii];
                newTECRequested[ii] = false;
            }
            if (lockInit[ii]) {
                integratorValues[ii] = tecLast[ii];     //  Start integrator from current TEC value
                lockInit[ii] = false;
                lockOn[ii] = true;
                errorsLast[ii] = 0.0;
            }
            if (temperaturesNew[ii] > constants::TEMPERATURE_CUTOUT) {
                if (tecNew[ii] > 0.0) {
                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Warning : Channel %d has temperature %f, which is over cutoff temperature %f; turning off TEC", 
                        globalTime_ms*0.001, ii, temperaturesNew[ii], constants::TEMPERATURE_CUTOUT);    
                    tecNew[ii] = 0;
                }
                if (lockOn[ii]) {
                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Warning : Channel %d has temperature %f, which is over cutoff temperature %f; turning lock off channel %d", 
                        globalTime_ms*0.001, ii, temperaturesNew[ii], constants::TEMPERATURE_CUTOUT, ii);    
                    lockOn[ii] = false;
                }
            }
            if (lockOn[ii]) {
                errorsNew[ii] = temperatureSetPoint[ii] - temperaturesNew[ii];
                proportionalValues[ii] = PIDDirectionSign * kProp * errorsNew[ii];
                integratorValues[ii] = integratorValues[ii] + PIDDirectionSign * kInteg * constants::DATA_UPDATE_INTERVAL_SEC * 0.5 * (errorsNew[ii] + errorsLast[ii]);
                if (integratorValues[ii] > 100) {     //   Implement anti-windup using static integrator clamping
                    integratorValues[ii] = 100;
                } else if (integratorValues[ii] < 0) {
                    integratorValues[ii] = 0;
                }
                differentiatorValues[ii] = (2.0 * PIDDirectionSign * kDeriv * (-temperaturesNew[ii] + temperaturesLast[ii])
                    + (2.0 * differentiatorTimeConstant - constants::DATA_UPDATE_INTERVAL_SEC) * differentiatorValues[ii])   
                    / (2.0 * differentiatorTimeConstant + constants::DATA_UPDATE_INTERVAL_SEC);	//  formula for band-limited differentiator, minus sign for derivative on input
                tecNew[ii] = proportionalValues[ii] + integratorValues[ii] + differentiatorValues[ii];
//                if (tecNew[ii] > constants::ANALOG_OUT_MAX_COUNT ) {       //      Clamp output
//                    tecNew[ii] = constants::ANALOG_OUT_MAX_COUNT;
//                } else if (tecTargets[ii] < 0) {
//                    tecNew[ii] = 0;
//                }
                errorsLast[ii] = errorsNew[ii];
                if ( ii == 0) {
                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Lock_On state; errorValue: %f, proportional: %f, integrator: %f, differentiator: %f, servoTargetValue: %f, servoActualValue: %f", globalTime_ms*0.001,  errorValue, proportional, integrator, differentiator, servoTargetValue, servoActualValue);    
                    Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f In Lock_On state; proportionalValues[ii]: %f, integratorValues[ii]: %f, differentiatorValues[ii]: %f, errorsNew[ii]: %f, tecNew[ii]: %f", globalTime_ms*0.001, 
                        proportionalValues[ii], integratorValues[ii], differentiatorValues[ii], errorsNew[ii], tecNew[ii] );        
                }
            } else {
            }
//            tecNew[ii] = tecTargets[ii];
            if( tecNew[ii] < 0.0) tecNew[ii] = 0.0;
            if( tecNew[ii] > 100.0) tecNew[ii] = 100.0;
            if(abs(tecLast[ii] - tecNew[ii]) > 0.001) {  //  Update PWM output, which should be the same as pwmArrayLast
                analogWrite(constants::TEC_OUT_PIN_LIST[ii], tecNew[ii]*0.01*constants::ANALOG_OUT_MAX_COUNT);   //  Write PWM if changed
                tecLast[ii]=tecNew[ii];
//                pwmChanged = true;
            }
        }
        return;
    }

    void StateControl::initArrays() {
        for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
            newTECRequested[ii] = false;
            tecLast[ii] = 0.0;
        }
    }