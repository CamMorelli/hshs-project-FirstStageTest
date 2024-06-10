
#ifndef __state_control__
#define __state_control__

#include "constants.hpp"
//#include "usb_serial.h"

//enum stateType {Ready = 0, Reset = 1, Data_Off = 2, Lock_Init = 3, Lock_On = 4, Scan_Init = 5, Scan_On = 6, LED_Scan_Init = 7, LED_Scan_On = 8, Dither_Init = 9, Dither_On = 10, Shutdown = 11, Zero_Init = 12, Zero_Cal = 13};
//enum stateType {Standby = 0, Ready = 1, Reset = 2, Data_Off = 3, Scan_Init = 4, Scan_On = 5, Shutdown_Init = 6, Shutdown = 7};
enum stateType {Standby = 0, Ready = 1, Reset = 2, Data_Off = 3, Scan_Init = 4, Scan_On = 5, Shutdown_Init = 6, Shutdown = 7, Zero_Init = 8, Zero_Cal = 9};

class  StateControl {     //  Updates state parameters from arguments; resets state time, initializes Fit
  public:
    StateControl();
    stateType state, stateLast;
    uint16_t stateStageCounter;

    bool dataOn, dataOnLast;
    double photodiodeDifferenceMeasure;    //  Has maximal range of -1000 to 1000?
    float crispLEDTargetValue, crispLEDActualValue, crispLEDSaveValue;
    double pd1Value, pd2Value, pd1Raw, pd2Raw, servoTargetValue, servoActualValue, servoMaxValue;
    double temperaturesNew[constants::NUM_CHANNELS];
//    double laserCurrentValue, tecCurrentValue, laserCurrentRaw, tecCurrentRaw, laserCurrentZero, tecCurrentZero;
    double laserCurrentValue, tecCurrentValue;
    double laserCurrentVoltsRaw, laserCurrentVoltsZero, tecCurrentVoltsRaw, tecCurrentVoltsZero;
    double kProp, kInteg, kDeriv;
    double servoTimeConst, photodiodeDifferenceSetPointTarget, photodiodeDifferenceSetPoint, scanTime, scanRange, scanStart, scanStop, servoValueInitial, reportInterval;
    uint32_t errorSumCount, zeroSumCount;
    double zeroSum1, zeroSum2, pd1Zero, pd2Zero;
    float errorSumBuffer, errorSumBufferLast, averageError, pd1SumBuffer, averagePD1, pd2SumBuffer, averagePD2, pdDiffSumBuffer, averagePDDiff;
    float averageTherm[constants::NUM_CHANNELS], averageLaserCurrent, averageTECCurrent;
    float thermBuffer[constants::NUM_CHANNELS], laserCurrentBuffer, tecCurrentBuffer;
    float laserNew[constants::NUM_CHANNELS], tecNew[constants::NUM_CHANNELS], fanPWM;
    float laserLast[constants::NUM_CHANNELS], tecLast[constants::NUM_CHANNELS], fanPWMLast;
    float laserSave[constants::NUM_CHANNELS], tecSave[constants::NUM_CHANNELS];
    float temperatureSetPoint[constants::NUM_CHANNELS];

    float ledScanTime, ledScanLow, ledScanHigh, ledScanInitial;
    bool ditherState;
    float hopValue, ditherError;
    uint32_t analogReadCount;
//    uint32_t zeroCycleCount;
    int32_t ledFlashCounter, ledFlashCounterLast;
    double lastErrorValue;
    double lastInput;
//    double differenceSetPoint;
//    double kp, ki, kd;
    double differentiatorTimeConstant;
    double errorValue;
//    double newOutput;
    double samplingPeriod;
    double proportional;
    double integrator;
    double differentiator;
    float PIDDirectionSign;
    double proportionalValues[constants::NUM_CHANNELS];
    double integratorValues[constants::NUM_CHANNELS];
    double differentiatorValues[constants::NUM_CHANNELS];
    double temperaturesLast[constants::NUM_CHANNELS];
    bool lockInit[constants::NUM_CHANNELS];
    bool lockOn[constants::NUM_CHANNELS];
    double errorsNew[constants::NUM_CHANNELS];
    double errorsLast[constants::NUM_CHANNELS];
    float tecTargets[constants::NUM_CHANNELS];
    float tecFromGUI[constants::NUM_CHANNELS];
    bool newTECRequested[constants::NUM_CHANNELS];

//    bool pidForward, pidDirectionTarget;

//  Variable returns: crispLEDValue, photodiodeDifferenceMeasure, pdOneVolts, pdTwoVolts, servoLevel, (PIDgains), time, state (ready, lock, scan), integrationTime

    uint16_t debuggingPointCountInterval;
//    uint32_t waitForFrameTime, readFrameTime, calculateT0Time, writeDataTime, updateOutputsTime, innerLoopTime, handleSerialTime;
    uint32_t writeDataTime, updateOutputsTime, innerLoopTime, handleSerialTime;
    uint16_t temperature_sum_count;

    uint32_t globalCounter;     //  Clock attributes
    uint32_t stateCounter;
    boolean resetStateTime;
    boolean resetCameraClock;
    uint32_t stateStartTime_ms;
    uint32_t globalStartTime_ms;
    uint32_t stateTime_ms;
    uint32_t globalTime_ms;
    boolean resetGlobalTime;

    bool fittingInProcess;


    int16_t fitNumPoints;
    uint32_t fitStartCounter;

    double sumx, sumy, sumxy, sumx2;
    float slopeVals, interceptVals, debugData;

    uint32_t stateTimeOutSec;
    bool firstPass;
    int32_t numFailedFrames, maxNumFailedFrames;
    uint16_t currentChannelZeroCounts;
    bool shutdownFlag;


    errorType switchState(stateType stateNew, uint8_t stateStageCounterNew );
    void resetFitParameters ();
    void appendPoints (uint32_t globalCounter, float xVal, float yVals);
    void calcFits ();
    bool endFits(stateType stateNew);
    bool updateClocks();
    bool updateOutputs();
    void updateState( );

    errorType reset_clock ();
    bool mcuSetup();


    bool writeData(StateControl& _sc );

//    bool updateCRISPLED();
//    bool updateServo();
//    bool updatePID();
    bool updateLasers();
    bool updateFan();
    float computeTempFromAnalogRead( int vcount );
    void updateTECs();
    void initArrays();

};

#endif