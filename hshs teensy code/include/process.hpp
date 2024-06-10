#ifndef __process_hpp__
#define __process_hpp__

#include "line_parser.hpp"
#include "state_control.hpp"


//#include "usb_serial.h"

//class StateControl;

namespace process {
    errorType processServoLevel( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processLedLine( LineParsingResult& parsingResult );
    errorType processSensorLine( LineParsingResult& parsingResult, StateControl& _sc );
    errorType process_uint16_t( LineParsingResult& parsingResult, uint16_t& newValue );
    errorType process_float( LineParsingResult& parsingResult, float& newValue );
    errorType process_double( LineParsingResult& parsingResult, double& newValue );
    errorType processScan(LineParsingResult&  parsingResult , StateControl& _sc );
//    errorType processLEDScan(LineParsingResult&  parsingResult , StateControl& _sc );
//    errorType processDither( LineParsingResult&  parsingResult , StateControl& _sc );
//    errorType processLock(LineParsingResult&  parsingResult , StateControl& _sc );
    void printPIDInitializationStatus ( bool startSucceeded );
    errorType processPIDGainCommand( LineParsingResult& parsingResult , StateControl& _sc);
    errorType processPIDDirectionCommand( LineParsingResult& parsingResult , StateControl& _sc);
    errorType processPIDSetPointCommand( LineParsingResult& parsingResult );
    errorType processPIDSetDifferentiatorTimeConstantCommand( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processResetPID(LineParsingResult& parsingResult );
    errorType processPIDState(LineParsingResult& parsingResult );
    errorType processZeroCal( LineParsingResult& parsingResult, StateControl& _sc);
    errorType processBlueLedLine( LineParsingResult& parsingResult );
    errorType processGreenLedLine( LineParsingResult& parsingResult );
    errorType processYellowLedLine( LineParsingResult& parsingResult );
    errorType processRedLedLine( LineParsingResult& parsingResult );
    errorType processLaserPWM( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processTECPWM( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processFanPWM( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processTempSetPoint( LineParsingResult& parsingResult, StateControl& _sc );
//    errorType processLock( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processLockNew( LineParsingResult& parsingResult, StateControl& _sc );
    errorType processShutdown( LineParsingResult& parsingResult, StateControl& _sc);
}

#endif
