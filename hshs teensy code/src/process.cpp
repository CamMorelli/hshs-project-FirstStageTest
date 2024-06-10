#include <Arduino.h>
#include "process.hpp"
#include "constants.hpp"
#include <conversions.hpp>
#include "simple_pid.hpp"
#include "serial_comm.hpp"

namespace process {

    SimplePID PIDsProcess[2];
    double lastTempMeasurements[2] = {0};

    errorType processServoLevel( LineParsingResult& parsingResult, StateControl& _sc ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtofResult inputServoLevel;
        conversions::atof(parsingResult.getToken(1), inputServoLevel);
        if ( !inputServoLevel.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if (inputServoLevel.value > constants::SERVO_MAX_VALUE) {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Warning, inputServoLevel of: %.3f is too high, setting to %0.3f", _sc.globalTime_ms*0.001, inputServoLevel, constants::SERVO_MAX_VALUE);
            _sc.servoTargetValue = constants::SERVO_MAX_VALUE;
        }
        else if (inputServoLevel.value < 0) {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Warning, inputServoLevel of: %.3f is too low, setting to %0.3f", _sc.globalTime_ms*0.001, inputServoLevel, 0.0);
            _sc.servoTargetValue = constants::SERVO_MAX_VALUE;
        } else {
            _sc.servoTargetValue = inputServoLevel.value;
        }
        return NO_ERROR;
    }

    errorType processLedLine( LineParsingResult& parsingResult ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
            analogWrite(constants::PIN_STATUS_LED, constants::ANALOG_OUT_MAX_COUNT);     //  turn on LED
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            analogWrite(constants::PIN_STATUS_LED, 0);     //  turn off LED
            return NO_ERROR;
        }
        return INVALID_ARGUMENT_TYPE;
    }

    errorType processSensorLine( LineParsingResult& parsingResult, StateControl& _sc) {   //  TODO: How to handle case when already in desired state
//    int16_t status;
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
//            if (status == 0) {
                _sc.dataOn = true;
            _sc.switchState(Ready, 0 );
                return NO_ERROR;
//            } else {
//                dataOn = false;
//                return CANNOT_COMPLETE_COMMAND;
///            }
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {        //  Off stops transmitting data on the serial port, but not stop I2C
            _sc.dataOn = false;
            _sc.switchState(Data_Off, 0 );
//            _sc.waitForFrameTime = 0; _sc.readFrameTime = 0; _sc.calculateT0Time = 0; 
            return NO_ERROR;
        }
        return INVALID_ARGUMENT_TYPE;
    }
/*
    errorType processZeroCal( LineParsingResult& parsingResult, StateControl& _sc) {
        if ( parsingResult.getTokenCount() != 1 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        _sc.zeroSumCount = 0; _sc.zeroSum1 = 0; _sc.zeroSum2 = 0;   //  Initialize zero cal parameters
        _sc.crispLEDSaveValue = _sc.crispLEDTargetValue;    //  Store LED level
        _sc.crispLEDTargetValue = 0;    //  Turn off CRISP LED
        _sc.switchState(Zero_Init, 0 );  //  This is used for proper handling of writeTransition
        return NO_ERROR;
    }
*/

    errorType processShutdown( LineParsingResult& parsingResult, StateControl& _sc) {
        if ( parsingResult.getTokenCount() != 1 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        _sc.shutdownFlag = true;
        _sc.resetGlobalTime = true;     //  Reset clocks
        _sc.resetStateTime = true;
        _sc.dataOn = false;
        _sc.switchState(Shutdown_Init, 0 );  //  This is used for proper handling of writeTransition
        return NO_ERROR;
    }

    errorType process_uint16_t( LineParsingResult& parsingResult, uint16_t& newValue ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtoiResult newValueI;  //  The first token is command, the second is integer
        conversions::atoi(parsingResult.getToken(1), newValueI);
        if ( !newValueI.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        newValue = newValueI.value; 
        return NO_ERROR;
    }

    errorType process_float( LineParsingResult& parsingResult, float& newValue ) { //  Parse single float from serial as reference
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtofResult newValueF;  //  The first token is command, the second is float
        conversions::atof(parsingResult.getToken(1), newValueF);
        if ( !newValueF.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        newValue = newValueF.value; 
        return NO_ERROR;
    }

//  TODO: Create conversions for double input?
    errorType process_double( LineParsingResult& parsingResult, double& newValue ) { //  Parse single float from serial as reference
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtofResult newValueF;  //  The first token is command, the second is double
        conversions::atof(parsingResult.getToken(1), newValueF);
        if ( !newValueF.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        newValue = newValueF.value; 
        return NO_ERROR;
    }

    errorType processScan(LineParsingResult&  parsingResult , StateControl& _sc ) {
            if (_sc.state != Ready) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
                return CANNOT_COMPLETE_COMMAND;
            } else if ( parsingResult.getTokenCount() != 3 ) {
                return INVALID_ARGUMENT_COUNT;
            }
                conversions::AtofResult newValue;
                for (uint16_t i=0; i < 2; i++ ) {   
                conversions::atof(parsingResult.getToken(i+1), newValue); //  Second and third tokens are floats
                if ( !newValue.valid ) {
                    return INVALID_ARGUMENT_TYPE;
                } else { switch (i) {
                    case 0: _sc.scanTime = newValue.value; break; 
                    case 1: _sc.scanRange = newValue.value; break; }
                }
            }
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f scanTime: [%f], scanRange: [%f]", 
//            _sc.globalTime_ms*0.001, _sc.scanTime, _sc.scanRange);
            _sc.switchState(Scan_Init, 0 );
            return NO_ERROR;
        }

/*
    errorType processLEDScan(LineParsingResult&  parsingResult , StateControl& _sc ) {
            if (_sc.state != Ready) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
                return CANNOT_COMPLETE_COMMAND;
            } else if ( parsingResult.getTokenCount() != 4 ) {
                return INVALID_ARGUMENT_COUNT;
            }
                conversions::AtofResult newValue;
                for (uint16_t i=0; i < 3; i++ ) {   
                conversions::atof(parsingResult.getToken(i+1), newValue); //  Second and third tokens are floats
                if ( !newValue.valid ) {
                    return INVALID_ARGUMENT_TYPE;
                } else { switch (i) {
                    case 0: _sc.ledScanTime = newValue.value; break; 
                    case 1: _sc.ledScanLow = newValue.value; break;
                    case 2: _sc.ledScanHigh = newValue.value; break; }
                }
            }
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f ledScanTime: [%f], ledScanLow: [%f] ledScanHigh: [%f]", 
//            _sc.globalTime_ms*0.001, _sc.ledScanTime, _sc.ledScanLow, _sc.ledScanHigh);
            _sc.switchState(LED_Scan_Init, 0 );
            return NO_ERROR;
        }
*/

/*
    errorType processDither(LineParsingResult&  parsingResult , StateControl& _sc ) {
            if (_sc.state != Ready) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
                return CANNOT_COMPLETE_COMMAND;
            } else if ( parsingResult.getTokenCount() != 2 ) {
                return INVALID_ARGUMENT_COUNT;
            }
            conversions::AtofResult newValue;
            conversions::atof(parsingResult.getToken(1), newValue);
            if ( !newValue.valid ) {
                return INVALID_ARGUMENT_TYPE;
            } else { 
                _sc.hopValue = newValue.value;
            }
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f Start Dither, hopValue: [%f]", 
//            _sc.globalTime_ms*0.001, _sc.hopValue);
            _sc.switchState(Dither_Init, 0 );
            return NO_ERROR;
        }
*/

/*
    errorType processLock(LineParsingResult&  parsingResult , StateControl& _sc ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
//            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f getToken = %s", _sc.globalTime_ms*0.001, parsingResult.getToken(1));
            if (_sc.state == Lock_On) {
                return NO_ERROR;
            } else if (_sc.state != Ready) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
                return CANNOT_COMPLETE_COMMAND;
            } else {
            _sc.switchState(Lock_Init, 0 );
            return NO_ERROR;
            }
        } else if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            if (_sc.state != Lock_On) {
                Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in lock state", _sc.globalTime_ms*0.001);
                return CANNOT_COMPLETE_COMMAND;
            }
            _sc.switchState(Ready, 0 );
            return NO_ERROR;
        }
        return INVALID_ARGUMENT_TYPE;
    }
*/

    errorType processLaserPWM( LineParsingResult& parsingResult, StateControl& _sc ) {
        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
            returnValue = NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 3 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtoiResult channel;
        conversions::atoi(parsingResult.getToken(1), channel);
        if ( !channel.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( channel.value < 0 ) || ( channel.value > (constants::NUM_CHANNELS - 1) ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        conversions::AtofResult pwmValue;
        conversions::atof(parsingResult.getToken(2), pwmValue);
        if ( !pwmValue.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( pwmValue.value < 0 ) || ( pwmValue.value > 100 ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.laserNew[channel.value] = pwmValue.value;
        return returnValue;
    }

    errorType processTECPWM( LineParsingResult& parsingResult, StateControl& _sc ) {
        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
        returnValue = NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 3 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtoiResult channel;
        conversions::atoi(parsingResult.getToken(1), channel);
        if ( !channel.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( channel.value < 0 ) || ( channel.value > (constants::NUM_CHANNELS - 1) ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        conversions::AtofResult pwmValue;
        conversions::atof(parsingResult.getToken(2), pwmValue);
        if ( !pwmValue.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( pwmValue.value < 0 ) || ( pwmValue.value > 100 ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.tecFromGUI[channel.value] = pwmValue.value;
        _sc.newTECRequested[channel.value] = true;
        return returnValue;
    }

    errorType processFanPWM( LineParsingResult& parsingResult, StateControl& _sc ) {
        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
        returnValue = NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtofResult pwmValue;
        conversions::atof(parsingResult.getToken(1), pwmValue);
        if ( !pwmValue.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( pwmValue.value < 0 ) || ( pwmValue.value > 100 ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.fanPWM = pwmValue.value;
        return returnValue;
    }
    errorType processZeroCal( LineParsingResult& parsingResult, StateControl& _sc ) {
        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
        returnValue = NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 1 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        _sc.switchState(Zero_Init, 0 );
        return returnValue;
    }

    errorType processTempSetPoint( LineParsingResult& parsingResult, StateControl& _sc ) {
        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
        returnValue = NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 3 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtoiResult channel;
        conversions::atoi(parsingResult.getToken(1), channel);
        if ( !channel.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( channel.value < 0 ) || ( channel.value > (constants::NUM_CHANNELS - 1) ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        conversions::AtofResult tempSetPnt;
        conversions::atof(parsingResult.getToken(2), tempSetPnt);
        if ( !tempSetPnt.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( tempSetPnt.value < 0 ) || ( tempSetPnt.value > 100 ) ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.temperatureSetPoint[channel.value] = tempSetPnt.value;
        return returnValue;
    }

    errorType processBlueLedLine( LineParsingResult& parsingResult ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
            digitalWrite(constants::BLUE_LED_PIN, HIGH);
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            digitalWrite(constants::BLUE_LED_PIN, LOW);
            return NO_ERROR;
        }
        return CANNOT_COMPLETE_COMMAND;
    }

    errorType processGreenLedLine( LineParsingResult& parsingResult ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
            digitalWrite(constants::GREEN_LED_PIN, HIGH);
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            digitalWrite(constants::GREEN_LED_PIN, LOW);
            return NO_ERROR;
        }
        return CANNOT_COMPLETE_COMMAND;
    }

    errorType processYellowLedLine( LineParsingResult& parsingResult ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
            digitalWrite(constants::YELLOW_LED_PIN, HIGH);
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            digitalWrite(constants::YELLOW_LED_PIN, LOW);
            return NO_ERROR;
        }
        return CANNOT_COMPLETE_COMMAND;
    }

    errorType processRedLedLine( LineParsingResult& parsingResult ) {
        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "on" ) == 0 ) {
            digitalWrite(constants::RED_LED_PIN, HIGH);
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "off" ) == 0 ) {
            digitalWrite(constants::RED_LED_PIN, LOW);
            return NO_ERROR;
        }
        return CANNOT_COMPLETE_COMMAND;
    }


    void printPIDInitializationStatus ( bool startSucceeded ) {

        if ( startSucceeded ) {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "PID is now completely initialized" );
        }
        else {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "PID is still partially initialized" );
        }
    }

    errorType processPIDGainCommand( LineParsingResult& parsingResult, StateControl& _sc ) {

        if ( parsingResult.getTokenCount() != 4 ) {  
            return INVALID_ARGUMENT_COUNT;
        }

        conversions::AtoiResult tmpA;
        conversions::atoi(parsingResult.getToken(1), tmpA);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "kProp[%.3f]", tmpA.value );
        if ( !tmpA.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.kProp = tmpA.value;
//        if ( pid_index < 0 || pid_index > 1 ) {
//            return INVALID_ARGUMENT_TYPE;
//        }

        conversions::AtofResult tmpB;
        conversions::atof(parsingResult.getToken(2), tmpB);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "kInteg:[%.3f]", tmpB.value );
        if ( !tmpB.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.kInteg = tmpB.value;

        conversions::AtofResult tmpC;
        conversions::atof(parsingResult.getToken(3), tmpC);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "kDeriv:[%.3f]", tmpC.value );
        if ( !tmpB.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.kDeriv = tmpC.value;

//        conversions::atof(parsingResult.getToken(4), tmpB);
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "Kd:[%.3f]", tmpB.value );
//        if ( !tmpB.valid ) {
//            return INVALID_ARGUMENT_TYPE;
//        }
//        const double kd = tmpB.value;

//        PIDsProcess[pid_index].stop(); 
//        PIDsProcess[pid_index].reset(); 
//        PIDsProcess[pid_index].setSamplingPeriod( constants::DATA_UPDATE_INTERVAL_SEC ); 
//        PIDsProcess[pid_index].setGains( kp, ki, kd ); 

//        const bool startSucceeded = PIDsProcess[pid_index].start();
//        PIDsProcess[pid_index].printState();
//        printPIDInitializationStatus ( startSucceeded );

        return NO_ERROR;
    }
    
    errorType processPIDDirectionCommand( LineParsingResult& parsingResult, StateControl& _sc ) {

        if ( parsingResult.getTokenCount() != 2 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        if ( strcmp( parsingResult.getToken(1), "F" ) == 0 ) {
            _sc.PIDDirectionSign = 1;
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(1), "R" ) == 0 ) {
            _sc.PIDDirectionSign = -1;
            return NO_ERROR;
        }
        return INVALID_ARGUMENT_TYPE;
    }

    errorType processPIDSetPointCommand( LineParsingResult& parsingResult ) {

        if ( parsingResult.getTokenCount() != 3 ) {  
            return INVALID_ARGUMENT_COUNT;
        }

        conversions::AtoiResult tmpA;
        conversions::atoi(parsingResult.getToken(1), tmpA);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "pid index:[%d]", tmpA.value );
        if ( !tmpA.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        const int pid_index = tmpA.value;
        if ( pid_index < 0 || pid_index > 1 ) {
            return INVALID_ARGUMENT_TYPE;
        }

        conversions::AtofResult tmpB;
        conversions::atof(parsingResult.getToken(2), tmpB);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "set point:[%.3f]", tmpB.value );
        if ( !tmpB.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }

        PIDsProcess[pid_index].stop();
        PIDsProcess[pid_index].reset(); 
        PIDsProcess[pid_index].setOutputSetPoint( (double) tmpB.value );

        const bool startSucceeded = PIDsProcess[pid_index].start();
        PIDsProcess[pid_index].printState();
        printPIDInitializationStatus ( startSucceeded );

        return NO_ERROR;
    }

    errorType processPIDSetDifferentiatorTimeConstantCommand( LineParsingResult& parsingResult, StateControl& _sc ) {

        if ( parsingResult.getTokenCount() != 2 ) {  
            return INVALID_ARGUMENT_COUNT;
        }

//        conversions::AtoiResult tmpA;
//        conversions::atoi(parsingResult.getToken(1), tmpA);
//        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "pid index:[%d]", tmpA.value );
//        if ( !tmpA.valid ) {
//            return INVALID_ARGUMENT_TYPE;
//        }
//        const int pid_index = tmpA.value;
//        if ( pid_index < 0 || pid_index > 1 ) {
//            return INVALID_ARGUMENT_TYPE;
//        }

        conversions::AtofResult tmpB;
        conversions::atof(parsingResult.getToken(1), tmpB);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "differentiator time constant:[%.3f]", tmpB.value );
        if ( !tmpB.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        _sc.differentiatorTimeConstant = tmpB.value;

//        PIDsProcess[pid_index].stop(); 
//        PIDsProcess[pid_index].reset(); 
//        PIDsProcess[pid_index].setDifferentiatorTimeConstant( differentiator_time_constant ); 

//        const bool startSucceded = PIDsProcess[pid_index].start();
//        PIDsProcess[pid_index].printState();
//        printPIDInitializationStatus ( startSucceded );

        return NO_ERROR;
    }

    errorType processResetPID(LineParsingResult& parsingResult ) {
    
        if ( parsingResult.getTokenCount() != 2 ) {  
            return INVALID_ARGUMENT_COUNT;
        }

        conversions::AtoiResult tmpA;
        conversions::atoi(parsingResult.getToken(1), tmpA);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "pid index:[%d]", tmpA.value );
        if ( !tmpA.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        const int pid_index = tmpA.value;
        if ( pid_index < 0 || pid_index > 1 ) {
            return INVALID_ARGUMENT_TYPE;
        }

        PIDsProcess[pid_index].reset();
        Serial_IO::printToConsole( Serial_IO::Receiver::Both, "Resetting PID" );
        PIDsProcess[pid_index].printState();

        return NO_ERROR;
    }

    errorType processPIDState(LineParsingResult& parsingResult ) {
    
        if ( parsingResult.getTokenCount() != 2 ) {  
            return INVALID_ARGUMENT_COUNT;
        }

        conversions::AtoiResult tmpA;
        conversions::atoi(parsingResult.getToken(1), tmpA);
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "pid index:[%d]", tmpA.value );
        if ( !tmpA.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        const int pid_index = tmpA.value;
        if ( pid_index < 0 || pid_index > 1 ) {
            return INVALID_ARGUMENT_TYPE;
        }

        PIDsProcess[pid_index].printState();

        return NO_ERROR;
    }

    errorType processLockNew( LineParsingResult& parsingResult, StateControl& _sc ) {
//        errorType returnValue = NO_ERROR;
        if (_sc.state != Ready) {
            Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f System not in ready state", _sc.globalTime_ms*0.001);
            return NOT_READY;
        }
        if ( parsingResult.getTokenCount() != 3 ) {
            return INVALID_ARGUMENT_COUNT;
        }
        conversions::AtoiResult channel;
        conversions::atoi(parsingResult.getToken(1), channel);
        if ( !channel.valid ) {
            return INVALID_ARGUMENT_TYPE;
        }
        if ( ( channel.value < 0 ) || ( channel.value > (constants::NUM_CHANNELS - 1) ) ) {
            return INVALID_ARGUMENT_TYPE;
        }

        if ( strcmp( parsingResult.getToken(2), "on" ) == 0 ) {
            _sc.lockInit[channel.value] = true;
            return NO_ERROR;
        }
        if ( strcmp( parsingResult.getToken(2), "off" ) == 0 ) {
            _sc.lockOn[channel.value] = false;
            return NO_ERROR;
        }
        return CANNOT_COMPLETE_COMMAND;
    }
}
