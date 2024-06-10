#include "serial_comm.hpp"
#include "state_control.hpp"
#include <line_delimiter.hpp>  //  These are part of common libraries
#include "process.hpp"
#include <string>
#include <TeensyID.h>

//#include <stdlib.h>
//#include <ctype.h>
//#include <limits.h>
//#include <errno.h>
//#include <cstdlib>

static LineDelimiter lineDelimiter(256);
static LineParser lineParser(256);
static LineParsingResult parsingResult(24, 24);

static Serial_IO::Receiver currentReceiver = Serial_IO::Receiver::Human;

void Serial_IO::setReceiver( Receiver r ) {
    currentReceiver = r;
}

Serial_IO::Receiver Serial_IO::getReceiver() {
    return currentReceiver;
}

bool Serial_IO::startSerial() {
    Serial.begin(115200); //  This is optional on Teensy and the baud rate setting is ignored with USB serial On Teensy, Teensy USB Serial object always communicates at native USB speed, either 12 or 480 Mbit/sec
    while (!Serial && millis() < 2000) {    //  Variable Serial is an external object of type usb_serial_class.
        // wait up to 2 seconds for Arduino Serial Monitor
        if (Serial) return true;
    }
    return false;
}

void Serial_IO::printToConsole( Receiver intendedReceiver, const char * format, ... ) {

    static uint64_t line_count = 0;
    static char buffer[256];

    va_list args;
    va_start( args, format );
    memset(buffer, 0, sizeof(buffer));
    vsnprintf( buffer, sizeof(buffer)-1, format, args );
    va_end( args );

    if ( currentReceiver == Receiver::Human  && 
            (intendedReceiver == Receiver::Human || intendedReceiver == Receiver::Both) ) {
        Serial.write(0xFD);     //  Using high bit set to denote start of new chunk of data
        Serial.printf( "[%s]", teensySN() );
        Serial.printf( " %06d ", line_count++ );
        Serial.printf( "%s", buffer );
        Serial.printf( "\r\n" );
    }

    if ( currentReceiver == Receiver::Machine  && 
        (intendedReceiver == Receiver::Machine || intendedReceiver == Receiver::Both) ) {
        Serial.write(0xFD);     //  Using high bit set to denote start of new chunk of data
        Serial.printf( "%s", buffer );
        Serial.printf( "\r\n" );
    }
}

boolean Serial_IO::writeTwoCodedBytes (uint16_t intVal ) {  //  Write data coded as two 7-bit values into 8-bit bytes
      Serial.write( (u_int8_t) ((intVal >> 7) & 0x007F) ); // Place upper 7 bits into lower 7 bits of first byte
      Serial.write( (u_int8_t) (intVal & 0x007F) ); // Place lower 7 bits into lower 7 bits of second byte
  return (true);
}

boolean Serial_IO::writeFourCodedBytes (uint32_t intVal ) {
      Serial.write( (u_int8_t) ((intVal >> 21) & 0x007F) ); // Place upper 7 bits into lower 7 bits of first byte
      Serial.write( (u_int8_t) ((intVal >> 14) & 0x007F) ); // Place next 7 bits into lower 7 bits of second byte
      Serial.write( (u_int8_t) ((intVal >> 7) & 0x007F) ); // Place next 7 bits into lower 7 bits of third byte
      Serial.write( (u_int8_t) (intVal & 0x007F) ); // Place lower 7 bits into lower 7 bits of second byte
  return (true);
}

boolean Serial_IO::writeTransitionData (uint8_t transitionType, uint32_t firstPoint, uint16_t numPoints, float slopeValues, float interceptValues, 
    float debugData, float offset) {
    Serial.write(0xFC); //  Write transtion fit data;  0xFC signals start of data
    writeTwoCodedBytes((uint16_t) transitionType );
    writeFourCodedBytes((uint32_t) firstPoint );
    writeTwoCodedBytes((uint16_t) numPoints );
    writeFourCodedBytes((uint32_t) (slopeValues*10000.0+110000000) );
    writeFourCodedBytes((uint32_t) (interceptValues*10000.0+110000000) );
    writeFourCodedBytes((uint32_t) (debugData*1000.0) );
    writeTwoCodedBytes((uint16_t) offset*100 );
  return (true);
}

bool Serial_IO::writeBufferCode(uint32_t code ) {
  Serial.write(code);
  return true;
}

/*
bool Serial_IO::writeHSHSData(StateControl& _sc) {

//        float photodiodeDifferenceMeasure, crispLEDValue, pd1Value, pd2Value, servoValue, kProp, kInteg, kDeriv, servoTimeConst, errorOffset, scanTime, scanStart, scanStop, reportInterval;

//  N.B. Always use (unit32_t) with writeFourCodedBytes
    writeBufferCode(0xFE ); //  0xFE signals start of frame
    writeFourCodedBytes(_sc.globalTime_ms );
    writeFourCodedBytes(_sc.stateTime_ms );
    writeFourCodedBytes(_sc.globalCounter );
    writeFourCodedBytes(_sc.stateCounter );

    writeTwoCodedBytes((uint16_t)(150*_sc.stateStageCounter+_sc.state) );  //  Write state enum and counter
    writeTwoCodedBytes((uint16_t)(10000*_sc.averagePD1) );  //  Write state enum and counter; for two bytes, maximum number without overflow is 2^14 = 16384
    writeTwoCodedBytes((uint16_t)(10000*_sc.averagePD2) );  //  Write state enum and counter
    writeFourCodedBytes((uint32_t) (_sc.averagePDDiff*100000.0+110000000) ); //  photodiodeDifferenceMeasure has range from -1 to 1 for four bytes, the maximum number without overflow is 2^28 = 268,435,456
//    writeTwoCodedBytes((uint16_t)(10000*_sc.pd1Value) );  //  Write state enum and counter; for two bytes, maximum number without overflow is 2^14 = 16384
//    writeTwoCodedBytes((uint16_t)(10000*_sc.pd2Value) );  //  Write state enum and counter
//    writeFourCodedBytes((uint32_t) (_sc.photodiodeDifferenceMeasure*100000.0+110000000) ); //  photodiodeDifferenceMeasure has range from -1 to 1 for four bytes, the maximum number without overflow is 2^28 = 268,435,456
    writeFourCodedBytes((uint32_t) (_sc.photodiodeDifferenceSetPoint*100000.0+110000000) );

    if (_sc.state == Dither_On) {
        writeFourCodedBytes((uint32_t) (_sc.ditherError*50000.0+110000000) ); //  Assuming set point has same range as photodiodeDifferenceMeasure, error could range from -2000 to 2000 for four bytes, the maximum number without overflow is 2^28 = 268,435,456
// Can't write to console in middle of data package!!!        printToConsole( Receiver::Human, "%.3f writing for ditherState, _sc.ditherError = %f",  _sc.globalTime_ms*0.001, _sc.ditherError );
    } else {
        writeFourCodedBytes((uint32_t) (_sc.errorValue*50000.0+110000000) ); //  Assuming set point has same range as photodiodeDifferenceMeasure, error could range from -2000 to 2000 for four bytes, the maximum number without overflow is 2^28 = 268,435,456
    }
    writeFourCodedBytes((uint32_t) (_sc.averageError*50000.0+110000000) );

    writeFourCodedBytes((uint32_t)(10000*_sc.servoActualValue) );  //  Write state enum and counter
    writeTwoCodedBytes((uint16_t)(100*_sc.crispLEDActualValue) );  //  Write state enum and counter

    return true;
}
*/

bool Serial_IO::writeHSHSDataNew(StateControl& _sc) {

//        float photodiodeDifferenceMeasure, crispLEDValue, pd1Value, pd2Value, servoValue, kProp, kInteg, kDeriv, servoTimeConst, errorOffset, scanTime, scanStart, scanStop, reportInterval;

//  N.B. Always use (unit32_t) with writeFourCodedBytes
    writeBufferCode(0xFE ); //  0xFE signals start of frame
    writeFourCodedBytes(_sc.globalTime_ms );
    writeFourCodedBytes(_sc.stateTime_ms );
    writeFourCodedBytes(_sc.globalCounter );
    writeFourCodedBytes(_sc.stateCounter );
    writeTwoCodedBytes((uint16_t)(100*_sc.stateStageCounter+_sc.state) );  //  Write state enum and counter

    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
        writeTwoCodedBytes((uint16_t)(150*_sc.laserNew[ii]) );
    }
    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
        writeTwoCodedBytes((uint16_t)(150*_sc.tecNew[ii]) );
    }
    writeTwoCodedBytes((uint16_t)(150*_sc.fanPWM) );
    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
        writeTwoCodedBytes((uint16_t)(150*_sc.averageTherm[ii]) );
    }
    for (uint32_t ii = 0; ii < constants::NUM_CHANNELS; ii++) {
        writeTwoCodedBytes((uint16_t)(150*_sc.temperatureSetPoint[ii]) );
    }
    writeTwoCodedBytes((uint16_t)(1500 *_sc.averageLaserCurrent) );
    writeTwoCodedBytes((uint16_t)(1500 *_sc.averageTECCurrent) );

    return true;
}

void Serial_IO::printVersion( ) {
    printToConsole( Receiver::Human, "led-controller app teensy-a - git version %s", GIT_VERSION);
}

void Serial_IO::printHelp( ) {
    printToConsole( Receiver::Human, "");
    printVersion( );
    printToConsole( Receiver::Human, "command line options:");
    printToConsole( Receiver::Human, "  help");
    printToConsole( Receiver::Human, "  version");
    printToConsole( Receiver::Human, "  status_led on|off");
    printToConsole( Receiver::Human, "  crisp_led value (0 to 100)");
    printToConsole( Receiver::Human, "  lock on|off");
    printToConsole( Receiver::Human, "  servo on|off");
    printToConsole( Receiver::Human, "  update_set_point (none-current or 0-1)");
    printToConsole( Receiver::Human, "  set_set_point value (-1 to 1)");
    printToConsole( Receiver::Human, "  servo_level value (-1 to 1)");
    printToConsole( Receiver::Human, "  scan seconds range");
    printToConsole( Receiver::Human, "  led_scan seconds low high (0 to 100)");
    printToConsole( Receiver::Human, "  report_interval value_msec");
    printToConsole( Receiver::Human, "  servo_time_const value_msec");
    printToConsole( Receiver::Human, "  dither hopvalue");

    printToConsole( Receiver::Human, "  laser (0-3) (0-100)");
    printToConsole( Receiver::Human, "  tec (0-3) (0-100)");
    printToConsole( Receiver::Human, "  fan (0-100)");
    printToConsole( Receiver::Human, "  temper (0-3) value(C)");
    printToConsole( Receiver::Human, "  lock (0-3) on/off");

    printToConsole( Receiver::Human, "  set_pid_gain kp ki kd");
    printToConsole( Receiver::Human, "  set_pid_point index set_point");
    printToConsole( Receiver::Human, "  set_pid_diff_time_const index differentiator_time_constant");
    printToConsole( Receiver::Human, "  set_pid_direction F or R");

    printToConsole( Receiver::Human, "  reset_pid index");
    printToConsole( Receiver::Human, "  get_pid_state index");
    printToConsole( Receiver::Human, "  get_temp index");

    printToConsole( Receiver::Human, "  zero_cal");
    printToConsole( Receiver::Human, "  blue on|off");
    printToConsole( Receiver::Human, "  green on|off");
    printToConsole( Receiver::Human, "  yellow on|off");
    printToConsole( Receiver::Human, "  red on|off");
//    printToConsole( Receiver::Human, "  reset_current");
    printToConsole( Receiver::Human, "  shutdown");
    printToConsole( Receiver::Human, "  set_log_type human|machine");    
    printToConsole( Receiver::Human, "");


}

errorType Serial_IO::processSetLogReceiverCommand( LineParsingResult& parsingResult ) {
    if ( parsingResult.getTokenCount() != 2 ) {
        return INVALID_ARGUMENT_COUNT;
    }
    if ( strcmp( parsingResult.getToken(1), "human" ) == 0 ) {
        Serial_IO::setReceiver( Serial_IO::Receiver::Human );
        return NO_ERROR;
    }
    if ( strcmp( parsingResult.getToken(1), "machine" ) == 0 ) {
        Serial_IO::setReceiver( Serial_IO::Receiver::Machine );
        return NO_ERROR;
    }
    return INVALID_ARGUMENT_TYPE;
}

errorType Serial_IO::processInputLine( LineParsingResult& parsingResult, StateControl& _sc  ) {
    
    if ( parsingResult.getTokenCount() < 1) {
        return INVALID_COMMAND;
    }

    if ( strcmp( parsingResult.getToken(0), "help" ) == 0 ) { 
        printHelp( );
        return NO_ERROR;
    }

    if ( strcmp( parsingResult.getToken(0), "version" ) == 0 ) {
        printToConsole( Receiver::Machine, "%s", GIT_VERSION);
        printVersion();
        return NO_ERROR;
    }

    if ( strcmp( parsingResult.getToken(0), "status_led" ) == 0 ) {
        return process::processLedLine( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "reset_state" ) == 0 ) {
      return  _sc.switchState(Reset, 0 );
    }

    if ( strcmp( parsingResult.getToken(0), "reset_clock" ) == 0 ) {
        return _sc.reset_clock();
    }

    if ( strcmp( parsingResult.getToken(0), "sensor" ) == 0 ) {
        return process::processSensorLine( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "debug_interval" ) == 0 ) {
        return process::process_uint16_t( parsingResult, _sc.debuggingPointCountInterval );
    }

    if ( strcmp( parsingResult.getToken(0), "set_log_type" ) == 0 ) {
        return Serial_IO::processSetLogReceiverCommand( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "crisp_led" ) == 0 ) {
        return process::process_float( parsingResult, _sc.crispLEDTargetValue );
    }

    if ( strcmp( parsingResult.getToken(0), "servo_level" ) == 0 ) {
        return process::processServoLevel(parsingResult , _sc);
    }

    if ( strcmp( parsingResult.getToken(0), "set_set_point" ) == 0 ) {
        return process::process_double( parsingResult, _sc.photodiodeDifferenceSetPointTarget );
    }

    if ( strcmp( parsingResult.getToken(0), "scan" ) == 0 ) {
//        return  _sc.switchState(Scan_On, 0 );
        return process::processScan( parsingResult , _sc );
        }

//    if ( strcmp( parsingResult.getToken(0), "dither" ) == 0 ) {
//        return  _sc.switchState(Scan_On, 0 );
//        return process::processDither( parsingResult , _sc );
//        }

//    if ( strcmp( parsingResult.getToken(0), "led_scan" ) == 0 ) {
//        return  _sc.switchState(Scan_On, 0 );
//        return process::processLEDScan( parsingResult , _sc );
//        }

    if ( strcmp( parsingResult.getToken(0), "lock" ) == 0 ) {
//        return  _sc.switchState(Lock_On, 0 );
        return process::processLockNew( parsingResult , _sc );
//        return process::processLock( parsingResult , _sc );
        }

    if ( strcmp( parsingResult.getToken(0), "update_set_point" ) == 0 ) {
        _sc.photodiodeDifferenceSetPointTarget = _sc.averagePDDiff;
        Serial_IO::printToConsole( Serial_IO::Receiver::Human, "%.3f, Updating error offset from actual; old value: %f, new value: %f, error value: %f",
            _sc.globalTime_ms*0.001, _sc.photodiodeDifferenceSetPoint, _sc.photodiodeDifferenceSetPointTarget, _sc.photodiodeDifferenceMeasure);

        return NO_ERROR;
        }

    if ( strcmp( parsingResult.getToken(0), "set_pid_point" ) == 0 ) {
        return process::processPIDSetPointCommand( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "set_pid_gain" ) == 0 ) {
        return process::processPIDGainCommand( parsingResult, _sc );
    }
    if ( strcmp( parsingResult.getToken(0), "set_pid_diff_time_const" ) == 0 ) {
        return process::processPIDSetDifferentiatorTimeConstantCommand( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "set_pid_direction" ) == 0 ) {
        return process::processPIDDirectionCommand( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "reset_pid" ) == 0 ) {
        return process::processResetPID( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "get_pid_state" ) == 0 ) {
        return process::processPIDState( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "zero_cal" ) == 0 ) {
        return  process::processZeroCal( parsingResult , _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "blue" ) == 0 ) {
        return process::processBlueLedLine( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "green" ) == 0 ) {
        return process::processGreenLedLine( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "yellow" ) == 0 ) {
        return process::processYellowLedLine( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "red" ) == 0 ) {
        return process::processRedLedLine( parsingResult );
    }

    if ( strcmp( parsingResult.getToken(0), "laser" ) == 0 ) {
        return process::processLaserPWM( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "tec" ) == 0 ) {
        return process::processTECPWM( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "fan" ) == 0 ) {
        return process::processFanPWM( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "temper" ) == 0 ) {
        return process::processTempSetPoint( parsingResult, _sc );
    }

    if ( strcmp( parsingResult.getToken(0), "shutdown" ) == 0 ) {
        return  process::processShutdown( parsingResult , _sc );
    }

//    if ( strcmp( parsingResult.getToken(0), "reset_current" ) == 0 ) {
//        return  process::processZeroCurrent( parsingResult , _sc );
//    }

    return INVALID_COMMAND;
}

void Serial_IO::handleSerialLine(StateControl& _sc ) {
    while ( Serial.available() ) {
        const char lastCharRead = (char)Serial.read();
        lineDelimiter.addChar( lastCharRead );
        if ( lineDelimiter.isReady() ) {
            char *nextLine = lineDelimiter.getLine();
            if ( strlen( nextLine ) == 0 ) {
                printHelp();
            }
            else {
                const bool ok1 = lineParser.process( nextLine, ' ', parsingResult );
                if ( !ok1 ) {
                    printToConsole( Receiver::Human, "cannot process command: [%s]", nextLine);
                }
                else {
                    printToConsole( Receiver::Human, "will process command: [%s]", nextLine);
                    const errorType _errorType = processInputLine( parsingResult, _sc );
                    switch (_errorType)
                      {
                        case NO_ERROR: {
                          printToConsole( Receiver::Human, "command succeeded");
                        } break;
                        case INVALID_COMMAND: {
                          printToConsole( Receiver::Human, "invalid command");
                          printHelp();
                        } break;
                        case INVALID_ARGUMENT_COUNT: {
                          printToConsole( Receiver::Human, "invalid argument count");
                        } break;
                        case INVALID_ARGUMENT_TYPE: {
                          printToConsole( Receiver::Human, "invalid argument type");
                        } break;
                        case CANNOT_COMPLETE_COMMAND: {
                          printToConsole( Receiver::Human, "cannot complete command");
                        } break;
                        case NOT_READY: {
                          printToConsole( Receiver::Human, "not in ready state");
                        } break;
                      }
                }
                printToConsole( Receiver::Human, "ready");
            }
        }
    }
}
