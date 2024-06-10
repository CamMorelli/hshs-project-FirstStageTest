
#ifndef __serial_comm__
#define __serial_comm__

#include <Arduino.h>
#include "constants.hpp"
//#include "usb_serial.h"
#include <line_parser.hpp>  //  These are part of common libraries

class StateControl;

class  Serial_IO {
    public:

    enum Receiver { Machine, Human, Both } ;
    static void setReceiver( Receiver );
    Receiver getReceiver();
    static bool startSerial();
    static void printToConsole( Receiver r, const char * format, ... );
    static boolean writeTwoCodedBytes (uint16_t intVal );
    static boolean writeFourCodedBytes (uint32_t intVal );
    static boolean writeTransitionData (uint8_t transitionType, uint32_t firstPoint, uint16_t numPoints, float slopeValues, float interceptValues, 
        float debugData, float offset );
    static bool writeBufferCode(uint32_t code );
    static bool writeHSHSData(StateControl& _sc);
    static bool writeHSHSDataNew(StateControl& _sc);
    static void printVersion();
    static void printHelp();
    static errorType processSetLogReceiverCommand( LineParsingResult& parsingResult );
    static errorType processInputLine( LineParsingResult& parsingResult, StateControl& _sc  );
    static void handleSerialLine(StateControl& _sc );

};

#endif