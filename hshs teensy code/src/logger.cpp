#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <cstdlib>
#include <TeensyID.h>
#include "logger.hpp"

namespace logger {

    static Receiver currentReceiver = Receiver::Human;

    void setReceiver( Receiver r ) {
        currentReceiver = r;
    }

    Receiver getReceiver() {
        return currentReceiver;
    }
    
    void printToConsole( Receiver intendedReceiver, usb_serial_class& serial, const char * format, ... ) {

        static uint64_t line_count = 0;
        static char buffer[256];

        va_list args;
        va_start( args, format );
        memset(buffer, 0, sizeof(buffer));
        vsnprintf( buffer, sizeof(buffer)-1, format, args );
        va_end( args );

        if ( currentReceiver == Receiver::Human  && 
             (intendedReceiver == Receiver::Human || intendedReceiver == Receiver::Both) ) {
            serial.printf( "[%s]", teensySN() );
            serial.printf( " %06d ", line_count++ );
            serial.printf( "%s", buffer );
            serial.printf( "\r\n" );
        }

        if ( currentReceiver == Receiver::Machine  && 
            (intendedReceiver == Receiver::Machine || intendedReceiver == Receiver::Both) ) {
            serial.printf( "%s", buffer );
            serial.printf( "\r\n" );
        }
    }
}

