#ifndef __log_hpp__
#define __log_hpp__

#include <Arduino.h>
#include "usb_serial.h"

namespace logger {
    enum Receiver { Machine, Human, Both } ;
    void setReceiver( Receiver );
    Receiver getReceiver();
    void printToConsole( Receiver r, usb_serial_class& serial, const char * format, ... );
}

#endif
