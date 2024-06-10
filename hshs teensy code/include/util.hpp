#ifndef __util_hpp__
#define __util_hpp__

#include <Arduino.h>
#include "usb_serial.h"

namespace util {
    void trim( char *str );
    void deleteRepeatedChars(char *str, char r);
    void deleteChars(char *str, char r);
    int clamp( int in, int min, int max );
    int max3( int a, int b, int c );
}

#endif
