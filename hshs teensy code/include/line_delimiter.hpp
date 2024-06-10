#ifndef __line_delimiter_hpp__
#define __line_delimiter_hpp__

#include <Arduino.h>
//#include "usb_serial.h"
#include "serial_comm.hpp"

class LineDelimiter {
  private:
    const unsigned int capacity;
    char* accumBuffer;
    char* resultBuffer;
    unsigned int next;
    bool ready;
  public:
    LineDelimiter(int capacity);
    void addChar(char);
    bool isReady();
    char* getLine();
    void printState(usb_serial_class& serial);
};

#endif
