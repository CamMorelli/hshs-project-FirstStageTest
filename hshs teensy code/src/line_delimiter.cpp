#include <string.h>
#include <strings.h>
#include "line_delimiter.hpp"
#include "logger.hpp"


LineDelimiter::LineDelimiter(int c): capacity(c) {
    resultBuffer = new char[capacity];
    memset( resultBuffer, 0, capacity );
    accumBuffer = new char[capacity];
    memset( accumBuffer, 0, capacity );
    next = 0;
    ready = false;
}

void LineDelimiter::addChar(char in) {

    // ignore CRs, so it works the same in both linux & windows
    if ( in == '\x0d' ) {
        return;
    }

    // prevent a buffer overflow
    if ( next > capacity-2 ) {
        next = 0;
        ready = false;
        memset( accumBuffer, 0, capacity );
        return;
    }

    // LF finishes a line, return to the caller that the line is ready
    if ( in == '\x0a' ) {
        accumBuffer[next] = 0;
        strcpy( resultBuffer, accumBuffer );
        memset( accumBuffer, 0, capacity );
        ready = true;
        next = 0;
        return;
    }

    // otherwise, accumulate the last received char, and return that the line is not ready yet
    accumBuffer[next] = in;
    ready = false;
    next++;
}

bool LineDelimiter::isReady() {
    return ready;
}

char* LineDelimiter::getLine() {
    return resultBuffer;
}

void LineDelimiter::printState(usb_serial_class& serial) {
   Serial_IO::printToConsole( Serial_IO::Receiver::Human, "LineDelimiter.printState -- BEGIN");
   Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  capacity:%d", capacity);
   Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  next:%d", next);
   Serial_IO::printToConsole( Serial_IO::Receiver::Human, "  ready:%d", ready);
   Serial_IO::printToConsole( Serial_IO::Receiver::Human, "LineDelimiter.printState -- END");
}


