#include <string.h>
#include <strings.h>
#include "line_parser.hpp"
#include "util.hpp"

LineParsingResult::LineParsingResult(int a, int b): maxTokens(a), maxLengthPerToken(b) {
    result = new char*[maxTokens];
    for ( unsigned int i=0; i<maxTokens; i++ ) {
        result[i] = new char[maxLengthPerToken];
    }
    next = 0;
}

void LineParsingResult::reset() {
    next = 0;
}

int LineParsingResult::getTokenCount() {
    return next;
}

char* LineParsingResult::getToken(int i) {
    return result[i];
}

LineParser::LineParser(int c): capacity(c) {
    buffer = new char[capacity];
}

bool LineParsingResult::addToken(char * in) {
    if ( next > (maxTokens-1) ) {
        return false;
    }
    if ( strlen( in ) > (maxLengthPerToken-1) ) {
        return false;
    }

    strcpy( result[next], in );
    next++;
    return true;
}

bool LineParser::process(char* in, char separator, LineParsingResult& pr) {

    pr.reset();
    memset( buffer, 0, capacity );
    next = 0;

    util::trim( in );
    util::deleteRepeatedChars(in, ' ');

    for (unsigned int i=0; i<strlen(in); i++) {
        if ( next > capacity-2 ) {
            return false;
        }
        else {
            buffer[next] = in[i];
            next++;
        }
    }

    unsigned int count = 0;
    char sepString[] = {separator, '\0'};
    char* token = strtok(buffer, sepString);
    while( token != NULL ) {
        util::trim( token );
        bool ok = pr.addToken(token);
        if ( !ok ) {
            return false;
        }
        token = strtok(NULL, sepString);
        count++;
    }

    return true;
}
