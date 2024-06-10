#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <cstdlib>
#include "util.hpp"

namespace util {

    void trim(char *str)
    {
        size_t len = 0;
        char *frontp = str;
        char *endp = NULL;

        if( str == NULL ) { 
            return; 
        }
        
        if( str[0] == '\0' ) { 
            return; 
        }

        len = strlen(str);
        endp = str + len;

        /* Move the front and back pointers to address the first non-whitespace
        * characters from each end.
        */
        while( isspace((unsigned char) *frontp) ) { ++frontp; }
        if( endp != frontp )
        {
            while( isspace((unsigned char) *(--endp)) && endp != frontp ) {}
        }

        if( frontp != str && endp == frontp ) {
            *str = '\0';
        }
        else if( str + len - 1 != endp ) {
            *(endp + 1) = '\0';
        }

        /* Shift the string so that it starts at str so that if it's dynamically
        * allocated, we can still free it on the returned pointer.  Note the reuse
        * of endp to mean the front of the string buffer now.
        */
        endp = str;
        if( frontp != str )
        {
            while( *frontp ) { 
                *endp++ = *frontp++; 
            }
            *endp = '\0';
        }

        return;
    }

    void deleteRepeatedChars(char *str, char r) {
        
        int outIndex = 0;
        int count = 0;

        for ( size_t i=0; i<strlen(str); i++ ) {
            if ( str[i] == r ) {
                count++;
                if ( count == 1 ) {
                    str[outIndex++] = str[i];
                }
            }
            else {
                count = 0;
                str[outIndex++] = str[i];
            }
        } 

        str[outIndex] = '\0';
    }

    void deleteChars(char *str, char r) {
        int outIndex = 0;
        for ( size_t i=0; i<strlen(str); i++ ) {
            if (str[i] != r) {
                str[outIndex++] = str[i]; // here count is
            }
        }
        str[outIndex] = '\0';
    }

    int clamp( int in, int min, int max ) {

        if ( in < min ) {
            return min;
        }

        if ( in > max ) {
            return max;
        }

        return in;
    }

    int max3( int a, int b, int c ) {
        int m = a;
        if ( b > m ) {
            m = b;
        }
        if ( c > m ) {
            m = c;
        }
        return m;
    }
}