#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <cstdlib>
#include "conversions.hpp"

namespace conversions {

    void atoi( const char* in, AtoiResult& r ) {

        errno = 0; 
        char *endptr;
        const long val = strtol(in, &endptr, 10);

        if ( (errno == ERANGE && (val == LONG_MAX || val == LONG_MIN))
                || 
             (errno != 0 && val == 0) 
                || 
             (endptr == in) ) {
            r.valid = false;
            return;
        }

        r.value = val;
        r.valid = true;
    }

    void atof( const char* in, AtofResult& r ) {

        errno = 0; 
        char *endptr;
        const float val = strtof(in, &endptr);

        if ( (errno == ERANGE )
                || 
             (errno != 0 && val == 0) 
                || 
             (endptr == in) ) {
            r.valid = false;
            return;
        }

        r.value = val;
        r.valid = true;
    }

}