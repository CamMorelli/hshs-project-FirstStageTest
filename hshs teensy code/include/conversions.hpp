#ifndef __conversions_hpp__
#define __conversions_hpp__

#include <Arduino.h>

namespace conversions {

    struct AtoiResult{
        long value;
        bool valid;
    };

    struct AtofResult{
        float value;
        bool valid;
    };

    void atoi( const char* a, AtoiResult& );
    void atof( const char* a, AtofResult& );
}

#endif
