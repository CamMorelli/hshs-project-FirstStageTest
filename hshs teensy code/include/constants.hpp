#ifndef __constants_hpp__
#define __constants_hpp__
#include <Arduino.h>

    enum errorType { NO_ERROR = 0, INVALID_COMMAND = -1, INVALID_ARGUMENT_COUNT = -2, INVALID_ARGUMENT_TYPE = -3, CANNOT_COMPLETE_COMMAND = -4, NOT_READY = -5};

namespace constants {

    static const uint16_t NUM_CHANNELS = 4;    // This is for sizing arrays
    static const float TEMPERATURE_CUTOUT = 50; //  Thermistor temperature that turns off laser and TEC
    static const float REFERENCE_RESISTANCE = 0.1;  //  Resistance for summed laser and summed tec monitor
    static const float HALL_SENSOR_VOLTS_PER_AMP = 0.264;    //  Response of ACS725LLCTR-10AU-S is 264 mV/A

    static const float DATA_UPDATE_INTERVAL_SEC = 0.01;
    static const uint16_t SAMPLES_TO_READ = DATA_UPDATE_INTERVAL_SEC / 0.0000405;    //  Approximately 40.5 microseconds for analog read of two photodiode channels
    static const float REPORT_INTERVAL_SEC = 0.2;
    static const uint16_t CYCLES_PER_DATA_POINT = REPORT_INTERVAL_SEC/DATA_UPDATE_INTERVAL_SEC;
    static const uint32_t CYCLES_FOR_ZERO_INIT = 3; //  Number of cycles with autofocus LED off before zero calibration
    static const uint32_t CYCLES_FOR_ZERO_CAL = 50; //  Number of cycles with autofocus LED off for zero calibration
    static const uint8_t PIN_IN_PD_1 = 14;      //  Pins to read photodiode voltages
    static const uint8_t PIN_IN_PD_2 = 15;
    static const uint8_t PIN_OUT_LED_LEVEL = 22;
    static const uint8_t PIN_OUT_SERVO_LEVEL = 23;
    static const float SERIAL_UPDATE_INTERVAL_SEC = 0.2;
    static const uint16_t SERIAL_TICK_INTERVAL = SERIAL_UPDATE_INTERVAL_SEC/DATA_UPDATE_INTERVAL_SEC;    //  
    static const double SERVO_MAX_VALUE = 1000;     //  Arbitrary maximum servo parameter to facilitate small changes in GUI (250 µm range means servo change of 0.1 is 0.025 µm)

    static const uint8_t RED_LED_PIN = 9;    //  3rd/Status LED
    static const uint8_t YELLOW_LED_PIN = 10;    //  3rd/Status LED
    static const uint8_t GREEN_LED_PIN = 11;    //  3rd/Status LED
    static const uint8_t BLUE_LED_PIN = 12;    //  3rd/Status LED
    static const uint8_t PIN_STATUS_LED = 13;    //  3rd/Status LED

    static const uint8_t CALC_START = 23; //Pin that goes high/low when calculations are complete to visualize timing on oscilloscope

//  Output pins
    static const uint8_t LASER_OUT_PIN_LIST[NUM_CHANNELS] = { 0, 1, 2, 3 };  // Laser power control pins
    static const uint8_t TEC_OUT_PIN_LIST[NUM_CHANNELS] = { 4, 5, 6, 7 };  // TEC power control pins
    static const uint8_t FAN_OUT_PIN = 8;  // fan control pin
    static const float LASER_OUT_FREQUENCY =  1000.0;      //  Slow frequency due to low pass filter on MOSFET gate
    static const float TEC_OUT_FREQUENCY =  1000.0;      //  Slow frequency due to low pass filter on MOSFET gate
    static const float FAN_OUT_FREQUENCY =  1000.0;      //  Slow frequency due to low pass filter on MOSFET gate


//  Input pins
    static const uint8_t CLOCK_DIN_PIN = 14;  // Clock/sync digital input pin
    static const uint8_t CAMERA_A_DIN_PIN = 10;  // Camera A digital input pin
    static const uint8_t CAMERA_B_DIN_PIN = 11;  // Camera B digital input pin
    static const uint8_t CAMERA_C_DIN_PIN = 12;  // Camera C digital input pin
    static const uint8_t THERMISTER_AIN_PIN_LIST[NUM_CHANNELS] = { 15, 16, 17, 18 };  // Thermistor analog input pins
    static const uint8_t LASER_CURRENT_AIN_PIN = 19;  // Summed laser current analog input pin
    static const uint8_t TEC_CURRENT_AIN_PIN = 20;  // Summed TEC current analog input pin
    static const uint8_t TTL_DIN_PIN = 9;  //  TTL digital input pin

//  Spare pin, wired to DA-15 connector currently, can be output; digital and analog input
    static const uint8_t SPARE_PIN = 21;  // Spare pin

    //  Pins 13 (built in LED), 22, and 23 are currently not connected


    static const uint16_t ANALOG_OUT_RESOLUTION_BITS = 13;       //  N.B. Actual analog resolution is mapped to the available resolution depending on the PWM frequency
    static const uint16_t ANALOG_OUT_MAX_COUNT = (1 << ANALOG_OUT_RESOLUTION_BITS) - 1;
    static const uint16_t ANALOG_IN_RESOLUTION_BITS = 12;       //  N.B. Actual analog resolution is mapped to the available resolution depending on the PWM frequency
    static const uint16_t ANALOG_IN_MAX_COUNT = (1 << ANALOG_IN_RESOLUTION_BITS) - 1;
    static const float ANALOG_IN_REFERENCE_VOLTAGE = 3.3;
    static const uint16_t CRISP_LED_ANALOG_OUT_RESOLUTION_BITS = 8;      //  Maximize frequency for CRISP led at cost of resolution, Teensy struggles to pull down LED control pin on CRISP with higher series resistor
    static const uint16_t CRISP_LED_ANALOG_OUT_MAX_COUNT = (1 << CRISP_LED_ANALOG_OUT_RESOLUTION_BITS) - 1;      //  Maximize frequency for CRISP led at cost of resolution, Teensy struggles to pull down LED control pin on CRISP with higher series resistor
    static const float CRISP_LED_PWM_FREQUENCY =  585937.5;      //  Maximize frequency for CRISP led at cost of resolution, Teensy struggles to pull down LED control pin on CRISP with higher series resistor
    static const uint16_t SERVO_ANALOG_OUT_RESOLUTION_BITS = 13;      //  Maximize resolution for servo at cost of frequency, need resolution for accurate autofocus
    static const uint16_t SERVO_ANALOG_OUT_MAX_COUNT = (1 << SERVO_ANALOG_OUT_RESOLUTION_BITS) - 1;      //  Maximize resolution for servo at cost of frequency, need resolution for accurate autofocus
    static const float SERVO_PWM_FREQUENCY =  18310.55;      //  Maximize resolution for servo at cost of frequency, need resolution for accurate autofocus
    //  Set analog resolution to highest value, values written will be reduced when written based on the PWM frequency for that channel
    //  For reference, see https://www.pjrc.com/teensy/td_pulse.html
    //  PJRC "Pulsed Output: PWM & Tone"
};

#endif
