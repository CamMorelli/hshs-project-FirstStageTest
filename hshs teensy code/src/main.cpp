/*
Code acquires photodiode signals from CRISP autofocus and uses PID to calculate servo values for autofocus control
Written for Teensy 4.0

Using bit 7 high (MSB) to indicate start of dataset; FE: PID data (binary encoding); FD: text
Data sent in lower 7 bits (14 or 28 bits total), with highest bits first, e.g., temperature*100 in 14 bits as lower 7 bits over two subsequent bytes
*/

#include <Arduino.h>
//#include "usb_serial.h"
#include <cstdlib>
#include <util.hpp>
#include <conversions.hpp>
#include "constants.hpp"
#include "state_control.hpp"
#include "serial_comm.hpp"

extern float tempmonGetTemp(void);    //  For temperature of Teensy?

static StateControl _sc;    //  Instantiate class and reset clocks
uint32_t tic, toc, startLoopTime;

void setup() {
    _sc.initArrays();
    Serial_IO::startSerial();

    _sc.mcuSetup();   //  Configure required MCU outputs

    startLoopTime = micros();

}       //  End setup

void loop() {       //  Flow: handle serial inputs, update clocks, update state and control ouputs, get temperatures, write data to serial output (latest PID related values)
                    //  Loop rate is limited by the analogRead rate in updateOutputs

    if ( _sc.globalCounter % constants::SERIAL_TICK_INTERVAL == 0 ) {
        Serial_IO::handleSerialLine(_sc);   //  Handle any commands from serial line
        tic = micros();
        _sc.handleSerialTime = tic - startLoopTime;
    } else {
        tic = micros();
    }

    _sc.updateOutputs();      //  Read photodiode voltages, update states, update servo and LED levels
    toc = micros();
    _sc.updateOutputsTime = toc - tic;

    tic = micros();
    _sc.writeData(_sc );          //  Write new pid related parameters, also software and hardware debugging outputs
    startLoopTime = micros();
    _sc.writeDataTime = startLoopTime - tic;

}   //  End loop
