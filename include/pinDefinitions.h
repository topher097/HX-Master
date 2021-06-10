#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

namespace pinDefinitions
{
    const int BLINK = 32;      // Status LED
    const int P1 = A14;         // Inlet upstream pressure transducer
    const int P2 = A15;         // Inlet downstream pressure transducer
    const int P3 = A16;         // Outlet vapor pressure transducer
    const int P4 = A17;         // Outlet liquid pressure transducer
    const int HMT1 = A0;       // Heater module 1 thermistor
    const int HMT2 = A1;       // Heater module 2 thermistor
    const int HMT3 = A2;       // Heater module 3 thermistor
    const int HMT4 = A3;       // Heater module 4 thermistor
    const int HMT5 = A6;       // Heater module 5 thermistor
    const int BST1 = A13;      // Boil surface thermistor 1
    const int BST2 = A12;      // Boil surface thermistor 2
    const int BST3 = A11;      // Boil surface thermistor 3
    const int BST4 = A10;      // Boil surface thermistor 4
    const int RX1  = 0;        // UART RX1
    const int TX1  = 1;        // UART TX1
    const int HMD1 = 2;        // MOSFET for heater module 1
    const int HMD2 = 3;        // MOSFET for heater module 2
    const int HMD3 = 4;        // MOSFET for heater module 3
    const int HMD4 = 5;        // MOSFET for heater module 4
    const int HMD5 = 6;        // MOSFET for heater module 5
    const int RHD = 7;         // MOSFET for rope heater
    const int IFT_CS = 10;     // CS pin for thermocouple MAX31855 module
    const int IFT_MISO = 12;   // MISO pin for thermocouple MAX31855 module
    const int IFT_SCK = 13;    // SCK pin for thermocouple MAX31855 
}

#endif




