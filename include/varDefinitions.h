#ifndef VARIABLE_DEFINITIONS_H
#define VARIABLE_DEFINITIONS_H

#include <stdint.h>
#include <string>
#include <defaultPiezo.h>

using namespace defaultPiezoProperties;

namespace varDefinitions
{
    // Pressure transducer vars
    float tempPressure;
    float inletPressureUpstream = 1.0;
    float inletPressureDownstream = 1.0;
    float outletPressureVapor = 1.0;
    float outletPressureLiquid = 1.0;
    float pressureDriveVoltage = 10.0;          // Voltage driving the pressure sensors
    float inletFlowRate;

    // Thermistor temperature vars
    float tempTemperature;
    float heaterTemperature1 = 1.0;
    float heaterTemperature2 = 1.0;
    float heaterTemperature3 = 1.0;
    float heaterTemperature4 = 1.0;
    float heaterTemperature5 = 1.0;
    float boilSurfaceTemperature1 = 1.0;
    float boilSurfaceTemperature2 = 1.0;
    float boilSurfaceTemperature3 = 1.0;
    float boilSurfaceTemperature4 = 1.0;
    float averageBoilSurfaceTemp = 1.0;
    
    // Thermocouple for inlet flow temp vars
    float inletFluidTemperature = 1.0;

    // Logging vars
    unsigned int dataDelay = 500;                // time between data read and transfer in milliseconds
    unsigned int dataStartTime;                 // start time after data read and sent
    unsigned long int testTime;                 // time since program started in milliseconds
    unsigned long int testTimeStart;            // time of when program starts

    // Piezo control vars
    bool inPhase = true;        // Piezos are in phase with each other
    String phaseText = "";      // Text for LCD screen

    // Heater module cartridge controller vars
    int heatEnergyDensity = 50;         // Desired effective wattage per cm^2 to be pumped through heater modules

    // Protection vars
    bool safeHeater = true;
    bool safeBoilSurface = true;
    bool safeInletTemp = true;
    float heaterMaxTemp = 120.0;        // degree celcius
    float boilSurfaceMaxTemp = 90.0;    // degree celcius
    float inletMaxTemp = 65;            // degree celcius

    // Serial communcation between teensys vars
    uint16_t rx1_byte;                  // Teensy slave rx
    uint16_t tx1_byte;                  // Teensy slave tx

    // Blink vars
    int ledState = LOW;
    const int blinkDelay = 500;         // delay in ms
}

#endif