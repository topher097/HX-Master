#ifndef VARIABLE_DEFINITIONS_H
#define VARIABLE_DEFINITIONS_H

#include <stdint.h>
#include <string>
#include <defaultPiezo.h>
#include <vector>

using namespace defaultPiezoProperties;
using namespace std;

namespace varDefinitions
{
    // Pressure transducer vars
    float tempPressure;
    float inletPressureUpstream = 1.0;
    float inletPressureDownstream = 1.0;
    float outletPressureVapor = 1.0;
    float outletPressureLiquid = 1.0;
    float pressureDriveVoltage = 10.0;          // Voltage driving the pressure sensors
    float pressureOffset = 0.7;                 // PSI
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
    float heaterModuleTempOffset = 0.0;
    float boilSurfaceTempOffset = -366.25;
    
    // Thermocouple for inlet flow temp vars
    float inletFluidTemperature = 1.0;

    // Valve potentiometer vars
    int16_t valveRotation;
    int16_t minRotation = 0;                // Value when valve is fully closed
    int16_t maxRotation = 4095;             // Value when valve is fully open
    vector<float> cv1 = {0.0, 0.0001, 0.0007, 0.0017, 0.0027, 0.0037, 0.0047, 0.0057, 0.0067, 0.0077, 0.0087, 0.0097, 0.0107, 0.0117, 0.0127, 0.0138, 0.0148, 0.0159, 0.0169, 0.0180, 0.0190};       // Cv for valve 1
    vector<float> cv2 = {0.0, 0.0053, 0.0120, 0.0184, 0.0245, 0.0303, 0.0358, 0.0410, 0.0458, 0.0504, 0.0546, 0.0585, 0.0623, 0.0656, 0.0688, 0.0715, 0.0742, 0.0764, 0.0786, 0.0802, 0.0818};       // Cv for valve 2


    // Logging vars
    uint16_t dataDelay = 50;                // time between data read and transfer in milliseconds
    uint64_t dataStartTime;                 // start time after data read and sent
    uint64_t testTime;                 // time since program started in milliseconds
    uint64_t testTimeStart;            // time of when program starts
    double testTimePrint;

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