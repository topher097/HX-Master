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
    float pressureOffset = 0;                 // PSI
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
    //float boilSurfaceTempOffset = -366.25;
    float boilSurfaceTempOffset = 0;
    
    // Thermocouple for inlet flow temp vars
    float inletFluidTemperature     = 1.0;
    float inletFluidTemperatureOld  = 1.0;
    float targetFluidTemperature    = 55.0;
    bool enableRopeHeater = false;

    // Valve potentiometer vars
    int16_t valveRotation;
    int16_t potRead;
    float percentOpen;
    int16_t minRotation = 9;                // Value when valve is fully closed
    int16_t maxRotation = 3655;             // Value when valve is fully open
    vector<float> cv1 = {0.0, 0.0001, 0.0007, 0.0017, 0.0027, 0.0037, 0.0047, 0.0057, 0.0067, 0.0077, 0.0087, 0.0097, 0.0107, 0.0117, 0.0127, 0.0138, 0.0148, 0.0159, 0.0169, 0.0180, 0.0190};       // Cv for valve 1
    vector<float> cv2 = {0.0, 0.0053, 0.0120, 0.0184, 0.0245, 0.0303, 0.0358, 0.0410, 0.0458, 0.0504, 0.0546, 0.0585, 0.0623, 0.0656, 0.0688, 0.0715, 0.0742, 0.0764, 0.0786, 0.0802, 0.0818};       // Cv for valve 2
    vector<float> cv3 = {0.0, 0.0176, 0.0353, 0.0509, 0.0644, 0.0762, 0.0863, 0.0948, 0.1021, 0.1081, 0.1130, 0.1167, 0.1204, 0.1229, 0.1254, 0.1273, 0.1292, 0.1312, 0.1331, 0.1357, 0.1383};       // Cv for valve 3
    vector<float> cv4 = {0.0, 0.0236, 0.0443, 0.0635, 0.0814, 0.0980, 0.1133, 0.1273, 0.1402, 0.1520, 0.1628, 0.1720, 0.1812, 0.1886, 0.1960, 0.2018, 0.2075, 0.2119, 0.2162, 0.2201, 0.2240};       // Cv for valve 4

    // Logging vars
    uint16_t dataDelay = 250;               // time between data read and transfer in milliseconds
    uint64_t dataStartTime;                 // start time after data read and sent
    uint64_t testTime;                      // time since program started in milliseconds
    uint64_t testTimeStart;                 // time of when program starts
    double testTimePrint;                   // Print the time to serial for MATLAB
    bool endTesting = 1;                // Bool to end the test
    bool tempEnd = endTesting;

    // Heater module cartridge controller vars
    int heatEnergyDensity       = 10;           // Desired effective wattage per cm^2 to be pumped through heater modules
    int heatEnergyDensityMax    = 100;          // Max achievable energy density with current cartridges 
    bool enableHeaters          = false;         // Bool for enabling the heater cartridges
    int heatEnergyDensityOld    = heatEnergyDensity;

    // Protection vars
    bool safeHeater = true;
    bool safeBoilSurface = true;
    bool safeInletTemp = true;
    //float heaterMaxTemp = 300.0;        // degree celcius
    //float boilSurfaceMaxTemp = 80.0;    // degree celcius
    //float inletMaxTemp = 65;            // degree celcius

    float heaterMaxTemp = 300.0;        // degree celcius
    float boilSurfaceMaxTemp = 150.0;    // degree celcius
    float inletMaxTemp = 95;            // degree celcius

    // Blink vars
    int ledState = LOW;
    const int blinkDelay = 500;         // delay in ms
}

#endif