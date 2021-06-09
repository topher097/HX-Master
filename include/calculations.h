#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <coeffDefinitions.h>
#include <pinDefinitions.h>
#include <varDefinitions.h>


using namespace varDefinitions;
using namespace pinDefinitions;
using namespace coeffDefinitions;
using namespace std;


// Calculate the temperature of the heater module thermistor using interpolation function and voltage read from pin
float calcTempHeaterModuleThermistor(float V){
  // Note that the analog value is anwhere from 0 to 4095, so that value is converted to 0 to 3.3V in interpolation function automatically
  float tempTemperature = HMcoef0*pow(V, 0) + HMcoef1*pow(V, 1) + HMcoef2*pow(V, 2) + HMcoef3*pow(V, 3) + HMcoef4*pow(V, 4) + HMcoef5*pow(V, 5) + HMcoef6*pow(V, 6) + HMcoef7*pow(V, 7) + HMcoef8*pow(V, 8) + HMcoef9*pow(V, 9) + HMcoef10*pow(V, 10) + HMcoef11*pow(V, 11);
  return tempTemperature;
}

// Calculate the temperature of the boil surface thermistor using interpolation function and voltage read from pin
float calcTempBoilSurfaceThermistor(float V){
  // Note that the analog value is anwhere from 0 to 4095, so that value is converted to 0 to 3.3V in interpolation function automatically
  float tempTemperature = BScoef0*pow(V, 0) + BScoef1*pow(V, 1) + BScoef2*pow(V, 2) + BScoef3*pow(V, 3) + BScoef4*pow(V, 4) + BScoef5*pow(V, 5) + BScoef6*pow(V, 6) + BScoef7*pow(V, 7);
  return tempTemperature;
}

// Calculate the pressure of the pressure transducer using interpolation function and voltage read from pin
// 30 psia sensor, 10V-15V drive, 100mV output
float calcPressure(float V){
  // Get the analog value and convert to psi
  float tempPressure = Pcoef0*pow(V, 0) + Pcoef1*pow(V, 1);
  return tempPressure;
}

// Calculate the inlet flow rate using the two pressure measurements from inlet sensors
float calcInletFlowRate(){
  float valveOpenCv = 0.0818;                 // gal/min fully open on valve 2
  float galpermin_to_mlpermin = 3785.41;      // 1 gal/min is 3785.41 mL/min
  float sg = 1.54;                            // fluid specific gravity
  inletFlowRate = valveOpenCv * sqrt(abs(inletPressureUpstream - inletPressureDownstream)/sg);
  return inletFlowRate;
}


#endif