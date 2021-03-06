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
  float tempTemperature = HMcoef0*pow(V, 0) + HMcoef1*pow(V, 1);
  return tempTemperature + heaterModuleTempOffset;
}

// Calculate the temperature of the boil surface thermistor using interpolation function and voltage read from pin
float calcTempBoilSurfaceThermistor(float V){
  // Note that the analog value is anwhere from 0 to 4095, so that value is converted to 0 to 3.3V in interpolation function automatically
  float tempTemperature = BScoef0*pow(V, 0) + BScoef1*pow(V, 1) + BScoef2*pow(V, 2) + BScoef3*pow(V, 3) + BScoef4*pow(V, 4) + BScoef5*pow(V, 5) + BScoef6*pow(V, 6) + BScoef7*pow(V, 7);
  return tempTemperature; //+ boilSurfaceTempOffset;
}

// Calculate the pressure of the pressure transducer using interpolation function and voltage read from pin
// 30 psia sensor, 10V-15V drive, 100mV output
float calcPressure(float V){
  // Get the analog value and convert to psi
  float tempPressure = Pcoef0*pow(V, 0) + Pcoef1*pow(V, 1);
  return tempPressure + pressureOffset;
}

// Calculate the inlet flow rate using the two pressure measurements from inlet sensors
float calcInletFlowRate(float V, float pressureUpstream, float pressureDownstream, float potValue){
  // Calculate the Cv value given vector and the rotation of the pot
  //percentOpen = V/((float)maxRotation - (float)minRotation);                  // percent the valve is open 0-1
  percentOpen = (map(potValue, minRotation, maxRotation, 0, 100))/100;
  float valveRotation = (cv2.size()-1)*percentOpen;
  uint16_t upperRotation = ceil(valveRotation);    // index of lower Cv value index
  uint16_t lowerRotation = floor(valveRotation);   // index of upper Cv value index
  float upperValveCv = cv2[upperRotation];                  // lower value of Cv
  float lowerValveCv = cv2[lowerRotation];                  // upper value of Cv
  float valveCv = lowerValveCv + ((valveRotation-lowerRotation)*(upperValveCv - lowerValveCv))/(upperRotation - lowerRotation);   // linear interpolation of Cv value
  float galpermin_to_mlpermin = 3785.41;                    // 1 gal/min is 3785.41 mL/min
  //float sg = 1.54;                                          // fluid specific gravity
  float sg = 1.0;                                           // water specific gravity
  inletFlowRate = valveCv * sqrt(abs(inletPressureUpstream - inletPressureDownstream)/sg);            // gal/min
  if (isnan(inletFlowRate) || inletFlowRate < 0.0){inletFlowRate = 0.0;}
  return inletFlowRate * galpermin_to_mlpermin;             // mL/min
}


#endif