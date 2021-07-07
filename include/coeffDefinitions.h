#ifndef COEFFICIENT_DEFINITIONS_H
#define COEFFICIENT_DEFINITIONS_H

namespace coeffDefinitions
{
    // Boil surface thermistor interpolation curve coefficients
    const float BScoef0 = -45.2;
    const float BScoef1 = 276;
    const float BScoef2 = -601;
    const float BScoef3 = 797;
    const float BScoef4 = -594;
    const float BScoef5 = 248;
    const float BScoef6 = -54;
    const float BScoef7 = 4.8;

    // Heater module thermistor interpolation curve coefficients
    const float HMcoef0 = -9.74;
    const float HMcoef1 = 129;

    // Pressure sensor interpolation curve coefficients
    const float Pcoef0 = 0;
    const float Pcoef1 = 9.3;
}

#endif
