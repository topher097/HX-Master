#ifndef COEFFICIENT_DEFINITIONS_H
#define COEFFICIENT_DEFINITIONS_H

namespace coeffDefinitions
{
    // Boil surface thermistor interpolation curve coefficients
    const float BScoef0 = 374.55779068854577;
    const float BScoef1 = 17.40816141969277;
    const float BScoef2 = 0.32103832076171;
    const float BScoef3 = 0.0016992484227292349;
    const float BScoef4 = -4.3962486759910275e-05;
    const float BScoef5 = -3.7897772892121576e-07;
    const float BScoef6 = 6.9241378036387565e-09;
    const float BScoef7 = -2.3683363162345936e-11;

    // Heater module thermistor interpolation curve coefficients
    const float HMcoef0 = 34.796569085903165;
    const float HMcoef1 = 2.1377274062093172;
    const float HMcoef2 = 0.037710766520922305;
    const float HMcoef3 = 0.00019364441743986369;
    const float HMcoef4 = 1.4000629550389751e-05;
    const float HMcoef5 = 6.521125926259332e-08;
    const float HMcoef6 = -3.0040976596479196e-09;
    const float HMcoef7 = 1.993233371349954e-11;
    const float HMcoef8 = -3.860489367096544e-14;
    const float HMcoef9 = -7.748547176657863e-17;
    const float HMcoef10 = 4.0350228079546027e-19;
    const float HMcoef11 = -4.352448435319378e-22;

    // Pressure sensor interpolation curve coefficients
    const float Pcoef0 = -7.125683319833918e-06;
    const float Pcoef1 = 129480.71625391858;
}

#endif
