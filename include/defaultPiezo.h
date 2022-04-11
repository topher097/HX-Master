#ifndef DEFAULT_PIEZO_PROPERTIES_H
#define DEFAULT_PIEZO_PROPERTIES_H

namespace defaultPiezoProperties
{
    float default_frequency1 = 1000.0;  // Frequency of left channel piezo in Hz
    float default_frequency2 = 1000.0;  // Frequency of right channel piezo in Hz
    float default_amplitude1 = 0.8;     // Amplitude of sine wave 1 (left cahnnel); 0-1
    float default_amplitude2 = 0.8;     // Amplitude of sine wave 2 (right channel); 0-1
    float default_phase1 = 0.0;         // Phase of left channel signal in degrees
    float default_phase2 = 0.0;         // Phase of right channel signal in degrees
    int default_enable1 = HIGH;          // Enable pin for piezo driver 1
    int default_enable2 = HIGH;          // Enable pin for piezo driver 2
    int v_mult = 95;                   // Multiplier of amplitude to get V p-p
}

#endif