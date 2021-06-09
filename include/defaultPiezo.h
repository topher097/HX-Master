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
    int default_enable1 = LOW;          // Enable pin for piezo driver 1
    int default_enable2 = LOW;          // Enable pin for piezo driver 2

    float frequency1 = default_frequency1;    // Frequency of left channel piezo in Hz
    float frequency2 = default_frequency2;    // Frequency of right channel piezo in Hz
    float amplitude1 = default_amplitude1;    // Amplitude of sine wave 1 (left cahnnel); 0-1
    float amplitude2 = default_amplitude2;    // Amplitude of sine wave 2 (right channel); 0-1
    float phase1 = default_phase1;            // Phase of left channel signal in degrees
    float phase2 = default_phase2;            // Phase of right channel signal in degrees
    int enable1 = default_enable1;            // Enable pin for piezo driver 1
    int enable2 = default_enable2;            // Enable pin for piezo driver 2
}

#endif