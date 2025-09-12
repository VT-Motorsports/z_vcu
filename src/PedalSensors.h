// If PEDALSENSORS_H is not defined, define it (start of header guard)
#ifndef PEDALSENSORS_H     
// Define PEDALSENSORS_H to prevent multiple inclusions
#define PEDALSENSORS_H  

#include <stdio.h>

struct PedalSensor{
    const int inputPin;
    const int zeroPos;
    const int fullPos;
    const int onLedPin;
    const int ocLedPin;
    const int scLedPin;

    float pedalPercentage();
    bool checkOpenCircuit();
    bool checkShortCircuit();
    bool checkRangeFaults();
};

bool pedalAgreementFault(PedalSensor p1, PedalSensor p2);
bool brakeFault(PedalSensor p1, PedalSensor p2);
bool checkAllFaults(PedalSensor p1, PedalSensor p2)

#endif