/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "CanInitializer.h"
#include <stdio.h>
#include <sys/_intsup.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

#define DEBUGGING 0

// Placeholder values while we do not have real numbers (most likely same as 24 car)
const int ocThreshold = 1; // Open Circuit threshold
const int scThreshold = 2;  // Short Circuit threshold

// Pin Configuration NOT REAL VALUES
const int brake_switch_pin = 14;
const int brake_out_pin = 2;
const int error_out_pin = 9;
const int rtds_horn_pin = 21;
const int custom_pedal_pin = A14;


// Class for the pedals contains how to read pedal positions and check Open Circuit and Short Circuit faults
struct pedalSensor {
    const int inputPin;
    const int zeroPos;
    const int fullPos;
    const int onLedPin;
    const int ocLedPin;
    const int scLedPin;
}


float pedalSensorPercentage(pedalSensor pedal) const {
    float percent = ((float) pedal.analogRead(pedal.inputPin) - pedal.zeroPos) / (pedal.fullPos - pedal.zeroPos);
            if (percent < 0.0f) {
                percent = 0.0f;
            } else if (percent > 1.0f) {
                percent = 1.0f;
            }
            return percent;
}

// Checks for a Short Circuit by reading the value of the sensor and comparing it to the threshold defined above
bool checkOpenCircuit(pedalSensor pedal) const {
    bool isOpenCircuit = pedal.analogRead(pedal.inputPin) < pedal.ocThreshold;
    digitalWrite(pedal.ocLedPin, isOpenCircuit);
    #ifdef DEBUGGING
    printf("Pedal %d: Open Circuit %s\n", pedal.inputPin, isOpenCircuit ? "detected" : "not detected");
    #endif
    return isOpenCircuit;
}

// Checks for a Short Circuit by reading the value of the sensor and comparing it to the threshold defined above
bool checkShortCircuit(PedalSensor pedal) const {
    bool isShortCircuit = analogRead(pdeal.inputPin) > pedal.scThreshold;
    digitalWrite(pedal.scLedPin, isShortCircuit);
    #ifdef DEBUGGING
    printf("Pedal %d: Short Circuit %s\n", pedal.inputPin, isShortCircuit ? "detected" : "not detected");
    #endif
    return isShortCircuit;
}

// Checks both faults for a pedal
bool checkRangeFaults(pedalSensor pedal) const {
      return checkOpenCircuit(pedal) || checkShortCircuit(pedal);
    }

// Pedal sensor1 WRITE DOWN RIGHT OR LEFT WHEN WE FIGURE IT OUT also update values
const pedalSensor p1 = {
    0,
    0,
    0,
    0,
    0,
    0
};

// Pedal sensor2 WRITE DOWN RIGHT OR LEFT WHEN WE FIGURE IT OUT also update values
const pedalSensor p2 = {
    0,
    0,
    0,
    0,
    0,
    0
};

const pedalSensor[] pedals = [p1, p2];

//******
// This would be where pedal map was defined with old apps
//  */


// By rules this will trip when
bool pedalAgreementFault() {

}



/* The devicetree node identifier for the "led0" alias. */

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

int main(void) {}
