/*
 * LowPassFilter.h
 *
 *  Created on: Nov 15, 2025
 *      Author: Najd Ben Saad
 */

/* Written by: Philip M. Salmony @ philsal.co.uk */

#ifndef LPF_H
#define LPF_H

#include <stdint.h>

#define LPF_TYPE_BESSEL 1

typedef struct {
    float out;          // Current filter output (y[n])
    float buf[2];       // History buffer (y[n-1], y[n-2])
    float coeffNum;     // Numerator coefficient
    float coeffDen[2];  // Denominator coefficients
} LPFTwoPole;

void LPFTwoPole_Init(LPFTwoPole *lpf, uint8_t type, float cutoffFrequency, float sampleTime);
float LPFTwoPole_Update(LPFTwoPole *lpf, float val);

#endif
