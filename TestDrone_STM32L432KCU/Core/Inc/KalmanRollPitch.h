/*
 * KalmanRollPitch.h
 *
 *  Created on: Nov 15, 2025
 *      Author: Najd Ben Saad
 */
/*
*
* Kalman Filter - Two States (Roll and Pitch)
*
* Implements a continuous-discrete Kalman filter for estimation of roll and pitch angles.
* Sensors: gyro (p, q, r), accelerometer (ax, ay, az), pitot tube and differential pressure sensor (Va)
*
* Written by: Philip M. Salmony @ philsal.co.uk
* Last changed: 02 Dec 2019
**/
/*
 * Description:
 * Header file for the Extended Kalman Filter (EKF) used to estimate
 * Roll (Phi) and Pitch (Theta) angles from Gyroscope and Accelerometer data.
 */

#ifndef KALMAN_ROLL_PITCH_H
#define KALMAN_ROLL_PITCH_H

#include <math.h>
#include <stdint.h>

/* Gravity constant (m/s^2) - Must match the unit of accelerometer input */
#define g 9.81f

/**
 * @brief Kalman Filter State Structure
 */
typedef struct {
    float phi;    // Estimated Roll angle [radians]
    float theta;  // Estimated Pitch angle [radians]

    float P[4];   // Error Covariance Matrix (2x2, flattened to 1D array)
                  // P[0]=P00, P[1]=P01, P[2]=P10, P[3]=P11

    float Q[2];   // Process Noise Covariance (Gyro noise)
                  // Q[0]=Roll noise, Q[1]=Pitch noise

    float R[3];   // Measurement Noise Covariance (Accelerometer noise)
                  // R[0]=Ax noise, R[1]=Ay noise, R[2]=Az noise

    float gyr[3]; // Storage for the most recent gyro rates [rad/s]
} KalmanRollPitch;

/**
 * @brief Initialize the Kalman Filter
 * @param kal Pointer to the filter structure
 * @param Pinit Initial uncertainty (e.g., 10.0f)
 * @param Q Array of process noise covariance
 * @param R Array of measurement noise covariance
 */
void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R);

/**
 * @brief Prediction Step (Time Update)
 * Uses Gyroscope data to predict the next state (integrate angles).
 * @param kal Pointer to the filter structure
 * @param gyr Array of gyro rates [p, q, r] in rad/s
 * @param T Sample time in seconds (dt)
 */
void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyr, float T);

/**
 * @brief Update Step (Measurement Correction)
 * Uses Accelerometer data to correct the predicted state (drift compensation).
 * @param kal Pointer to the filter structure
 * @param acc Array of acceleration [ax, ay, az] in m/s^2
 * @param Va True airspeed (Set to 0.0f for multirotors/hover)
 * @return 1 if successful, 0 if matrix inversion failed
 */
uint8_t KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc, float Va);

#endif /* KALMAN_ROLL_PITCH_H */
