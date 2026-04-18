/*
 * KalmanRollPitch.c
 *
 * Created on: Nov 15, 2025
 * Author: Najd Ben Saad
 * */

/* Written by: Philip M. Salmony @ philsal.co.uk */

/* Description:
 * Implementation of the Extended Kalman Filter for attitude estimation.
 * Based on Euler angle kinematics.
 */

/* Modified */

#include "KalmanRollPitch.h"

void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R) {
	// Reset states
	kal->phi = 0.0f;
	kal->theta = 0.0f;

	// Initialize Covariance Matrix P (Diagonal)
	kal->P[0] = Pinit;
	kal->P[1] = 0.0f;
	kal->P[2] = 0.0f;
	kal->P[3] = Pinit;

	// Set Noise Covariances
	kal->Q[0] = Q[0];
	kal->Q[1] = Q[1];
	kal->R[0] = R[0];
	kal->R[1] = R[1];
	kal->R[2] = R[2];

	// Reset Gyro buffer
	kal->gyr[0] = 0.0f;
	kal->gyr[1] = 0.0f;
	kal->gyr[2] = 0.0f;
}

void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyr, float T) {
	/* 1. Extract Gyro Measurements [rad/s] */
	float p = gyr[0]; // Roll rate
	float q = gyr[1]; // Pitch rate
	float r = gyr[2]; // Yaw rate

	// Store for use in Update step if needed
	kal->gyr[0] = p;
	kal->gyr[1] = q;
	kal->gyr[2] = r;

	/* 2. Pre-compute Trigonometry */
	float sp = sin(kal->phi);
	float cp = cos(kal->phi);
	float ct = cos(kal->theta);

	/* 3. Gimbal Lock Protection */
	// If Pitch is near +/- 90 degrees, cos(theta) approaches 0.
	// Division by zero would occur in tan(theta) calculation.
	if (fabs(ct) < 0.01f) {
		return; // Skip prediction to prevent NaN (Not a Number) errors
	}

	// Calculate tan(theta)
	float tt = sin(kal->theta) / ct;

	/* 4. State Prediction (Priori) */
	// Euler Kinematics Equations:
	// dPhi   = p + tan(theta)*(q*sin(phi) + r*cos(phi))
	// dTheta = q*cos(phi) - r*sin(phi)
	// x(k+1) = x(k) + T * dx/dt
	kal->phi = kal->phi + T * (p + tt * (q * sp + r * cp));
	kal->theta = kal->theta + T * (q * cp - r * sp);

	/* 5. Jacobian Matrix A (Linearization of the system) */
	// We re-compute sin/cos with the NEW predicted angles for better linearization
	sp = sin(kal->phi);
	cp = cos(kal->phi);

	// A = df/dx (Partial derivatives of state transition function)
	float A[4] = { tt * (q * cp - r * sp), (r * cp + q * sp) * (tt * tt + 1.0f),
			-(r * cp + q * sp), 0.0f };

	/* 6. Covariance Prediction: P = A*P*A' + Q */
	// This propagates the uncertainty of our estimate forward in time
	float Ptmp[4] = { T
			* (kal->Q[0] + 2.0f * A[0] * kal->P[0] + A[1] * kal->P[1]
					+ A[1] * kal->P[2]), T
			* (A[0] * kal->P[1] + A[2] * kal->P[0] + A[1] * kal->P[3]
					+ A[3] * kal->P[1]), T
			* (A[0] * kal->P[2] + A[2] * kal->P[0] + A[1] * kal->P[3]
					+ A[3] * kal->P[2]), T
			* (kal->Q[1] + A[2] * kal->P[1] + A[2] * kal->P[2]
					+ 2.0f * A[3] * kal->P[3]) };

	kal->P[0] += Ptmp[0];
	kal->P[1] += Ptmp[1];
	kal->P[2] += Ptmp[2];
	kal->P[3] += Ptmp[3];
}

uint8_t KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc, float Va) {
	/* 1. Extract Measurements */
	float p = kal->gyr[0];
	float q = kal->gyr[1];
	float r = kal->gyr[2];

	float ax = acc[0];
	float ay = acc[1];

	/* Coordinate Frame Correction (CRITICAL) */
	// The MPU6050 measures reaction force (+1g UP when flat).
	// The Kalman model expects the Gravity Vector (-1g DOWN).
	// Therefore, we invert the Z-axis reading.
	float az = -acc[2];

	/* 2. Trigonometry */
	float sp = sin(kal->phi);
	float cp = cos(kal->phi);
	float st = sin(kal->theta);
	float ct = cos(kal->theta);

	/* 3. Measurement Model h(x) */
	// Predicted Accelerometer readings based on current angles.
	// Assuming Va (Airspeed) = 0 for hover.
	float h[3] = { q * Va * st + g * st,                     // Expected Ax
	r * Va * ct - p * Va * st - g * ct * sp,  // Expected Ay
	-q * Va * ct - g * ct * cp                 // Expected Az
	};

	/* 4. Jacobian Matrix C = dh/dx */
	// Relates changes in state (angles) to changes in measurements (accel)
	float C[6] = { 0.0f, q * Va * ct + g * ct, -g * cp * ct, -r * Va * st
			- p * Va * ct + g * sp * st,
	g * sp * ct, (q * Va + g * cp) * st };

	/* 5. Kalman Gain K = P*C' * inv(C*P*C' + R) */
	// Compute the term S = C*P*C' + R (Innovation Covariance)
	float G[9] = { kal->P[3] * C[1] * C[1] + kal->R[0], C[1] * C[2] * kal->P[2]
			+ C[1] * C[3] * kal->P[3], C[1] * C[4] * kal->P[2]
			+ C[1] * C[5] * kal->P[3],

	C[1] * (C[2] * kal->P[1] + C[3] * kal->P[3]), kal->R[1]
			+ C[2] * (C[2] * kal->P[0] + C[3] * kal->P[2])
			+ C[3] * (C[2] * kal->P[1] + C[3] * kal->P[3]), C[4]
			* (C[2] * kal->P[0] + C[3] * kal->P[2])
			+ C[5] * (C[2] * kal->P[1] + C[3] * kal->P[3]),

	C[1] * (C[4] * kal->P[1] + C[5] * kal->P[3]), C[2]
			* (C[4] * kal->P[0] + C[5] * kal->P[2])
			+ C[3] * (C[4] * kal->P[1] + C[5] * kal->P[3]), kal->R[2]
			+ C[4] * (C[4] * kal->P[0] + C[5] * kal->P[2])
			+ C[5] * (C[4] * kal->P[1] + C[5] * kal->P[3]) };

	// Calculate Determinant of S (G)
	float Gdet = (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8]
			+ G[1] * G[5] * G[6] + G[2] * G[3] * G[7] - G[2] * G[4] * G[6]);

	// Check for singularity (matrix inversion impossible)
	if (fabs(Gdet) > 0.000001f) {
		float Gdetinv = 1.0f / Gdet;

		// Compute inverse of S
		float Ginv[9] = { Gdetinv * (G[4] * G[8] - G[5] * G[7]), -Gdetinv
				* (G[1] * G[8] - G[2] * G[7]), Gdetinv
				* (G[1] * G[5] - G[2] * G[4]), -Gdetinv
				* (G[3] * G[8] - G[5] * G[6]), Gdetinv
				* (G[0] * G[8] - G[2] * G[6]), -Gdetinv
				* (G[0] * G[5] - G[2] * G[3]), Gdetinv
				* (G[3] * G[7] - G[4] * G[6]), -Gdetinv
				* (G[0] * G[7] - G[1] * G[6]), Gdetinv
				* (G[0] * G[4] - G[1] * G[3]) };

		// Compute Kalman Gain K
		float K[6] = { Ginv[3] * (C[2] * kal->P[0] + C[3] * kal->P[1])
				+ Ginv[6] * (C[4] * kal->P[0] + C[5] * kal->P[1])
				+ C[1] * Ginv[0] * kal->P[1], Ginv[4]
				* (C[2] * kal->P[0] + C[3] * kal->P[1])
				+ Ginv[7] * (C[4] * kal->P[0] + C[5] * kal->P[1])
				+ C[1] * Ginv[1] * kal->P[1], Ginv[5]
				* (C[2] * kal->P[0] + C[3] * kal->P[1])
				+ Ginv[8] * (C[4] * kal->P[0] + C[5] * kal->P[1])
				+ C[1] * Ginv[2] * kal->P[1], Ginv[3]
				* (C[2] * kal->P[2] + C[3] * kal->P[3])
				+ Ginv[6] * (C[4] * kal->P[2] + C[5] * kal->P[3])
				+ C[1] * Ginv[0] * kal->P[3], Ginv[4]
				* (C[2] * kal->P[2] + C[3] * kal->P[3])
				+ Ginv[7] * (C[4] * kal->P[2] + C[5] * kal->P[3])
				+ C[1] * Ginv[1] * kal->P[3], Ginv[5]
				* (C[2] * kal->P[2] + C[3] * kal->P[3])
				+ Ginv[8] * (C[4] * kal->P[2] + C[5] * kal->P[3])
				+ C[1] * Ginv[2] * kal->P[3] };

		/* 6. Update Error Covariance Matrix P = (I - K*C)*P */
		float PtmpUpdate[4];
		PtmpUpdate[0] = -kal->P[2] * (C[1] * K[0] + C[3] * K[1] + C[5] * K[2])
				- kal->P[0] * (C[2] * K[1] + C[4] * K[2] - 1.0f);
		PtmpUpdate[1] = -kal->P[3] * (C[1] * K[0] + C[3] * K[1] + C[5] * K[2])
				- kal->P[1] * (C[2] * K[1] + C[4] * K[2] - 1.0f);
		PtmpUpdate[2] = -kal->P[2]
				* (C[1] * K[3] + C[3] * K[4] + C[5] * K[5] - 1.0f)
				- kal->P[0] * (C[2] * K[4] + C[4] * K[5]);
		PtmpUpdate[3] = -kal->P[3]
				* (C[1] * K[3] + C[3] * K[4] + C[5] * K[5] - 1.0f)
				- kal->P[1] * (C[2] * K[4] + C[4] * K[5]);

		kal->P[0] += PtmpUpdate[0];
		kal->P[1] += PtmpUpdate[1];
		kal->P[2] += PtmpUpdate[2];
		kal->P[3] += PtmpUpdate[3];

		/* 7. Update State (Correction) x = x + K(y - h) */
		// Calculate the measurement residual (y - h) = (Actual Accel - Predicted Accel)
		kal->phi += K[0] * (ax - h[0]) + K[1] * (ay - h[1])
				+ K[2] * (az - h[2]);
		kal->theta += K[3] * (ax - h[0]) + K[4] * (ay - h[1])
				+ K[5] * (az - h[2]);

		return 1; // Success
	}

	return 0; // Failed (Matrix Singular)
}
