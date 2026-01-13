/*
 * defines.h
 *
 * Created on: Nov 15, 2025
 * Author: Najd Ben Saad
 * Updated: Dec 2025 for STM32L432KCU + MPU6050 ONLY (Bluetooth Control)
 *
 * Description:
 * Configuration parameters for the Flight Controller.
 */

#ifndef DEFINES_H
#define DEFINES_H

/* LOOP TIMING & SYNCHRONIZATION */
/* Accelerometer and Gyroscope sampling rate (10ms = 100Hz) */
//#define SAMPLE_TIME_ACC_MS  10  // Synced with Gyro for Kalman stability
//#define SAMPLE_TIME_GYR_MS  10
/* Sample times (OLD)
#define SAMPLE_TIME_ACC_MS  16
#define SAMPLE_TIME_GYR_MS  10*/

/* Sample times */
#define SAMPLE_TIME_ACC_MS  16
#define SAMPLE_TIME_GYR_MS  10
#define SAMPLE_TIME_CTRL_MS 4
#define SAMPLE_TIME_LED_MS  1000


/* Command update rate (Bluetooth/RC) */
#define SAMPLE_TIME_RC_MS   20
//#define SAMPLE_TIME_RC_MS   10

/* Main Control Loop Frequency (10ms = 100Hz) */
//#define SAMPLE_TIME_CTRL_MS 10


/* LED Status blink rate (1000ms = 1Hz) - Heartbeat */
#define SAMPLE_TIME_LED_MS  1000

/* DIGITAL LOW-PASS FILTERS */
/* Filters mechanical vibration from sensors */
#define LPF_ACC_CUTOFF_HZ 10.0f
#define LPF_GYR_CUTOFF_HZ 30.0f
/* Low-pass filters (OLD)
#define LPF_ACC_CUTOFF_HZ 10.0f
#define LPF_GYR_CUTOFF_HZ 32.0f*/

/* Complementary filter (yaw angle estimation)
#define CF_ALPHA 0.01f*/

/*KALMAN FILTER CONSTANTS*/
#define EKF_P_INIT 10.0f
#define EKF_N_GYR 0.00000191056f // Gyro process noise
#define EKF_N_ACC 0.000067666f   // Accel measurement noise

/*PID CONTROLLERS (Stability Settings)*/
/* SAFETY VALUES: Low P, Zero I/D for first test to prevent shaking */
/* ROLL AXIS */
#define CTRL_ROLL_P  0.80f   // Reduced for safety
#define CTRL_ROLL_I  0.00f   // 0 for tuning
#define CTRL_ROLL_D  0.00f
#define CTRL_ROLL_FF 0.00f
#define CTRL_ROLL_LIM_MIN -4.36f // Output limit (radians)
#define CTRL_ROLL_LIM_MAX  4.36f
/* PITCH AXIS */
#define CTRL_PITCH_P  0.80f   // Reduced for safety
#define CTRL_PITCH_I  0.00f
#define CTRL_PITCH_D  0.00f
#define CTRL_PITCH_FF 0.00f
#define CTRL_PITCH_LIM_MIN -4.36f
#define CTRL_PITCH_LIM_MAX  4.36f
/* Controllers (OLD)
#define CTRL_ROLL_P  1.23f
#define CTRL_ROLL_I  0.10f
#define CTRL_ROLL_D  0.00f
#define CTRL_ROLL_FF 0.00f
#define CTRL_ROLL_LIM_MIN -4.36f
#define CTRL_ROLL_LIM_MAX  4.36f

#define CTRL_PITCH_P  1.08f
#define CTRL_PITCH_I  0.08f
#define CTRL_PITCH_D  0.00f
#define CTRL_PITCH_FF 0.00f
#define CTRL_PITCH_LIM_MIN -4.36f
#define CTRL_PITCH_LIM_MAX  4.36f */

/*COMMAND LIMITS (Safety)*/
#define PWM_THR_LIMIT 600

/* Deadband for Joystick/Bluetooth app center stick (0.05 = 5%)
#define RC_DEADBAND 0.05f*/

/*INPUT SCALING (Bluetooth Command -> Drone Angle)*/
/* How much the drone tilts when you send "Max Forward" command */
#define RC_TO_ROLL_ANGLE_SETPOINT  (20.0f * DEG_TO_RAD)
#define RC_TO_PITCH_ANGLE_SETPOINT (20.0f * DEG_TO_RAD)
#define RC_TO_YAW_RATE_SETPOINT    (45.0f * DEG_TO_RAD)

#define CMD_TILT_ANGLE (40.0f * DEG_TO_RAD)

/* (OLD)
#define RC_TO_ROLL_ANGLE_SETPOINT (10.0f * DEG_TO_RAD)
#define RC_TO_PITCH_ANGLE_SETPOINT (10.0f * DEG_TO_RAD)
#define RC_TO_YAW_RATE_SETPOINT (25.0f * DEG_TO_RAD)*/

/* Conversions */
#define RAD_TO_DEG 57.2957795131f
#define DEG_TO_RAD 0.01745329251f

/*SENSOR CALIBRATION */
#define CALIB_ACC_SCALE_X 1.0f
#define CALIB_ACC_SCALE_Y 1.0f
#define CALIB_ACC_SCALE_Z 1.0f

#define CALIB_ACC_BIAS_X  0.0f
#define CALIB_ACC_BIAS_Y  0.0f
#define CALIB_ACC_BIAS_Z  0.0f

#endif /* DEFINES_H */
