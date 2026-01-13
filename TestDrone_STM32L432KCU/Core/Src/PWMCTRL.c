/*
 * PWMCTRL.c
 *
 * Created on: Nov 18, 2025
 * Author: Najd Ben Saad
 * Updated: Dec 2025
 *
 * Description:
 * Implementation of Motor Control and Mixing Logic.
 * Wiring Assumption (Based on our drone):
 * - TIM1 CH1 -> RR (Rear Right) (PA8)
 * - TIM1 CH2 -> FL (Front Left)  Changed to //- TIM2 CH1 -> FL (Front Left) (PA9)-> (PA5)-> (PA3) //
 * - TIM1 CH3 -> FR (Front Right) Changed to //- TIM2 CH2 -> FR (Front Right)(PA10)-> (PA1) //
 * - TIM1 CH4 -> RL (Rear Left) (PA11)
 */

#include "PWMCTRL.h"

/* Debug variables to see PWM values in Live Expression */
volatile uint16_t pwm_fr_debug = 0;
volatile uint16_t pwm_fl_debug = 0;
volatile uint16_t pwm_rr_debug = 0;
volatile uint16_t pwm_rl_debug = 0;

/**
 * @brief Helper: Clamp value within range
 */
static inline int16_t clamp(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Helper: Convert 0-CMD_MAX (SAFE_MAX) command to 1000-2000us PWM
 */
static inline uint16_t command_to_pwm(int16_t command) {

    // Scale command to PWM range
    uint32_t pwm = PWM_MIN_US + ((uint32_t)command * PWM_RANGE_US) / MOTOR_FULL_SCALE;

    if (pwm > PWM_MAX_US) return PWM_MAX_US;
    return (uint16_t)pwm;
}

void ESC_Init(ESC_CONF *esc) {
    esc->FR = 0; esc->FL = 0; esc->RR = 0; esc->RL = 0;
    esc->throttle = 0;
    esc->state = DISARMED;

    // Start PWM Hardware
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // RR
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // RL

    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // FL
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // FL
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // FR


    // Initial output: Min PWM
    ESC_SetSpeed(esc);

    // Allow ESCs to boot
    HAL_Delay(100);
}

void ESC_Calibrate(ESC_CONF *esc) {
    // WARNING: PROPS MUST BE REMOVED

    // 1. High Signal
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_MAX_US);
    HAL_Delay(2000); // Wait for beep

    // 2. Low Signal
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_MIN_US);
    HAL_Delay(2000); // Wait for confirmation beep

    esc->state = DISARMED;
}

void ESC_Arm(ESC_CONF *esc) {
    esc->throttle = 0; // Safety reset
    esc->state = ARMED;
}

void ESC_Disarm(ESC_CONF *esc) {
    esc->state = DISARMED;
    esc->throttle = 0;
    ESC_SetSpeed(esc); // Apply stop immediately
}

void ESC_UpdateCommands(ESC_CONF *esc, int16_t throttle, float ctrl_roll, float ctrl_pitch, float ctrl_yaw) {
    if (esc->state == DISARMED) {
        esc->FR = 0; esc->FL = 0; esc->RR = 0; esc->RL = 0;
        return;
    }

    esc->throttle = clamp(throttle, MOTOR_CMD_MIN, MOTOR_SAFE_MAX);

    // Scale PID output (Radians/sec) to Motor Command units
    // Start with 50. Increase if drone is sluggish correcting itself.
    float scale = 50.0f;

    int16_t P_corr = (int16_t)(ctrl_pitch * scale);
    int16_t R_corr = (int16_t)(ctrl_roll * scale);
    int16_t Y_corr = (int16_t)(ctrl_yaw * scale);

    /* MIXING LOGIC (Standard Quad X)
     * * Logic Check (Pitch):
     * If Nose Up (Angle > 0) -> PID Error is Neg -> P_corr is Neg.
     * We want to LOWER Front and RAISE Rear.
     * Front = Throttle + P_corr (Since P is neg, this subtracts/lowers) -> CORRECT
     * Rear  = Throttle - P_corr (Since P is neg, this adds/raises)      -> CORRECT
     *
     * Logic Check (Roll):
     * If Right Down (Angle > 0) -> PID Error is Neg -> R_corr is Neg.
     * We want to RAISE Right and LOWER Left.
     * Right = Throttle - R_corr (Adds) -> CORRECT
     * Left  = Throttle + R_corr (Subtracts) -> CORRECT
     */

    esc->FR = esc->throttle + P_corr - R_corr - Y_corr; // Front Right
    esc->FL = esc->throttle + P_corr + R_corr + Y_corr; // Front Left
    esc->RR = esc->throttle - P_corr - R_corr + Y_corr; // Rear Right
    esc->RL = esc->throttle - P_corr + R_corr - Y_corr; // Rear Left

    // Clamp values to prevent motor cutoff or overflow
    esc->FR = clamp(esc->FR, MOTOR_CMD_MIN, MOTOR_SAFE_MAX);
    esc->FL = clamp(esc->FL, MOTOR_CMD_MIN, MOTOR_SAFE_MAX);
    esc->RR = clamp(esc->RR, MOTOR_CMD_MIN, MOTOR_SAFE_MAX);
    esc->RL = clamp(esc->RL, MOTOR_CMD_MIN, MOTOR_SAFE_MAX);
}

void ESC_SetSpeed(ESC_CONF *esc) {
	uint16_t pwm1, pwm2, pwm3, pwm4;

    if (esc->state == DISARMED) {
        pwm1 = pwm2 = pwm3 = pwm4 = PWM_MIN_US;
    } else {
        // Map according to wiring setup:
        // CH1=RR, CH2=FL, CH3=FR, CH4=RL
        pwm1 = command_to_pwm(esc->RR); // Channel 1
        pwm2 = command_to_pwm(esc->FL); // Channel 2
        pwm3 = command_to_pwm(esc->FR); // Channel 3
        pwm4 = command_to_pwm(esc->RL); // Channel 4
    }

    // Update Global Debug Variables
    pwm_rr_debug = pwm1;
    pwm_fl_debug = pwm2;
    pwm_fr_debug = pwm3;
    pwm_rl_debug = pwm4;

    // Write to Hardware Registers
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1); // RR
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm4); // RL

    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, pwm2); // FL
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm3); // FR
}

void ESC_EmergencyStop(ESC_CONF *esc) {
    ESC_Disarm(esc);
    // Redundant safety set
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_MIN_US);
}

uint8_t ESC_SafetyCheck(ESC_CONF *esc, float roll_deg, float pitch_deg) {
    // If drone tilts too much (crash or manual handling), kill motors
    if (fabs(roll_deg) > SAFETY_ANGLE_MAX || fabs(pitch_deg) > SAFETY_ANGLE_MAX) {
        ESC_EmergencyStop(esc);
        return 0; // Check Failed
    }
    return 1; // Safe
}
