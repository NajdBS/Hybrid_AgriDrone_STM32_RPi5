
/*
 * PWMCTRL.h
 *
 * Created on: Nov 18, 2025
 * Author: Najd Ben Saad
 * Updated: Dec 2025
 *
 * Description:
 * Driver to control 4 ESCs (Electronic Speed Controllers) via PWM.
 * Includes Motor Mixing logic for Quadcopter X configuration.
 */
/*
* Controls 4 ESCs via PWM (TIM3)
 * Motor layout (X configuration):
 *       FRONT
 *     FL     FR
 *       \ /
 *       / \
 *     RL     RR
 *       REAR
 */

#ifndef INC_PWMCTRL_H_
#define INC_PWMCTRL_H_

#include "stm32l4xx_hal.h"
#include <math.h>

/* Timer Handler defined in main.c */
extern TIM_HandleTypeDef htim1;

/* PWM Signal Constants (microseconds) */
#define PWM_MIN_US          1000    // Motor Stopped
#define PWM_MAX_US          2000    // Full Throttle
#define PWM_RANGE_US        1000    // Range

/* Motor command settings */
#define MOTOR_CMD_MIN       0
#define MOTOR_FULL_SCALE    1000
#define MOTOR_SAFE_MAX       600     // Safety Limit for testing (max 60%)
//#define MOTOR_CMD_MAX       600

/* Safety */
#define SAFETY_ANGLE_MAX    45.0f   // Max angle before auto-kill

/* State definitions */
typedef enum {
    DISARMED = 0,
    ARMED = 1
} ESC_STATE;

/* Main Struct */
typedef struct {
    int16_t FR;       // Front Right
    int16_t FL;       // Front Left
    int16_t RR;       // Rear Right
    int16_t RL;       // Rear Left
    int16_t throttle; // Base throttle
    ESC_STATE state;  // Arming state
} ESC_CONF;

/* Function Prototypes */
void ESC_Init(ESC_CONF *esc);
void ESC_Calibrate(ESC_CONF *esc);
void ESC_Arm(ESC_CONF *esc);
void ESC_Disarm(ESC_CONF *esc);

/* Core update function - Mixes PID outputs into motor speeds */
void ESC_UpdateCommands(ESC_CONF *esc, int16_t throttle, float ctrl_roll, float ctrl_pitch, float ctrl_yaw);

/* Hardware update - Sends PWM to pins */
void ESC_SetSpeed(ESC_CONF *esc);

/* Safety functions */
void ESC_EmergencyStop(ESC_CONF *esc);
uint8_t ESC_SafetyCheck(ESC_CONF *esc, float roll_deg, float pitch_deg);

#endif /* INC_PWMCTRL_H_ */

