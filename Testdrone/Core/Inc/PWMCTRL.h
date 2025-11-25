/*
 * PWMCTRL.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Najd Ben Saad
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

#include "stm32f4xx_hal.h"    // ou #include "stm32f4xx_hal_i2c.h"

extern TIM_HandleTypeDef htim3;

/* PWM constants (in microseconds) */
#define PWM_MIN_US          1000    // Minimum PWM (motor stopped)
#define PWM_MAX_US          2000    // Maximum PWM (full throttle)
#define PWM_RANGE_US        1000    // Range: 2000 - 1000

/* Motor command limits */
#define MOTOR_CMD_MIN       0       // Minimum motor command
#define MOTOR_CMD_MAX       600     // Maximum motor command (safety limit)

/* Safety thresholds */
#define SAFETY_ANGLE_MAX    45.0f   // Max tilt angle (degrees) before auto-disarm


extern uint16_t pwm_fr;
extern uint16_t pwm_fl;
extern uint16_t pwm_rr;
extern uint16_t pwm_rl;

typedef enum{
	DISARMED = 0,
	ARMED = 1
}ESC_STATE;



typedef struct{

	int16_t FR;
	int16_t FL;
	int16_t RR;
	int16_t RL;

	int16_t throttle ;//throttle

	ESC_STATE state;

}ESC_CONF;




extern char cmd_rx[1];


void ESC_Init(ESC_CONF * ESC_speed);

void ESC_Calibrate(ESC_CONF * ESC_speed);

void ESC_Arm(ESC_CONF *esc);

void ESC_Disarm(ESC_CONF *esc);

/************/
void ESC_UpdateCommands(ESC_CONF *esc, int16_t throttle, float ctrl_roll, float ctrl_pitch, float ctrl_yaw);

void ESC_SetSpeed(ESC_CONF *esc);

void ESC_EmergencyStop(ESC_CONF *esc);

uint8_t ESC_SafetyCheck(ESC_CONF *esc, float roll_deg, float pitch_deg);

//void ESC_followCmd(ESC_CONF * ESC_speed, IMU_MEASURE * MPU_measure,  char cmd);

//void ESC_setSpeed(ESC_CONF * ESC_speed);

//void CMD_transform(ESC_CONF * ESC_speed, char cmd);



#endif /* INC_PWMCTRL_H_ */
