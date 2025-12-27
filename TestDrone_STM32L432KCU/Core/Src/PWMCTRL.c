/*
 * PWMCTRL.C
 *
 *  Created on: Nov 18, 2025
 *      Author: Najd Ben Saad
 */

#include "PWMCTRL.h"
#include <math.h>


//char cmd_rx[1];

// FUNCTIONS *****************************

/**
 * @brief Clamp a value between min and max
 */
static inline int16_t clamp(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static inline uint16_t command_to_pwm(int16_t command) {
    // Map 0-600 to 1000-2000 us
    return PWM_MIN_US + ((command * PWM_RANGE_US) / MOTOR_CMD_MAX);
}


/*void Motors_Init(void)
{
    printf("Initializing motors (ESC)...\r\n");

    // Start PWM on all motor channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Set minimum throttle (1000 µs pulse)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MIN);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN);

    motors_armed = 0;  // Safety: motors OFF

    printf("Motors initialized (PWM @ TIM3). ESC waiting for ARM.\r\n");
}*/

void ESC_Init(ESC_CONF *esc) {
    // Initialize all values to zero
    esc->FR = 0;
    esc->FL = 0;
    esc->RR = 0;
    esc->RL = 0;
    esc->throttle = 0;
    esc->state = DISARMED;

    // Start PWM on all 4 channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Set all motors to minimum PWM (stopped)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN_US);

    // ESC initialization sequence (some ESCs need this)
    HAL_Delay(100);

    printf("Motors initialized (PWM @ TIM3). ESC waiting for ARM.\r\n");
}

void ESC_Calibrate(ESC_CONF *esc) {

    // Send max PWM for 2 seconds
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MAX_US);
    HAL_Delay(2000);

    // Send min PWM for 2 seconds
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN_US);
    HAL_Delay(2000);

    esc->state = DISARMED;
}

void ESC_Arm(ESC_CONF *esc) {
    esc->state = ARMED;
}

void ESC_Disarm(ESC_CONF *esc) {
    esc->FR = 0;
    esc->FL = 0;
    esc->RR = 0;
    esc->RL = 0;
    esc->throttle = 0;
    esc->state = DISARMED;

    // Immediately set motors to minimum
    ESC_SetSpeed(esc);
}

void ESC_UpdateCommands(ESC_CONF *esc, int16_t throttle, float ctrl_roll, float ctrl_pitch, float ctrl_yaw) {

    // If disarmed, all motors to zero
    if (esc->state == DISARMED) {
        esc->FR = 0;
        esc->FL = 0;
        esc->RR = 0;
        esc->RL = 0;
        esc->throttle = 0;
        return;
    }

    // Store throttle
    esc->throttle = clamp(throttle, MOTOR_CMD_MIN, MOTOR_CMD_MAX);

    // Convert controller outputs (typically in rad/s) to motor commands
    // Scale factor: adjust based on your controller gains
    // Typical range: ctrl outputs are -4 to +4 rad/s, map to ±200 motor units
    float scale = 50.0f;  // Adjust this based on testing

    int16_t roll_correction  = (int16_t)(ctrl_roll * scale);
    int16_t pitch_correction = (int16_t)(ctrl_pitch * scale);
    int16_t yaw_correction   = (int16_t)(ctrl_yaw * scale);

    // Motor mixing for X configuration
    // Roll:  positive = tilt right → increase left motors (FL, RL)
    // Pitch: positive = tilt forward → increase rear motors (RR, RL)
    // Yaw:   positive = rotate CW → increase CCW motors (FL, RR)

    esc->FR = esc->throttle - pitch_correction - roll_correction + yaw_correction;
    esc->FL = esc->throttle - pitch_correction + roll_correction - yaw_correction;
    esc->RR = esc->throttle + pitch_correction - roll_correction - yaw_correction;
    esc->RL = esc->throttle + pitch_correction + roll_correction + yaw_correction;

    // Clamp all motor commands to safe range
    esc->FR = clamp(esc->FR, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
    esc->FL = clamp(esc->FL, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
    esc->RR = clamp(esc->RR, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
    esc->RL = clamp(esc->RL, MOTOR_CMD_MIN, MOTOR_CMD_MAX);
}

void ESC_SetSpeed(ESC_CONF *esc) {

   // extern uint16_t pwm_fr, pwm_fl, pwm_rr, pwm_rl;

    if (esc->state == DISARMED) {
        // All motors to minimum PWM
        pwm_fr = PWM_MIN_US;
        pwm_fl = PWM_MIN_US;
        pwm_rr = PWM_MIN_US;
        pwm_rl = PWM_MIN_US;
    } else {
        // Convert commands to PWM values
        pwm_fr = command_to_pwm(esc->FR);
        pwm_fl = command_to_pwm(esc->FL);
        pwm_rr = command_to_pwm(esc->RR);
        pwm_rl = command_to_pwm(esc->RL);
    }

    // Apply PWM to hardware
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_fr);  // Front Right
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_rl);  // Front Left
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_fl);  // Rear Right
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_rr);  // Rear Left*/
}

void ESC_EmergencyStop(ESC_CONF *esc) {
    ESC_Disarm(esc);

    // Force hardware PWM to minimum immediately
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MIN_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MIN_US);
}

uint8_t ESC_SafetyCheck(ESC_CONF *esc, float roll_deg, float pitch_deg) {
    // Check if angles exceed safe limits
    if (fabs(roll_deg) > SAFETY_ANGLE_MAX || fabs(pitch_deg) > SAFETY_ANGLE_MAX) {
        ESC_EmergencyStop(esc);
        return 0;  // Unsafe
    }
    return 1;  // Safe
}

/*void ESC_followCmd(ESC_CONF * ESC_speed, IMU_MEASURE * MPU_measure, char cmd){

	int cmd_pitch = 0;
	int cmd_roll  = 0;

	int16_t low_lim = 0;
	int16_t high_lim = 600;

	// PID values
	int P_pitch = 2;
	int D_pitch  = 1;
	//int I_pitch = 0;
	int P_roll  = 2;
	int D_roll  = 1;
	//int I_roll = 0;


	if(cmd == 'W'){	// forward
		cmd_pitch = -10;
		//PRINTF(" FORWARD \n\r");
	}else if(cmd == 'A'){ // left
		cmd_roll = -10;
		//PRINTF(" LEFT \n\r");
	}else if(cmd == 'S'){ // back
		cmd_pitch = 10;
		//PRINTF(" BACK \n\r");
	}else if(cmd == 'D'){ // right
		cmd_roll = +10;
		//PRINTF(" RIGHT \n\r");
	}else if(cmd == 'I'){ // up
		ESC_speed->up_value += 10;
		//PRINTF(" UP \n\r");
	}else if(cmd == 'J'){ // rotate left
		//PRINTF(" ROT LEFT \n\r");
	}else if(cmd == 'K'){ // down
		ESC_speed->up_value -= 50;
		//PRINTF(" DOWN \n\r");
	}else if(cmd == 'L'){ // rotate right
		//PRINTF(" ROT RIGHT \n\r");
	}else if(cmd == 'X'){ // disarm drone
		ESC_speed->state = DISARMED;
		//PRINTF(" DISARM \n\r");
	}else if(cmd == 'C'){ // arms DRONE
		ESC_speed->state = ARMED;
		//PRINTF(" ARM \n\r");
	}

	// Security check to avoid dead bodies in the field
	if(MPU_measure->angle_X > 10 || MPU_measure->angle_Y > 10 || MPU_measure->angle_X < -10 || MPU_measure->angle_Y < -10){
		ESC_speed->state == DISARMED;
	}

	if(ESC_speed->state == ARMED){

		double pitch_err   = 0;//cmd_pitch - MPU_measure->angle_X;
		double pitch_d_err = 0;//MPU_measure->gyro_angle_dX;

		double roll_err    = 0;//cmd_roll  - MPU_measure->angle_Y;
		double roll_d_err  = 0;//MPU_measure->gyro_angle_dY;

		int16_t FR_tmp = ESC_speed->up_value + pitch_err*P_pitch - pitch_d_err*D_pitch - roll_err*P_roll + roll_d_err*D_roll;
		int16_t FL_tmp = ESC_speed->up_value + pitch_err*P_pitch - pitch_d_err*D_pitch + roll_err*P_roll - roll_d_err*D_roll;
		int16_t RR_tmp = ESC_speed->up_value - pitch_err*P_pitch + pitch_d_err*D_pitch - roll_err*P_roll + roll_d_err*D_roll;
		int16_t RL_tmp = ESC_speed->up_value - pitch_err*P_pitch + pitch_d_err*D_pitch + roll_err*P_roll - roll_d_err*D_roll;

		if(FR_tmp > low_lim && FR_tmp < high_lim){
			ESC_speed->FR = FR_tmp;
		}

		if(FL_tmp > low_lim && FL_tmp < high_lim){
			ESC_speed->FL = FL_tmp;
		}

		if(RR_tmp > low_lim && RR_tmp < high_lim){
			ESC_speed->RR = RR_tmp;
		}

		if(RL_tmp > low_lim && RL_tmp < high_lim){
			ESC_speed->RL = RL_tmp;
		}

	}else if(ESC_speed->state == DISARMED){

		ESC_speed->FR = 0;
		ESC_speed->FL = 0;
		ESC_speed->RR = 0;
		ESC_speed->RL = 0;
		ESC_speed->up_value =0;
	}
}


void ESC_setSpeed(ESC_CONF * ESC_speed){

	TIM3->CCR1 = ESC_speed->FR + PWM_1_MS;
	TIM3->CCR2 = ESC_speed->FL + PWM_1_MS;
	TIM3->CCR3 = ESC_speed->RR + PWM_1_MS;
	TIM3->CCR4 = ESC_speed->RL + PWM_1_MS;
}
*/






