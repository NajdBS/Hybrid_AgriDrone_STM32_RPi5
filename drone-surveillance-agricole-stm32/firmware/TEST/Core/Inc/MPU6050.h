/*
 * MPU6050.h
 *
 *  Created on: Oct 7, 2025
 *      Author: Najd Ben Saad
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stdio.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


// Variables globales externes
extern I2C_HandleTypeDef hi2c1;

extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
extern int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;

extern float Ax, Ay, Az;
extern float Gx, Gy, Gz;

//extern uint8_t check_test;
// Fonctions
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);






#endif /* INC_MPU6050_H_ */
