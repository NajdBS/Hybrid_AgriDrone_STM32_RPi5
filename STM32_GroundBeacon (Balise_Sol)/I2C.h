#ifndef I2C_H
#define I2C_H

#include "stm32f4xx.h"


// DS1621 I2C Address and commands for the temperature sensor
#define DS1621_ADDR_WRITE  (0x90)  // DS1621 write address
#define DS1621_ADDR_READ   (0x91)  // DS1621 read address

// DS1621 Registers
#define DS1621_TEMP_REG    (0xAA)  // Register to read temperature
#define DS1621_CTRL_REG    (0xAC)  // Register to write control settings
#define DS1621_STATUS_REG  (0xA8)  // Status register

// Commands for controlling the DS1621
#define DS1621_START_CONVERSION  (0xEE)  // Start temperature conversion
#define DS1621_STOP_CONVERSION   (0x22)  // Stop temperature conversion



// Initialize I2C1
void I2C1_Init(void);

// Start I2C communication
void I2C_Start(void);

// Set I2C address
void I2C_Address(uint8_t Address);

// Write data to I2C
void I2C_Write(uint8_t data);

// Read data from I2C
void I2C_Read(uint8_t Address, uint8_t *buffer, uint8_t size);

// Stop I2C communication
void I2C_Stop(void);

// DS1621 Specific Functions
void DS1621_Write(uint8_t Address, uint8_t Reg, uint8_t Data);
void DS1621_Command(uint8_t Address, uint8_t Command);
void DS1621_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size);
void DS1621_Init(void);

// Function to read temperature from DS1621
float DS1621_Read_Temp(void);

#endif
