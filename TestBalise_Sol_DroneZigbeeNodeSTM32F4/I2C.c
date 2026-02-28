#include "I2C.h"


void I2C1_Init()
{
    // Enable clock for I2C1 (APB1)
    RCC->APB1ENR |= (1 << 21);

    // Enable clock for GPIOB
    RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB (for PB6 and PB7)

    // Configure PB6 and PB7 for I2C1 (Alternate Function)
    GPIOB->MODER &= ~((3 << (2 * 6)) | (3 << (2 * 7)));  // Clear mode bits for PB6 and PB7
    GPIOB->MODER |= ((2 << (2 * 6)) | (2 << (2 * 7)));   // Set PB6 and PB7 to alternate function (AF)

    GPIOB->AFR[0] &= ~((0xF << (4 * 6)) | (0xF << (4 * 7)));  // Clear AF bits for PB6 and PB7
    GPIOB->AFR[0] |= ((4 << (4 * 6)) | (4 << (4 * 7)));  // Select AF4 function for PB6 and PB7

    GPIOB->OTYPER |= ((1 << 7) | (1 << 6));  // Set PB6 and PB7 as open-drain

    // Reset the I2C
    I2C1->CR1 |= (1 << 15);
    I2C1->CR1 &= ~(1 << 15);

    // Input clock in I2C_CR2 Register
    I2C1->CR2 |= (16 << 0);  // PCLK1 frequency in MHz

    // Configure the clock control registers
    I2C1->CCR = 80 << 0;

    // Configure the rise time register
    I2C1->TRISE = 17;

    // Enable the peripheral
    I2C1->CR1 |= (1 << 0);  // Enable I2C
}

void I2C_Start(void)
{
    I2C1->CR1 |= (1 << 8);  // Generate START condition
    while (!(I2C1->SR1 & (1 << 0)));  // Wait for SB bit to set
}

void I2C_Address(uint8_t Address)
{
    I2C1->DR = Address;  // Send the address
    while (!(I2C1->SR1 & (1 << 1)));  // Wait for ADDR bit to set
    I2C1->SR2;  // Clear ADDR bit
}

void I2C_Write(uint8_t data)
{
    while (!(I2C1->SR1 & (1 << 7)));  // Wait for TXE bit to set
    I2C1->DR = data;
}

void I2C_Read(uint8_t Address, uint8_t *buffer, uint8_t size)
{
    int remaining = size;

    if (size == 1)
    {
        I2C1->DR = Address;
        while (!(I2C1->SR1 & (1 << 1)));  // Wait for ADDR bit to set
        I2C1->CR1 &= ~(1 << 10);  // Disable ACK
        I2C1->SR2;  // Clear ADDR bit
        I2C1->CR1 |= (1 << 9);  // Generate STOP condition

        while (!(I2C1->SR1 & (1 << 6)));  // Wait for RxNE to set
        buffer[size - remaining] = I2C1->DR;  // Read the data
    }
    else
    {
        I2C1->DR = Address;
        while (!(I2C1->SR1 & (1 << 1)));  // Wait for ADDR bit to set
        I2C1->SR2;  // Clear ADDR bit

        I2C1->CR1 |= (1 << 10);  // Enable ACK

        while (remaining > 2)
        {
            while (!(I2C1->SR1 & (1 << 6)));  // Wait for RxNE
            buffer[size - remaining] = I2C1->DR;  // Copy data
            remaining--;
        }

        while (!(I2C1->SR1 & (1 << 6)));  // Wait for RxNE
        buffer[size - remaining] = I2C1->DR;
        I2C1->CR1 |= (1 << 9);  // Generate STOP condition

        remaining--;

        I2C1->CR1 &= ~(1 << 10);  // Clear ACK bit
        while (!(I2C1->SR1 & (1 << 6)));  // Wait for RxNE
        buffer[size - remaining] = I2C1->DR;
    }
}

void I2C_Stop(void)
{
    I2C1->CR1 |= (1 << 9);  // Generate STOP condition
}

void DS1621_Write(uint8_t Address, uint8_t Reg, uint8_t Data)
{
    I2C_Start();
    I2C_Address(Address);
    I2C_Write(Reg);
    I2C_Write(Data);
    I2C_Stop();
}

void DS1621_Command(uint8_t Address, uint8_t Command)
{
    I2C_Start();
    I2C_Address(Address);
    I2C_Write(Command);
    I2C_Stop();
}

void DS1621_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
    I2C_Start();
    I2C_Address(Address);
    I2C_Write(Reg);
    I2C_Start();  // Repeated start
    I2C_Read(Address + 0x01, buffer, size);
    I2C_Stop();
}

void DS1621_Init(void)
{
    DS1621_Command(DS1621_ADDR_WRITE, DS1621_START_CONVERSION);
}

float DS1621_Read_Temp(void)
{
    uint8_t Rx_data[2];  // Array to store the raw data from the sensor
    int16_t val;         // Variable to hold the processed temperature value
    float temp;          // Variable to store the final temperature in Celsius

    // Read 2 bytes of data from the DS1621 temperature register
    DS1621_Read(DS1621_ADDR_WRITE, DS1621_TEMP_REG, Rx_data, 2);

    // Combine the two received bytes to form a 9-bit value
    val = (int16_t)((Rx_data[0] << 1) | (Rx_data[1] >> 7));

    if (val & 0x0100) {
        val |= 0xFF00;  // Extend the sign for negative values
        temp = (float)val * 0.5;  // Convert the raw value to temperature (negative value)
    } else {
        temp = (float)val * 0.5;  // Convert the raw value to temperature (positive value)
    }

    return temp;  // Return the temperature in Celsius
}
