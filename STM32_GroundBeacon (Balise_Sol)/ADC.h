#ifndef ADC_H
#define ADC_H

#include "stm32f4xx.h"   // Header pour STM32F4
#include <stdbool.h>  // Inclure l'en-tête nécessaire pour bool

// Define the size of ADC channels buffer
#define BUFFER_SIZE 3
#define dataUploadInterval 10000 //ARR (ms)

// Declare global variables
extern uint16_t ADC_Value[BUFFER_SIZE]; // Array to store ADC conversion results
extern bool flag;

// Function declarations
void config_GPIO_ADC_Channnel(void);  // Function to configure GPIO for ADC channels
void config_TIMER3(void);             // Function to configure Timer 3 for ADC trigger
void config_ADC1(void);               // Function to configure ADC1 with DMA
void config_DMA2(void);               // Function to configure DMA2 for ADC1

// Interrupt handler for DMA2 Stream0
void DMA2_Stream0_IRQHandler(void);    // DMA2 Stream0 interrupt handler for ADC

#endif // ADC_H
