#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"   // Header for STM32F4
#include "UTIL.h"
#include "string.h"  

#define TX_RX_BUFFER_SIZE 500  // Buffer size
#define CPY_BUFFER_SIZE 500 

extern char CpyBuffer[CPY_BUFFER_SIZE];
extern char RX2_Buffer[TX_RX_BUFFER_SIZE];  
extern int j;

// Initialization function for USART3
void USART3_Init(void);

// Function to send a single character via USART3
void SendChar_usart3(char C);

// Function to send a string via USART3
void SendTxt_usart3(char *ADR);

// Initialization function for USART2
void USART2_Init(void);

// Function to send a single character via USART2
void SendChar_usart2(char C);

// Function to send a string via USART2
void SendTxt_usart2(char *ADR);

// USART2 interrupt handler declaration
void USART2_IRQHandler(void);

#endif // USART_H
