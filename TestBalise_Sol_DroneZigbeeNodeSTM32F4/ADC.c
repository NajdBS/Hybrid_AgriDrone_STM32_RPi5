#include "ADC.h"

void config_GPIO_ADC_Channnel(void)
{
    RCC->AHB1ENR |= 0x1;   // Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 2); // Enable GPIOC clock (for PC4 and PC5)

    // Configure PA1, PC4, and PC5 as Analog Input
    GPIOA->MODER |= (0x3 << 2*1);    // PA1 as Analog (00: Analog mode)
    GPIOC->MODER |= (0x3 << 2*4);    // PC4 as Analog (00: Analog mode)
    GPIOC->MODER |= (0x3 << 2*5);    // PC5 as Analog (00: Analog mode)
}

void config_TIMER3(void)
{
    // Enable clock for Timer 3 (APB1)
    RCC->APB1ENR |= (1 << 1); //enable TIM3
	
    // Configure the prescaler (PSC) and auto-reload value (ARR)
    TIM3->PSC = 15999;     // Prescaler: 16 MHz / (15999 + 1) = 1 kHz
    TIM3->ARR = dataUploadInterval;       // Auto-reload value for 1-second interval
	  
		// Configure TRGO (Trigger Output) to send an "Update Event" signal to ADC
		TIM3->CR2 &= ~(7 << 4); // Clear MMS bits (bits 6:4)
		TIM3->CR2 |= (2 << 4);  // Set MMS = 010 (Update Event as TRGO)
		//TIM3->CR1 |= (1 << 7);
		// Start Timer 3
    TIM3->CR1 |= 0x1;
}


void config_ADC1(void)
{
    // Enable ADC1 clock
    RCC->APB2ENR |= (1 << 8);    // Enable ADC1 clock

    // Configure ADC1
    ADC1->CR1 |= (1 << 8);       // SCAN mode enabled (multi-channel)
    ADC1->CR1 |= (1 << 11);      // Discontinuous mode enabled (DISCEN)
    ADC1->CR1 &= ~(0b111 << 13); // Clear [15:13] bits DISCNUM 
    ADC1->CR1 |= (2 << 13);      // Set DISCNUM = 010 (3 channel)

    // Configure sequence for channels 1, 14, and 15
    ADC1->SQR1 &= ~(0xF << 20);  // Clear L bits [23:20]
    ADC1->SQR1 |= 2 << 20;       // Set L = 2 (Total 3 conversions: L+1 = 3)
    ADC1->SQR3 |= (1 << 0) | (14 << 5) | (15 << 10); // Configure channels 1, 14, and 15
    
    // Configure sampling time for the channels
    // ADC1->SMPR2 &= ~((0b111 << 3)); // Clear [5:3] bits, N1 = 3
    // ADC1->SMPR1 &= ~((0b111 << 12) | (0b111 << 15)); // Clear [5:3] bits, N1 = 3
		// Sampling time: tconv = (3+12) * (1/8) ~= 2µs / PCLK2 = 2

    // Configure ADC/DMA
    ADC1->CR2 |= (1 << 8) | (1 << 9);  // Enable DMA Mode and Continuous DMA Requests

    // Select External Trigger Source: TIM3_TRGO (EXTSEL = 1000)
    ADC1->CR2 &= ~(15 << 24);  // Clear EXTSEL bits [27:24]
    ADC1->CR2 |= (8 << 24);    // Set EXTSEL = 1000 (TIM3_TRGO)
    ADC1->CR2 &= ~(3 << 28);   // Clear EXTEN bits [29:28]
    ADC1->CR2 |= (1 << 28);    // Set EXTEN = 01 (Trigger on rising edge)
    
    // Enable ADC
    ADC1->CR2 |= 1;            // Turn on ADC
}

void config_DMA2(void)
{
		RCC->AHB1ENR |= 1<<22;// Enable DMA2 clock
	
		DMA2_Stream0->CR &= ~(0b111);// Select Channel 0 for ADC1
		//DMA2_Stream0->CR &= ~(0b11<<16);// Low Level Priority
		DMA2_Stream0->CR |= 1<<8;// Enable circular mode
		
		DMA2_Stream0->CR |= 1<<13;// Memory data size 16-bit
		DMA2_Stream0->CR |= 1<<11;// Peripheral data size 16-bit
		//DMA2_Stream0->CR &= ~(1<<6);// Data direction : Peripheral to Memory
		DMA2_Stream0->CR |= 1<<10;// Memory increment
		DMA2_Stream0->CR |= 1<<4;// Enable TCIE
		//DMA2_Stream0->CR &= ~(1<<18);// DBM=0;
		
		DMA2_Stream0->NDTR = 3;// Number of data to transfer
		DMA2_Stream0->M0AR = (uint32_t)&ADC_Value[0];// Set memory address : Address destination
		DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;// Set peripheral address : Address source
		
		//DMA2_Stream0->FCR &=~ (1<<2);// Enable Direct Mode and disable FIFO
		NVIC_EnableIRQ(DMA2_Stream0_IRQn);   // Enable DMA2_Stream0 global interrupt	
		DMA2_Stream0->CR |= 1<<0;// Enable STREAM   
}

void DMA2_Stream0_IRQHandler(void) {
		if (DMA2->LISR & (1 << 5)) // Check if Transfer Complete (TCIF0) is set
		{  
		flag=true;
		DMA2->LIFCR |= (1 << 5);// Clear the interrupt flag
    }
}


