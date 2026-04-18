#include "USART.h"


void USART3_Init(void)
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB clock(bit 1 AHB1ENR)

    // Set PB10 (TX) and PB11 (RX) to alternate function (AF)
    GPIOB->MODER &= ~( (3 << (2 * 10)) | (3 << (2 * 11)) );   // Clear mode bits for PB10 and PB11
    GPIOB->MODER |= ( (2 << (2 * 10)) | (2 << (2 * 11)) );    // Set PB10 and PB11 to alternate function (AF)

    // Set AF7 (USART3) for PB10 and PB11
    GPIOB->AFR[1] &= ~( (0xF << (4 * (10 - 8))) | (0xF << (4 * (11 - 8))) );  // Clear AF bits
    GPIOB->AFR[1] |= ( (7 << (4 * (10 - 8))) | (7 << (4 * (11 - 8))) );       // Set AF7 (USART3) for PB10 and PB11
    //AF7 is the alternate function assigned to USART3_TX and USART3_RX on PB10 (transmit) and PB11 (receive), respectively.
		
	  //Config USART3		
		RCC->APB1ENR |= 1<<18; // Enable USART3 clock
		//USART3->BRR=834; 	// for 8 MHZ and 9600 Baudrate 
		USART3->BRR=16000000 / 115200; 	// for 16 MHZ and 9600 Baudrate 
		USART3->CR1 |= 1<<3; 	// Enable TE
	  USART3->CR1 |= 1<<2; 	// Enable RE
		USART3->CR1 |= 1<<5; 	// Enable RXNEIE
		USART3->CR1 |= 1<<4; 	// Enable IDLEIE
	  USART3->CR1 |= 1<<13; // Enable USART3
	  NVIC_EnableIRQ(USART3_IRQn);    // Enable USART3 global interrupt
		
}
// Function to send a character via USART3
void SendChar_usart3(char C)
{
	while((USART3->SR & 0x80) == 0){}// Check if the Transmit Data Register (TDR) is empty
  USART3->DR=C;  // Load the character into the Data Register (DR) for transmission
   
}


// Function to send a string via USART3
void SendTxt_usart3(char *ADR)
{
		while(*ADR)
		{
		SendChar_usart3(*ADR);
		ADR++;
		}
}


void USART2_Init(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA clock (bit 0 in AHB1ENR)

    // Set PA2 (TX) and PA3 (RX) to alternate function (AF)
    GPIOA->MODER &= ~( (3 << (2 * 2)) | (3 << (2 * 3)) );   // Clear mode bits for PA2 and PA3
    GPIOA->MODER |= ( (2 << (2 * 2)) | (2 << (2 * 3)) );    // Set PA2 and PA3 to alternate function (AF)

    // Set AF7 (USART2) for PA2 and PA3
    GPIOA->AFR[0] &= ~( (0xF << (4 * 2)) | (0xF << (4 * 3)) );  // Clear AF bits
    GPIOA->AFR[0] |= ( (7 << (4 * 2)) | (7 << (4 * 3)) );       // Set AF7 (USART2) for PA2 and PA3

    // Configure USART2
    RCC->APB1ENR |= (1 << 17);  // Enable USART2 clock (bit 17 in APB1ENR)
    
    USART2->BRR = 16000000 / 115200;         // Baud rate 9600 for 16 MHz clock (BRR = 16000000 / 9600)
    
    USART2->CR1 |= (1 << 3);    // Enable transmitter (TE)
    USART2->CR1 |= (1 << 2);    // Enable receiver (RE)
    USART2->CR1 |= (1 << 5);    // Enable RXNE interrupt (RXNEIE)
    USART2->CR1 |= (1 << 4);    // Enable IDLE interrupt (IDLEIE)
    USART2->CR1 |= (1 << 13);   // Enable USART2 (UE)
    
    NVIC_EnableIRQ(USART2_IRQn);  // Enable USART2 global interrupt
}

// Function to send a character via USART2
void SendChar_usart2(char C)
{
	while ((USART2->SR & 0x80) == 0) {} // Wait until the Transmit Data Register (TDR) is empty
	USART2->DR = C;                     // Load the character into the Data Register for transmission
}

// Function to send a string via USART2
void SendTxt_usart2(char *ADR)
{
	while (*ADR)
	{
		SendChar_usart2(*ADR);           // Send each character
		ADR++;                           // Move to the next character
	}
}

void USART2_IRQHandler()
{
    // Check if RXNE (Receive Not Empty) flag is set
    if ((USART2->SR & 0x20) != 0)  
    {
        if (j < TX_RX_BUFFER_SIZE - 1) 
        {
				j++;  
				RX2_Buffer[j-1] = USART2->DR;  
				}
    }
    
    // Check if IDLE flag is set (indicates end of transmission)
    if ((USART2->SR & 0x10) != 0)  
    {
        SendTxt_usart3(RX2_Buffer); 
				memcpy(CpyBuffer, RX2_Buffer, TX_RX_BUFFER_SIZE);
				ClearBuffer(RX2_Buffer, sizeof(RX2_Buffer));
        j = 0;                       
        USART2->SR;                  
        USART2->DR;                 
    }
}



