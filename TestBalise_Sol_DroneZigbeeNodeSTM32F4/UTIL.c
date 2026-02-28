#include "UTIL.h"

// Function to clear a buffer
void ClearBuffer(char *buffer, int size) 
{
    for (int j = 0; j < size; j++) {

			buffer[j] = '\0';
    }
}	

void delay_ms(uint32_t ms) {
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Load the value for 1ms delay
    SysTick->VAL = 0; // Reset the current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick

    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // Wait for the overflow flag
    }

    SysTick->CTRL = 0; // Disable SysTick after usage
}
//IWDG
void IWDG_Init(void)
{
	IWDG->KR	|=	0x5555;
	IWDG->PR	|=	0x06;
	IWDG->RLR	=	IWDG_TimeOut * (32000/256)/1000;
	IWDG->KR	|=	0xAAAA;
	IWDG->KR	|=	0xCCCC;
}
void IWDG_Refresh(void)
{
	//SendTxt_usart2("\t#################### System REFRESHING ####################\n");
	IWDG->KR	|=	0xAAAA;
}

void enter_sleep_mode(void)
{
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;  // Sleep mode 
    //SCB->SCR &= ~(1<<2); //sleepdeep bit =0
    __WFI();  // Wait For Interrupt
}


