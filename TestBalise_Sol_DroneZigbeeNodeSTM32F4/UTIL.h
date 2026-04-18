#ifndef UTILS_H
#define UTILS_H

#include "stm32f4xx.h"  


void ClearBuffer(char *buffer, int size);
void delay_ms(uint32_t ms);

#define IWDG_TimeOut 60000

void IWDG_Init(void);
void IWDG_Refresh(void);
void enter_sleep_mode(void);


#endif // UTILS_H
