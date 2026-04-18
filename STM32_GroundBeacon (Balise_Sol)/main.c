#include "stm32f4xx.h"
#include <stdio.h>       
#include "string.h"
#include "USART.h"
#include "UTIL.h"
#include "ADC.h"
#include "I2C.h"

#define CMD_BUFFER_SIZE 500
#define DATA_BUFFER_SIZE 500

// Buffers
char cmdBuffer[CMD_BUFFER_SIZE];
char RX2_Buffer[TX_RX_BUFFER_SIZE];
char DataBuffer[DATA_BUFFER_SIZE];
char CpyBuffer[DATA_BUFFER_SIZE];

int j = 0;
//Var
bool flag = false;
float temperature=0;
uint16_t ADC_Value[BUFFER_SIZE];



int main(void)
{
    USART2_Init();
    USART3_Init();
    IWDG_Init();
		config_GPIO_ADC_Channnel();
		config_ADC1();  
		config_TIMER3();
		config_DMA2();
		I2C1_Init();
	  //DS1621_Init(); // Uncomment this line when the DS1621 sensor is connected
    while (1) 
    {
    if (flag == true){
		//temperature = DS1621_Read_Temp (); // Uncomment this line when the DS1621 sensor is connected
		sprintf(DataBuffer, "CH1=%d CH2=%d CH3=%d Temp_DS1621=%.2f C\r\n", ADC_Value[0], ADC_Value[1], ADC_Value[2], temperature);
		SendTxt_usart2(DataBuffer);
		flag = false;
		}
		// Refresh watchdog periodically
    IWDG_Refresh();
		
		SendTxt_usart3("Entering Sleep Mode...\r\n");
	 
		// Enter mode Sleep
    enter_sleep_mode();
	
	  SendTxt_usart3("Woke up from Sleep Mode.\r\n");
    }
}




