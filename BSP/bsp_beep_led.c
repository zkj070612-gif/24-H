#include "bsp_beep_led.h"


void OPEN_MCULED(void)
{
    DL_GPIO_setPins(LED_PORT,LED_MCU_PIN);
}

void CLOSE_MCULED(void)
{
    DL_GPIO_clearPins(LED_PORT,LED_MCU_PIN);
}


void Buzzer_open_state(void)
{
    DL_GPIO_setPins(BEEP_PORT,BEEP_Buzzer_PIN);
}

void Buzzer_close_state(void)
{
    DL_GPIO_clearPins(BEEP_PORT,BEEP_Buzzer_PIN);
}


void BEEP_RGB()
{
	if(RGB_BEEP_flag==1)
	{
	Buzzer_open_state();
	Control_RGB_ALL(Cyan_RGB);
	
	}
	else
	{
	
		Buzzer_close_state();
	 Control_RGB_ALL(OFF);
//		delay_ms(10);
	
	
	}





}