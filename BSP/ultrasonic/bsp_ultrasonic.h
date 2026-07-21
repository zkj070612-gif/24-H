#ifndef _BSP_ULTRASONIC_H_
#define _BSP_ULTRASONIC_H_

#include "AllHeader.h"

#define SR04_TRIG(x)  ( x ? DL_GPIO_setPins(SR04_PORT,SR04_TRIG_PIN) : DL_GPIO_clearPins(SR04_PORT,SR04_TRIG_PIN) )
#define SR04_ECHO()   ( ( ( DL_GPIO_readPins(SR04_PORT,SR04_ECHO_PIN) & SR04_ECHO_PIN ) > 0 ) ? 1 : 0 )

void Ultrasonic_Init(void);//超声波初始化 



uint32_t Get_Time(void)   ;
#endif

