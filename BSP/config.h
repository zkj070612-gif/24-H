#ifndef __CONGIF_H_
#define __CONGIF_H_


#include "AllHeader.h"


#define COMPILE_ENT 0  //编译环境,1:CCS 0:keil  此处和printf相关环境不对,无法打印

#define Motor_Switch (1)

/*
 * 1: run the HC-06 BLE UART1 echo/OLED test.
 * 0: run the original H-track competition program.
 */
#define BLE_UART1_TEST_MODE (0)
#define MAGNET_MAX_ON_MS (3000U)
#define IRRMOTE (0) //红外遥控打开开关





#endif

