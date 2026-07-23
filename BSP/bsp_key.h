#ifndef __BSP_KEY_H_
#define __BSP_KEY_H_

#include "AllHeader.h"


// 按键状态，与实际电平相反。
#define KEY_PRESS      1
#define KEY_RELEASE    0

#define KEY_MODE_ONE_TIME   1
#define KEY_MODE_ALWAYS     0


#define KEY1_PRES      1
#define KEY2_PRES      2
#define KEY3_PRES      3
 

uint8_t Key1_State(uint8_t mode);
uint8_t Key1_Long_Press(uint16_t timeout);

uint8_t Key2_State(uint8_t mode);
uint8_t Key2_Long_Press(uint16_t timeout);

uint8_t Key3_State(uint8_t mode);
uint8_t Key3_Long_Press(uint16_t timeout);

uint8_t KEY_Scan(void);
uint8_t get_key();
#endif
