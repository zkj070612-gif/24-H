#ifndef _BSP_IR_H_
#define _BSP_IR_H_

#include "AllHeader.h"

#define IR_Num 8

extern volatile u8 IR_recv_complete_flag;
extern volatile u8 IR_Data_number[IR_Num];
extern volatile u8 IR_data_parsed_flag;
extern volatile u8 oledbuf[13];

void IR_usart_config(void);
void IRDataAnalysis(void);
void IR_DATA(void);
bool IR_HasValidData(void);
uint32_t IR_LastFrameAgeMs(void);
uint32_t IR_GetFrameCount(void);

#endif
