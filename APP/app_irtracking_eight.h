#ifndef __APP_IRTRACKING_H__
#define __APP_IRTRACKING_H__




#include "AllHeader.h"

#define BLACK_IR       1        //榛戠嚎black
#define WHITE_IR       0        //鐧界嚎white


#define IRR_SPEED_LIMIT 			  30 
#define u8 uint8_t
#define u16 uint16_t
extern u8 x1,x2,x3,x4,x5,x6,x7,x8;
int LineCheck(void);
//void Line_Tracke(void);

int LineStop();


void deal_IRdata(u8 *x1,u8 *x2,u8 *x3,u8 *x4,u8 *x5,u8 *x6,u8 *x7,u8 *x8);

void Copy_HD_Data();
void Line_Tracke(int state);
//void LineWalking(int line_l,int line_r,int state);

#endif

