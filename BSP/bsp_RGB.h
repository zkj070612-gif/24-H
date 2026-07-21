#ifndef _BSP_WS2812_H_
#define _BSP_WS2812_H_
 
#include "AllHeader.h"

#define WS2812_MAX        2   //彩灯最大个数
#define WS2812_NUMBERS    2   //彩灯个数
 

//用户修改参数区
//#define WS2812_FREQUENCY
#define RGB_PIN_L()       DL_GPIO_clearPins(RGB_PORT, RGB_WQ2812_PIN)  //控制彩灯引脚（需要配置为强推挽输出）
#define RGB_PIN_H()       DL_GPIO_setPins(RGB_PORT, RGB_WQ2812_PIN)    //控制彩灯引脚（需要配置为强推挽输出）



#define RED               0xff0000                  //红色
#define GREEN             0x00ff00                  //绿色
#define BLUE              0x0000ff                  //蓝色
#define YELLOW            0xffff00                  //黄色
#define PURPLE            0xff00ff                  //紫色
#define CYAN              0x00ffff                  //青色

#define BLACK             0x000000                  //熄灭
#define WHITE             0xffffff                  //白色



//8.3 -8  0.000000083 
//4.16 -9 0.00000000416
void Ws2812b_WriteByte(unsigned char byte);//发送一个字节数据（@12.000MHz,理论每个机器周期83ns,测试约为76ns）                                                      
void setLedCount(unsigned char count);//设置彩灯数目，范围0-25.                                                           
unsigned char getLedCount();//彩灯数目查询函数                                                                     
void rgb_SetColor(unsigned char LedId, unsigned long color);//设置彩灯颜色                                     
void rgb_SetRGB(unsigned char LedId, unsigned long red, unsigned long green, unsigned long blue);//设置彩灯颜色
void rgb_SendArray();//发送彩灯数据   

void WS2812_GPIO_Init(void);

void RGB_LED_Write1(void);
void RGB_LED_Reset(void);


void rgb_GetRGB(unsigned char LedId, unsigned long i);

void delay_0_25us(void);

#endif
