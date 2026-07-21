#include "bsp_RGB.h"


unsigned char LedsArray[WS2812_MAX * 3];      //定义颜色数据存储数组
unsigned int  ledsCount   = WS2812_NUMBERS;   //定义实际彩灯默认个数
unsigned int  nbLedsBytes = WS2812_NUMBERS*3; //定义实际彩灯颜色数据个数

// 延时0.25us
void delay_0_25us(void)
{
    //volatile
    for( int i = 0; i < 13; i++); //5:32MHZ  13：80Mhz
}


/******************************************************************
 * 函 数 名 称：rgb_SetColor
 * 函 数 说 明：设置彩灯颜色
 * 函 数 形 参：LedId控制的第几个灯  color颜色数据
 * 函 数 返 回：无
******************************************************************/

void rgb_SetColor(unsigned char LedId, unsigned long color)
{
    if( LedId > ledsCount )
    {
        return;    //to avoid overflow
    }
    LedsArray[LedId * 3]     = (color>>8)&0xff;
    LedsArray[LedId * 3 + 1] = (color>>16)&0xff;
    LedsArray[LedId * 3 + 2] = (color>>0)&0xff;
}

/******************************************************************
 * 函 数 名 称：rgb_SetRGB
 * 函 数 说 明：设置彩灯颜色(三原色设置)
 * 函 数 形 参：LedId控制的第几个灯 red红色数据  green绿色数据  blue蓝色数据
 * 函 数 返 回：无
******************************************************************/
void rgb_SetRGB(unsigned char LedId, unsigned long red, unsigned long green, unsigned long blue)
{
    unsigned long Color=red<<16|green<<8|blue;
    rgb_SetColor(LedId,Color);
}







/******************************************************************
 * 函 数 名 称：rgb_SendArray
 * 函 数 说 明：发送彩灯数据
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void rgb_SendArray(void)
{
    unsigned int i;
    //发送数据
    for(i=0; i<nbLedsBytes; i++)
        Ws2812b_WriteByte(LedsArray[i]);
}

/******************************************************************
 * 函 数 名 称：RGB_LED_Reset
 * 函 数 说 明：复位ws2812
 * 函 数 形 参：无
 * 函 数 返 回：无
******************************************************************/
void RGB_LED_Reset(void)
{
        RGB_PIN_L();              
        delay_us(285);
}

/******************************************************************
 * 函 数 名 称：Ws2812b_WriteByte
 * 函 数 说 明：向WS2812写入单字节数据
 * 函 数 形 参：byte写入的字节数据
 * 函 数 返 回：无
 * 备       注：1码的时序 = 高电平580ns~1us    再低电平220ns~420ns
 *              0码的时序 = 高电平220ns~380ns  再低电平580ns~1us
******************************************************************/
void Ws2812b_WriteByte(unsigned char byte)
{
    int i = 0, j = 0, k = 0;
        for(i = 0; i < 8; i++ )
        {
            if( byte & (0x80 >> i) )//当前位为1
            { 
                RGB_PIN_H();

                //0.75us
                delay_us(1);


                RGB_PIN_L();

                delay_0_25us(); //0.25us
            }
            else//当前位为0
            {
                RGB_PIN_H();
                
                delay_0_25us(); //0.25us
                RGB_PIN_L();

                //0.833us
                delay_us(1);
            }
        }
}
    
     