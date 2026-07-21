#include "bsp_ultrasonic.h"



volatile unsigned int msHcCount = 0;//ms计数
volatile float distance = 0;


volatile uint32_t systick_counter = 0;

/******************************************************************
 * 函 数 名 称：bsp_ultrasonic
 * 函 数 说 明：超声波初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：TRIG引脚负责发送超声波脉冲串
******************************************************************/
void Ultrasonic_Init(void)
{
    //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_1ms_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_1ms_INST_INT_IRQN);
		DL_TimerG_startCounter(TIMER_1ms_INST); 
        
}
/******************************************************************
 * 函 数 名 称：Open_Timer
 * 函 数 说 明：打开定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：
******************************************************************/
void Open_Timer(void)
{
        
    DL_TimerG_setTimerCount(TIMER_1ms_INST, 0);   // 清除定时器计数  
        
    msHcCount = 0;  
        
    DL_TimerG_startCounter(TIMER_1ms_INST);   // 使能定时器
}

/******************************************************************
 * 函 数 名 称：Get_TIMER_Count
 * 函 数 说 明：获取定时器定时时间
 * 函 数 形 参：无
 * 函 数 返 回：数据
 * 备       注：
******************************************************************/
uint32_t Get_TIMER_Count(void)
{
    uint32_t time  = 0;  
    time   = msHcCount*1000;                         // 得到us 
    time  += DL_TimerG_getTimerCount(TIMER_1ms_INST);  // 得到ms 
        
    DL_TimerG_setTimerCount(TIMER_1ms_INST, 0);   // 清除定时器计数  
    delay_ms(1);
    return time ;          
}

/******************************************************************
 * 函 数 名 称：Close_Timer
 * 函 数 说 明：关闭定时器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：
******************************************************************/
void Close_Timer(void)
{
    DL_TimerG_stopCounter(TIMER_1ms_INST);     // 关闭定时器 
}


/******************************************************************
 * 函 数 名 称：TIMER_0_INST_IRQHandler
 * 函 数 说 明：定时器中断服务函数
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：1ms进入一次
******************************************************************/
//void TIMER_1ms_INST_IRQHandler(void)
//{
//    //如果产生了定时器中断
//    switch( DL_TimerG_getPendingInterrupt(TIMER_1ms_INST) )
//    {
//        case DL_TIMERA_IIDX_LOAD: //重加载
//                msHcCount++;
//            break;
//				
//				
////				case DL_TIMERG_IIDX_ZERO:
////				 systick_counter++; // 每1ms自动+1      +1 per sencond
////            break;
////				
//        
//        default://其他的定时器中断
//            break;
//    }
//}

void TIMER_1ms_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_1ms_INST) )
    {
        case DL_TIMERA_IIDX_LOAD: //重加载
                msHcCount++;
            break;
				
				
//				case DL_TIMERG_IIDX_ZERO:
//				 systick_counter++; // 每1ms自动+1      +1 per sencond
//            break;
				
        
        default://其他的定时器中断
            break;
    }
}



uint32_t Get_Time(void)    
{
    return msHcCount;
}




