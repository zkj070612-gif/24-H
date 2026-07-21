#include "bsp_timer.h"

volatile float my_yaw;

//volatile float kal_mpu_out;
//20ms定时器




void Timer_20ms_Init(void)
{
    //打开20ms定时器
    DL_TimerA_startCounter(TIMER_20ms_INST);
    NVIC_ClearPendingIRQ(TIMER_20ms_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_20ms_INST_INT_IRQN);

}

u8 gled_cnt = 0;
//20ms定时器中断
void TIMER_20ms_INST_IRQHandler(void)
{
    //20ms归零中断触发
	if( DL_TimerA_getPendingInterrupt(TIMER_20ms_INST) == DL_TIMER_IIDX_ZERO )
	{
		//编码器更新
//				encoder_update();
//        Motion_Handle(); //小车测速
	
        gled_cnt++;
        if(gled_cnt>=10)
        {
            gled_cnt=0;
            DL_GPIO_togglePins(LED_PORT,LED_MCU_PIN);
        }

	}

}


//void TIMER_1ms_INST_IRQHandler(void)
//{
//    switch( DL_TimerG_getPendingInterrupt(TIMER_1ms_INST) )
//    {
//        case DL_TIMER_IIDX_ZERO://如果是0溢出中断  If it is a 0 overflow interrupt
////            Buzzer_Handle();
//            systick_counter++; // 每1ms自动+1      +1 per sencond
//            break;

//        default:
//            break;
//    }
//    
//}

//uint32_t Get_Time(void)    
//{
//    return systick_counter;
//}

