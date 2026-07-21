#include "bsp_ir_receiver.h"

#include "bsp_motor.h"


extern int CarSpeedControl ;






typedef struct INFRARED_DATA{
    
    uint8_t AddressCode;            //地址码
    uint8_t AddressInverseCode;     //地址反码
    uint8_t CommandCode;            //命令码
    uint8_t CommandInverseCode;     //命令反码
    
}_INFRARED_DATA_STRUCT_;


_INFRARED_DATA_STRUCT_ InfraredData;

//红外初始化
void infrared_config(void)
{     
    #if IRRMOTE && Motor_Switch==0
        NVIC_EnableIRQ(GPIOB_INT_IRQn);//开启按键引脚的GPIOB端口中断
    #endif
}


//获取红外低电平时间
//以微秒us作为时间参考
void get_infrared_low_time( uint32_t *low_time )
{
    uint32_t time_val = 0;

    while( GET_OUT == 0 )
    {
        if( time_val>= 500 ) 
        {
            *low_time = time_val;
            return;
        } 
        delay_us(20);           
        time_val++;
    }
    *low_time = time_val;
}

//获取红外高电平时间
//以微秒us作为时间参考
void get_infrared_high_time(uint32_t *high_time)
{
    uint32_t time_val = 0;
    while( GET_OUT == 1 )
    {
        if( time_val >= 250 ) 
        {
            *high_time = time_val;
            return;
        } 
        delay_us(20);
        time_val++;
    }
    *high_time = time_val;
}

/******************************************************************
 * 函 数 名 称：guide_and_repeat_code_judgment
 * 函 数 说 明：引导 和 重复 码 判断
 * 函 数 形 参：无
 * 函 数 返 回：1：不是引导码   2：重复码  0：引导码
 * 备       注：以20微秒us作为时间参考
                引导码：由一个 9ms 的低电平和一个 4.5ms 的高电平组成
                重复码：由一个 9ms 的低电平和一个 2.5ms 的高电平组成
******************************************************************/
uint8_t guide_and_repeat_code_judgment(void)
{
    uint32_t out_time=0;
    get_infrared_low_time(&out_time);
    //time>10ms             time <8ms
    if((out_time > 500) || (out_time < 400))
    {
        return 1;
    } 
    get_infrared_high_time(&out_time);
    // x>5ms  或者 x<2ms
    if((out_time > 250) || (out_time < 100))
    {
        return 1;
    } 
    
    //如果是重复码  2ms < time < 3ms 
    if((out_time > 100) && (out_time < 150))
    {
        return 2;
    } 
    
    return 0;
}

//红外数据是否正确判断
uint8_t infrared_data_true_judgment(uint8_t *value)
{
    //判断地址码是否正确
    if( value[0] != (uint8_t)(~value[1]) )  return 0;
    //判断命令码是否正确
    if( value[2] != (uint8_t)(~value[3]) )  return 1;


    printf("%x %x %x %x\r\n",value[0],value[1],value[2],value[3]);
    //保存正确数据
    InfraredData.AddressCode        = value[0];
    InfraredData.AddressInverseCode = value[1];
    InfraredData.CommandCode        = value[2];
    InfraredData.CommandInverseCode = value[3];
        
        return 0;
}

//接收红外数据
uint8_t receiving_infrared_data()
{
    uint16_t group_num = 0,data_num = 0;
    uint32_t time=0;
    uint8_t bit_data = 0;
    uint8_t ir_value[5] = {0};
    
    uint8_t guide_and_repeat_code = 0;
    
    //等待引导码
    guide_and_repeat_code = guide_and_repeat_code_judgment();
    //如果不是引导码则结束解析
    if(  guide_and_repeat_code == 1 ) 
    {
//        printf("err\r\n");
//        return ;
    }

    //共有4组数据
    //地址码+地址反码+命令码+命令反码
    for(group_num = 0; group_num < 4; group_num++ )                
        {                                
        //接收一组8位的数据
        for( data_num = 0; data_num < 8; data_num++ )        
        {
            //接收低电平
            get_infrared_low_time(&time);
            //如果不在0.56ms内的低电平,数据错误
            if((time > 60) || (time < 20))
            {
//                return ;
            }
            
            time = 0;
            //接收高电平
            get_infrared_high_time(&time);
            //如果是在1200us<t<2000us范围内则判断为1
            if((time >=60) && (time < 100))   
            {
                bit_data = 1;
            }
            //如果是在200us<t<1000us范围内则判断为0
            else if((time >=10) && (time < 50))  
            {
                bit_data = 0;
            }
                
            //groupNum表示第几组数据
            ir_value[ group_num ] <<= 1;         
            
            //接收的第1个数为高电平;在第二个for循环中，数据会向右移8次
            ir_value[ group_num ] |= bit_data;   
            
            //用完时间要重新赋值        
            time=0;                                
        }
    }
    //判断数据是否正确，正确则保存数据
    infrared_data_true_judgment(ir_value);
		return ir_value[ group_num ];
}

//获取红外发送过来的命令
uint8_t get_infrared_command(void)
{
		return InfraredData.CommandCode;
}
//清除红外发送过来的数据
void clear_infrared_command(void)
{
    InfraredData.CommandCode = 0xFF; //设置一个不是遥控器的键值
}


//因为这里和电机读数有冲突，所以做下防冲突
#if IRRMOTE && Motor_Switch==0
void GROUP1_IRQHandler(void)//Group1的中断服务函数
{
    //读取Group1的中断寄存器并清除中断标志位
    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
    {
        //检查是否是GPIOB端口中断，注意是GPIO_INT_IIDX，不是GPIO_OUT_IIDX 
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            if( GET_OUT == 0 )// 如果是低电平
                    receiving_infrared_data(); //接收一次红外数据
                        
        break;
    }
}
#endif

//void Deal_ir_Motor_Data(void)
//{


//	//小车的前进,后退,左转,右转,停止动作 The forward, backward, left, right, and stop movements of the car
//	
//		switch (receiving_infrared_data())
//		{
//			case run_car:   get_CarState = MOTION_RUN;  break;
//			case back_car:  get_CarState = MOTION_BACK;  break;
//			case left_car:  get_CarState = MOTION_LEFT;  break;
//			case right_car: get_CarState = MOTION_RIGHT;  break;
//			case stop_car:  get_CarState = MOTION_STOP;  break;
//			default: get_CarState = MOTION_STOP; break;
//		}
//	wheel_State(get_CarState,CarSpeedControl);
//		uart0_send_char(receiving_infrared_data());
//}


//void ir_active()
//{
//	
//	receiving_infrared_data();
//	infrared_data_true_judgment();





//}
