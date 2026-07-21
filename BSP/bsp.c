#include "bsp.h"

//选择是否在初始化完成后，小车自主向前走，0为否，1为是(自主避让案例使用)
//Select whether the car moves forward autonomously after initialization is completed, 0 for no, 1 for yes (used in autonomous avoidance cases)

int Car_Auto_Drive = 0;

void board_init(void)
{
	// SYSCFG初始化
	SYSCFG_DL_init();
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
}


void bsp_init(void)
{
    board_init();


    //模块adc采集
 
//    //红外接收初始化
    infrared_config();
	 
    OLED_Init();  //初始化OLED
//	
		delay_ms(500);
	
	

	
		MPU6050_Init();
	
	
		DMP_Init();
	
		encoder_init();
		Init_Motor_PWM();
	   Ultrasonic_Init();
 
		

//	
		PID_Param_Init();

	
		
		

		W25Q64_Init();
    

//    
//    
//    //超声波

 
//    

//    




}
