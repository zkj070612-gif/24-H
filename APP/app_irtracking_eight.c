#include "app_irtracking_eight.h"

#define IRTrack_Trun_KP (-3.60)
#define IRTrack_Trun_KI (-0.000000)  // 0.01
#define IRTrack_Trun_KD (-0.8000)
#define IRR_TURN_LIMIT 31
/* The YB-CVT01 motor channels turn opposite to the original tracking
 * template's differential polarity. Keep this switch explicit so it can be
 * changed without retuning the sensor weights or PID gains. */
#define IRR_STEERING_POLARITY (-1)


u8 x1,x2,x3,x4,x5,x6,x7,x8;
int pid_output_IRR = 0;
u8 trun_flag = 0;


 int8_t err = 0;
#define IRR_SPEED 			  73  //巡线速度
#define IRTrack_Minddle    0 //巡线中值



float  Track_PID(int8_t actual_value)
{
    float IRTrackTurn = 0;
	int8_t error;
	static int8_t error_last=0;
	static float IRTrack_Integral;
	

	error=actual_value-IRTrack_Minddle;
	
	IRTrack_Integral +=error;
    
    if(IRTrack_Integral>100)
        IRTrack_Integral =100;
    else if(IRTrack_Integral<-100)
        IRTrack_Integral = -100;
	
	//	pid
	IRTrackTurn=error*IRTrack_Trun_KP
							+IRTrack_Trun_KI*IRTrack_Integral
							+(error - error_last)*IRTrack_Trun_KD;
	error_last = error;
	return IRTrackTurn;

}

////检测现在位于黑线还是在白线上	Detection is now on the black line or on the white line



void Copy_HD_Data()
{

			x1 =IR_Data_number[0];
			x2 =IR_Data_number[1]; 
			x3 =IR_Data_number[2];
			x4 =IR_Data_number[3];
			x5 =IR_Data_number[4];
			x6 =IR_Data_number[5];
			x7 =IR_Data_number[6];
			x8 =IR_Data_number[7];
	
	
//		OLED_ShowString(0,25,(uint8_t *)"IR:",8,1);
//	
//		OLED_ShowNum(20 + 1 * 10, 20, x1, 1, 8, 1);    
//		OLED_ShowNum(20 + 2 * 10, 20, x2, 1, 8, 1);    
//		OLED_ShowNum(20 + 3 * 10, 20, x3, 1, 8, 1);
//		OLED_ShowNum(20 + 4 * 10, 20, x4, 1, 8, 1);
//		OLED_ShowNum(20 + 5 * 10, 20, x5, 1, 8, 1);
//		OLED_ShowNum(20 + 6 * 10, 20, x6, 1, 8, 1); 
//		OLED_ShowNum(20 + 7 * 10, 20, x7, 1, 8, 1); 
//		OLED_ShowNum(20 + 8 * 10, 20, x8, 1, 8, 1);   	
//		OLED_Refresh();
}
int LineCheck(void)
{
  
	IR_DATA();
	Copy_HD_Data();
	
	if(x1==1&& x2==1&& x3==1&& x4==1&& x5==1&& x6==1&& x7==1 && x8==1)
	{
		return WHITE_IR;
	}
	else if(x1+x2+x3+x4+x5+x6+x7+x8<8)
	{
		return BLACK_IR;
	}
	return WHITE_IR;
}


void Line_Tracke(int state) {
//	

	
		IR_DATA();
	Copy_HD_Data();
	



    int weights[8] = {-20, -15, -10, -5, 5, 10, 15, 20};
    int weighted_sum = 0;
    int sensor_active_count = 0;

    // 八路模块数字量协议：0 表示检测到黑线，1 表示白底。
    if (x1 == 0) { weighted_sum += weights[0]; sensor_active_count++; }
    if (x2 == 0) { weighted_sum += weights[1]; sensor_active_count++; }
    if (x3 == 0) { weighted_sum += weights[2]; sensor_active_count++; }
    if (x4 == 0) { weighted_sum += weights[3]; sensor_active_count++; }
    if (x5 == 0) { weighted_sum += weights[4]; sensor_active_count++; }
    if (x6 == 0) { weighted_sum += weights[5]; sensor_active_count++; }
    if (x7 == 0) { weighted_sum += weights[6]; sensor_active_count++; }
    if (x8 == 0) { weighted_sum += weights[7]; sensor_active_count++; }

        if (sensor_active_count > 0) {
        // 计算平均误差，减少传感器数量变化的干扰
        err = weighted_sum / sensor_active_count;
    } 
				
		else {
        // 所有传感器都未检测到黑线，进入寻线模式
        if (err > 0) {
            err = 20; // 上次偏右，则继续右转寻找
        } else if (err < 0) {
            err = -20; // 上次偏左，则继续左转寻找
        } else {
            err = 0; 
					
        }
    }
	 
			if(state==1)
		{
		
	  pid_output_IRR = (int)(Track_PID(err) * IRR_STEERING_POLARITY);
		if (pid_output_IRR > IRR_TURN_LIMIT) pid_output_IRR = IRR_TURN_LIMIT;
		if (pid_output_IRR < -IRR_TURN_LIMIT) pid_output_IRR = -IRR_TURN_LIMIT;
		
		Set_PID_Motor(IRR_SPEED+pid_output_IRR ,IRR_SPEED-pid_output_IRR,0);
		}
		else 
		{
			
				IR_DATA();
			Copy_HD_Data();
	

		
		}

}




