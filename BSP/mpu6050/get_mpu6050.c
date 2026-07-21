#include "get_mpu6050.h"
#include "Filter.h"
#define Pi 3.14159265
short angle[3];
short accel[3];
float pitch,roll,yaw;

volatile float kal_mpu_out;

volatile float Filter_out;

volatile float Filter_out_last;

volatile float Yaw_Err_Lowout_last;
volatile float Yaw_Err_Lowout;

volatile float erro_sumr;
volatile float error;

volatile float gyro_x,gyro_y,gyro_z,accel_z;
volatile float Accel_Angle_X,Accel_Angle_Y;
uint8_t Way_Angle=2; 
Bias_t Angle;

const int CALIB_SAMPLES = 2; // 閲囨牱娆℃暟  Number of samples
float Angle_Balance,Gyro_Balance,Gyro_Turn; //骞宠　鍊捐 骞宠　闄€铻轰华 杞悜闄€铻轰华
float Acceleration_Z;
// 璁＄畻鍋忕Щ閲?  Number of samples
float yawBias = 0, pitchBias = 0, rollBias = 0;

float calibratedYaw = 0, calibratedPitch = 0, calibratedRoll = 0;

void AngleOffsetCalc(void)
{
	
		
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        
			
			
//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
        calibratedYaw += calibratedYaw;

       
//        delay_ms(20);
    }
//    printf("yawsum:%d\r\n", (int)yawSum);
    // 璁＄畻鍋忕Щ閲?  Calculate offset
    calibratedYaw = calibratedYaw / CALIB_SAMPLES;
		
    
//    printf("yawBias:%.2f, pitchBias:%.2f, rollBias:%.2f", yawBias, pitchBias, rollBias);
}
//鑾峰彇宸叉牎鍑嗙殑瑙掑害  Get the calibrated angle
void Get_CalibratedAngles(void)
{
	
	
	
	
    if(Filter_out < -2)
    {
        calibratedYaw = -Filter_out;
    }
    else if(Filter_out >= 2)
    {
        calibratedYaw = 360 - Filter_out; //杞﹀ご鏈濆墠yaw椤烘椂閽堜负璐熸暟  閫嗘椂閽堜负姝ｆ暟 yaw clockwise is negative
    }
    else
    {
        calibratedYaw = 0; 	//杩囨护鎺?鍜?60闄勮繎鐨勬暟鎹? Filter out data near 0 and 360
			
    }

}

float get_Filter(float 	Yaw_Err)	
{
	
	
	float a=0.0150;

	Yaw_Err_Lowout=(1-a)*Yaw_Err+a*Yaw_Err_Lowout_last;//浣垮緱娉㈠舰鏇村姞骞虫粦锛屾护闄ら珮棰戝共鎵?
	Yaw_Err_Lowout_last=Yaw_Err_Lowout;
//			if(error<00.55)
//	{
//		erro_sumr+=error;

//		error=Yaw_Err_Lowout-Yaw_Err_Lowout_last;

//		Yaw_Err_Lowout-=error;
//}
	return Yaw_Err_Lowout;
}




void Get_EulerAngles(void)
{
    //鑾峰彇娆ф媺瑙?Get Euler angles
   


	
//	    for (int i = 0; i < CALIB_SAMPLES; i++)
//    {
         mpu_dmp_get_data(&pitch,&roll,&yaw);
			
			
	  Filter_out=get_Filter(yaw);
			
			
//			calibratedYaw= Filter_out+Filter_out_last;
//			Filter_out_last=Filter_out;
			
			
//        printf("yaw:%.2f, pitch:%.2f, roll:%.2f\r\n", yaw, pitch, roll);
//        calibratedYaw += calibratedYaw;

       
////        delay_ms(20);
////    }
//	  calibratedYaw = calibratedYaw / CALIB_SAMPLES;
	
	



       


	

	
	

//		calibratedYaw=Filter_out+180;

//	  delay_ms(10);//鏍规嵁璁剧疆鐨勯噰鏍风巼锛屼笉鍙缃欢鏃惰繃澶?According to the set sampling rate, the delay cannot be set too large
//	printf("pitch :%3.2f roll :%3.2f yaw:%3.2f\r\n",pitch,roll,Filter_out);

}

//瑙掑害鐜疨ID鎺у埗 Angle ring PID control
float dir_kp = 2.0,dir_ki=0.000,dir_kd = 5.00;
int Integral_Max = 300; //300
int pid_max = 1000; //3000
float Dir_PID(float error)
{ //5*20/1000*188
    float result = 0;
    static int16_t err_last = 0; //涓婃鐨勮宸垵濮嬩负0  Last error
    static float Integral = 0; // 鍒濆鍖栫Н鍒嗛」 Initialize integral term

//   if(error == 0)          
//   {
//       Integral = 0;          //绉垎娓呴浂   Integral cleared
//   }
    Integral += error;           // 鏇存柊绉垎椤癸紝骞惰繘琛岄檺骞?Update the integral term and limit it
    if (Integral > Integral_Max) Integral = Integral_Max;               //绉垎闄愬箙 Integral limiting
    if (Integral < -Integral_Max) Integral = -Integral_Max;             //绉垎闄愬箙 Integral limiting

    // 浣嶇疆寮?PID
    result = dir_kp * error +dir_ki*Integral+ dir_kd * (error - err_last);
  
	 err_last = error;       // 鏇存柊绉垎椤癸紝骞惰繘琛岄檺骞?Update the integral term and limit it

    // 瀵硅緭鍑鸿繘琛岄檺骞匫utput limiting value
    if (result > pid_max) result = pid_max;
    if (result < -pid_max) result = -pid_max;
    return -result;
}
//灏嗚埅鍚戣闄愬埗涓?0-360 搴︼紙闃叉鍥犲姞鍑忚繍绠楀鑷磋埅鍚戣鑼冨洿瓒呰繃 0-360 搴︼級
//Limit the heading_angle to 0-360 degrees(to prevent the range of heading_angle over 0-360 degrees beacuse of Addition or subtraction operations )
float navigetion_0_360_limit(float angle)
{
	while (angle >= 360.0f) angle -= 360.0f;
	while (angle < 0.0f) angle += 360.0f;
	return angle;
}

//璁＄畻 0-360 瀵艰埅鍧愭爣绯讳腑鐨勫皬鍦嗗姬鍋忓樊锛堥€嗘椂閽堟柟鍚戠殑璐熷渾寮ц搴︿负姝ｏ紝椤烘椂閽堟柟鍚戠殑涓鸿礋锛?
//Calculate the minor arc deviation in the 0-360 Navigation Coordinate System (Counterclockwise negative arc Angle is Positive, Clockwise is Negative)  
//璁＄畻 0-360 瀵艰埅鍧愭爣绯讳腑鐨勫皬鍦嗗姬鍋忓樊锛堥€嗘椂閽堟柟鍚戠殑璐熷渾寮ц搴︿负姝ｏ紝椤烘椂閽堟柟鍚戠殑涓鸿礋锛?
//Calculate the minor arc deviation in the 0-360 Navigation Coordinate System (Counterclockwise negative arc Angle is Positive, Clockwise is Negative)  
float get_minor_arc(float azimuth,float headingAngle)  
{
    float angle_err = 0.0; 
    if(azimuth >= 180 + headingAngle)  
    {
        angle_err = azimuth - headingAngle - 360;  
    }
    else if(headingAngle > 180 + azimuth)
    {
        angle_err = azimuth - headingAngle + 360;
    }
    else
    {
        angle_err =  azimuth - headingAngle;
    }
    return -angle_err;
}


 /**************************************************************************
Function: Get angle
Input   : way锛歍he algorithm of getting angle 1锛欴MP  2锛歬alman  3锛欳omplementary filtering
Output  : none
鍑芥暟鍔熻兘锛氳幏鍙栬搴?
鍏ュ彛鍙傛暟锛歸ay锛氳幏鍙栬搴︾殑绠楁硶 1锛欴MP  2锛氬崱灏旀浖 3锛氫簰琛ユ护娉?
杩斿洖  鍊硷細鏃?
**************************************************************************/	
void Get_Angle(uint8_t way)
{ 

	if(way==1)                           //DMP鐨勮鍙栧湪	鏁版嵁閲囬泦涓柇璇诲彇锛屼弗鏍奸伒寰椂搴忚姹?
	{	
		if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
        { 
          //  printf("\r\npitch =%d\r\n", (int)pitch);
           // printf("\r\nroll =%d\r\n", (int)roll);
//            printf("\r\nyaw =%3.2f\r\n", yaw);
        }      
        delay_ms(10);//鏍规嵁璁剧疆鐨勯噰鏍风巼锛屼笉鍙缃欢鏃惰繃澶?
	}			
	else
	{
		
		MPU6050ReadGyro(angle);
		MPU6050ReadAcc(accel);
		Accel_Angle_X=atan2(accel[0],accel[2])*180/Pi;     //璁＄畻鍊捐锛岃浆鎹㈠崟浣嶄负搴?
		Accel_Angle_Y=atan2(accel[1],accel[2])*180/Pi;     //璁＄畻鍊捐锛岃浆鎹㈠崟浣嶄负搴?
		accel_z=accel[2]*1.962/32768;
		gyro_z = angle[2] * 2000 / 32768; // 闄€铻轰华閲忕▼杞崲
		delay_ms(10);
		if(Way_Angle==2)		  	
		{
			pitch= -Kalman_Filter_x(Accel_Angle_X,gyro_x);//鍗″皵鏇兼护娉?
			roll = -Kalman_Filter_y(Accel_Angle_Y,gyro_y);


		}
		else if(Way_Angle==3) 
		{  
			 pitch = -Complementary_Filter_x(Accel_Angle_X,gyro_x);//浜掕ˉ婊ゆ尝
			 roll = -Complementary_Filter_y(Accel_Angle_Y,gyro_y);
		}
		Angle_Balance=pitch;                              //鏇存柊骞宠　鍊捐
		Gyro_Turn=angle[2];                                 //鏇存柊杞悜瑙掗€熷害
		Acceleration_Z=accel[2];                           //鏇存柊Z杞村姞閫熷害璁?
		
	}

} 


