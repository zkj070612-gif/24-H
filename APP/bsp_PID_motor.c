#include "bsp_PID_motor.h"



PID_TypeDef  veer_pid;

PID_TypeDef  k230_pid;


int8_t veer = 0;

static int get_offset_yaw = 0;

volatile float l_pid_out;
volatile float r_pid_out;

volatile float kal_lpid_out;
volatile float kal_rpid_out;
PID_t pid_motor[2];


volatile	float Yawpid_out ;
//	float SetPoint ;
//volatile	float err;
volatile	float now_yawout;
volatile	int16_t target_yaw;

extern float pitch,roll,yaw;

extern volatile float my_yaw;



float get_yaw = 0;                    //The target heading_Angle of the car movement(杞﹁締杩愬姩鐨勭洰鏍囪埅鍚戣)

float first_yaw = 0;
// YAW鍋忚埅瑙?
//YAW yaw angle
PID pid_Yaw = {0, 0.4, 0, 0.1, 0, 0, 0};

void PID_param_init(PID_TypeDef *pid)
{
	pid->Target = 0;//鐩爣鍊?
	pid->PID_out = 0;
    pid->Kp = 0;
    pid->Ki= 0;
	pid->Kd = 0;
    pid->Err = 0.0f;
    pid->LastErr = 0.0f;
	pid->PenultErr = 0.0f;
    pid->Integral = 0.0f;//绉垎鍊?
	
	pid->KP_polarity = 1;
	pid->KI_polarity = 1;
	pid->KD_polarity = 1;
}



void Set_PID_Veer(void)
{
	PID_TypeDef veerpid;
	PID_param_init(&veerpid);
//	set_pid_target(&veerpid, target_yaw);
	set_p_i_d(&veerpid, veer_p, 0, veer_d);



}
/**
 * @brief       pid闂幆鎺у埗璁＄畻
 * @param       *PID锛歅ID缁撴瀯浣撳彉閲忓湴鍧€
 * @param       CurrentValue锛氬綋鍓嶆祴閲忓€?
 * @retval      鏈熸湜杈撳嚭鍊?
 */
float PID_Calculate(PID_TypeDef *PID,float CurrentValue)
{
	PID->Integral += PID->Err;
	/*绉垎鍒嗙*/
//	if( (PID->Err > 36) || (PID->Err < -36) ){
//		PID->Integral = 0;
//		//PID->PID_out += PID->IntegralConstant * PID->Integral;
//	}
	
    PID->Err =  PID->Target - CurrentValue;
    PID->PID_out = PID->Kp * PID->Err 										/*姣斾緥*/
				 + PID->Ki * PID->Integral  								/*绉垎*/
			     + PID->Kd * (PID->Err - PID->LastErr);						/*寰垎*/
	
	/*绉垎闄愬箙*/
	if(PID->Integral > 1000){
		PID->Integral = 1000;
	}
	if(PID->Integral < -1000){
		PID->Integral = -1000;
	}
	PID->LastErr = PID->Err;
    return PID->PID_out;
}

// 鍒濆鍖朠ID鍙傛暟
//Initialize PID parameters
void PID_Param_Init(void)
{
    /* 閫熷害鐩稿叧鍒濆鍖栧弬鏁?*/
	//Speed dependent initialization parameters
    for (int i = 0; i < MAX_MOTOR; i++)
    {
        pid_motor[i].target_val = 0.0;
        pid_motor[i].pwm_output = 0.0;
        pid_motor[i].err = 0.0;
        pid_motor[i].err_last = 0.0;
        pid_motor[i].err_next = 0.0;
        pid_motor[i].integral = 0.0;

        pid_motor[i].Kp = PID_DEF_KP;
        pid_motor[i].Ki = PID_DEF_KI;
        pid_motor[i].Kd = PID_DEF_KD;
    }

    pid_Yaw.Proportion = PID_YAW_DEF_KP;
    pid_Yaw.Integral = PID_YAW_DEF_KI;
    pid_Yaw.Derivative = PID_YAW_DEF_KD;
}

// Set PID parameters 璁剧疆PID鍙傛暟
void PID_Set_Parm(PID_t *pid, float p, float i, float d)
{
    pid->Kp = p; // Set Scale Factor 璁剧疆姣斾緥绯绘暟 P
    pid->Ki = i; // Set integration coefficient 璁剧疆绉垎绯绘暟 I
    pid->Kd = d; // Set differential coefficient 璁剧疆寰垎绯绘暟 D
}

// Set the target value of PID 璁剧疆PID鐨勭洰鏍囧€?
void PID_Set_Target(PID_t *pid, float temp_val)
{
    pid->target_val = temp_val; // Set the current target value 璁剧疆褰撳墠鐨勭洰鏍囧€?
}

// Obtain PID target value 鑾峰彇PID鐩爣鍊?
float PID_Get_Target(PID_t *pid)
{
    return pid->target_val; // Set the current target value 璁剧疆褰撳墠鐨勭洰鏍囧€?
}

// Incremental PID calculation formula 澧為噺寮廝ID璁＄畻鍏紡
float PID_Incre_Calc(PID_t *pid, float actual_val)
{
    /*璁＄畻鐩爣鍊间笌瀹為檯鍊肩殑璇樊*/
	//Calculate the error between the target value and the actual value
    pid->err = pid->target_val - actual_val;
    /*PID绠楁硶瀹炵幇*/
    //PID algorithm implementation
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    /*浼犻€掕宸?/
    //transmission error
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    /*杩斿洖PWM杈撳嚭鍊?/
    /*Return PWM output value*/

    /* The original source accidentally commented out these two state updates. */
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    if (pid->pwm_output > (MAX_SPEED - MOTOR_DEAD_ZONE))
        pid->pwm_output = (MAX_SPEED - MOTOR_DEAD_ZONE);
    if (pid->pwm_output < (MOTOR_DEAD_ZONE - MAX_SPEED))
        pid->pwm_output = (MOTOR_DEAD_ZONE - MAX_SPEED);

    return pid->pwm_output;
}

// 浣嶇疆寮廝ID璁＄畻鏂瑰紡
//Position PID calculation method
float PID_Location_Calc(PID_t *pid, float actual_val)
{
    /*璁＄畻鐩爣鍊间笌瀹為檯鍊肩殑璇樊*/
	/*Calculate the error between the target value and the actual value*/
    pid->err = pid->target_val - actual_val;

    /* 闄愬畾闂幆姝诲尯 */
    /*Limited closed-loop dead zone*/
    if ((pid->err >= -40) && (pid->err <= 40))
    {
        pid->err = 0;
        pid->integral = 0;
    }

    /* 绉垎鍒嗙锛屽亸宸緝澶ф椂鍘绘帀绉垎浣滅敤 */
    /*Integral separation, removing the integral effect when the deviation is large*/
    if (pid->err > -1500 && pid->err < 1500)
    {
        pid->integral += pid->err; // error accumulation 璇樊绱Н

        /* Limit the integration range to prevent integration saturation 闄愬畾绉垎鑼冨洿锛岄槻姝㈢Н鍒嗛ケ鍜?*/
        if (pid->integral > 4000)
            pid->integral = 4000;
        else if (pid->integral < -4000)
            pid->integral = -4000;
    }

    /*PID绠楁硶瀹炵幇*/ /*PID algorithm implementation*/
    pid->output_val = pid->Kp * pid->err +
                      pid->Ki * pid->integral +
                      pid->Kd * (pid->err - pid->err_last);
		

    /*璇樊浼犻€?/ /*Error transmission*/
    pid->err_last = pid->err;

    /*杩斿洖褰撳墠瀹為檯鍊?/ /*Returns the current actual value*/
    return pid->output_val;
}

// PID璁＄畻杈撳嚭鍊?PID calculation output value
void PID_Calc_Motor(motor_data_t *motor)
{
    int i;
    // float pid_out[4] = {0};
    // for (i = 0; i < MAX_MOTOR; i++)
    // {
    //     pid_out[i] = PID_Location_Calc(&pid_motor[i], 0);
    //     PID_Set_Motor_Target(i, pid_out[i]);
    // }

    for (i = 0; i < MAX_MOTOR; i++)
    {
        motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i]);
    }
}

// PID鍗曠嫭璁＄畻涓€鏉￠€氶亾 PID calculates one channel separately
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed)
{
    if (motor_id >= MAX_MOTOR)
        return 0;
//    return PID_Incre_Calc(&pid_motor[motor_id], now_speed);
//		motor->speed_pwm[motor_id]=PID_Location_Calc(&pid_motor[motor_id], now_speed);
		return PID_Location_Calc(&pid_motor[motor_id], now_speed);
		
		
}

// 璁剧疆PID鍙傛暟锛宮otor_id=4璁剧疆鎵€鏈夛紝=0123璁剧疆瀵瑰簲鐢垫満鐨凱ID鍙傛暟銆?
//Set PID parameters, motor_ Id=4 Set all,=0123 Set the PID parameters of the corresponding motor.
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id > MAX_MOTOR)
        return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = kp;
            pid_motor[i].Ki = ki;
            pid_motor[i].Kd = kd;
        }
    }
    else
    {
        pid_motor[motor_id].Kp = kp;
        pid_motor[motor_id].Ki = ki;
        pid_motor[motor_id].Kd = kd;
    }
}

void Set_PID_motor(void)
{
		PID_Set_Motor_Parm(0, PID_DEF_KP,PID_DEF_KI, PID_DEF_KD);
		PID_Set_Motor_Parm(1, PID_DEF_KP,PID_DEF_KI, PID_DEF_KD);
}

// 娓呴櫎PID鏁版嵁
//Clear PID data
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id > MAX_MOTOR)
        return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0;
            pid_motor[i].err = 0.0;
            pid_motor[i].err_last = 0.0;
            pid_motor[i].err_next = 0.0;
            pid_motor[i].integral = 0.0;
        }
    }
    else
    {
        pid_motor[motor_id].pwm_output = 0.0;
        pid_motor[motor_id].err = 0.0;
        pid_motor[motor_id].err_last = 0.0;
        pid_motor[motor_id].err_next = 0.0;
        pid_motor[motor_id].integral = 0.0;
    }
}


void Wheel_Yaw_PID(float yaw,float l_motor,float r_motor)
{
    float yaw_offset = PID_Yaw_Calc(yaw);
  

	
		
    float speed_L1 = l_motor - yaw_offset;
    float speed_R1 = r_motor + yaw_offset;

 
   Set_PID_Motor(speed_L1 ,speed_R1,0);

}

// 璁剧疆PID鐩爣閫熷害锛屽崟浣嶄负锛歮m/s
//Set PID target speed in mm/s
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > MAX_MOTOR)
        return;

    if (motor_id == MAX_MOTOR)
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].target_val = target;
        }
    }
    else
    {
        pid_motor[motor_id].target_val = target;
    }
}

// 杩斿洖PID缁撴瀯浣撴暟缁?
//Returns an array of PID structures
PID_t *Pid_Get_Motor(void)
{
    return pid_motor;
}

/*
 ****************缁撳悎imu鐨凱ID***********
 */
// 閲嶇疆鍋忚埅瑙掔殑鐩爣鍊?
//Reset the target value of yaw angle
void PID_Yaw_Reset(float yaw)
{
    pid_Yaw.SetPoint = yaw;
    pid_Yaw.SumError = 0;
    pid_Yaw.LastError = 0;
    pid_Yaw.PrevError = 0;
}

// 璁＄畻鍋忚埅瑙掔殑杈撳嚭鍊?
//Calculate the output value of yaw angle
float PID_Yaw_Calc(float NextPoint)
{
    float dError, Error;
    Error = pid_Yaw.SetPoint - NextPoint;           // deviation 鍋忓樊
    pid_Yaw.SumError += Error;                      // integral 绉垎
    dError = pid_Yaw.LastError - pid_Yaw.PrevError; // Current differential 褰撳墠寰垎
    pid_Yaw.PrevError = pid_Yaw.LastError;
    pid_Yaw.LastError = Error;

    double omega_rad = pid_Yaw.Proportion * Error            // proportional 姣斾緥椤?
                       + pid_Yaw.Integral * pid_Yaw.SumError // Integral term 绉垎椤?
                       + pid_Yaw.Derivative * dError;        // differential term 寰垎椤?

    if (omega_rad > PI / 6)
        omega_rad = PI / 6;
    if (omega_rad < -PI / 6)
        omega_rad = -PI / 6;
    return omega_rad;
}

float get_pid_target(PID_TypeDef *pid)
{
  return pid->Target;    // 璁剧疆褰撳墠鐨勭洰鏍囧€?
}

void set_pid_target(PID_TypeDef *pid, float target)
{
	pid->Target = target;
}

/**
  * @brief  璁剧疆姣斾緥銆佺Н鍒嗐€佸井鍒嗙郴鏁?
  * @param  p锛氭瘮渚嬬郴鏁?P
  * @param  i锛氱Н鍒嗙郴鏁?i
  * @param  d锛氬井鍒嗙郴鏁?d
  *	@note 	鏃?
  * @retval 鏃?
  */
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d)
{
  	pid->Kp = p * (pid->KP_polarity);    // 璁剧疆姣斾緥绯绘暟 P
	pid->Ki = i * (pid->KI_polarity);    // 璁剧疆绉垎绯绘暟 I
	pid->Kd = d * (pid->KD_polarity);    // 璁剧疆寰垎绯绘暟 D
}

void set_pid_polarity(PID_TypeDef *pid, int8_t p_polarity, int8_t i_polarity, int8_t d_polarity)
{
	pid->KP_polarity = p_polarity;
	pid->KI_polarity = i_polarity;
	pid->KD_polarity = d_polarity;
}




// Set parameters for yaw angle PID 璁剧疆鍋忚埅瑙扨ID鐨勫弬鏁?
void PID_Yaw_Set_Parm(float kp, float ki, float kd)
{
    pid_Yaw.Proportion = kp;
    pid_Yaw.Integral = ki;
    pid_Yaw.Derivative = kd;
}


void Set_PID_Motor(float set_l ,float set_r,float turn_out)
{
		
		
		l_pid_out =	PID_Calc_One_Motor(0, set_l);
		r_pid_out =	PID_Calc_One_Motor(1, set_r);
//		kal_lpid_out=KalmanFilter(&kfp_line,l_pid_out);
//		kal_rpid_out=KalmanFilter(&kfp_line,r_pid_out);
//		PWM_Control_Car(kal_lpid_out , kal_rpid_out );
		PWM_Control_Car(l_pid_out+turn_out , r_pid_out-turn_out );

}






//void turn_pid(int16_t dir, int8_t v)
//{
//	
//	float Yawpid_out ;
////	float SetPoint ;
//	uint16_t err;
//	float my_yaw;
//	my_yaw = yaw + 180;
////	Get_EulerAngles();
//	int16_t target_yaw = get_targ(my_yaw, dir);
////	int16_t target_yaw = get_targ(my_yaw, __dir);

//	PID_Yaw_Reset(my_yaw);

////	Yawpid_out= PID_Yaw_Calc(__v);
//	
//	
////	if(__dir == BACK){
////		PID_Yaw_Set_Parm(0.6, 0, 2);
////	}

//	
////	while(1){
//			err = calc_min_angle_direction(my_yaw, target_yaw);
//			PID_Yaw_Calc(target_yaw-err);
////		PID_Calculate(&veerpid, target_yaw-err);
//		//printf("yaw=%f, targ=%d \r\n", yaw, target_yaw);
//		if(dir == RIGHT){
////			set_pid_target(&l_pid, veerpid.PID_out);
////			PID_Calc_One_Motor(0, Yawpid_out);
////			PID_Calc_One_Motor(1, 0);
//			Set_PID_Motor(Yawpid_out ,0);
////			set_pid_target(&r_pid, 0);
//		}else if(dir == LEFT ){
////			set_pid_target(&l_pid, 0);
////			set_pid_target(&r_pid, -veerpid.PID_out);
////			PID_Calc_One_Motor(1,Yawpid_out);
////			PID_Calc_One_Motor(0, 0);
//			Set_PID_Motor(0 ,Yawpid_out);
//			
//		}else {
////			set_pid_target(&l_pid, veerpid.PID_out);
////			set_pid_target(&r_pid, -veerpid.PID_out);
////			PID_Calc_One_Motor(0, Yawpid_out);
////			PID_Calc_One_Motor(1, -Yawpid_out);
//			Set_PID_Motor(Yawpid_out ,-Yawpid_out);
//		}
//		
////		
//	printf("dir:%3.2f\r\n",dir);
////		if(My_abs(err) < 10 ){
////			break;
////		}
////	}

//	Set_PID_Motor(0 ,0);
//	delay_ms(100);
//}


