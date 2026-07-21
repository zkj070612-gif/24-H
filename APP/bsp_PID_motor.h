#ifndef __BSP_PID_MOTOR_H
#define __BSP_PID_MOTOR_H

#include "bsp.h"

#define PI (3.1415926f)

//#define PID_MOTOR_KP (1.5f)
//#define PID_MOTOR_KI (0.08f)
//#define PID_MOTOR_KD (0.5f)

//#define PID_DEF_KP (-000.8f) //0.8
//#define PID_DEF_KI (-0.04f) //0.06
//#define PID_DEF_KD (-001.6f) //0.5


#define PID_DEF_KP (-000.8f) //0.8
#define PID_DEF_KI (-0.030f) //0.06
#define PID_DEF_KD (-000.8f) //0.5



//#define PID_DEF_KP (1.2f) //0.8
//#define PID_DEF_KI (0.000f) //0.06
//#define PID_DEF_KD (0.0f) //0.5

#define PID_YAW_DEF_KP (5.0)
#define PID_YAW_DEF_KI (0.00)
#define PID_YAW_DEF_KD (0.50)

#define veer_p  (-046.0f)
#define veer_i  (-0.00f)
#define veer_d  (-0.00f) 


#define k230_p  (00.90f)
#define k230_i  (0.00f)
#define k230_d  (0.01f) 

#define k230_p1  (00.5f)
#define k230_i1  (0.00f)
#define k230_d1  (0.001f) 

#define k230_p2  (005.5f)
#define k230_i2  (0.00f)
#define k230_d2  (0.001f) 


#define k230_p3  (000.3f)
#define k230_i3  (0.00f)
#define k230_d3  (0.001f) 
#define RIGHT	 (-90)
#define LEFT	(90)
#define BACK	 (180)

typedef struct _pid
{
    float target_val; // 鐩爣鍊?
    float output_val; // 杈撳嚭鍊?
    float pwm_output; // PWM杈撳嚭鍊?
    float Kp, Ki, Kd; // 瀹氫箟姣斾緥銆佺Н鍒嗐€佸井鍒嗙郴鏁?
    float err;        // 瀹氫箟鍋忓樊鍊?
    float err_last;   // 瀹氫箟涓婁竴涓亸宸€?

    float err_next; // 瀹氫箟涓嬩竴涓亸宸€? 澧為噺寮?
    float integral; // 瀹氫箟绉垎鍊硷紝浣嶇疆寮?
} PID_t;

typedef struct
{
    float SetPoint;   // 璁惧畾鐩爣Desired value
    float Proportion; // 姣斾緥甯告暟Proportional Const
    float Integral;   // 绉垎甯告暟Integral Const
    float Derivative; // 寰垎甯告暟Derivative Const
    float LastError;  // Error[-1]
    float PrevError;  // Error[-2]
    float SumError;   // Sums of Errors
} PID;

typedef struct _motor_data_t
{
    float speed_mm_s[2];  // 杈撳叆鍊硷紝缂栫爜鍣ㄨ绠楅€熷害
    float speed_pwm[2];   // 杈撳嚭鍊硷紝PID璁＄畻鍑篜WM鍊?
    int16_t speed_set[2]; // 閫熷害璁剧疆鍊?
} motor_data_t;

typedef struct{
    float Kp;    //姣斾緥绯绘暟
    float Ki;   //绉垎绯绘暟
	float Kd;     //寰垎绯绘暟
    float Err; //鍋忓樊鍊?
	float LastErr;//涓婁竴涓亸宸?
	float PenultErr; //涓嬩竴涓亸宸?
    float Integral;//绉垎鍜?
    float Target;//鐩爣鍊?
	float PID_out; //PID杈撳嚭
	
	int8_t KP_polarity;
	int8_t KI_polarity;
	int8_t KD_polarity;
}PID_TypeDef;

extern PID_TypeDef  veer_pid;

extern int8_t veer;

void PID_param_init(PID_TypeDef *pid);

void PID_Param_Init(void);

float PID_Location_Calc(PID_t *pid, float actual_val);
void PID_Calc_Motor(motor_data_t *motor);
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);
void PID_Set_Motor_Target(uint8_t motor_id, float target);
void PID_Clear_Motor(uint8_t motor_id);
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);
float PID_Incre_Calc(PID_t *pid, float actual_val);

//void Set_PID_Motor(float set_l ,float set_r);
void Set_PID_motor(void);


void set_pid_target(PID_TypeDef *pid, float target);
float get_pid_target(PID_TypeDef *pid);
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d);

void Wheel_Yaw_PID(float yaw,float l_motor,float r_motor);

float PID_Calculate(PID_TypeDef *PID,float CurrentValue);
void Set_PID_Veer(void);
//int16_t get_targ(int16_t now, int16_t angle);
//int calc_min_angle_direction(int now, int targ);
//int get_targ(int now, int angle);
void Set_PID_Motor(float set_l ,float set_r,float turn_out);
void PID_Yaw_Reset(float yaw);
float PID_Yaw_Calc(float NextPoint);
void PID_Yaw_Set_Parm(float kp, float ki, float kd);
int My_abs(int x);

#endif
