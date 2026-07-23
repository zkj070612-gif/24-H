/**
 * ============================================================
 * bsp_PID_motor.h  —  PID 电机控制头文件（带详细批注版）
 * ============================================================
 *
 * 【文件说明】
 *   本文件定义了 PID 控制器的数据结构、宏常量和函数声明。
 *   适用于 MSPM0G3507 智能小车项目，用于控制直流电机的转速
 *   以及偏航角（Yaw）的闭环控制。
 *
 * 【什么是 PID？】
 *   PID 是 Proportional（比例）、Integral（积分）、Derivative（微分）
 *   三个英文单词的缩写。它是一种经典的闭环控制算法，广泛应用于
 *   工业控制、机器人、无人机等领域。
 *
 *   - 比例(P)：根据"当前误差"的大小来调整输出，误差越大，修正力度越大。
 *   - 积分(I)：根据"历史累计误差"来调整输出，用于消除稳态误差（比如
 *              电机因为摩擦力始终达不到目标速度，积分项会慢慢补上）。
 *   - 微分(D)：根据"误差变化趋势"来调整输出，相当于"刹车"，防止超调
 *              （比如速度冲过头了，微分项会反向拉回来）。
 *
 *   简单类比：你洗澡调水温——
 *     P：水太冷就猛开热水（比例控制，响应快）
 *     I：开了一会儿还是不够热，再慢慢多开一点（积分控制，消除残余误差）
 *     D：水温快到了，提前关小一点防止烫到（微分控制，抑制超调）
 * ============================================================
 */

#ifndef __BSP_PID_MOTOR_H
#define __BSP_PID_MOTOR_H

#include "bsp.h"

/* ============================================================
 * 圆周率 π 的宏定义
 * 用于偏航角 PID 中角度与弧度的转换计算
 * ============================================================ */
#define PI (3.1415926f)

/* ============================================================
 * PID 参数宏定义区域
 * -----------------------------------------------------------
 * KP: 比例系数 — 值越大响应越快，但过大容易震荡
 * KI: 积分系数 — 值越大消除稳态误差越快，但过大容易超调
 * KD: 微分系数 — 值越大抑制震荡越强，但过大系统会变得迟钝
 *
 * 注意：这里有些参数是负值（如 -0.8），这是因为电机的
 * 正转/反转方向与编码器读数方向相反，用负号做方向校正。
 * ============================================================ */

// 以下是调试过程中尝试过的不同 PID 参数组（已注释掉）
//#define PID_MOTOR_KP (1.5f)
//#define PID_MOTOR_KI (0.08f)
//#define PID_MOTOR_KD (0.5f)

//#define PID_DEF_KP (-000.8f) //0.8
//#define PID_DEF_KI (-0.04f) //0.06
//#define PID_DEF_KD (-001.6f) //0.5

/* ---------- 当前使用的默认电机速度 PID 参数 ---------- */
#define PID_DEF_KP (-000.8f) // 比例系数 -0.8
#define PID_DEF_KI (-0.030f) // 积分系数 -0.03
#define PID_DEF_KD (-000.8f) // 微分系数 -0.8

// 另一组尝试过的参数（纯比例控制，已弃用）
//#define PID_DEF_KP (1.2f)
//#define PID_DEF_KI (0.000f)
//#define PID_DEF_KD (0.0f)

/* ---------- 偏航角（Yaw）PID 的默认参数 ---------- */
#define PID_YAW_DEF_KP (5.0)   // 偏航角比例系数
#define PID_YAW_DEF_KI (0.00)  // 偏航角积分系数（为0，纯PD控制）
#define PID_YAW_DEF_KD (0.50)  // 偏航角微分系数

/* ---------- 转向修正（veer）PID 参数 ---------- */
#define veer_p  (-046.0f)  // 转向修正比例系数
#define veer_i  (-0.00f)   // 转向修正积分系数
#define veer_d  (-0.00f)   // 转向修正微分系数（纯P控制）

/* ---------- K230 电机的多组 PID 参数（针对不同场景调参）---------- */
#define k230_p   (00.90f)   // K230 比例系数 第1组
#define k230_i   (0.00f)    // K230 积分系数
#define k230_d   (0.01f)    // K230 微分系数

#define k230_p1  (00.5f)    // K230 比例系数 第2组
#define k230_i1  (0.00f)
#define k230_d1  (0.001f)

#define k230_p2  (005.5f)   // K230 比例系数 第3组
#define k230_i2  (0.00f)
#define k230_d2  (0.001f)

#define k230_p3  (000.3f)   // K230 比例系数 第4组
#define k230_i3  (0.00f)
#define k230_d3  (0.001f)

/* ---------- 运动方向角度定义 ---------- */
#define RIGHT  (-90)   // 右转 -90°
#define LEFT   (90)    // 左转 90°
#define BACK   (180)   // 后退 180°

/* ============================================================
 * 结构体定义区域
 * ============================================================ */

/**
 * PID_t — 电机速度 PID 控制用的结构体（增量式 PID）
 *
 * 这个结构体为每个电机保存一套完整的 PID 控制数据，
 * 包括目标值、输出值、误差历史、积分累计等。
 */
typedef struct _pid
{
    float target_val;  // 【目标值】希望电机达到的速度（mm/s）
    float output_val;  // 【输出值】PID 计算后的速度输出
    float pwm_output;  // 【PWM输出】最终转换为 PWM 占空比的输出值
    float Kp, Ki, Kd;  // 【PID三参数】比例(Kp)、积分(Ki)、微分(Kd)系数
    float err;         // 【当前误差】目标值 - 实际值
    float err_last;    // 【上一次误差】用于微分项计算
    float err_next;    // 【下一次误差】（增量式PID用，保存更早的误差）
    float integral;    // 【积分累计】位置式PID用，累加所有历史误差
} PID_t;

/**
 * PID — 偏航角（Yaw）PID 控制用的结构体（位置式 PID）
 *
 * 这个结构体专门用于保持小车直行的偏航角控制。
 * 命名风格与 PID_t 不同，是项目中较早定义的版本。
 */
typedef struct
{
    float SetPoint;   // 【设定目标】期望的偏航角度
    float Proportion; // 【比例常数】Proportional Const
    float Integral;   // 【积分常数】Integral Const
    float Derivative; // 【微分常数】Derivative Const
    float LastError;  // 【上次误差】Error[-1]
    float PrevError;  // 【上上次误差】Error[-2]
    float SumError;   // 【误差累计和】Sums of Errors
} PID;

/**
 * motor_data_t — 电机数据打包结构体
 *
 * 将两个电机的输入（编码器测速）和输出（PID计算PWM）
 * 打包在一起，方便在 app_motor.c 中传递。
 */
typedef struct _motor_data_t
{
    float   speed_mm_s[2];  // 【输入】编码器计算出的实际速度（mm/s）
    float   speed_pwm[2];   // 【输出】PID 计算出的 PWM 输出值
    int16_t speed_set[2];   // 【设定】用户设定的目标速度
} motor_data_t;

/**
 * PID_TypeDef — 通用 PID 结构体（位置式/增量式均可使用）
 *
 * 这是项目中最通用的 PID 结构体，巡线、RGB呼吸灯等
 * 场景都使用这个结构体。支持极性控制（改变输出方向）。
 */
typedef struct
{
    float Kp;          // 【比例系数】
    float Ki;          // 【积分系数】
    float Kd;          // 【微分系数】
    float Err;         // 【当前偏差值】
    float LastErr;     // 【上一次偏差】
    float PenultErr;   // 【上上一次偏差】（名字有点误导，实际是更早的误差）
    float Integral;    // 【积分累加和】
    float Target;      // 【目标值】
    float PID_out;     // 【PID 输出值】

    int8_t KP_polarity;  // 【Kp 极性】1=正常方向, -1=反向（乘以系数时用）
    int8_t KI_polarity;  // 【Ki 极性】
    int8_t KD_polarity;  // 【Kd 极性】
} PID_TypeDef;

/* ============================================================
 * 全局变量声明（extern）
 * 这些变量在 bsp_PID_motor.c 中定义，其他文件可以引用
 * ============================================================ */
extern PID_TypeDef  veer_pid;   // 转向修正 PID 实例
extern int8_t       veer;        // 转向状态标志

/* ============================================================
 * 函数声明区域
 * ============================================================ */

/** 初始化一个 PID_TypeDef 结构体，将所有字段清零 */
void PID_param_init(PID_TypeDef *pid);

/** 初始化所有 PID 参数（电机速度PID + 偏航角PID） */
void PID_Param_Init(void);

/** 位置式 PID 计算（使用 PID_t 结构体）
 *  @param pid        指向 PID_t 结构体的指针
 *  @param actual_val 传感器测得的实际值
 *  @return           PID 计算后的输出值 */
float PID_Location_Calc(PID_t *pid, float actual_val);

/** 对所有电机执行 PID 计算
 *  @param motor 包含电机输入/输出数据的结构体指针 */
void PID_Calc_Motor(motor_data_t *motor);

/** 对单个电机执行 PID 计算（位置式）
 *  @param motor_id  电机编号（0=左电机, 1=右电机）
 *  @param now_speed 当前实际速度
 *  @return          PID 计算后的 PWM 输出值 */
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed);

/** 设置单个电机的目标速度
 *  @param motor_id 电机编号，MAX_MOTOR 表示设置全部
 *  @param target   目标速度（mm/s） */
void PID_Set_Motor_Target(uint8_t motor_id, float target);

/** 清除电机的 PID 历史数据（误差、积分等归零） */
void PID_Clear_Motor(uint8_t motor_id);

/** 设置电机的 PID 三参数
 *  @param motor_id 电机编号
 *  @param kp, ki, kd 新的 PID 参数 */
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd);

/** 增量式 PID 计算（使用 PID_t 结构体）
 *  @param pid        指向 PID_t 结构体的指针
 *  @param actual_val 当前实际值
 *  @return           PID 计算后的 PWM 输出值 */
float PID_Incre_Calc(PID_t *pid, float actual_val);

/** 设置两个电机的默认 PID 参数 */
void Set_PID_motor(void);

/* ---- PID_TypeDef 结构体的 getter/setter 函数 ---- */

/** 设置 PID 控制器的目标值 */
void set_pid_target(PID_TypeDef *pid, float target);

/** 获取 PID 控制器的当前目标值 */
float get_pid_target(PID_TypeDef *pid);

/** 设置 PID 控制器的 P/I/D 三个系数（会自动乘以极性） */
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d);

/* ---- 偏航角（Yaw）PID 相关函数 ---- */

/** 综合偏航角 PID 控制：根据当前偏航角调整左右电机速度
 *  @param yaw     当前偏航角
 *  @param l_motor 左电机基础速度
 *  @param r_motor 右电机基础速度 */
void Wheel_Yaw_PID(float yaw, float l_motor, float r_motor);

/** 通用 PID 计算（位置式，使用 PID_TypeDef 结构体）
 *  @param PID          指向 PID_TypeDef 的指针
 *  @param CurrentValue 当前测量值
 *  @return             PID 输出值 */
float PID_Calculate(PID_TypeDef *PID, float CurrentValue);

/** 设置转向修正 PID 的参数 */
void Set_PID_Veer(void);

/** 设置电机速度（同时传入左右电机目标 + 转向修正）
 *  @param set_l    左电机目标速度
 *  @param set_r    右电机目标速度
 *  @param turn_out 转向补偿量 */
void Set_PID_Motor(float set_l, float set_r, float turn_out);

/** 重置偏航角 PID 的目标值和历史数据 */
void PID_Yaw_Reset(float yaw);

/** 偏航角 PID 计算：根据当前角度计算角速度修正量
 *  @param NextPoint 当前实际偏航角
 *  @return          角速度修正量（弧度/秒） */
float PID_Yaw_Calc(float NextPoint);

/** 设置偏航角 PID 的三参数 */
void PID_Yaw_Set_Parm(float kp, float ki, float kd);

/** 求绝对值（整数版） */
int My_abs(int x);

#endif
