/**
 * ============================================================
 * bsp_PID_motor.c  —  PID 电机控制实现文件（带详细批注版）
 * ============================================================
 *
 * 【文件说明】
 *   本文件实现了小车的所有 PID 控制算法，包括：
 *   1. 电机速度 PID（增量式 & 位置式）
 *   2. 偏航角（Yaw）PID（保持直行方向）
 *   3. 通用 PID 计算（用于巡线、转向等场景）
 *
 * 【PID 的两种主要形式】
 *   - 位置式 PID：输出 = Kp*err + Ki*∫err + Kd*(err - last_err)
 *     输出的是"绝对控制量"，比如电机的PWM占空比。每次计算都
 *     需要累加所有历史误差，积分项会一直增长。
 *
 *   - 增量式 PID：Δ输出 = Kp*(err - last_err) + Ki*err + Kd*(err - 2*last_err + prev_err)
 *     输出的是"控制量的变化量"，只与最近3次误差有关。优点是
 *     计算量小，且切换目标时不会有突变。
 *
 *   本项目中：
 *   - 电机速度控制主要使用增量式 PID（PID_Incre_Calc）
 *   - 偏航角控制使用位置式 PID（PID_Yaw_Calc）
 *   - 其他场景（巡线等）使用位置式 PID（PID_Calculate）
 *
 * 【PID 调参口诀】
 *   参数整定找最佳，从小到大顺序查；
 *   先是比例后积分，最后再把微分加；
 *   曲线振荡很频繁，比例系数要放大；   ← 震荡 → 先调小 Kp
 *   曲线漂浮绕大弯，比例系数往小扳；   ← 响应慢 → 调大 Kp
 *   曲线偏离回复慢，积分时间往下降；   ← 有残余误差 → 调大 Ki
 *   曲线波动周期长，积分时间再加长；   ← 超调 → 调小 Ki
 *   曲线振荡频率快，先把微分降下来；   ← 高频震荡 → 调小 Kd
 *   动差大来波动慢，微分时间应加长。   ← 迟钝 → 调大 Kd
 *
 * ============================================================
 */

#include "bsp_PID_motor.h"

/* ============================================================
 * 全局变量定义区域
 * ============================================================ */

// 转向修正用的 PID 控制器实例（全局变量，外部可引用）
PID_TypeDef  veer_pid;

// K230 电机专用的 PID 控制器实例
PID_TypeDef  k230_pid;

// 转向状态标志：0=不转向, 非0=正在转向
int8_t veer = 0;

// 偏航角偏移量（内部使用）
static int get_offset_yaw = 0;

// 左右电机的 PID 输出值（volatile 表示可能在中断中修改）
volatile float l_pid_out;
volatile float r_pid_out;

// 卡尔曼滤波后的 PID 输出（预留，当前未使用）
volatile float kal_lpid_out;
volatile float kal_rpid_out;

// 两个电机的 PID 控制器数组
// pid_motor[0] = 左电机, pid_motor[1] = 右电机
PID_t pid_motor[2];

// 偏航角 PID 输出相关变量
volatile float   Yawpid_out;    // 偏航角 PID 的当前输出值
volatile float   now_yawout;    // 当前偏航角输出
volatile int16_t target_yaw;    // 目标偏航角

// 引用外部定义的 IMU 欧拉角变量（来自姿态解算模块）
extern float pitch, roll, yaw;

// 引用外部定义的偏航角测量值
extern volatile float my_yaw;

// 目标航向角 & 初始航向角
float get_yaw = 0;     // 小车运动的目标航向角
float first_yaw = 0;   // 初始记录的航向角

// 偏航角 PID 控制器实例（结构体方式初始化）
// 初始参数: P=0.4, I=0, D=0.1（纯 PD 控制）
PID pid_Yaw = {0, 0.4, 0, 0.1, 0, 0, 0};

/* ============================================================
 * PID_param_init() — 初始化 PID_TypeDef 结构体
 * ------------------------------------------------------------
 * 功能：将一个 PID_TypeDef 结构体的所有字段设置为初始值。
 *       - 目标值 = 0
 *       - 输出值 = 0
 *       - 三个系数 = 0（需要后续用 set_p_i_d 设置）
 *       - 误差/积分历史 = 0
 *       - 极性全部设为 1（正方向）
 *
 * 典型用法：
 *   PID_TypeDef my_pid;
 *   PID_param_init(&my_pid);           // 先初始化
 *   set_p_i_d(&my_pid, 1.2, 0.05, 0.3); // 再设置参数
 *   set_pid_target(&my_pid, 100);       // 设置目标
 * ============================================================ */
void PID_param_init(PID_TypeDef *pid)
{
    pid->Target = 0;     // 目标值初始化为0
    pid->PID_out = 0;    // 输出值初始化为0
    pid->Kp = 0;         // 比例系数 = 0
    pid->Ki = 0;         // 积分系数 = 0
    pid->Kd = 0;         // 微分系数 = 0
    pid->Err = 0.0f;     // 当前误差 = 0
    pid->LastErr = 0.0f; // 上一次误差 = 0
    pid->PenultErr = 0.0f; // 上上一次误差 = 0
    pid->Integral = 0.0f;  // 积分累加值 = 0

    // 极性全部设为 1，即输出方向不变
    // 如果某个系数设为负数且极性也是负数，负负得正，实际效果为正
    pid->KP_polarity = 1;
    pid->KI_polarity = 1;
    pid->KD_polarity = 1;
}

/**
 * Set_PID_Veer() — 设置转向修正 PID
 * ------------------------------------------------------------
 * 功能：初始化转向修正用的 PID 控制器，使用 veer_p/veer_d 宏定义
 *       的系数。注意这个函数创建的是局部变量，实际使用时需要通过
 *       全局变量 veer_pid 来操作。
 *
 * 注意：这个函数目前只初始化了局部变量 veerpid，没有赋值给全局变量，
 *       可能是遗留代码或需要配合其他函数使用。
 */
void Set_PID_Veer(void)
{
    PID_TypeDef veerpid;          // 创建局部 PID 变量
    PID_param_init(&veerpid);      // 初始化为零
    // set_pid_target(&veerpid, target_yaw);  // 已注释：不在这里设置目标
    set_p_i_d(&veerpid, veer_p, 0, veer_d);  // 设置 P 和 D 参数（纯 PD 控制，无 I）
}

/* ============================================================
 * PID_Calculate() — 通用位置式 PID 计算
 * ------------------------------------------------------------
 * 【核心公式】
 *   输出 = Kp * 当前误差
 *        + Ki * 误差累计和
 *        + Kd * (当前误差 - 上次误差)
 *
 * 【执行流程】
 *   第1步：累加当前误差到积分项（注意：先加旧误差，再算新误差）
 *   第2步：计算新的误差 = 目标 - 实际
 *   第3步：套用 PID 公式计算输出
 *   第4步：积分限幅（防止积分项过大导致失控）
 *   第5步：保存当前误差为"上次误差"，供下次使用
 *
 * 【积分限幅的作用】
 *   如果误差一直存在（比如电机被卡住了），积分项会无限增长，
 *   一旦障碍解除，巨大的积分值会导致电机突然猛转——这叫"积分饱和"。
 *   限幅 ±1000 就是为了防止这种情况。
 *
 * @param PID          指向 PID_TypeDef 控制器的指针
 * @param CurrentValue 传感器测得的当前实际值
 * @return             计算后的 PID 输出值
 * ============================================================ */
float PID_Calculate(PID_TypeDef *PID, float CurrentValue)
{
    // 【第1步】先累加上一次的误差到积分项
    PID->Integral += PID->Err;

    // 以下是被注释掉的积分分离代码（调试用）
    // 积分分离：当误差太大时，暂时不累加积分，防止积分饱和
    //	if( (PID->Err > 36) || (PID->Err < -36) ){
    //		PID->Integral = 0;
    //	}

    // 【第2步】计算当前误差 = 目标值 - 实际值
    PID->Err = PID->Target - CurrentValue;

    // 【第3步】PID 核心公式
    //   Kp * Err              ← 比例项：根据当前误差大小调整输出
    //   Ki * Integral         ← 积分项：根据历史累计误差调整输出
    //   Kd * (Err - LastErr)  ← 微分项：根据误差变化趋势调整输出（"刹车"作用）
    PID->PID_out = PID->Kp * PID->Err                     /* 比例 */
                 + PID->Ki * PID->Integral                 /* 积分 */
                 + PID->Kd * (PID->Err - PID->LastErr);    /* 微分 */

    // 【第4步】积分限幅：把积分值限制在 [-1000, 1000] 范围内
    // 防止积分项过大导致的失控（积分饱和现象）
    if (PID->Integral > 1000)
    {
        PID->Integral = 1000;
    }
    if (PID->Integral < -1000)
    {
        PID->Integral = -1000;
    }

    // 【第5步】保存本次误差，作为下一次计算的"上次误差"
    PID->LastErr = PID->Err;

    // 返回计算出的 PID 输出值
    return PID->PID_out;
}

/* ============================================================
 * PID_Param_Init() — 初始化所有 PID 参数
 * ------------------------------------------------------------
 * 功能：在上电或复位时调用，设置电机 PID 和偏航角 PID 的默认参数。
 *
 * 初始化内容：
 *   1. 两个电机的 PID_t 结构体清零 + 设置默认 Kp/Ki/Kd
 *   2. 偏航角 PID 的参数设置（P=5.0, I=0, D=0.5）
 * ============================================================ */
void PID_Param_Init(void)
{
    /* 速度相关初始化参数 */
    for (int i = 0; i < MAX_MOTOR; i++)   // MAX_MOTOR = 2（左右两个电机）
    {
        pid_motor[i].target_val = 0.0;    // 目标速度 = 0
        pid_motor[i].pwm_output  = 0.0;   // PWM 输出 = 0
        pid_motor[i].err         = 0.0;   // 当前误差 = 0
        pid_motor[i].err_last    = 0.0;   // 上次误差 = 0
        pid_motor[i].err_next    = 0.0;   // 上上次误差 = 0
        pid_motor[i].integral    = 0.0;   // 积分累计 = 0

        // 使用头文件中定义的默认 PID 参数
        pid_motor[i].Kp = PID_DEF_KP;     // 比例系数
        pid_motor[i].Ki = PID_DEF_KI;     // 积分系数
        pid_motor[i].Kd = PID_DEF_KD;     // 微分系数
    }

    // 初始化偏航角 PID 参数
    pid_Yaw.Proportion = PID_YAW_DEF_KP;  // 比例 = 5.0
    pid_Yaw.Integral   = PID_YAW_DEF_KI;  // 积分 = 0（纯 PD 控制，不用积分）
    pid_Yaw.Derivative = PID_YAW_DEF_KD;  // 微分 = 0.5
}

/**
 * PID_Set_Parm() — 设置 PID 三参数
 * ------------------------------------------------------------
 * 这是 PID_t 结构体的 setter 函数，用于在运行时动态调整 PID 参数。
 *
 * @param pid 目标 PID 控制器
 * @param p   新的比例系数
 * @param i   新的积分系数
 * @param d   新的微分系数
 */
void PID_Set_Parm(PID_t *pid, float p, float i, float d)
{
    pid->Kp = p;  // 设置比例系数
    pid->Ki = i;  // 设置积分系数
    pid->Kd = d;  // 设置微分系数
}

/**
 * PID_Set_Target() — 设置 PID 目标值
 * ------------------------------------------------------------
 * @param pid      目标 PID 控制器
 * @param temp_val 新的目标值（比如期望速度 200 mm/s）
 */
void PID_Set_Target(PID_t *pid, float temp_val)
{
    pid->target_val = temp_val;
}

/**
 * PID_Get_Target() — 获取 PID 目标值
 * ------------------------------------------------------------
 * @param pid 目标 PID 控制器
 * @return    当前设置的目标值
 */
float PID_Get_Target(PID_t *pid)
{
    return pid->target_val;
}

/* ============================================================
 * PID_Incre_Calc() — 增量式 PID 计算
 * ------------------------------------------------------------
 * 【核心公式】（增量式）
 *   Δ输出 = Kp * (eₖ - eₖ₋₁)           ← 比例增量：误差变化量
 *         + Ki * eₖ                      ← 积分增量：当前误差
 *         + Kd * (eₖ - 2*eₖ₋₁ + eₖ₋₂)  ← 微分增量：误差变化趋势
 *
 *   然后：输出 = 上次输出 + Δ输出
 *
 * 【为什么用增量式？】
 *   1. 计算量小，只关心最近3次误差
 *   2. 输出是"增量"，切换目标时不会突变（无积分饱和问题）
 *   3. 误码率影响小（一次算错，下次能自动修正）
 *
 * 【误差变量含义】
 *   err       = 当前误差（最新）
 *   err_next  = 上一次误差（errₖ₋₁）
 *   err_last  = 上上次误差（errₖ₋₂）
 *
 * 【pwm_output 限幅】
 *   MAX_SPEED - MOTOR_DEAD_ZONE = 电机能响应的最大有效 PWM 值
 *   死区（Dead Zone）是电机刚能转动的最小 PWM 值，小于这个值
 *   电机不转，所以有效范围是 [dead_zone, max_speed-dead_zone]
 *
 * @param pid        电机 PID 控制器
 * @param actual_val 编码器测得的当前实际速度
 * @return           计算后的 PWM 输出值
 * ============================================================ */
float PID_Incre_Calc(PID_t *pid, float actual_val)
{
    /* 计算目标值与实际值的误差 */
    pid->err = pid->target_val - actual_val;

    /* PID 算法实现 — 增量式公式 */
    // Kp * (err - err_next)
    //   → 比例项：如果误差在变大（err > err_next），就加大输出
    // Ki * err
    //   → 积分项：误差越大，积分修正越多
    // Kd * (err - 2*err_next + err_last)
    //   → 微分项：误差变化的"加速度"，预判未来趋势
    pid->pwm_output += pid->Kp * (pid->err - pid->err_next)
                     + pid->Ki * pid->err
                     + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);

    /* 传递误差：把误差历史往前推一格 */
    // err_last ← err_next ← err（最新的变成"上一次"）
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    /* 以下两行是原始代码中意外重复注释的误差传递（冗余代码，与上面相同）*/
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;

    /* 输出限幅：确保 PWM 值在电机有效工作范围内 */
    // 上限：不超过 MAX_SPEED - MOTOR_DEAD_ZONE
    if (pid->pwm_output > (MAX_SPEED - MOTOR_DEAD_ZONE))
        pid->pwm_output = (MAX_SPEED - MOTOR_DEAD_ZONE);
    // 下限：不低于 -(MAX_SPEED - MOTOR_DEAD_ZONE)，负值表示反转
    if (pid->pwm_output < (MOTOR_DEAD_ZONE - MAX_SPEED))
        pid->pwm_output = (MOTOR_DEAD_ZONE - MAX_SPEED);

    return pid->pwm_output;
}

/* ============================================================
 * PID_Location_Calc() — 位置式 PID 计算
 * ------------------------------------------------------------
 * 【核心公式】（位置式）
 *   输出 = Kp * err                  ← 比例项
 *        + Ki * integral            ← 积分项（累加）
 *        + Kd * (err - err_last)    ← 微分项
 *
 * 这个函数相比 PID_Calculate() 多了以下保护机制：
 *
 * 【死区处理】（Dead Zone）
 *   当误差在 [-40, 40] 范围内时，认为已经"够好了"，不再调整。
 *   这可以防止电机在小误差范围内频繁微调导致的抖动和噪音。
 *
 * 【积分分离】
 *   只有当误差在 [-1500, 1500] 范围内时才累加积分。
 *   如果误差很大（说明偏离目标很远），暂时不用积分作用，
 *   只用 P+D 快速逼近。等误差变小后再加上积分来消除残余误差。
 *   这可以防止大误差下积分过度累积导致的严重超调。
 *
 * 【积分限幅】
 *   积分值限制在 [-4000, 4000]，防止积分饱和。
 *
 * @param pid        电机 PID 控制器
 * @param actual_val 当前实际速度
 * @return           位置式 PID 的计算输出
 * ============================================================ */
float PID_Location_Calc(PID_t *pid, float actual_val)
{
    /* 计算目标值与实际值的误差 */
    pid->err = pid->target_val - actual_val;

    /* 死区处理：误差很小（±40以内），不再调整 */
    // 例如：目标 200，实际 195~205 都算"到达目标"
    if ((pid->err >= -40) && (pid->err <= 40))
    {
        pid->err = 0;        // 强制误差为0
        pid->integral = 0;   // 清零积分
    }

    /* 积分分离：只在小误差范围内累加积分 */
    // 误差在 ±1500 以内 → 启用积分
    if (pid->err > -1500 && pid->err < 1500)
    {
        pid->integral += pid->err;  // 累加误差到积分项

        /* 积分限幅：防积分饱和，限制在 ±4000 */
        if (pid->integral > 4000)
            pid->integral = 4000;
        else if (pid->integral < -4000)
            pid->integral = -4000;
    }
    // 注意：误差超过 ±1500 时积分不变（不会归零，只是不累加）
    // 这是一种软性积分分离策略

    /* PID 算法实现 — 位置式公式 */
    pid->output_val = pid->Kp * pid->err                 /* 比例：当前误差 */
                    + pid->Ki * pid->integral             /* 积分：累计误差 */
                    + pid->Kd * (pid->err - pid->err_last); /* 微分：误差变化 */

    /* 误差传递：保存本次误差为"上次误差" */
    pid->err_last = pid->err;

    /* 返回当前实际值（注意：返回的是 output_val，不是 actual_val）*/
    return pid->output_val;
}

/* ============================================================
 * PID_Calc_Motor() — 对所有电机执行 PID 计算
 * ------------------------------------------------------------
 * 遍历两个电机，用增量式 PID 根据编码器测得的实际速度
 * 计算出应有的 PWM 输出值。
 *
 * 调用时机：在 Motion_Get_Speed() 中被调用（每 20ms 一次）
 *
 * @param motor 包含所有电机输入/输出数据的结构体
 *              motor->speed_mm_s[i] = 编码器测得的实际速度（输入）
 *              motor->speed_pwm[i]  = PID 计算出的 PWM 值（输出）
 * ============================================================ */
void PID_Calc_Motor(motor_data_t *motor)
{
    int i;
    // 以下是注释掉的旧代码（原来用位置式 PID，后来改为增量式）
    // for (i = 0; i < MAX_MOTOR; i++)
    // {
    //     pid_out[i] = PID_Location_Calc(&pid_motor[i], 0);
    //     PID_Set_Motor_Target(i, pid_out[i]);
    // }

    for (i = 0; i < MAX_MOTOR; i++)
    {
        // 对每个电机：
        //   输入 = motor->speed_mm_s[i]（编码器测的实际速度）
        //   输出 = motor->speed_pwm[i]（PID 计算的 PWM 值）
        motor->speed_pwm[i] = PID_Incre_Calc(&pid_motor[i], motor->speed_mm_s[i]);
    }
}

/* ============================================================
 * PID_Calc_One_Motor() — 单独计算一个电机的 PID
 * ------------------------------------------------------------
 * 与 PID_Calc_Motor 不同，这个函数只计算一个电机，
 * 且使用的是位置式 PID（PID_Location_Calc）。
 *
 * 调用场景：红外遥控控制、巡线转向等需要单独控制某个电机的场景。
 *
 * @param motor_id  电机编号（0=左, 1=右）
 * @param now_speed 当前实际速度
 * @return          PID 输出值；如果 motor_id 无效则返回 0
 * ============================================================ */
float PID_Calc_One_Motor(uint8_t motor_id, float now_speed)
{
    if (motor_id >= MAX_MOTOR)   // 安全检查：电机编号不能越界
        return 0;

    // 使用位置式 PID 计算
    // 当前代码：使用 PID_Location_Calc
    // 被注释掉的备选方案：PID_Incre_Calc（增量式）
    return PID_Location_Calc(&pid_motor[motor_id], now_speed);
}

/* ============================================================
 * PID_Set_Motor_Parm() — 设置电机的 PID 参数
 * ------------------------------------------------------------
 * 可以设置单个电机或全部电机的 PID 参数。
 *
 * @param motor_id 电机编号
 *        = 0 → 左电机
 *        = 1 → 右电机
 *        = MAX_MOTOR(2) → 设置全部电机
 * @param kp, ki, kd 新的 PID 参数
 * ============================================================ */
void PID_Set_Motor_Parm(uint8_t motor_id, float kp, float ki, float kd)
{
    if (motor_id > MAX_MOTOR)   // 越界检查
        return;

    if (motor_id == MAX_MOTOR)   // motor_id == 2 → 批量设置所有电机
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].Kp = kp;
            pid_motor[i].Ki = ki;
            pid_motor[i].Kd = kd;
        }
    }
    else   // motor_id == 0 或 1 → 单独设置
    {
        pid_motor[motor_id].Kp = kp;
        pid_motor[motor_id].Ki = ki;
        pid_motor[motor_id].Kd = kd;
    }
}

/**
 * Set_PID_motor() — 用默认参数设置两个电机的 PID
 * ------------------------------------------------------------
 * 调用 PID_Set_Motor_Parm，使用头文件中定义的默认参数。
 */
void Set_PID_motor(void)
{
    PID_Set_Motor_Parm(0, PID_DEF_KP, PID_DEF_KI, PID_DEF_KD);  // 左电机
    PID_Set_Motor_Parm(1, PID_DEF_KP, PID_DEF_KI, PID_DEF_KD);  // 右电机
}

/* ============================================================
 * PID_Clear_Motor() — 清除电机的 PID 历史数据
 * ------------------------------------------------------------
 * 将电机 PID 控制器的误差、积分、输出等历史数据全部清零。
 * 在小车停止或切换运动模式时调用，防止历史数据影响新的控制。
 *
 * @param motor_id 电机编号（同 PID_Set_Motor_Parm 的规则）
 * ============================================================ */
void PID_Clear_Motor(uint8_t motor_id)
{
    if (motor_id > MAX_MOTOR)
        return;

    if (motor_id == MAX_MOTOR)   // 清除所有电机
    {
        for (int i = 0; i < MAX_MOTOR; i++)
        {
            pid_motor[i].pwm_output = 0.0;  // PWM 输出清零
            pid_motor[i].err        = 0.0;  // 当前误差清零
            pid_motor[i].err_last   = 0.0;  // 历史误差清零
            pid_motor[i].err_next   = 0.0;
            pid_motor[i].integral   = 0.0;  // 积分清零
        }
    }
    else   // 清除单个电机
    {
        pid_motor[motor_id].pwm_output = 0.0;
        pid_motor[motor_id].err        = 0.0;
        pid_motor[motor_id].err_last   = 0.0;
        pid_motor[motor_id].err_next   = 0.0;
        pid_motor[motor_id].integral   = 0.0;
    }
}

/* ============================================================
 * Wheel_Yaw_PID() — 综合偏航角 PID 控制（带 IMU）
 * ------------------------------------------------------------
 * 功能：根据 IMU 测得的当前偏航角，计算左右电机的速度修正量，
 *       使小车保持直行。如果小车偏左，就加速左轮/减速右轮；
 *       如果偏右，则反过来。
 *
 * 工作原理：
 *   1. PID_Yaw_Calc(yaw) 输入当前偏航角，输出角速度修正量 yaw_offset
 *   2. 左轮速度 = 基础速度 - yaw_offset（偏左时 yaw_offset>0，左轮减速回正）
 *   3. 右轮速度 = 基础速度 + yaw_offset（偏左时 yaw_offset>0，右轮加速回正）
 *
 * @param yaw      IMU 测得的当前偏航角
 * @param l_motor  左电机基础速度（用户设定的速度）
 * @param r_motor  右电机基础速度（用户设定的速度）
 * ============================================================ */
void Wheel_Yaw_PID(float yaw, float l_motor, float r_motor)
{
    // 【第1步】计算偏航角修正量
    float yaw_offset = PID_Yaw_Calc(yaw);

    // 【第2步】左轮减速、右轮加速 → 用差速实现转向修正
    float speed_L1 = l_motor - yaw_offset;   // 左轮减去修正量
    float speed_R1 = r_motor + yaw_offset;   // 右轮加上修正量

    // 【第3步】设置电机速度（turn_out=0，因为修正已经在左右速度里了）
    Set_PID_Motor(speed_L1, speed_R1, 0);
}

/**
 * PID_Set_Motor_Target() — 设置电机目标速度
 * ------------------------------------------------------------
 * @param motor_id 电机编号（同 PID_Set_Motor_Parm 的规则）
 * @param target   目标速度，单位 mm/s
 */
void PID_Set_Motor_Target(uint8_t motor_id, float target)
{
    if (motor_id > MAX_MOTOR)
        return;

    if (motor_id == MAX_MOTOR)   // 设置所有电机
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

/**
 * Pid_Get_Motor() — 获取电机 PID 结构体数组的指针
 * ------------------------------------------------------------
 * @return 指向 pid_motor[2] 数组的指针
 */
PID_t *Pid_Get_Motor(void)
{
    return pid_motor;
}

/* ============================================================
 *              偏航角（Yaw）PID 控制函数
 * ============================================================ */

/**
 * PID_Yaw_Reset() — 重置偏航角 PID
 * ------------------------------------------------------------
 * 将偏航角 PID 的目标值设为当前角度，并清空所有历史数据。
 * 通常在开始新的直线运动前调用，确保 PID 从零开始计算。
 *
 * @param yaw 当前 IMU 偏航角（作为新的目标值）
 */
void PID_Yaw_Reset(float yaw)
{
    pid_Yaw.SetPoint  = yaw;   // 以当前角度为目标
    pid_Yaw.SumError  = 0;     // 误差累计清零
    pid_Yaw.LastError = 0;     // 上次误差清零
    pid_Yaw.PrevError = 0;     // 上上次误差清零
}

/* ============================================================
 * PID_Yaw_Calc() — 偏航角 PID 计算
 * ------------------------------------------------------------
 * 【核心公式】
 *   角速度修正量 = P * 角度误差 + I * 累计误差 + D * 误差变化率
 *
 * 【微分项的计算方式】
 *   这里用的是"误差的差分"：dError = LastError - PrevError
 *   等同于：误差变化量 / 时间（假设固定时间间隔）
 *
 * 【输出限幅】
 *   输出限制在 ±π/6（约 ±30°），防止修正量过大导致小车打转。
 *
 * @param NextPoint 当前 IMU 测得的偏航角
 * @return          角速度修正量（弧度/秒）
 * ============================================================ */
float PID_Yaw_Calc(float NextPoint)
{
    float dError, Error;

    // 【第1步】计算角度误差
    Error = pid_Yaw.SetPoint - NextPoint;   // 偏差 = 目标角度 - 实际角度

    // 【第2步】累加积分
    pid_Yaw.SumError += Error;              // 积分累加

    // 【第3步】计算微分（误差的变化量）
    dError = pid_Yaw.LastError - pid_Yaw.PrevError;  // 当前微分

    // 【第4步】更新误差历史
    pid_Yaw.PrevError = pid_Yaw.LastError;   // 上上次 ← 上次
    pid_Yaw.LastError = Error;               // 上次 ← 本次

    // 【第5步】PID 计算
    // 比例项：Proportion * Error        → 角度偏差越大，修正越强
    // 积分项：Integral   * SumError     → 长时间偏航，积分补偿
    // 微分项：Derivative * dError       → 预判角度变化趋势
    double omega_rad = pid_Yaw.Proportion * Error             /* 比例项 */
                     + pid_Yaw.Integral * pid_Yaw.SumError    /* 积分项 */
                     + pid_Yaw.Derivative * dError;           /* 微分项 */

    // 【第6步】输出限幅：±PI/6 弧度（约 ±30°/秒）
    // 防止修正量过大导致小车剧烈摆动
    if (omega_rad > PI / 6)
        omega_rad = PI / 6;
    if (omega_rad < -PI / 6)
        omega_rad = -PI / 6;

    return omega_rad;
}

/* ---- PID_TypeDef 的 getter/setter 函数 ---- */

/**
 * get_pid_target() — 读取 PID 控制器的目标值
 */
float get_pid_target(PID_TypeDef *pid)
{
    return pid->Target;
}

/**
 * set_pid_target() — 设置 PID 控制器的目标值
 */
void set_pid_target(PID_TypeDef *pid, float target)
{
    pid->Target = target;
}

/**
 * set_p_i_d() — 设置 PID 三参数（含极性处理）
 * ------------------------------------------------------------
 * 注意：这里系数会乘以极性值！
 *   - 如果极性 = 1：实际系数 = 你传入的值（正方向控制）
 *   - 如果极性 = -1：实际系数 = -你传入的值（反方向控制）
 *
 * 极性机制允许你在不同的硬件接线/安装方向下，用同样的正系数值
 * 来调参，然后通过修改极性来适配实际方向。
 *
 * @param pid PID 控制器
 * @param p   比例系数（正数）
 * @param i   积分系数（正数）
 * @param d   微分系数（正数）
 */
void set_p_i_d(PID_TypeDef *pid, float p, float i, float d)
{
    pid->Kp = p * (pid->KP_polarity);   // 系数 × 极性 = 实际比例系数
    pid->Ki = i * (pid->KI_polarity);   // 系数 × 极性 = 实际积分系数
    pid->Kd = d * (pid->KD_polarity);   // 系数 × 极性 = 实际微分系数
}

/**
 * set_pid_polarity() — 设置 PID 三参数的极性
 * ------------------------------------------------------------
 * @param p_polarity 比例系数极性（1=正常, -1=反向）
 * @param i_polarity 积分系数极性
 * @param d_polarity 微分系数极性
 */
void set_pid_polarity(PID_TypeDef *pid, int8_t p_polarity, int8_t i_polarity, int8_t d_polarity)
{
    pid->KP_polarity = p_polarity;
    pid->KI_polarity = i_polarity;
    pid->KD_polarity = d_polarity;
}

/**
 * PID_Yaw_Set_Parm() — 设置偏航角 PID 参数
 * ------------------------------------------------------------
 */
void PID_Yaw_Set_Parm(float kp, float ki, float kd)
{
    pid_Yaw.Proportion = kp;
    pid_Yaw.Integral   = ki;
    pid_Yaw.Derivative = kd;
}

/* ============================================================
 * Set_PID_Motor() — 设置电机速度（核心运动控制函数）
 * ------------------------------------------------------------
 * 这是小车运动控制的总入口函数之一。它同时设置左右电机的
 * 目标速度，并通过 PID 计算 PWM 输出，最后驱动电机。
 *
 * 工作流程：
 *   1. 对左电机：PID_Calc_One_Motor(0, set_l) → 得到左轮 PWM
 *   2. 对右电机：PID_Calc_One_Motor(1, set_r) → 得到右轮 PWM
 *   3. 加上转向补偿 turn_out
 *   4. 调用 PWM_Control_Car() 驱动电机
 *
 * @param set_l    左电机目标速度
 * @param set_r    右电机目标速度
 * @param turn_out 转向补偿量（正值=向右转）
 * ============================================================ */
void Set_PID_Motor(float set_l, float set_r, float turn_out)
{
    // 对左右电机分别做 PID 计算
    l_pid_out = PID_Calc_One_Motor(0, set_l);   // 左电机 PID 输出
    r_pid_out = PID_Calc_One_Motor(1, set_r);   // 右电机 PID 输出

    // 以下是被注释掉的卡尔曼滤波方案（预留）
    // kal_lpid_out = KalmanFilter(&kfp_line, l_pid_out);
    // kal_rpid_out = KalmanFilter(&kfp_line, r_pid_out);
    // PWM_Control_Car(kal_lpid_out, kal_rpid_out);

    // 驱动电机：左轮 + turn_out，右轮 - turn_out → 差速转向
    PWM_Control_Car(l_pid_out + turn_out, r_pid_out - turn_out);
}
