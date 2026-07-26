#include "get_bno085.h"
#include "Filter.h"

#define Pi 3.14159265f

/* 全局变量 - 与 MPU6050 兼容 */
short angle[3] = {0};       // 陀螺仪原始数据 (deg/s * 10)
short accel[3] = {0};      // 加速度原始数据 (mg)
float pitch = 0, roll = 0, yaw = 0;
float yawBias = 0, pitchBias = 0, rollBias = 0;
float calibratedYaw = 0, calibratedPitch = 0, calibratedRoll = 0;

volatile float kal_mpu_out = 0;
volatile float Filter_out = 0;
volatile float Filter_out_last = 0;
volatile float Yaw_Err_Lowout_last = 0;
volatile float Yaw_Err_Lowout = 0;
volatile float erro_sumr = 0;
volatile float error = 0;

volatile float gyro_x = 0, gyro_y = 0, gyro_z = 0, accel_z = 0;
volatile float Accel_Angle_X = 0, Accel_Angle_Y = 0;
uint8_t Way_Angle = 2;
Bias_t Angle = {0};

const int CALIB_SAMPLES = 2;
float Angle_Balance = 0, Gyro_Balance = 0, Gyro_Turn = 0;
float Acceleration_Z = 0;

/* PID 参数 - 与 MPU6050 兼容 */
float dir_kp = 2.0f, dir_ki = 0.000f, dir_kd = 5.00f;
int Integral_Max = 300;
int pid_max = 1000;

/* 初始化 - 替代 DMP_Init */
void BNO085_DMP_Init(void)
{
    uint8_t i = 1;
    char error_code = 0;

    OLED_ShowString(0, 9, "BNO085 init...", 8, 1);
    OLED_ShowString(0, 18, "Attempts:", 8, 1);
    OLED_Refresh();

    do {
        error_code = BNO085_Init();

        OLED_ShowNum(60, 18, i++, 2, 8, 1);
        OLED_ShowNum(42, 0, error_code, 3, 8, 1);
        OLED_Refresh();

        delay_ms(200);
    } while(error_code != 0);

    OLED_ShowString(60, 10, "OK!", 8, 1);
    OLED_Refresh();
    delay_ms(1000);
    OLED_Clear();
}

/* 替代 Get_EulerAngles - 直接读取 BNO085 输出的欧拉角 */
void BNO085_Get_EulerAngles(void)
{
    BNO085_Euler_t euler;
    BNO085_ReadEuler(&euler);

    // 更新全局变量
    pitch = euler.pitch;
    roll = euler.roll;
    yaw = euler.yaw;

    // 应用滤波 (保持与 MPU6050 兼容的滤波逻辑)
    Filter_out = BNO085_Get_Filter(yaw);

    // 调试输出
    // printf("pitch: %.2f roll: %.2f yaw: %.2f\r\n", pitch, roll, yaw);
}

/* 替代 Get_Filter - 低通滤波 */
float BNO085_Get_Filter(float Yaw_Err)
{
    float a = 0.0150f;

    Yaw_Err_Lowout = (1 - a) * Yaw_Err + a * Yaw_Err_Lowout_last;
    Yaw_Err_Lowout_last = Yaw_Err_Lowout;

    return Yaw_Err_Lowout;
}

/* 替代 Dir_PID - 方向环 PID 控制 */
float BNO085_Dir_PID(float error)
{
    float result = 0;
    static int16_t err_last = 0;
    static float Integral = 0;

    Integral += error;
    if(Integral > Integral_Max) Integral = Integral_Max;
    if(Integral < -Integral_Max) Integral = -Integral_Max;

    result = dir_kp * error + dir_ki * Integral + dir_kd * (error - err_last);
    err_last = error;

    if(result > pid_max) result = pid_max;
    if(result < -pid_max) result = -pid_max;

    return -result;
}

/* 替代 AngleOffsetCalc - 角度偏移计算 */
void BNO085_AngleOffsetCalc(void)
{
    // BNO085 自动校准，这里简化处理
    calibratedYaw = yaw;
    calibratedPitch = pitch;
    calibratedRoll = roll;
}

/* 替代 Get_CalibratedAngles - 获取校准后的角度 */
void BNO085_Get_CalibratedAngles(void)
{
    // 转换 BNO085 的 yaw 到 0-360 度导航坐标系
    // BNO085 的 yaw 已经是 0-360 度

    if(Filter_out < -2.0f) {
        calibratedYaw = -Filter_out;
    } else if(Filter_out >= 2.0f) {
        calibratedYaw = 360.0f - Filter_out;
    } else {
        calibratedYaw = 0.0f;
    }
}

/* 替代 Get_Angle - 通用角度获取 */
void BNO085_Get_Angle(uint8_t way)
{
    if(way == 1) {
        // 方式1: 直接读取 BNO085 (内置融合)
        BNO085_Get_EulerAngles();
        delay_ms(10);
    } else {
        // 方式2/3: 读取原始数据 (如果需要自己的滤波算法)
        BNO085_ReadGyro_Short(angle);
        BNO085_ReadAccel_Short(accel);

        // 计算加速度角度
        Accel_Angle_X = atan2f((float)accel[0], (float)accel[2]) * 180.0f / Pi;
        Accel_Angle_Y = atan2f((float)accel[1], (float)accel[2]) * 180.0f / Pi;

        accel_z = accel[2] * 1.962f / 32768.0f;
        gyro_z = angle[2] * 2000.0f / 32768.0f;

        delay_ms(10);

        if(way == 2) {
            // 卡尔曼滤波 (如果实现了 Kalman_Filter_x/y)
            // pitch = -Kalman_Filter_x(Accel_Angle_X, gyro_x);
            // roll = -Kalman_Filter_y(Accel_Angle_Y, gyro_y);
        } else if(way == 3) {
            // 互补滤波 (如果实现了 Complementary_Filter_x/y)
            // pitch = -Complementary_Filter_x(Accel_Angle_X, gyro_x);
            // roll = -Complementary_Filter_y(Accel_Angle_Y, gyro_y);
        }

        // 更新平衡角度
        Angle_Balance = pitch;
        Gyro_Turn = angle[2];
        Acceleration_Z = accel[2];
    }
}

/* 替代 navigetion_0_360_limit - 限制角度在 0-360 度 */
float BNO085_navigetion_0_360_limit(float angle)
{
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f) angle += 360.0f;
    return angle;
}

/* 替代 get_minor_arc - 计算小圆弧偏差 */
float BNO085_get_minor_arc(float azimuth, float headingAngle)
{
    float angle_err = 0.0f;

    if(azimuth >= 180.0f + headingAngle) {
        angle_err = azimuth - headingAngle - 360.0f;
    } else if(headingAngle > 180.0f + azimuth) {
        angle_err = azimuth - headingAngle + 360.0f;
    } else {
        angle_err = azimuth - headingAngle;
    }

    return -angle_err;
}
