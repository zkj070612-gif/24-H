#ifndef _GET_BNO085_H_
#define _GET_BNO085_H_

#include "AllHeader.h"
#include "bsp_bno085.h"

/**
 * @brief BNO085 姿态获取接口
 * @details 与 get_mpu6050.h 接口兼容的包装层
 *          直接替换 MPU6050，使用 BNO085 的 9 轴融合输出
 */

/* 全局变量 - 与 MPU6050 兼容 */
extern float pitch, roll, yaw;
extern float yawBias, pitchBias, rollBias;
extern float calibratedYaw, calibratedPitch, calibratedRoll;
extern short angle[3];      // 陀螺仪原始数据
extern short accel[3];     // 加速度原始数据

/* 校准结构体 - 与 MPU6050 兼容 */
typedef struct
{
    float Xoffset;
    float Yoffset;
    float Zoffset;
} Bias_t;

/* 初始化函数 */
void BNO085_DMP_Init(void);  // 替代 DMP_Init

/* 数据获取函数 - 与 MPU6050 兼容 */
void BNO085_Get_EulerAngles(void);  // 替代 Get_EulerAngles
float BNO085_Get_Filter(float Yaw_Err);
float BNO085_Dir_PID(float error);

/* 角度计算 - 与 MPU6050 兼容 */
void BNO085_AngleOffsetCalc(void);
void BNO085_Get_CalibratedAngles(void);
void BNO085_Get_Angle(uint8_t way);

/* 导航辅助函数 */
float BNO085_navigetion_0_360_limit(float angle);
float BNO085_get_minor_arc(float azimuth, float headingAngle);

#endif /* _GET_BNO085_H_ */
