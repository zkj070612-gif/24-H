#ifndef _BSP_BNO085_H_
#define _BSP_BNO085_H_

#include "AllHeader.h"

/**
 * @brief BNO085 9轴姿态传感器驱动
 * @details 内置 SH-2 传感器融合算法，直接输出四元数/欧拉角
 *          替代 MPU6050，解决航向角漂移问题
 *
 * I2C 地址: 0x4A (默认) 或 0x4B (ADR脚接高)
 *
 * 接线 (I2C):
 *   SCL (GPIO) → BNO085 SCL
 *   SDA (GPIO) → BNO085 SDA
 *   3.3V       → BNO085 VCC
 *   GND        → BNO085 GND
 *   3.3V       → BNO085 NRST (复位，可选)
 *   GPIO       → BNO085 INT (中断，可选)
 */

/* I2C 设备地址 */
#define BNO085_I2C_ADDR         0x4A    // 默认地址
#define BNO085_I2C_ADDR_ALT     0x4B    // ADR引脚接高电平

/* SHTP 报告 ID */
#define SHTP_REPORT_COMMAND_REQUEST             0xF2
#define SHTP_REPORT_COMMAND_RESPONSE            0xF1
#define SHTP_REPORT_PRODUCT_ID_REQUEST          0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE         0xF8
#define SHTP_REPORT_FRS_READ_REQUEST            0xF4
#define SHTP_REPORT_FRS_READ_RESPONSE           0xF3

/* 传感器报告 ID (特征报告) */
#define SENSOR_REPORTID_ACCELEROMETER           0x01
#define SENSOR_REPORTID_GYROSCOPE               0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD          0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION     0x04
#define SENSOR_REPORTID_ROTATION_VECTOR         0x05    // 游戏旋转矢量（无磁力计）
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION    0x09    // 地磁旋转矢量
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR    0x08    // 游戏旋转矢量
#define SENSOR_REPORTID_ARVR_STABILIZED_ROTATION 0x28 // AR/VR 稳定旋转

/* 命令 */
#define COMMAND_ME_CALIBRATE                    0x01
#define COMMAND_ERRORS                          0x01
#define COMMAND_COUNTER                         0x02
#define COMMAND_TARE                            0x03
#define COMMAND_INITIALIZE                      0x04

/* 校准状态 */
#define CALIB_STATUS_ACCEL                      0x01
#define CALIB_STATUS_GYRO                       0x02
#define CALIB_STATUS_MAG                        0x04
#define CALIB_STATUS_PLANAR_ACCEL               0x08

/* 数据精度 */
#define BNO085_Q_POINT_14                       14      // Q 点精度 (2^14)
#define BNO085_Q_POINT_12                       12      // 加速度/磁力计

/* 结构体定义 */
typedef struct {
    float qw;       // 四元数 W
    float qx;       // 四元数 X
    float qy;       // 四元数 Y
    float qz;       // 四元数 Z
    float accuracy; // 估计精度 (0-3)
} BNO085_Quaternion_t;

typedef struct {
    float yaw;      // 航向角 Z (0-360°)
    float pitch;    // 俯仰角 X (-90°~+90°)
    float roll;     // 横滚角 Y (-180°~+180°)
    float accuracy; // 估计精度 (0-3)
} BNO085_Euler_t;

typedef struct {
    float x;
    float y;
    float z;
} BNO085_Vector3_t;

typedef struct {
    uint8_t accel;
    uint8_t gyro;
    uint8_t mag;
    uint8_t planar;
} BNO085_CalibStatus_t;

/* 初始化函数 */
char BNO085_Init(void);
uint8_t BNO085_Reset(void);

/* 基本数据读取 */
void BNO085_ReadQuaternion(BNO085_Quaternion_t *quat);
void BNO085_ReadEuler(BNO085_Euler_t *euler);
void BNO085_ReadAccel(BNO085_Vector3_t *accel);
void BNO085_ReadGyro(BNO085_Vector3_t *gyro);
void BNO085_ReadMag(BNO085_Vector3_t *mag);
void BNO085_ReadLinearAccel(BNO085_Vector3_t *linAccel);

/* 校准相关 */
void BNO085_GetCalibStatus(BNO085_CalibStatus_t *status);
void BNO085_Calibrate(void);
void BNO085_SaveCalibration(void);
void BNO085_ClearTare(void);

/* 与 MPU6050 接口兼容的函数 */
void BNO085_ReadGyro_Short(short *gyroData);      // 兼容 MPU6050ReadGyro
void BNO085_ReadAccel_Short(short *accData);     // 兼容 MPU6050ReadAcc
float BNO085_GetTemp(void);                       // 兼容 MPU6050_GetTemp

/* 底层 I2C */
uint8_t BNO085_ReadByte(uint8_t reg);
char BNO085_WriteData(uint8_t reg, uint8_t *data, uint8_t len);
char BNO085_ReadData(uint8_t reg, uint8_t *data, uint8_t len);

/* SHTP 协议接口 */
uint16_t BNO085_GetPacketHeader(uint16_t *payloadLen);
uint8_t BNO085_SendPacket(uint8_t channel, uint8_t *data, uint16_t len);
uint8_t BNO085_ReceivePacket(uint8_t *channel, uint8_t *data, uint16_t *len);

/* 传感器使能 */
void BNO085_EnableRotationVector(uint32_t interval_ms);
void BNO085_EnableGameRotationVector(uint32_t interval_ms);
void BNO085_EnableAccelerometer(uint32_t interval_ms);
void BNO085_EnableGyroscope(uint32_t interval_ms);
void BNO085_EnableMagnetometer(uint32_t interval_ms);
void BNO085_EnableLinearAccelerometer(uint32_t interval_ms);

/* 四元数转欧拉角 */
void BNO085_QuaternionToEuler(const BNO085_Quaternion_t *quat, BNO085_Euler_t *euler);

#endif /* _BSP_BNO085_H_ */
