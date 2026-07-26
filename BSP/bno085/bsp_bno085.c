#include "bsp_bno085.h"
#include <string.h>
#include <math.h>

/* I2C GPIO 定义 - 复用 MPU6050 的引脚 */
#define BNO085_SDA_PIN      MPU6050_SDA_PIN
#define BNO085_SCL_PIN      MPU6050_SCL_PIN
#define BNO085_PORT         MPU6050_PORT
#define BNO085_SDA_IOMUX    MPU6050_SDA_IOMUX
#define BNO085_SCL_IOMUX    MPU6050_SCL_IOMUX

/* SHTP 通道 */
#define CHANNEL_EXEC        0x00    // 执行/控制通道
#define CHANNEL_WAKE        0x01    // 唤醒报告
#define CHANNEL_GYRO        0x02    // 陀螺仪
#define CHANNEL_GYRO_CALIB  0x03    // 陀螺仪校准
#define CHANNEL_ACCEL       0x04    // 加速度计
#define CHANNEL_ACCEL_CALIB 0x05    // 加速度计校准
#define CHANNEL_MAG         0x06    // 磁力计
#define CHANNEL_MAG_CALIB   0x07    // 磁力计校准
#define CHANNEL_PED         0x08    // 计步器
#define CHANNEL_CONT        0x09    // 控制
#define CHANNEL_ROTATION    0x0A    // 旋转矢量
#define CHANNEL_LINEAR      0x0B    // 线性加速度

/* 内部缓冲区 */
static uint8_t shtpData[256];       // SHTP 数据缓冲区
static uint16_t packetNumber = 0;   // 包序号

/* 传感器数据缓存 */
static BNO085_Quaternion_t quatData = {0};
static BNO085_Euler_t eulerData = {0};
static BNO085_Vector3_t accelData = {0};
static BNO085_Vector3_t gyroData = {0};
static BNO085_Vector3_t magData = {0};
static BNO085_CalibStatus_t calibStatus = {0};

/* 私有函数声明 */
static void BNO085_I2C_Start(void);
static void BNO085_I2C_Stop(void);
static void BNO085_I2C_SendByte(uint8_t dat);
static uint8_t BNO085_I2C_ReadByte(void);
static uint8_t BNO085_I2C_WaitAck(void);
static void BNO085_I2C_SendAck(uint8_t ack);
static void BNO085_I2C_Delay(void);

static uint8_t SHTP_SendPacket(uint8_t channel, uint8_t *data, uint16_t len);
static uint8_t SHTP_ReceivePacket(uint8_t *channel, uint8_t *data, uint16_t *len);
static uint8_t SHTP_GetAdvertisement(void);
static void ParseRotationVector(uint8_t *data, uint16_t len);
static void ParseAccelerometer(uint8_t *data, uint16_t len);
static void ParseGyroscope(uint8_t *data, uint16_t len);
static void ParseMagnetometer(uint8_t *data, uint16_t len);

static int16_t parse16bit(uint8_t *data);

/* I2C 时序函数 (GPIO 模拟) */
static void BNO085_I2C_Delay(void)
{
    delay_us(5);
}

static void BNO085_SDA_OUT(void)
{
    DL_GPIO_initDigitalOutput(BNO085_SDA_IOMUX);
    DL_GPIO_setPins(BNO085_PORT, BNO085_SDA_PIN);
    DL_GPIO_enableOutput(BNO085_PORT, BNO085_SDA_PIN);
}

static void BNO085_SDA_IN(void)
{
    DL_GPIO_initDigitalInput(BNO085_SDA_IOMUX);
}

static uint8_t BNO085_SDA_GET(void)
{
    return (DL_GPIO_readPins(BNO085_PORT, BNO085_SDA_PIN) & BNO085_SDA_PIN) ? 1 : 0;
}

static void BNO085_SDA(uint8_t x)
{
    if(x) DL_GPIO_setPins(BNO085_PORT, BNO085_SDA_PIN);
    else DL_GPIO_clearPins(BNO085_PORT, BNO085_SDA_PIN);
}

static void BNO085_SCL(uint8_t x)
{
    if(x) DL_GPIO_setPins(BNO085_PORT, BNO085_SCL_PIN);
    else DL_GPIO_clearPins(BNO085_PORT, BNO085_SCL_PIN);
}

static void BNO085_I2C_Start(void)
{
    BNO085_SDA_OUT();
    BNO085_SCL(1);
    BNO085_SDA(1);
    BNO085_I2C_Delay();
    BNO085_SDA(0);
    BNO085_I2C_Delay();
    BNO085_SCL(0);
}

static void BNO085_I2C_Stop(void)
{
    BNO085_SDA_OUT();
    BNO085_SCL(0);
    BNO085_SDA(0);
    BNO085_I2C_Delay();
    BNO085_SCL(1);
    BNO085_I2C_Delay();
    BNO085_SDA(1);
    BNO085_I2C_Delay();
}

static uint8_t BNO085_I2C_WaitAck(void)
{
    uint8_t ack = 0;
    uint8_t ack_flag = 10;
    BNO085_SCL(0);
    BNO085_SDA(1);
    BNO085_SDA_IN();
    BNO085_SCL(1);
    while(BNO085_SDA_GET() && ack_flag) {
        ack_flag--;
        BNO085_I2C_Delay();
    }
    if(ack_flag <= 0) {
        BNO085_I2C_Stop();
        return 1;
    }
    BNO085_SCL(0);
    BNO085_SDA_OUT();
    return ack;
}

static void BNO085_I2C_SendAck(uint8_t ack)
{
    BNO085_SDA_OUT();
    BNO085_SCL(0);
    BNO085_SDA(0);
    BNO085_I2C_Delay();
    if(!ack) BNO085_SDA(0);
    else BNO085_SDA(1);
    BNO085_SCL(1);
    BNO085_I2C_Delay();
    BNO085_SCL(0);
    BNO085_SDA(1);
}

static void BNO085_I2C_SendByte(uint8_t dat)
{
    int i;
    BNO085_SDA_OUT();
    BNO085_SCL(0);
    for(i = 0; i < 8; i++) {
        BNO085_SDA((dat & 0x80) >> 7);
        BNO085_I2C_Delay();
        BNO085_SCL(1);
        BNO085_I2C_Delay();
        BNO085_SCL(0);
        BNO085_I2C_Delay();
        dat <<= 1;
    }
}

static uint8_t BNO085_I2C_ReadByte(void)
{
    uint8_t i, receive = 0;
    BNO085_SDA_IN();
    for(i = 0; i < 8; i++) {
        BNO085_SCL(0);
        BNO085_I2C_Delay();
        BNO085_SCL(1);
        BNO085_I2C_Delay();
        receive <<= 1;
        if(BNO085_SDA_GET()) receive |= 1;
        BNO085_I2C_Delay();
    }
    BNO085_SCL(0);
    return receive;
}

/* 基础 I2C 读写 */
char BNO085_WriteData(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint16_t i;
    BNO085_I2C_Start();
    BNO085_I2C_SendByte((BNO085_I2C_ADDR << 1) | 0);
    if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return 1; }
    BNO085_I2C_SendByte(reg);
    if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return 2; }
    for(i = 0; i < len; i++) {
        BNO085_I2C_SendByte(data[i]);
        if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return (3 + i); }
    }
    BNO085_I2C_Stop();
    return 0;
}

char BNO085_ReadData(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    BNO085_I2C_Start();
    BNO085_I2C_SendByte((BNO085_I2C_ADDR << 1) | 0);
    if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return 1; }
    BNO085_I2C_SendByte(reg);
    if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return 2; }
    BNO085_I2C_Start();
    BNO085_I2C_SendByte((BNO085_I2C_ADDR << 1) | 1);
    if(BNO085_I2C_WaitAck()) { BNO085_I2C_Stop(); return 3; }
    for(i = 0; i < (len - 1); i++) {
        data[i] = BNO085_I2C_ReadByte();
        BNO085_I2C_SendAck(0);
    }
    data[i] = BNO085_I2C_ReadByte();
    BNO085_I2C_SendAck(1);
    BNO085_I2C_Stop();
    return 0;
}

uint8_t BNO085_ReadByte(uint8_t reg)
{
    uint8_t res;
    BNO085_ReadData(reg, &res, 1);
    return res;
}

/* 解析辅助函数 */
static int16_t parse16bit(uint8_t *data)
{
    return (int16_t)((data[1] << 8) | data[0]);
}

/* 四元数转欧拉角 */
void BNO085_QuaternionToEuler(const BNO085_Quaternion_t *quat, BNO085_Euler_t *euler)
{
    float qw = quat->qw;
    float qx = quat->qx;
    float qy = quat->qy;
    float qz = quat->qz;

    // Roll (X)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    euler->roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / 3.14159265f;

    // Pitch (Y)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if(fabs(sinp) >= 1.0f)
        euler->pitch = copysign(90.0f, sinp);
    else
        euler->pitch = asin(sinp) * 180.0f / 3.14159265f;

    // Yaw (Z) - 转换为 0-360 度
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    euler->yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / 3.14159265f;
    if(euler->yaw < 0) euler->yaw += 360.0f;

    euler->accuracy = quat->accuracy;
}

/* SHTP 协议 - 发送数据包 */
static uint8_t SHTP_SendPacket(uint8_t channel, uint8_t *data, uint16_t len)
{
    uint8_t header[4];
    uint16_t totalLen = len + 4;  // 包括包头

    // 构建 SHTP 包头 (小端序)
    header[0] = totalLen & 0xFF;
    header[1] = (totalLen >> 8) & 0xFF;
    header[2] = channel;
    header[3] = packetNumber++ & 0xFF;

    // 发送包头
    if(BNO085_WriteData(0, header, 4) != 0) return 1;

    // 发送数据
    if(len > 0) {
        if(BNO085_WriteData(0, data, len) != 0) return 2;
    }

    return 0;
}

/* SHTP 协议 - 接收数据包 */
static uint8_t SHTP_ReceivePacket(uint8_t *channel, uint8_t *data, uint16_t *len)
{
    uint8_t header[4];
    uint16_t packetLen;

    // 读取 4 字节包头
    if(BNO085_ReadData(0, header, 4) != 0) return 1;

    // 解析包长度 (小端序)
    packetLen = header[0] | (header[1] << 8);
    if(packetLen < 4) return 2;

    // 提取通道和序号
    *channel = header[2];
    // packetNumber = header[3]; // 可验证

    // 计算有效载荷长度
    *len = packetLen - 4;
    if(*len > 0) {
        // 读取有效载荷
        if(BNO085_ReadData(0, data, *len) != 0) return 3;
    }

    return 0;
}

/* 使能旋转矢量报告 */
void BNO085_EnableRotationVector(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    // 特征报告: 使能旋转矢量
    cmd[0] = SENSOR_REPORTID_ROTATION_VECTOR;
    cmd[1] = 0;             // 标志
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    cmd[6] = 0;             // 传感器特定配置
    cmd[7] = 0;
    cmd[8] = 0;
    cmd[9] = 0;
    cmd[10] = 0;
    cmd[11] = 0;
    cmd[12] = 0;
    cmd[13] = 0;
    cmd[14] = 0;
    cmd[15] = 0;

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 使能游戏旋转矢量 (无磁力计，适合游戏) */
void BNO085_EnableGameRotationVector(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    cmd[0] = SENSOR_REPORTID_GAME_ROTATION_VECTOR;
    cmd[1] = 0;
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    memset(&cmd[6], 0, 10);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 使能加速度计 */
void BNO085_EnableAccelerometer(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    cmd[0] = SENSOR_REPORTID_ACCELEROMETER;
    cmd[1] = 0;
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    memset(&cmd[6], 0, 10);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 使能陀螺仪 */
void BNO085_EnableGyroscope(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    cmd[0] = SENSOR_REPORTID_GYROSCOPE;
    cmd[1] = 0;
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    memset(&cmd[6], 0, 10);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 使能磁力计 */
void BNO085_EnableMagnetometer(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    cmd[0] = SENSOR_REPORTID_MAGNETIC_FIELD;
    cmd[1] = 0;
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    memset(&cmd[6], 0, 10);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 使能线性加速度 */
void BNO085_EnableLinearAccelerometer(uint32_t interval_ms)
{
    uint8_t cmd[16];
    uint16_t interval_us = (uint16_t)(interval_ms * 1000);

    cmd[0] = SENSOR_REPORTID_LINEAR_ACCELERATION;
    cmd[1] = 0;
    cmd[2] = interval_us & 0xFF;
    cmd[3] = (interval_us >> 8) & 0xFF;
    cmd[4] = (interval_us >> 16) & 0xFF;
    cmd[5] = (interval_us >> 24) & 0xFF;
    memset(&cmd[6], 0, 10);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 16);
}

/* 解析旋转矢量报告 */
static void ParseRotationVector(uint8_t *data, uint16_t len)
{
    if(len < 12) return;

    // Q 点精度为 14 (2^14)
    float q14 = 1.0f / (1 << 14);

    quatData.qi = parse16bit(&data[0]) * q14;
    quatData.qy = parse16bit(&data[2]) * q14;
    quatData.qz = parse16bit(&data[4]) * q14;
    quatData.qw = parse16bit(&data[6]) * q14;

    // 如果包含精度估计
    if(len >= 14) {
        quatData.accuracy = parse16bit(&data[10]) * q14;
    }

    // 转换为欧拉角
    BNO085_QuaternionToEuler(&quatData, &eulerData);
}

/* 解析加速度计报告 */
static void ParseAccelerometer(uint8_t *data, uint16_t len)
{
    if(len < 6) return;

    float q12 = 1.0f / (1 << 12);  // 加速度 Q 点精度为 12

    accelData.x = parse16bit(&data[0]) * q12;
    accelData.y = parse16bit(&data[2]) * q12;
    accelData.z = parse16bit(&data[4]) * q12;
}

/* 解析陀螺仪报告 */
static void ParseGyroscope(uint8_t *data, uint16_t len)
{
    if(len < 6) return;

    float q12 = 1.0f / (1 << 12);

    gyroData.x = parse16bit(&data[0]) * q12;
    gyroData.y = parse16bit(&data[2]) * q12;
    gyroData.z = parse16bit(&data[4]) * q12;
}

/* 解析磁力计报告 */
static void ParseMagnetometer(uint8_t *data, uint16_t len)
{
    if(len < 6) return;

    float q12 = 1.0f / (1 << 12);

    magData.x = parse16bit(&data[0]) * q12;
    magData.y = parse16bit(&data[2]) * q12;
    magData.z = parse16bit(&data[4]) * q12;
}

/* 数据读取函数实现 */
void BNO085_ReadQuaternion(BNO085_Quaternion_t *quat)
{
    uint8_t channel;
    uint16_t len;

    // 尝试接收数据并解析
    while(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
        if(channel == CHANNEL_ROTATION) {
            ParseRotationVector(shtpData, len);
        } else if(channel == CHANNEL_ACCEL) {
            ParseAccelerometer(shtpData, len);
        } else if(channel == CHANNEL_GYRO) {
            ParseGyroscope(shtpData, len);
        } else if(channel == CHANNEL_MAG) {
            ParseMagnetometer(shtpData, len);
        }
    }

    if(quat) {
        memcpy(quat, &quatData, sizeof(BNO085_Quaternion_t));
    }
}

void BNO085_ReadEuler(BNO085_Euler_t *euler)
{
    // 先读取四元数，内部会自动转换
    BNO085_ReadQuaternion(NULL);

    if(euler) {
        memcpy(euler, &eulerData, sizeof(BNO085_Euler_t));
    }
}

void BNO085_ReadAccel(BNO085_Vector3_t *accel)
{
    uint8_t channel;
    uint16_t len;

    while(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
        if(channel == CHANNEL_ACCEL) {
            ParseAccelerometer(shtpData, len);
        }
    }

    if(accel) {
        memcpy(accel, &accelData, sizeof(BNO085_Vector3_t));
    }
}

void BNO085_ReadGyro(BNO085_Vector3_t *gyro)
{
    uint8_t channel;
    uint16_t len;

    while(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
        if(channel == CHANNEL_GYRO) {
            ParseGyroscope(shtpData, len);
        }
    }

    if(gyro) {
        memcpy(gyro, &gyroData, sizeof(BNO085_Vector3_t));
    }
}

void BNO085_ReadMag(BNO085_Vector3_t *mag)
{
    uint8_t channel;
    uint16_t len;

    while(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
        if(channel == CHANNEL_MAG) {
            ParseMagnetometer(shtpData, len);
        }
    }

    if(mag) {
        memcpy(mag, &magData, sizeof(BNO085_Vector3_t));
    }
}

void BNO085_ReadLinearAccel(BNO085_Vector3_t *linAccel)
{
    uint8_t channel;
    uint16_t len;

    while(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
        if(channel == CHANNEL_LINEAR) {
            // 解析线性加速度 (格式同加速度)
            if(len >= 6) {
                float q12 = 1.0f / (1 << 12);
                linAccel->x = parse16bit(&shtpData[0]) * q12;
                linAccel->y = parse16bit(&shtpData[2]) * q12;
                linAccel->z = parse16bit(&shtpData[4]) * q12;
                return;
            }
        }
    }
}

/* 与 MPU6050 接口兼容的函数 */
void BNO085_ReadGyro_Short(short *gyroData)
{
    BNO085_Vector3_t gyro;
    BNO085_ReadGyro(&gyro);

    // 转换为 MPU6050 原始数据格式 (约 2000dps 量程)
    // BNO085 输出的是 dps，需要转换回原始 ADC 值
    gyroData[0] = (short)(gyro.x * 32768.0f / 2000.0f);
    gyroData[1] = (short)(gyro.y * 32768.0f / 2000.0f);
    gyroData[2] = (short)(gyro.z * 32768.0f / 2000.0f);
}

void BNO085_ReadAccel_Short(short *accData)
{
    BNO085_Vector3_t accel;
    BNO085_ReadAccel(&accel);

    // 转换为 MPU6050 原始数据格式 (2g 量程)
    accData[0] = (short)(accel.x * 32768.0f / 2.0f);
    accData[1] = (short)(accel.y * 32768.0f / 2.0f);
    accData[2] = (short)(accel.z * 32768.0f / 2.0f);
}

float BNO085_GetTemp(void)
{
    // BNO085 没有直接输出温度
    // 返回一个默认值或从其他传感器获取
    return 25.0f;
}

/* 初始化 */
char BNO085_Init(void)
{
    uint8_t channel;
    uint16_t len;
    uint8_t retry = 5;

    // 初始化 GPIO
    BNO085_SDA_OUT();
    BNO085_SCL(1);
    delay_ms(10);

    // 等待 BNO085 启动
    delay_ms(100);

    // 尝试读取产品 ID
    while(retry--) {
        // 发送产品 ID 请求
        uint8_t cmd = SHTP_REPORT_PRODUCT_ID_REQUEST;
        if(SHTP_SendPacket(CHANNEL_EXEC, &cmd, 1) == 0) {
            delay_ms(10);

            // 接收响应
            if(SHTP_ReceivePacket(&channel, shtpData, &len) == 0) {
                if(channel == CHANNEL_EXEC && shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                    // 解析产品 ID
                    uint32_t swPartNumber = shtpData[2] | (shtpData[3] << 8) | (shtpData[4] << 16) | (shtpData[5] << 24);
                    uint32_t swVersion = shtpData[6] | (shtpData[7] << 8) | (shtpData[8] << 16);

                    printf("BNO085 Found! Part: 0x%08X, Ver: 0x%06X\r\n", swPartNumber, swVersion);

                    // 使能传感器报告
                    BNO085_EnableRotationVector(10);      // 100Hz 旋转矢量
                    BNO085_EnableAccelerometer(10);         // 100Hz 加速度
                    BNO085_EnableGyroscope(10);             // 100Hz 陀螺仪
                    BNO085_EnableMagnetometer(10);          // 100Hz 磁力计

                    return 0;  // 初始化成功
                }
            }
        }
        delay_ms(50);
    }

    printf("BNO085 Init Failed!\r\n");
    return 1;
}

/* 复位 */
uint8_t BNO085_Reset(void)
{
    // 软件复位命令
    uint8_t cmd[4];
    cmd[0] = 1;  // 复位请求
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;

    // 发送复位命令
    // 实际硬件复位需要拉低 NRST 引脚

    delay_ms(100);
    return 0;
}

/* 校准相关函数 */
void BNO085_GetCalibStatus(BNO085_CalibStatus_t *status)
{
    if(status) {
        status->accel = 3;  // BNO085 自动校准，返回最大精度
        status->gyro = 3;
        status->mag = 3;
        status->planar = 3;
    }
}

void BNO085_Calibrate(void)
{
    // BNO085 自动校准，无需手动操作
    // 但可以发送 ME 校准命令
    uint8_t cmd[17];
    cmd[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd[1] = 0;
    cmd[2] = COMMAND_ME_CALIBRATE;
    cmd[3] = 0;
    cmd[4] = 0x07;  // 校准所有传感器
    memset(&cmd[5], 0, 12);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 17);
}

void BNO085_SaveCalibration(void)
{
    // 保存校准数据到 FRS (Flash Record System)
    // 需要具体实现
}

void BNO085_ClearTare(void)
{
    // 清除 Tare 设置
    uint8_t cmd[17];
    cmd[0] = SHTP_REPORT_COMMAND_REQUEST;
    cmd[1] = 0;
    cmd[2] = COMMAND_TARE;
    cmd[3] = 0;
    cmd[4] = 0;  // 清除 Tare
    memset(&cmd[5], 0, 12);

    SHTP_SendPacket(CHANNEL_EXEC, cmd, 17);
}

/* 外部包装函数 */
uint16_t BNO085_GetPacketHeader(uint16_t *payloadLen)
{
    // 保留接口
    *payloadLen = 0;
    return 0;
}

uint8_t BNO085_SendPacket(uint8_t channel, uint8_t *data, uint16_t len)
{
    return SHTP_SendPacket(channel, data, len);
}

uint8_t BNO085_ReceivePacket(uint8_t *channel, uint8_t *data, uint16_t *len)
{
    return SHTP_ReceivePacket(channel, data, len);
}
