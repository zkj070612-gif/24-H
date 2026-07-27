#include "get_mpu6050.h"
#include "Filter.h"

#define PI_F                          3.14159265f
#define YAW_FILTER_ALPHA              0.25f
#define YAW_DRIFT_MIN_CAL_MS          3000U
#define YAW_DRIFT_MAX_STILL_RATE_DPS  3.0f

short angle[3] = {0};
short accel[3] = {0};
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;

volatile float kal_mpu_out = 0.0f;
volatile float Filter_out = 0.0f;
volatile float Filter_out_last = 0.0f;
volatile float Yaw_Err_Lowout_last = 0.0f;
volatile float Yaw_Err_Lowout = 0.0f;
volatile float erro_sumr = 0.0f;
volatile float error = 0.0f;

volatile float gyro_x = 0.0f;
volatile float gyro_y = 0.0f;
volatile float gyro_z = 0.0f;
volatile float accel_z = 0.0f;
volatile float Accel_Angle_X = 0.0f;
volatile float Accel_Angle_Y = 0.0f;

uint8_t Way_Angle = 2U;
Bias_t Angle = {0};
float Angle_Balance = 0.0f;
float Gyro_Balance = 0.0f;
float Gyro_Turn = 0.0f;
float Acceleration_Z = 0.0f;

float yawBias = 0.0f;
float pitchBias = 0.0f;
float rollBias = 0.0f;
float calibratedYaw = 0.0f;
float calibratedPitch = 0.0f;
float calibratedRoll = 0.0f;

float dir_kp = 2.0f;
float dir_ki = 0.0f;
float dir_kd = 5.0f;
int Integral_Max = 300;
int pid_max = 1000;

static uint8_t yawFilterInitialized = 0U;
static uint8_t yawDriftCalibrating = 0U;
static uint8_t yawDriftHasPrevious = 0U;
static uint32_t yawDriftPreviousMs = 0U;
static uint32_t yawDriftElapsedMs = 0U;
static uint32_t yawCorrectionStartMs = 0U;
static float yawDriftPrevious = 0.0f;
static float yawDriftDeltaSum = 0.0f;
static float yawDriftRateDps = 0.0f;

static float Wrap180(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value <= -180.0f) value += 360.0f;
    return value;
}

float navigetion_0_360_limit(float value)
{
    while (value >= 360.0f) value -= 360.0f;
    while (value < 0.0f) value += 360.0f;
    return value;
}

/*
 * Capture the present heading as the mission zero. The previous implementation
 * added calibratedYaw to itself and therefore never measured an offset.
 */
void AngleOffsetCalc(void)
{
    yawBias = Filter_out;
    pitchBias = pitch;
    rollBias = roll;
    calibratedYaw = 0.0f;
    calibratedPitch = 0.0f;
    calibratedRoll = 0.0f;
}

void Get_CalibratedAngles(void)
{
    /* Preserve the vehicle convention: clockwise rotation is positive. */
    calibratedYaw = navigetion_0_360_limit(yawBias - Filter_out);
    calibratedPitch = pitch - pitchBias;
    calibratedRoll = roll - rollBias;
}

float get_Filter(float yawInput)
{
    float delta;

    if (yawFilterInitialized == 0U) {
        Yaw_Err_Lowout = Wrap180(yawInput);
        Yaw_Err_Lowout_last = Yaw_Err_Lowout;
        yawFilterInitialized = 1U;
        return Yaw_Err_Lowout;
    }

    /*
     * Filter the shortest angular arc. Scalar filtering causes a false
     * 180-degree excursion when DMP yaw crosses -180/180 degrees.
     */
    delta = Wrap180(yawInput - Yaw_Err_Lowout_last);
    Yaw_Err_Lowout = Wrap180(
        Yaw_Err_Lowout_last + YAW_FILTER_ALPHA * delta);
    Yaw_Err_Lowout_last = Yaw_Err_Lowout;
    return Yaw_Err_Lowout;
}

void Get_EulerAngles(void)
{
    float newPitch;
    float newRoll;
    float newYaw;
    float correctedYaw;
    uint32_t now;

    /* Do not reuse partial/stale FIFO values when no complete packet exists. */
    if (mpu_dmp_get_data(&newPitch, &newRoll, &newYaw) != 0U) {
        return;
    }

    now = Get_Time();
    if (yawDriftCalibrating != 0U) {
        if (yawDriftHasPrevious != 0U) {
            uint32_t dt = now - yawDriftPreviousMs;
            if ((dt > 0U) && (dt <= 100U)) {
                float delta = Wrap180(newYaw - yawDriftPrevious);
                float rate = fabsf(delta) * 1000.0f / (float)dt;

                /*
                 * Exclude intervals where the chassis is clearly rotating.
                 * The startup screen asks the user to keep the car still.
                 */
                if (rate <= YAW_DRIFT_MAX_STILL_RATE_DPS) {
                    yawDriftDeltaSum += delta;
                    yawDriftElapsedMs += dt;
                }
            }
        }
        yawDriftPrevious = newYaw;
        yawDriftPreviousMs = now;
        yawDriftHasPrevious = 1U;
    }

    correctedYaw = newYaw -
        yawDriftRateDps * ((float)(now - yawCorrectionStartMs) * 0.001f);
    pitch = newPitch;
    roll = newRoll;
    yaw = Wrap180(correctedYaw);
    Filter_out = get_Filter(yaw);
}

/*
 * Measure residual yaw drift while the car is stationary during startup.
 * MPU6050 has no magnetometer, so this compensates a nearly constant bias;
 * it cannot provide an absolute compass heading.
 */
void MPU6050_StartYawDriftCalibration(void)
{
    yawDriftCalibrating = 1U;
    yawDriftHasPrevious = 0U;
    yawDriftElapsedMs = 0U;
    yawDriftDeltaSum = 0.0f;
    yawDriftRateDps = 0.0f;
    yawCorrectionStartMs = Get_Time();
}

void MPU6050_FinishYawDriftCalibration(void)
{
    if (yawDriftElapsedMs >= YAW_DRIFT_MIN_CAL_MS) {
        yawDriftRateDps =
            yawDriftDeltaSum * 1000.0f / (float)yawDriftElapsedMs;
    } else {
        yawDriftRateDps = 0.0f;
    }

    yawDriftCalibrating = 0U;
    yawCorrectionStartMs = Get_Time();
}

float Dir_PID(float headingError)
{
    float result;
    static float errorLast = 0.0f;
    static float integral = 0.0f;

    integral += headingError;
    if (integral > (float)Integral_Max) integral = (float)Integral_Max;
    if (integral < (float)-Integral_Max) integral = (float)-Integral_Max;

    result = dir_kp * headingError +
             dir_ki * integral +
             dir_kd * (headingError - errorLast);
    errorLast = headingError;

    if (result > (float)pid_max) result = (float)pid_max;
    if (result < (float)-pid_max) result = (float)-pid_max;
    return -result;
}

float get_minor_arc(float azimuth, float headingAngle)
{
    float angleError;

    if (azimuth >= (180.0f + headingAngle)) {
        angleError = azimuth - headingAngle - 360.0f;
    } else if (headingAngle > (180.0f + azimuth)) {
        angleError = azimuth - headingAngle + 360.0f;
    } else {
        angleError = azimuth - headingAngle;
    }
    return -angleError;
}

void Get_Angle(uint8_t way)
{
    if (way == 1U) {
        (void)mpu_dmp_get_data(&pitch, &roll, &yaw);
        delay_ms(10);
        return;
    }

    MPU6050ReadGyro(angle);
    MPU6050ReadAcc(accel);
    Accel_Angle_X = atan2f((float)accel[0], (float)accel[2]) *
                    180.0f / PI_F;
    Accel_Angle_Y = atan2f((float)accel[1], (float)accel[2]) *
                    180.0f / PI_F;
    accel_z = (float)accel[2] * 1.962f / 32768.0f;
    gyro_x = (float)angle[0] * 2000.0f / 32768.0f;
    gyro_y = (float)angle[1] * 2000.0f / 32768.0f;
    gyro_z = (float)angle[2] * 2000.0f / 32768.0f;
    delay_ms(10);

    if (way == 2U) {
        pitch = -Kalman_Filter_x(Accel_Angle_X, gyro_x);
        roll = -Kalman_Filter_y(Accel_Angle_Y, gyro_y);
    } else if (way == 3U) {
        pitch = -Complementary_Filter_x(Accel_Angle_X, gyro_x);
        roll = -Complementary_Filter_y(Accel_Angle_Y, gyro_y);
    }

    Angle_Balance = pitch;
    Gyro_Turn = (float)angle[2];
    Acceleration_Z = (float)accel[2];
}
