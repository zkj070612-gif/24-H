#include "bsp_encoder.h"
#include "bsp_ir_receiver.h"

static ENCODER_RES motorL_encoder;
static ENCODER_RES motorR_encoder;

int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;

void encoder_init(void)
{
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    Timer_20ms_Init();
}

ENCODER_DIR get_encoderL_dir(void)
{
    return motorL_encoder.dir;
}

ENCODER_DIR get_encoderR_dir(void)
{
    return motorR_encoder.dir;
}

void Encoder_Get_ALL(int *encoder_all)
{
    encoder_all[0] = motorL_encoder.ALLcount;
    encoder_all[1] = motorR_encoder.ALLcount;
}

void Encoder_Get_Temp(int *encoder_temp)
{
    encoder_temp[0] = motorL_encoder.count;
    encoder_temp[1] = motorR_encoder.count;
}

void encoder_update(void)
{
    long long left_delta;
    long long right_delta;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    left_delta = motorL_encoder.temp_count;
    right_delta = motorR_encoder.temp_count;
    motorL_encoder.temp_count = 0;
    motorR_encoder.temp_count = 0;
    if (primask == 0U) {
        __enable_irq();
    }

    motorL_encoder.count = (int)left_delta;
    motorR_encoder.count = (int)right_delta;
    motorL_encoder.ALLcount += motorL_encoder.count;
    motorR_encoder.ALLcount += motorR_encoder.count;
    motorL_encoder.dir = (motorL_encoder.count >= 0) ? FORWARD : REVERSAL;
    motorR_encoder.dir = (motorR_encoder.count >= 0) ? FORWARD : REVERSAL;

    /* x2 encoder pulses; absolute travel is used because competition motion
     * is always forward. */
    if (encoder_odometry_flag) {
        int left_abs = (motorL_encoder.count < 0) ? -motorL_encoder.count : motorL_encoder.count;
        int right_abs = (motorR_encoder.count < 0) ? -motorR_encoder.count : motorR_encoder.count;
        odometry_sum += (left_abs + right_abs) / 2;
    }
}

void Get_Odometry(void)
{
    /* Kept for source compatibility; encoder_update() owns accumulation. */
}

#if Motor_Switch
void GROUP1_IRQHandler(void)
{
    uint32_t pending_source = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    if ((pending_source & DL_INTERRUPT_GROUP1_IIDX_GPIOB) != 0U) {
        uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFFU);

        if ((status & GPIO_ENCODER_L_H1A_PIN) != 0U) {
            if (DL_GPIO_readPins(GPIOB, GPIO_ENCODER_L_H1B_PIN) == 0U) {
                motorL_encoder.temp_count++;
            } else {
                motorL_encoder.temp_count--;
            }
        }
        if ((status & GPIO_ENCODER_L_H1B_PIN) != 0U) {
            if (DL_GPIO_readPins(GPIOB, GPIO_ENCODER_L_H1A_PIN) == 0U) {
                motorL_encoder.temp_count--;
            } else {
                motorL_encoder.temp_count++;
            }
        }
        if ((status & (GPIO_ENCODER_L_H1A_PIN | GPIO_ENCODER_L_H1B_PIN)) != 0U) {
            DL_GPIO_clearInterruptStatus(GPIOB,
                GPIO_ENCODER_L_H1A_PIN | GPIO_ENCODER_L_H1B_PIN);
        }

        if ((status & GPIO_ENCODER_R_H2A_PIN) != 0U) {
            if (DL_GPIO_readPins(GPIOA, GPIO_ENCODER_R_H2B_PIN) == 0U) {
                motorR_encoder.temp_count--;
            } else {
                motorR_encoder.temp_count++;
            }
            DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_R_H2A_PIN);
        }

#if IRRMOTE
        if ((status & IRContorl_GET_OUT_PIN) != 0U) {
            if (GET_OUT == 0U) {
                receiving_infrared_data();
            }
            DL_GPIO_clearInterruptStatus(GPIOB, IRContorl_GET_OUT_PIN);
        }
#endif
    }

    if ((pending_source & DL_INTERRUPT_GROUP1_IIDX_GPIOA) != 0U) {
        uint32_t status = DL_GPIO_getEnabledInterruptStatus(
            GPIOA, GPIO_ENCODER_R_H2B_PIN);

        if ((status & GPIO_ENCODER_R_H2B_PIN) != 0U) {
            if (DL_GPIO_readPins(GPIOB, GPIO_ENCODER_R_H2A_PIN) == 0U) {
                motorR_encoder.temp_count++;
            } else {
                motorR_encoder.temp_count--;
            }
            DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_R_H2B_PIN);
        }
    }
}
#endif
