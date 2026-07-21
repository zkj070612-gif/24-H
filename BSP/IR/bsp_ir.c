#include "bsp_ir.h"

#define IR_FRAME_BUFFER_SIZE 80U

volatile u8 IR_Data_number[IR_Num] = {1, 1, 1, 1, 1, 1, 1, 1};
volatile u8 IR_recv_complete_flag = 0;
volatile u8 IR_data_parsed_flag = 0;
volatile u8 oledbuf[13] = {0};

static volatile char s_frame[IR_FRAME_BUFFER_SIZE];
static volatile uint16_t s_frame_length = 0;
static volatile uint8_t s_receiving = 0;
static volatile uint32_t s_frame_count = 0;
static volatile uint32_t s_last_frame_ms = 0;

void IR_usart_config(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

static bool IR_ParseDigitalFrame(const char *frame, uint8_t values[IR_Num])
{
    uint8_t found_mask = 0;
    const char *p = frame;

    if ((frame[0] != '$') || (frame[1] != 'D')) {
        return false;
    }

    while ((*p != '\0') && (*p != '#')) {
        if ((*p == 'x') && (p[1] >= '1') && (p[1] <= '8') &&
            (p[2] == ':') && ((p[3] == '0') || (p[3] == '1'))) {
            uint8_t index = (uint8_t)(p[1] - '1');
            values[index] = (uint8_t)(p[3] - '0');
            found_mask |= (uint8_t)(1U << index);
            p += 4;
        } else {
            p++;
        }
    }

    return (found_mask == 0xFFU) && (*p == '#');
}

void IRDataAnalysis(void)
{
    char local_frame[IR_FRAME_BUFFER_SIZE];
    uint8_t values[IR_Num];
    uint16_t length;

    if (IR_recv_complete_flag == 0U) {
        return;
    }

    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    length = s_frame_length;
    if (length >= IR_FRAME_BUFFER_SIZE) {
        length = IR_FRAME_BUFFER_SIZE - 1U;
    }
    for (uint16_t i = 0; i < length; i++) {
        local_frame[i] = (char)s_frame[i];
    }
    local_frame[length] = '\0';
    IR_recv_complete_flag = 0U;
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    if (IR_ParseDigitalFrame(local_frame, values)) {
        for (uint8_t i = 0; i < IR_Num; i++) {
            IR_Data_number[i] = values[i];
        }
        IR_data_parsed_flag = 1U;
        s_last_frame_ms = Get_Time();
        s_frame_count++;
    }
}

void UART_0_INST_IRQHandler(void)
{
    uint8_t data;

    if (DL_UART_getPendingInterrupt(UART_0_INST) != DL_UART_IIDX_RX) {
        return;
    }

    data = DL_UART_Main_receiveData(UART_0_INST);

    /* Keep one complete frame intact until the foreground has copied it. */
    if (IR_recv_complete_flag != 0U) {
        return;
    }

    if (data == '$') {
        s_receiving = 1U;
        s_frame_length = 0U;
        s_frame[s_frame_length++] = (char)data;
        return;
    }

    if (s_receiving == 0U) {
        return;
    }

    if (s_frame_length >= (IR_FRAME_BUFFER_SIZE - 1U)) {
        s_receiving = 0U;
        s_frame_length = 0U;
        return;
    }

    s_frame[s_frame_length++] = (char)data;
    if (data == '#') {
        s_frame[s_frame_length] = '\0';
        s_receiving = 0U;
        IR_recv_complete_flag = 1U;
    }
}

void IR_DATA(void)
{
    IRDataAnalysis();
}

bool IR_HasValidData(void)
{
    return (IR_data_parsed_flag != 0U);
}

uint32_t IR_LastFrameAgeMs(void)
{
    if (IR_data_parsed_flag == 0U) {
        return UINT32_MAX;
    }
    return Get_Time() - s_last_frame_ms;
}

uint32_t IR_GetFrameCount(void)
{
    return s_frame_count;
}
