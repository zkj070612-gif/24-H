#include "usart1_ble.h"

#include "ti_msp_dl_config.h"

#define BLE_UART1_RX_BUFFER_SIZE 64U
#define BLE_UART1_RX_BUFFER_MASK (BLE_UART1_RX_BUFFER_SIZE - 1U)

static volatile uint8_t s_rx_buffer[BLE_UART1_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile uint32_t s_dropped_count;

void BLE_UART1_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_dropped_count = 0U;

    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

bool BLE_UART1_ReadByte(uint8_t *data)
{
    uint8_t tail;

    if (data == NULL) {
        return false;
    }

    tail = s_rx_tail;
    if (tail == s_rx_head) {
        return false;
    }

    *data = s_rx_buffer[tail];
    s_rx_tail = (uint8_t)((tail + 1U) & BLE_UART1_RX_BUFFER_MASK);
    return true;
}

void BLE_UART1_SendByte(uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, data);
}

void BLE_UART1_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        BLE_UART1_SendByte((uint8_t)*text++);
    }
}

uint32_t BLE_UART1_GetDroppedCount(void)
{
    return s_dropped_count;
}

/* AT 指令快捷实现 --------------------------------------------------------- */

void BLE_SetRoleMaster(void)
{
    BLE_UART1_SendString("AT+ROLE=M\r\n");
}

void BLE_SetRoleSlave(void)
{
    BLE_UART1_SendString("AT+ROLE=S\r\n");
}

void BLE_SetBaud(uint32_t baud)
{
    /* hc06V2.3_le 固件可能不支持 AT+BAUD 指令，
     * 此函数保留接口但效果待验证。
     * 如需改波特率，建议通过 USB-TTL 单独测试确认。
     */
    (void)baud; /* 当前未实现，避免编译警告 */
}

void BLE_SetName(const char *name)
{
    BLE_UART1_SendString("AT+NAME=");
    BLE_UART1_SendString(name);
    BLE_UART1_SendString("\r\n");
}

void BLE_SetPIN(const char *pin)
{
    /*
     * AT+PIN=xxxxxx 设置六位配对密码
     * 注：部分 BLE 模块可能不支持传统 PIN 码，使用配对确认方式
     * 若模块支持，密码应为 6 位数字
     */
    BLE_UART1_SendString("AT+PIN=");
    BLE_UART1_SendString(pin);
    BLE_UART1_SendString("\r\n");
}

void BLE_Reset(void)
{
    BLE_UART1_SendString("AT+RESET\r\n");
}

void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_1_INST) != DL_UART_IIDX_RX) {
        return;
    }

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t data = DL_UART_Main_receiveData(UART_1_INST);
        uint8_t next = (uint8_t)((s_rx_head + 1U) & BLE_UART1_RX_BUFFER_MASK);

        if (next == s_rx_tail) {
            s_dropped_count++;
        } else {
            s_rx_buffer[s_rx_head] = data;
            s_rx_head = next;
        }
    }
}
