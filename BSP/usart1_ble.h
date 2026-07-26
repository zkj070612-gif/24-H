#ifndef __USART1_BLE_H_
#define __USART1_BLE_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  HC-06 BLE 模块 UART1 驱动
 *
 * 实际模块型号: hc06V2.3_le (BLE 从机/主机一体)
 * 服务 UUID: FFE0, 特征 FFE1 (Write/Notify)
 *
 * 接线 (UART1):
 *   TX (PB4) → HC-06 RXD
 *   RX (PA9) → HC-06 TXD
 *   波特率: 9600, 8N1
 *
 * AT 指令实测结果 (hc06V2.3_le 固件):
 *   ┌──────────────┬─────────────────┬─────────────────────────────┐
 *   │ 指令         │ 回复            │ 说明                        │
 *   ├──────────────┼─────────────────┼─────────────────────────────┤
 *   │ AT           │ OK              │ 基础测试                    │
 *   │ AT+VERSION   │ hc06V2.3_le     │ 固件版本                    │
 *   │ AT+ROLE=M    │ OK+ROLE:M       │ ✅ 设主模式 (Master)        │
 *   │ AT+ROLE=S    │ OK+ROLE:S       │ ✅ 设从模式 (Slave)         │
 *   │ AT+NAME=xxx  │ OK+NAME:xxx     │ ✅ 修改蓝牙名称             │
 *   │ AT+PIN=xxxxxx│ OK+PIN:xxxxxx   │ ✅ 设置六位数字密码         │
 *   │ AT+RESET     │ OK+RESET        │ ✅ 软件复位                 │
 *   │ AT+BAUD=xxxx │ (待验证)        │ ⚠️ 可能不支持或格式不同     │
 *   └──────────────┴─────────────────┴─────────────────────────────┘
 *
 * 注意:
 *   - AT+ROLE? 查询格式无回复，设值必须用 =M / =S 字母格式。
 *   - AT+PIN 指令设置六位数字密码，用于传统蓝牙配对（BLE 模块可能用配对确认）。
 *   - AT+BAUD 待验证，如需改波特率建议单独测试。
 *   - 未连接蓝牙时（LED 快闪）AT 指令可用；已连接时 AT 模式失效。
 *   - 主模式时 LED 快闪=搜索中，慢闪=已连接从机。
 */

void BLE_UART1_Init(void);
bool BLE_UART1_ReadByte(uint8_t *data);
void BLE_UART1_SendByte(uint8_t data);
void BLE_UART1_SendString(const char *text);
uint32_t BLE_UART1_GetDroppedCount(void);

/* AT 指令快捷接口 */
void BLE_SetRoleMaster(void);   /* AT+ROLE=M */
void BLE_SetRoleSlave(void);    /* AT+ROLE=S */
void BLE_SetBaud(uint32_t baud); /* AT+BAUD=xxxx */
void BLE_SetName(const char *name); /* AT+NAME=xxx */
void BLE_SetPIN(const char *pin);   /* AT+PIN=xxxxxx (6位数字密码) */
void BLE_Reset(void);           /* AT+RESET */

#endif
