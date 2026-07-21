#include "AllHeader.h"

#define TRACKING_WARMUP_MS 20000U
#define TRACKING_RETRY_MS   1000U

static void WaitForTrackingModule(void)
{
    uint32_t last_display = UINT32_MAX;
    uint32_t last_request = 0U;

    Motor_Stop(1);
    while (Get_Time() < TRACKING_WARMUP_MS) {
        uint32_t remaining;
        Scheduler_Run();
        remaining = (TRACKING_WARMUP_MS - Get_Time() + 999U) / 1000U;
        if (remaining != last_display) {
            last_display = remaining;
            OLED_Clear();
            OLED_ShowString(0, 0, "IR warming:", 8, 1);
            OLED_ShowNum(88, 0, remaining, 2, 16, 1);
            OLED_ShowString(0, 20, "Keep car still", 8, 1);
            OLED_Refresh();
        }
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "Waiting IR UART", 8, 1);
    OLED_Refresh();

    while (!IR_HasValidData()) {
        uint32_t now = Get_Time();
        Scheduler_Run();
        IR_DATA();
        if ((now - last_request) >= TRACKING_RETRY_MS) {
            uart0_send_string("$0,0,1#");
            last_request = now;
        }
    }
}

int main(void)
{
    uint8_t mode;
    uint32_t last_display = 0U;

    bsp_init();
    Motor_Stop(1);
    Buzzer_close_state();
    Control_RGB_ALL(OFF);

    mode = switch_mode();
    WaitForTrackingModule();
    Competition_Init(mode);

    for (;;) {
        uint32_t now;
        Scheduler_Run();
        Competition_Update();
        now = Get_Time();

        if ((now - last_display) >= 100U) {
            last_display = now;
            OLED_Clear();
            OLED_ShowString(0, 0, "M:", 8, 1);
            OLED_ShowNum(16, 0, mode, 1, 8, 1);
            OLED_ShowString(32, 0, "L:", 8, 1);
            OLED_ShowNum(48, 0, Competition_GetLap(), 1, 8, 1);
            OLED_ShowString(64, 0, "S:", 8, 1);
            OLED_ShowNum(80, 0, Competition_GetState(), 1, 8, 1);
            OLED_ShowString(0, 12, "Yaw:", 8, 1);
            OLED_ShowSNum(32, 12, (int)calibratedYaw, 3, 8, 1);
            OLED_ShowString(0, 24, "IR:", 8, 1);
            for (uint8_t i = 0; i < IR_Num; i++) {
                OLED_ShowNum(24 + i * 10, 24, IR_Data_number[i], 1, 8, 1);
            }
            OLED_Refresh();
        }
    }
}
