#include "task.h"

Task tasks[] = {
    {20, 0, encoder_update},
    {20, 0, Motion_Handle},
    {5, 0, Get_EulerAngles},
    {10, 0, Get_CalibratedAngles},
};

void Scheduler_Run(void)
{
    uint32_t now = Get_Time();
    for (uint32_t i = 0; i < (sizeof(tasks) / sizeof(tasks[0])); i++) {
        if ((now - tasks[i].last_call) >= tasks[i].interval) {
            tasks[i].task();
            tasks[i].last_call = now;
        }
    }
}
