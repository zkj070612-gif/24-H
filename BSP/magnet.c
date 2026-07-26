#include "magnet.h"

#include "ti_msp_dl_config.h"

static bool s_magnet_is_on;

void Magnet_Init(void)
{
    Magnet_Off();
}

void Magnet_On(void)
{
    DL_GPIO_setPins(MAGNET_PORT, MAGNET_CTRL_PIN);
    s_magnet_is_on = true;
}

void Magnet_Off(void)
{
    DL_GPIO_clearPins(MAGNET_PORT, MAGNET_CTRL_PIN);
    s_magnet_is_on = false;
}

bool Magnet_IsOn(void)
{
    return s_magnet_is_on;
}
