#include "bsp_key.h"

uint16_t g_key1_long_press = 0;
uint16_t g_key2_long_press = 0;
uint16_t g_key3_long_press = 0;

static uint8_t Key1_is_Press(void)
{
    return ((DL_GPIO_readPins(KEY_button1_PORT, KEY_button1_PIN) &
        KEY_button1_PIN) != 0U) ? KEY_PRESS : KEY_RELEASE;
}

static uint8_t Key2_is_Press(void)
{
    return ((DL_GPIO_readPins(KEY_button2_PORT, KEY_button2_PIN) &
        KEY_button2_PIN) != 0U) ? KEY_PRESS : KEY_RELEASE;
}

static uint8_t Key3_is_Press(void)
{
    return ((DL_GPIO_readPins(KEY_button3_PORT, KEY_button3_PIN) &
        KEY_button3_PIN) != 0U) ? KEY_PRESS : KEY_RELEASE;
}

static uint8_t Key_OneShot(uint8_t pressed, uint8_t mode, uint8_t *state)
{
    if (pressed == KEY_RELEASE) {
        *state = 0U;
        return KEY_RELEASE;
    }
    if (mode == KEY_MODE_ALWAYS) {
        return KEY_PRESS;
    }
    if (*state == 0U) {
        *state = 1U;
        return KEY_PRESS;
    }
    return KEY_RELEASE;
}

uint8_t Key1_State(uint8_t mode)
{
    static uint8_t state = 0;
    return Key_OneShot(Key1_is_Press(), mode, &state);
}

uint8_t Key2_State(uint8_t mode)
{
    static uint8_t state = 0;
    return Key_OneShot(Key2_is_Press(), mode, &state);
}

uint8_t Key3_State(uint8_t mode)
{
    static uint8_t state = 0;
    return Key_OneShot(Key3_is_Press(), mode, &state);
}

uint8_t Key1_Long_Press(uint16_t timeout)
{
    (void)timeout;
    return 0U;
}

uint8_t Key2_Long_Press(uint16_t timeout)
{
    (void)timeout;
    return 0U;
}

uint8_t Key3_Long_Press(uint16_t timeout)
{
    (void)timeout;
    return 0U;
}

uint8_t KEY_Scan(void)
{
    static uint8_t latched = 0;

    if ((Key1_is_Press() == KEY_RELEASE) &&
        (Key2_is_Press() == KEY_RELEASE)) {
        latched = 0U;
        return 0U;
    }
    if (latched != 0U) {
        return 0U;
    }

    delay_ms(10);
    if (Key1_is_Press() == KEY_PRESS) {
        latched = 1U;
        return KEY1_PRES;
    }
    if (Key2_is_Press() == KEY_PRESS) {
        latched = 1U;
        return KEY2_PRES;
    }
    return 0U;
}

uint8_t get_key(void)
{
    return KEY_Scan();
}
