#include "usart0.h"

void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
}

void uart0_send_string(char *str)
{
    if (str == NULL) {
        return;
    }
    while (*str != '\0') {
        uart0_send_char(*str++);
    }
}

/* UART0 is dedicated to the tracking module. Discard printf output so a
 * diagnostic string can never corrupt the module command protocol. */
#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif
FILE __stdout;
void _sys_exit(int x)
{
    (void)x;
}
#endif

int fputc(int ch, FILE *stream)
{
    (void)stream;
    return ch;
}
