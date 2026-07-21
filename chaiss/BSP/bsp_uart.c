#include "bsp_uart.h"

void BSP_UART_SendChar(UART_Regs *uart, uint8_t data)
{
    while (DL_UART_isBusy(uart) == true) {
    
    }
    DL_UART_transmitData(uart, data);

}
void BSP_UART_SendString(UART_Regs *uart, uint8_t * data)
{
    while (*data!=0 && data!=0) 
    {
        BSP_UART_SendChar(uart, *data);
        data++;
    }

}

