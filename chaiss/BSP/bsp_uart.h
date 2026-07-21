#ifndef __BSP_UART_H_
#define __BSP_UART_H_

#include "ti_msp_dl_config.h"
void BSP_UART_SendChar(UART_Regs *uart, uint8_t data);
void BSP_UART_SendString(UART_Regs *uart, uint8_t * data);

#endif
