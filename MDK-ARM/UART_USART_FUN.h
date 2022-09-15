#ifndef UART_USART_FUN_H_
#define UART_USART_FUN_H_

#include "Struct_typedef.h"
#include "main.h"
#include "usart.h"

extern int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout);
extern void Uart_Send(uint8_t *buf , uint16_t len);

#endif

