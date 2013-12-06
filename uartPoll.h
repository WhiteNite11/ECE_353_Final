#ifndef __UART_POLL_H__
#define __UART_POLL_H__

#include <stdint.h>
#include "UART.h"

void uartTxPollString(uint32_t base, char *data);
void uartTxPollChar(uint32_t base, char data);

#endif
