#include "uartPoll.h"

/****************************************************************************
 * This routine transmits a character out the UART / COM port.
 * Only the lower 8 bits of the 'data' variable are transmitted.
 ****************************************************************************/
void uartTxPollString(uint32_t base, char *data)
{
  UART_PERIPH *myPeriph = (UART_PERIPH *)base;
  
  if ( data != 0)
  {
    while(*data != '\0')
    {
      while( ((myPeriph->Flag)&(UART_FR_TXFF)) != 0 );
      myPeriph->Data = *data;
      data++;
    }
  }
  return;
}

/****************************************************************************
 * This routine transmits a character out the UART / COM port.
 * Only the lower 8 bits of the 'data' variable are transmitted.
 ****************************************************************************/
void uartTxPollChar(uint32_t base, char data)
{
  UART_PERIPH *myPeriph = (UART_PERIPH *)base;
  
  while( ((myPeriph->Flag)&(UART_FR_TXFF)) != 0 );
  myPeriph->Data = data;

  return;
}
