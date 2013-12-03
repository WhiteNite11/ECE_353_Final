//*****************************************************************************
//
//*****************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "lm4f120h5qr.h"
#include "board_config.h"
#include "team.h"


/******************************************************************************
 * Defines
 *****************************************************************************/
#define PORTA                 0x40004000
#define PORTB                 0x40005000
#define PORTC                 0x40006000
#define PORTD                 0x40007000
#define PORTE                 0x40024000
#define PORTF                 0x40025000

extern void PLL_Init(void);
extern void initPortC(void);

/******************************************************************************
 * Global Variables
 *****************************************************************************/
 GPIO_PORT *GpioPortA = (GPIO_PORT *)PORTA;
 GPIO_PORT *GpioPortB = (GPIO_PORT *)PORTB;
 GPIO_PORT *GpioPortC = (GPIO_PORT *)PORTC;
 GPIO_PORT *GpioPortD = (GPIO_PORT *)PORTD;
 GPIO_PORT *GpioPortE = (GPIO_PORT *)PORTE;
 GPIO_PORT *GpioPortF = (GPIO_PORT *)PORTF;

/******************************************************************************
 * Functions
 *****************************************************************************/
//*****************************************************************************
// Initialize the GPIO port
//*****************************************************************************
bool  gpioPortInit( 
                    uint32_t baseAddress, 
                    uint8_t digitalEnableMask, 
                    uint8_t inputMask,
					uint8_t outputMask, 
                    uint8_t pullUpMask
                  )
{
  uint32_t delay;
  GPIO_PORT *myPort = (GPIO_PORT *)baseAddress;
  
  // Validate that a correct base address has been passed
  // Turn on the Clock Gating Register
  switch (baseAddress) 
  {
    case PORTA :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
      break;
    case PORTB :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
      break;
    case PORTC :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
      break;
    case PORTD :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
      break;
    case PORTE :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
      break;
    case PORTF :
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
      break;
    default:
      return false;
  }

  // Delay a bit
  delay = SYSCTL_RCGCGPIO_R;
  
  // Set the Direction Register
  myPort->Direction                   &= ~inputMask;
  myPort->Direction					  |= outputMask;
  
  // Enable Pull-Up Resistors
  myPort->PullUpSelect                |= pullUpMask;
  
  // Disable the Alternate Function Select
  myPort->AlternateFunctionSelect     = 0;
  
  // Enable Pins as Digital Pins
  myPort->DigitalEnable               |= digitalEnableMask;
  
  return true;
}

//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
int 
main(void)
{
  // Initialize the PLLs so the the main CPU frequency is 80MHz
  PLL_Init();
  
  //Configure Port C
  initPortC();
  

}

