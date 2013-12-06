//*****************************************************************************
//
//*****************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "lm4f120h5qr.h"
#include "board_config.h"
#include "team.h"
#include "led_chars.h"
#include "gpio.h"
#include "SPI.h"
#include "uartPoll.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define PORTA                 0x40004000
#define PORTB                 0x40005000
#define PORTC                 0x40006000
#define PORTD                 0x40007000
#define PORTE                 0x40024000
#define PORTF                 0x40025000

#define OUTPUT_ENABLE_B       0xEF
#define ENABLES_B             0x0F
#define nROW_0                ~PIN_0
#define RED_EN                PIN_4
#define GREEN_EN              PIN_5
#define BLUE_EN               PIN_6
#define ROW_EN                PIN_7
#define ENABLES_OFF           0x00
#define OUT_EN_B  						PIN_4


#define SAVED_MSG_LEN_MAX     20

#define POT_LEFT  1
#define POT_RIGHT 0

#define PHASE   1
#define POLARITY 1
/******************************************************************************
 * Global Variables
 *****************************************************************************/
 GPIO_PORT *GpioPortA = (GPIO_PORT *)PORTA;
 GPIO_PORT *GpioPortB = (GPIO_PORT *)PORTB;
 GPIO_PORT *GpioPortC = (GPIO_PORT *)PORTC;
 GPIO_PORT *GpioPortD = (GPIO_PORT *)PORTD;
 GPIO_PORT *GpioPortE = (GPIO_PORT *)PORTE;
 GPIO_PORT *GpioPortF = (GPIO_PORT *)PORTF;
 
 volatile bool refreshLED = false;
 volatile bool checkADC;
 volatile uint32_t pwm = 0;
 volatile bool nextLEDrow = false;
 volatile bool buttonPoll = false;
//*****************************************************************************
// External Functions
//*****************************************************************************
extern void PLL_Init(void);
extern void initPortC(void);

/******************************************************************************
 * Functions
 *****************************************************************************/
//*****************************************************************************
// Configure UART0 for Tx and Rx from computer
//*****************************************************************************
bool uartInitPolling(uint32_t base)
{
  uint32_t delay;
  UART_PERIPH *myUart;
  
  if ( base != UART0)
    return false;
  
  // *******************************************
  // Configure GPIO PA0 and PA1 to be UART Pins
  // *******************************************
  
  // Turn on the clock gating register for GPIO port A
  // Make sure not to turn of any of the other ports
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
  
  delay = SYSCTL_RCGCGPIO_R;
  
  // Set the Digital Enable
  GpioPortA->DigitalEnable |= PIN_0 | PIN_1;
  
  // Set the Alternate Function
  GpioPortA->AlternateFunctionSelect |=  PIN_0 | PIN_1;
  
  // Set the Port Control Register
  GpioPortA->PortControl |= GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX;
  
  
  // *******************************
  // Set up the UART registers
  // *******************************
  myUart = (UART_PERIPH *)base;
  
  // Eanble the clock gating register
  // ( Not found in the UART_PERIPH struct)
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
  
  delay = SYSCTL_RCGCUART_R;
  
  // Set the baud rate for 115200
  myUart->IntegerBaudRateDiv = 43;
  myUart->FracBaudRateDiv = 26;

  // Configure the Line Control for 8-n-1
  myUart->LineControl = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
  
  // Enable the UART - Need to enabel both TX and RX
  myUart->UARTControl = UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE ;
  
  // Wait until the UART is avaiable
  while( !(SYSCTL_PRUART_R & SYSCTL_PRUART_R0 ))
  {}
  
  delay = 500;
  while( delay != 0)
  {
    delay--;
  }
  
  return true;
}


//***************************************************************************
// This routine returns a character received from the UART/COM port.
// This routine blocks until a character is received
//***************************************************************************/
char uartRxPoll(uint32_t base)
{
  UART_PERIPH *myPeriph = (UART_PERIPH *)base;
  
  while(myPeriph->Flag & UART_FR_RXFE)
  {
    // Wait
  }
   return myPeriph->Data;
}

// *******************************************
// Configure GPIO PA5-PA2 as SPI
// *******************************************
void initializePortASpi0(void)
{
  uint32_t delay;
  
  // Turn on the clock gating register for GPIO port A
  // Make sure not to turn of any of the other ports
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
  
  // Delay while clock starts up
  delay = SYSCTL_RCGCGPIO_R;
  
  // Set the 4 pins used for the SPI interface in the Digital Enable Register
  GpioPortA->DigitalEnable |= PIN_5 | PIN_4 | PIN_3 | PIN_2;
  
  // Set the 4 pins used for the SPI interface in the Alternate Function Register
  GpioPortA->AlternateFunctionSelect |=  PIN_5 | PIN_4 | PIN_3 | PIN_2;
  
  // Set the Port Control Register ( See lm4f120h5qr.h starting at line 2045)
  GpioPortA->PortControl |= GPIO_PCTL_PA5_SSI0TX | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA3_SSI0FSS | GPIO_PCTL_PA2_SSI0CLK;
  
}

// *******************************************
// Configure SPI
// *******************************************
bool initializeSPI( uint32_t base, uint8_t phase, uint8_t polarity)
{
  uint32_t delay;
  SPI_PERIPH *myPeriph = (SPI_PERIPH *)base;

  // Turn on the Clock Gating Register
  switch (base) 
  {
    case SSI0 :
    {
      // Configure GPIO Port A to support SSI0/SPI0
      initializePortASpi0();
      SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;
        break;
    }
    default:
        return false;
  }
  
  delay = SYSCTL_RCGCSSI_R;

  // Disable the SSI interface
  myPeriph->SSICR1 &= ~SSI_CR1_SSE;

  // Enable Master Mode
  myPeriph->SSICR1 &= ~SSI_CR1_MS;
  
  // Assume that we hvae a 80MHz clock and want a 4MHz SPI clock
  // FSSIClk = FSysClk / (CPSDVSR * (1 + SCR))
  myPeriph->SSICPSR = SPI_CLK_CPSDVSR;
  myPeriph->SSICR0  &=  ~SSI_CR0_SCR_M; // Set SCR to 0
  
  // Cleare the phse and polarity bits
  myPeriph->SSICR0  &=  ~(SSI_CR0_SPH | SSI_CR0_SPO);
  
  if (phase == 1)
      myPeriph->SSICR0  |= SSI_CR0_SPH;
  
  if (polarity ==1)
      myPeriph->SSICR0  |= SSI_CR0_SPO;

  // Freescale SPI Mode with 8-Bit data
  myPeriph->SSICR0  = ( myPeriph->SSICR0 & ~( SSI_CR0_FRF_M | SSI_CR0_DSS_M ))  | ( SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8); 
  
  //Enable SSI
  myPeriph->SSICR1 |= SSI_CR1_SSE;

  return true;
}

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
  myPort->Direction					  				|= outputMask;
  
  // Enable Pull-Up Resistors
  myPort->PullUpSelect                |= pullUpMask;
  
  // Disable the Alternate Function Select
  myPort->AlternateFunctionSelect     = 0;
  
  // Enable Pins as Digital Pins
  myPort->DigitalEnable               |= digitalEnableMask;
  
  return true;
}

//*****************************************************************************
// Display the LED character
//*****************************************************************************
void displayLEDChar(uint8_t symbol, uint8_t color){
  static uint8_t rowIndex = 7;
  static uint8_t charIndex = 0;
  static uint8_t currPWM = 0;
  // Activate rowIndex 
  GpioPortC->Data = ROW_EN;
  GpioPortB->Data = ~(1 << rowIndex);
  GpioPortC->Data = ENABLES_OFF;
	
  // Disable all Outputs
  GpioPortF->Data |= ~OUTPUT_ENABLE_B;

  // CLEAR LEDs
  GpioPortC->Data = RED_EN;
  GpioPortB->Data = 0xFF;
  GpioPortC->Data = ENABLES_OFF;
  GpioPortC->Data = BLUE_EN;
  GpioPortB->Data = 0xFF;
  GpioPortC->Data = ENABLES_OFF;
  GpioPortC->Data = GREEN_EN;
  GpioPortB->Data = 0xFF;
  GpioPortC->Data = ENABLES_OFF;

  //Turn on LEDs only if pwm allows it
  currPWM++;
  if(currPWM == 100)
  	currPWM = 0;
  if(currPWM < pwm){
	  // Set color LEDs
	  //RED
	  if(color == 0){
	  GpioPortC->Data = RED_EN;
	  GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //YEllOW
	  else if (color == 1){
	    GpioPortC->Data = RED_EN;
	    GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
		GpioPortC->Data = GREEN_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //GREEN
	  else if (color == 2){
		GpioPortC->Data = GREEN_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //BLUE
	  else if (color == 3){
		GpioPortC->Data = BLUE_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //INDIGO
	  else if (color == 4){
		GpioPortC->Data = GREEN_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
		GpioPortC->Data = BLUE_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //PURPLE
	  else if (color == 5){
		GpioPortC->Data = BLUE_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
		GpioPortC->Data = RED_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }
	  //White
	  else if (color == 6){
		GpioPortC->Data = RED_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
		GpioPortC->Data = BLUE_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
		GpioPortC->Data = GREEN_EN;
	  	GpioPortB->Data = ~(ucDispChar[symbol][charIndex]);
	  }	
	  GpioPortC->Data = ENABLES_OFF;
  }
  // Enable All Output
  GpioPortF->Data &= OUTPUT_ENABLE_B;
	
	//Increment the current row index to turn on
	if(nextLEDrow){
	  nextLEDrow = false;
	  if (rowIndex == 0)
	  	rowIndex = 7;
	  else
		rowIndex--;
	  //Increment the LEDchars that turn on
	  if (charIndex == 7)
		charIndex = 0;
	  else
		charIndex++; 
	}
}

//*****************************************************************************
// Debounce the shift registers
//*****************************************************************************
uint16_t debounce(uint32_t base, uint8_t pin, uint16_t shiftReg){
	//A 16 bit int that needs to be in form 
	//1000 0000 0000 0000 to have been debounced	

	//Tells whether the pin is currently high or low
	uint8_t high = 1;
	GPIO_PORT *currPort = (GPIO_PORT *)base;
	//Keep sampling the button every 1 ms until it's debounced 
	//The timer interrupts every 1ms
	if ((currPort->Data & pin) == 0)
		high = 0;
	else
		high = 1;
	shiftReg = (shiftReg << 1) + high;
	return shiftReg;

}

//*****************************************************************************
// Determine whether there is a button press
//*****************************************************************************
bool checkPB(uint16_t shiftReg){
	if (shiftReg == 0x8000){
		return true;
	}
	else 
		return false;
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
// Initialize the ADC on portE
//*****************************************************************************
void  ADCInit(){
  uint32_t delay= 10;
  
   // Enable Port E
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
  //Wait 10ms for enable
  sysTickSleep(delay);
  // Set the direction as an input
  GPIO_PORTE_DIR_R &= ~(PIN_2 | PIN_3);
  
  // Set the alternate function
  GPIO_PORTE_AFSEL_R |= ( PIN_2 | PIN_3 );
  
  // Disable the Digital Enable
  GPIO_PORTE_DEN_R &= ~( PIN_2 | PIN_3);
  
  // Enable Analog 
  GPIO_PORTE_AMSEL_R |= ( PIN_2 | PIN_3);
  
  SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
  //Wait 10ms for enable
  sysTickSleep(delay);
  
  // disable the sample sequencer by writing a 0 to the corresponding ASENn bit in the ADCACTSS register 
  ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;

  // Sequencer 3 is the lowest priority
  ADC0_SSPRI_R = ADC_SSPRI_SS3_4TH | ADC_SSPRI_SS2_3RD | ADC_SSPRI_SS1_2ND | ADC_SSPRI_SS0_1ST;

  ADC0_EMUX_R &= ~ADC_EMUX_EM3_ALWAYS;

  ADC0_SSMUX3_R &=  ~ADC_SSMUX3_MUX0_M;

  ADC0_SSCTL3_R = ADC_SSCTL3_IE0 | ADC_SSCTL3_END0; 

  // Clear Averaging Bits
  ADC0_SAC_R &= ~ADC_SAC_AVG_M  ;
  
  // Average 64 samples
  ADC0_SAC_R |= ADC_SAC_AVG_64X;  
}
//*****************************************************************************
// Get the ADC value of a given ADC Channel
//*****************************************************************************
uint32_t GetADCval(uint32_t Channel)
{
  uint32_t result;

  ADC0_SSMUX3_R = Channel;      // Set the channel
  ADC0_ACTSS_R  |= ADC_ACTSS_ASEN3; // Enable SS3
  ADC0_PSSI_R = ADC_PSSI_SS3;     // Initiate SS3

  while(0 == (ADC0_RIS_R & ADC_RIS_INR3)); // Wait for END of conversion
  result = ADC0_SSFIFO3_R & 0x0FFF;     // Read the 12-bit ADC result
  ADC0_ISC_R = ADC_ISC_IN3;         // Acknowledge completion

  return result;
}
int 
main(void)
{
  //Local Variables
  bool displayMode = true;//True if display mode, false if in data entry mode
	//Push buttons
	bool upPB = false;
	bool rightPB = false;
	bool downPB = false;
	bool leftPB = false;
	bool modePB = false; 
	//Push buttons shift registers
	uint16_t shiftRegSW2 = 0xFFFF;
	uint16_t shiftRegSW3 = 0xFFFF;
	uint16_t shiftRegSW4 = 0xFFFF;
	uint16_t shiftRegSW5 = 0xFFFF;
	uint16_t shiftRegSW6 = 0xFFFF;
	//Initial display message
  uint8_t displayArray [SAVED_MSG_LEN_MAX] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  uint8_t inputArray [17] =
		{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  //Mark the indexcies of the array
	uint8_t endIndex = 19;
	uint8_t currIndex = 0;
	uint8_t inputIndex = 0;
	uint8_t tempIndex = 0;
	//Flag for overflwoing max char array length
	bool overflow = false;
	//Init color
  uint8_t color;
  //ADCval1
  uint32_t ADCval1 = 0;
  uint32_t ADCval2 = 0;
  //Current string
  char myString[21];
  char myChar;
  int8_t stringIndex=-1;

  // Initialize the PLLs so the the main CPU frequency is 80MHz
  PLL_Init();
  
  //Configure Port C
  initPortC();
  
  //Configure Port B  DATA_7 through DATA_0 as outputs
  gpioPortInit(PORTB, 0xFF, 0x00, 0xFF, 0x00);
  //Configure Port F /OE as output and SW5 as input
  gpioPortInit(PORTF, 0x12, 0x02, 0x10, 0x12);
  //Configure Port A SW2 and SW3 as inputs
  gpioPortInit(PORTA, 0xC0, 0xC0, 0x00, 0xC0);
  //Configure Port D SW4 and SW5 as inputs
  gpioPortInit(PORTD, 0x0C, 0x0C, 0x00, 0x0C);
  
  // Initialize SPI0 interface
  initializeSPI(SSI0, PHASE, POLARITY);
  
  //Configure Timer0 1mS ticks
  TIMER0Config(80000);
  //Configure watchdog with 1s reset
  WatchdogTIMERConfig();
  //Configure the SYSTICK timer 12.5uS ticks
  SYSTICKConfig(1000, true);
  //Initialize UART0 and wait a bit
  uartInitPolling(UART0);
  sysTickSleep(1);
  //Init ADC
  ADCInit();
  //Get initial ADC values
  pwm = GetADCval(POT_RIGHT) / 40;
  ADCval2 = GetADCval(POT_LEFT) / 575;
  
  
  // Print out the current string
  uartTxPollString(UART0,"\n\r\n\rCurrent Greeint Message: ");
  uartTxPollString(UART0,"hello");
  uartTxPollString(UART0,"\n\r");

  while(1)
  {
	if(checkADC){
	 pwm = GetADCval(POT_RIGHT) / 40;
	 ADCval2 = GetADCval(POT_LEFT) / 600;
	 checkADC = false;
	}

    //On systick interrupt display the current character and poll the buttons
    if (refreshLED){
		if (displayMode){
			displayLEDChar(displayArray[currIndex], ADCval2);
		}
		else{
			displayLEDChar(inputArray[inputIndex], ADCval2);
		}
		refreshLED = false;
	}
	if (buttonPoll){
		//Check if any buttons have been pushed
		//If so debounce them 
		buttonPoll = false;
		//SW2
		shiftRegSW2 = debounce(PORTA, SW2, shiftRegSW2);
		upPB = checkPB(shiftRegSW2);
		//SW3	
		shiftRegSW3 = debounce(PORTA, SW3, shiftRegSW3);
		rightPB = checkPB(shiftRegSW3);			
		//SW4
		shiftRegSW4 = debounce(PORTD, SW4, shiftRegSW4);
		downPB = checkPB(shiftRegSW4);
		//SW5
		shiftRegSW5 = debounce(PORTD, SW5, shiftRegSW5);	
		leftPB = checkPB(shiftRegSW5);
		//SW6
		shiftRegSW6 = debounce(PORTF, SW6, shiftRegSW6);
		modePB = checkPB(shiftRegSW6);

	}//End polling
		
		//Display Mode
		if (displayMode){
		  //Check if right button is pressed
				if(rightPB){
					//Clear the button press
					rightPB = false;
					//Display the next character in order in green
					color = GREEN_EN;
					if(currIndex == endIndex)
						currIndex = 0;
					else
						currIndex++;
				}
				//Check if left button is pressed
				else if(leftPB){
					//Clear the button press
					leftPB = false;
					//Display the next character in reverse order in red
					color = RED_EN;
					if(currIndex == 0)
						currIndex = endIndex;
					else
						currIndex--;
				}
				//Check if mode button is pushed
				if(modePB){
					//Clear the button press
					modePB = false;
					//change mode and reset parameters
					displayMode = false;
					inputIndex = 0;
					currIndex = 0;
					endIndex = 0;
					overflow = false;
					tempIndex = 0;
					displayArray[0] = 0;
				}
		}
		
		//Input Mode
		else{
			//Check if right button is pressed
				if(rightPB || leftPB){
					//Clear the button press
					rightPB = false;
					leftPB = false;
					//Save the current input character in the display array if
					//there is enough space
					if (tempIndex >= SAVED_MSG_LEN_MAX){
						tempIndex= 0;
						overflow = true;
					}	
					displayArray[tempIndex] = inputArray[inputIndex];
					tempIndex++;					
					
				}
				//Check if up button is pressed
				else if(upPB){
					//Clear the button press
					upPB = false;
					//Display the next character in order in input array
					if(inputIndex == 15)
						inputIndex = 0;
					else
						inputIndex++;
				}
				//Check if down button is pressed
				else if(downPB){
					//Clear the button press
					downPB = false;
					//Display the next character in reverse order input array
					if(inputIndex == 0)
						inputIndex = 15;
					else
						inputIndex--;
				}
				if(modePB){
					//Clear the button press
					modePB = false;
					//change mode to display and reset parameters
					displayMode = true;
					color = GREEN_EN;
					if(overflow){
						endIndex = SAVED_MSG_LEN_MAX - 1;
					}
					else if (tempIndex != 0)
						endIndex = tempIndex - 1;
				}
		}
  }
}

