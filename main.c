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
#include "led_chars.h"
#include "gpio.h"
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
  static uint8_t rowIndex = 0;
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
	  if (rowIndex == 7)
	  	rowIndex = 0;
	  else
		rowIndex++;
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
  uint8_t displayArray [SAVED_MSG_LEN_MAX] = {12, 13, 10, 11, 10, 3, 5, 3};
  uint8_t inputArray [17] =
		{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  //Mark the indexcies of the array
	uint8_t endIndex = 7;
	uint8_t currIndex = 0;
	uint8_t inputIndex = 0;
	uint8_t tempIndex = 0;
	//Flag for overflwoing max char array length
	bool overflow = false;
	//Init color
  uint8_t color;

  uint32_t ADCval1 = 0;
  uint32_t ADCval2 = 0;


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
  
  //Configure Timer0 1mS ticks
  TIMER0Config(80000);
  
  //Configure the SYSTICK timer 12.5uS ticks
  SYSTICKConfig(1000, true);
  
  //Init ADC
  ADCInit();
  //Get initial ADC values
  pwm = GetADCval(POT_RIGHT) / 40;
  ADCval2 = GetADCval(POT_LEFT) / 575;
  
  while(1)
  {
	if(checkADC){
	 pwm = GetADCval(POT_RIGHT) / 40;
	 ADCval2 = GetADCval(POT_LEFT) / 600;
//	 if(ADCval1 == 0)
//	 	brightness = 1;
//	 else if(ADCval1 == 1)
//		brightness = 5;
//	 else if(ADCval1 == 2)
//		brightness = 10;
//	 else if(ADCval1 >= 3)
//	 	brightness = 15;

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

