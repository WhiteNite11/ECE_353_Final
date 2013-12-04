#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "lm4f120h5qr.h"


extern  volatile bool refreshLED; 
extern volatile bool checkADC;
extern volatile uint32_t brightness;

void TIMER0IntHandler(void){
}

 /****************************************************************************
 * The SysTick Handler
 ****************************************************************************/
void SYSTICKIntHandler(void)
{
  //Variable for 300 ms counter for ADC reads
  static uint16_t ADCtimer = 0;
  //Variable for refreshing LEDs
  static uint32_t refresh = 0;
  ADCtimer++;
  if (ADCtimer == 300){
	checkADC = true;
	ADCtimer = 0;
  }
  
  // Clear the SysTick Interrupt
  NVIC_ST_CURRENT_R = 0;
  //Set boolean trigger
  refresh++;
  if (refresh == brightness){
    refreshLED = true;
	refresh = 0;
  } 
  
}

 /****************************************************************************
 * The SysTick Configuration Routine
 ****************************************************************************/
void SYSTICKConfig(uint32_t loadVal, bool enableInterrupts)
{
  NVIC_ST_CTRL_R     = 0;            // disable SysTick timer
  NVIC_ST_RELOAD_R   = loadVal;      // Set max count to the reload reg
  NVIC_ST_CURRENT_R  = 0;            // clear the current count
  NVIC_ST_CTRL_R     = (NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_CLK_SRC);    // enable SysTick with Core Clock
  // Enable Interrupts
  if (enableInterrupts)
  {
  	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN;
  }
}
/****************************************************************************
 * The SysTick sleep routine
 ****************************************************************************/
void sysTickSleep(uint32_t milliSec)
{
  uint32_t delay= 0;
  while (delay != milliSec){
   	if (refreshLED == true){
	 	delay++;
		refreshLED = false;
	}
  }
}

