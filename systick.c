#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "lm4f120h5qr.h"


extern  volatile bool refreshLED; 
extern volatile bool checkADC;
extern volatile bool nextLEDrow;
extern volatile bool buttonPoll;

void TIMER0IntHandler(void){
  //Variable for 100 ms counter for ADC reads
  static uint16_t ADCtimer = 0;
  static uint8_t buttonTime = 0;
  ADCtimer++;
  buttonTime++;
  if (ADCtimer == 100){
	checkADC = true;
	ADCtimer = 0;
  }
  //Poll button every 1ms
  if (buttonTime == 1){
	buttonPoll = true;
	buttonTime = 0;
  }

   //Clear the interrupt
   TIMER0_ICR_R	|= TIMER_ICR_TATOCINT;
   //Pet the watchdog
   WATCHDOG0_LOAD_R   = 80000000;
}
 /****************************************************************************
 * The Timer0 Configuration Routine
 ****************************************************************************/
void TIMER0Config(uint32_t loadVal)
{
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0;
  TIMER0_CTL_R    &= ~(TIMER_CTL_TAEN);       // disable TIMER0
  TIMER0_CFG_R	  = TIMER_CFG_32_BIT_TIMER;  // Set Timer0 to 32 bit
  TIMER0_TAMR_R   = TIMER_TAMR_TAMR_PERIOD;  // Periodic timer 
  TIMER0_TAILR_R  = loadVal;	//Put load value into timer
  TIMER0_ICR_R    = TIMER_ICR_TATOCINT;
  TIMER0_IMR_R    |= TIMER_IMR_TATOIM;	 // Enable Interrupts
  NVIC_PRI4_R     = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; //Priority 2
  NVIC_EN0_R      |= NVIC_EN0_INT19;	//Enable interrupt 19 in NVIC
  TIMER0_CTL_R    |= TIMER_CTL_TAEN;    // enable TimerA
  
}

/****************************************************************************
 * The wATCHDOGTimer Configuration Routine
 ****************************************************************************/
void WatchdogTIMERConfig()
{
 SYSCTL_RCGCWD_R   |= SYSCTL_RCGCWD_R0;
 //Set the load value
 WATCHDOG0_LOAD_R   = 80000000;
 //Enable board reset on failure
 WATCHDOG0_CTL_R   |= WDT_CTL_RESEN;
 //Enable interrupts, and the timer
 WATCHDOG0_CTL_R   |= WDT_CTL_INTEN;
}
 /****************************************************************************
 * The SysTick Handler
 ****************************************************************************/
void SYSTICKIntHandler(void)
{ 
  //Variable for refreshing LEDs
  static uint32_t nextLED = 0;
  
  //Set boolean trigger
  nextLED++;
  if (nextLED == 100){
    nextLEDrow = true;
	nextLED = 0;
  }
  refreshLED = true; 
  // Clear the SysTick Interrupt
  NVIC_ST_CURRENT_R = 0;
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
  uint32_t delay = 0;
  uint32_t ticks = milliSec/.0125;
  while (delay != ticks){
   	if (refreshLED == true){
	 	delay++;
		refreshLED = false;
	}
  }
}

