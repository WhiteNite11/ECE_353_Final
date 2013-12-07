#ifndef __BOARD_CONFIG__
#define __BOARD_CONFIG__

#include <stdint.h>
#include <stdio.h>
#include "lm4f120h5qr.h"
#include "inc/hw_types.h"
#include "gpio.h"
#include "UART.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
 
#define NULL                  0

#define PIN_0     (1 << 0)
#define PIN_1     (1 << 1)
#define PIN_2     (1 << 2)
#define PIN_3     (1 << 3)
#define PIN_4     (1 << 4)
#define PIN_5     (1 << 5)
#define PIN_6     (1 << 6)
#define PIN_7     (1 << 7)

#define PA6_SW2       PIN_6
#define PA7_SW3       PIN_7
#define PD2_SW4       PIN_2
#define PD3_SW5       PIN_3
#define PF1_SW6       PIN_1


#define PA0_U0_RX  PIN_0
#define PA1_U0_TX  PIN_1

#define PA2_SPI0_CLK   PIN_2
#define PA3_SPI0_CS    PIN_3
#define PA4_SPI0_MISO  PIN_4
#define PA5_SPI0_MOSI  PIN_5

#define PB0_DATA0     PIN_0
#define PB1_DATA1     PIN_1
#define PB2_DATA2     PIN_2
#define PB3_DATA3     PIN_3
#define PB4_DATA4     PIN_4
#define PB5_DATA5     PIN_5
#define PB6_DATA6     PIN_6
#define PB7_DATA7     PIN_7

#define PC4_RED_EN    PIN_4
#define PC5_GRN_EN    PIN_5
#define PC6_BLU_EN    PIN_6
#define PC7_COL_EN    PIN_7
#define PF4_OUT_EN_B  PIN_4

#define PE3_ADC_0     PIN_3
#define PE2_ADC_1     PIN_2

#define PC0_TCK       PIN_0
#define PC1_TMS       PIN_1
#define PC2_TDI       PIN_2
#define PC3_TDO       PIN_3

#define DISABLED  0
#define NONE      0
#define PORT_CONTROL_DEFAULT  

/******************************************************************************
 * Initialization Structures
 *****************************************************************************/
 
/******************************************************************************
 * PORT A
 *****************************************************************************/
 GPIO_CONFIG portA_config = {
  (
    PA0_U0_RX  | 
    PA1_U0_TX  |
    PA6_SW2    |
    PA7_SW3
   
  ),                                // DigitalEnable
  (
    PA6_SW2    |
    PA7_SW3
   ),                               // Input
  NONE,                             // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  (
    PA6_SW2    |
    PA7_SW3
   ),                               // PullUp
  DISABLED,                         // AnalogEnable
  (
    PA0_U0_RX | 
    PA1_U0_TX 
  ),                                // AlternateFunctionEnable
  (
    GPIO_PCTL_PA0_U0RX    |
    GPIO_PCTL_PA1_U0TX    
  )                                 // PortControl
};

 /******************************************************************************
 * PORT B
 *****************************************************************************/
 GPIO_CONFIG portB_config = {
  (
   PB0_DATA0 |
   PB1_DATA1 |
   PB2_DATA2 |
   PB3_DATA3 |
   PB4_DATA4 |
   PB5_DATA5 |
   PB6_DATA6 |
   PB7_DATA7
  ),                                // DigitalEnable
  NONE,                             // Input
  (
   PB0_DATA0 |
   PB1_DATA1 |
   PB2_DATA2 |
   PB3_DATA3 |
   PB4_DATA4 |
   PB5_DATA5 |
   PB6_DATA6 |
   PB7_DATA7
  ),                                // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  DISABLED,                         // PullUp
  DISABLED,                         // AnalogEnable
  DISABLED,                         // AlternateFunctionEnable
  PORT_CONTROL_DEFAULT              // PortControl
};




 /******************************************************************************
 * PORT C
 *****************************************************************************/
 GPIO_CONFIG portC_config = {
  (
    PC4_RED_EN    |
    PC5_GRN_EN    |
    PC6_BLU_EN    |
    PC7_COL_EN    |   
    PC0_TCK       |
    PC1_TMS       |
    PC2_TDI       |
    PC3_TDO
  ),                                // DigitalEnable
  NONE,                             // Input
  (
    PC4_RED_EN |
    PC5_GRN_EN |
    PC6_BLU_EN |
    PC7_COL_EN 
  ),                                // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  ( 
    PC0_TCK       |
    PC1_TMS       |
    PC2_TDI       |
    PC3_TDO
  ),                                // PullUp
  DISABLED,                         // AnalogEnable
  ( 
    PC0_TCK       |
    PC1_TMS       |
    PC2_TDI       |
    PC3_TDO
  ),                                // AlternateFunctionEnable
  (
    GPIO_PCTL_PC0_TCK   |
    GPIO_PCTL_PC1_TMS   |
    GPIO_PCTL_PC2_TDI   |
    GPIO_PCTL_PC3_TDO
    
  )                                 // PortControl
};

 /******************************************************************************
 * PORT D
 *****************************************************************************/
 GPIO_CONFIG portD_config = {
  (
     PD2_SW4 |
     PD3_SW5
  ),                                // DigitalEnable
  (
     PD2_SW4 |
     PD3_SW5
  ),                                // Input
  NONE,                             // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  (
     PD2_SW4 |
     PD3_SW5
  ),                                // PullUp
  DISABLED,                         // AnalogEnable
  DISABLED,                         // AlternateFunctionEnable
  PORT_CONTROL_DEFAULT              // PortControl
};

 /******************************************************************************
 * PORT E
 *****************************************************************************/
 GPIO_CONFIG portE_config = {
  NONE,                             // DigitalEnable
  NONE,                             // Input
  NONE,                             // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  DISABLED,                         // PullUp
  DISABLED,                         // AnalogEnable
  DISABLED,                         // AlternateFunctionEnable
  PORT_CONTROL_DEFAULT              // PortControl
};

/******************************************************************************
 * PORT F
 *****************************************************************************/
GPIO_CONFIG portF_config = {
  (
    PF1_SW6 |
    PF4_OUT_EN_B
  ),                                // DigitalEnable
  (
    PF1_SW6
  ),                                // Input
  (
    PF4_OUT_EN_B
  ),                                // Output
  DISABLED,                         // InterruptEnable
  DISABLED,                         // InterruptLevel
  DISABLED,                         // InterruptLevelActiveHigh
  DISABLED,                         // InterruptEdge
  DISABLED,                         // InterruptEdgeRising
  DISABLED,                         // InterruptEdgeBoth
  DISABLED,                         // PullDown
  (
    PF1_SW6
  ),                                // PullUp
  DISABLED,                         // AnalogEnable
  DISABLED,                         // AlternateFunctionEnable
  PORT_CONTROL_DEFAULT              // PortControl
};


/******************************************************************************
 * Initialization Structure for UART0 Comms
 *****************************************************************************/ 
UART_CONFIG UART0_config = {
  ENABLED,    				// UART
  ENABLED,                  // UARTRx
  ENABLED,    				// UARTTx
  _8BITS,                   // WordLen
  115200,                     // BaudRate
  ENABLED,                  // FIFO
  DISABLED,                 // _2Stop
  DISABLED,                 // Parity
  DISABLED,                 // EvenParity
  DISABLED,                 // HighSpeed
  RX_ONE_EIGHTH,            // RxIntFIFO
  TX_ONE_EIGHTH,            // TxIntFIFO
  ENABLED,                  // RxIntMask
  ENABLED,                  // TxIntMask
  DISABLED,                 // RxTimeOutIntMask
  DISABLED,                 // OvrrIntMask
  DISABLED,                 // ParityErrIntMask
  DISABLED                  // FramingErrIntMask  
};




#endif
