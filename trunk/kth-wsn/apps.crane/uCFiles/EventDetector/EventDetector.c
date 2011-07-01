/* This file has been prepared for Doxygen automatic documentation generation. */
/*! \file *********************************************************************
 *
 * \brief  Event Detector Project. This application program would read various kind of system's output. It can be 
 * interfaced with at maximum 5 quadrature encoders, 5 analog signals from CT process sensors. It would also estimate
 * the full state of the system using available outputs. We would use first order low pass filters to estimate the speed
 * variables from output position states. In case of high noise we would use EKF filtering to estimate the speed variables.
 * The quadrature decoding would be done using dedicaded external HCTL2032 IC and also using built-in quadrature decoder of
 * ATxmega128A1. The program would sample the process @ 1 kHz and would compute the full state and then evaluate the event-generation
 * rule and then would generate the event/interrupt to trigger the communication of state vector to wireless mote on SPI channel and mote
 * would transmit it on the network. 
 * This spplicstion consists of three main modules. Module # 1: Sensor Interface Module, Module # 2: State Estimation Module, 
 * Module # 3: Event Generation Module
 * We have used one timer to read sensors periodically @ 1kHz and then we would estimate speeds (full state vector) and test the event-generation
 * rule. All three operations would be done inside timer overflow ISR. In case of event we would trigger the SPI communication from inside the timer
 * overflow interrupt. When the mote would receive the SPI data, it would immediately transmit it to the network! 
 *
 * \par Application note:
 *      See NetCon Wiki Page
 *      HCTL2032 Datasheet
 *      ATxmega128A1 Datasheet        
 *
 * \par Documentation
 *
 * \author
 *      Faisal Altaf: http://www.ee.kth.se
 *      Reglerteknik
 *      Support email: faltaf@kth.se
 *
 * $Revision: 0$
 * $Date: 2010-12-7 13:36 +0100 (on, 7 December 2010) $
 *
 * Copyright (c) 2010, Reglerteknik, KTH-Royal Institute of Technology. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of KTH may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 
 *****************************************************************************/


/* we are using the one provided by codevisionavr instead of that provided by atmel. 
copied from \inc directory to project directory but modified it to include F_CPU macro
which we copied from atmel's avr_compiler.h*/
#include "avr_compiler.h"


/* either copy the hctl_driver.lib, qdec_driver.lib and qdec_signal_generator.lib files from the  main project directory and put in .\lib directory
inside cavr2 directory or add the alternate paths to additional directories in the Project|Configure|C Compiler|Paths|Library paths menu*. The header files
hctl_driver.h, qdec_driver.h and qdec_signal_generator.h are present in main project directory.*/

#include "hctl2032_driver.h"
#include "qdec_driver.h"
#include "qdec_signal_generator.h"
#include "delay.h"
#include "math.h"

/*#define _ATXMEGA_DEVICE_*/
/* use the ATxmega128A1 USARTF0 for getchar and putchar functions. The following is required because by default putchar send data on USARTC0 */
#define _ATXMEGA_USART_ USARTF0
#include "stdio.h"


/*! \brief  If GENERATE_TEST_SIGNAL is defined the system generates a test signal
 *          with frequency equal to freq (RPM = freq*60). if external quadrature source is not available then generate 
 quadrature signal internally by MCU. Dont use double slash to comment GENERATE_TEST_SIGNAL instead use "slash and star".*/
/*#define GENERATE_TEST_SIGNAL*/

 /* To select between 2 MHz and 32 MHz clock source*/
#define SYSTEM_CLOCK_32MHZ
/*To select between 19200 and 115200bps baud rate*/
#define SYSTEM_CLOCK_32MHZ_115200

/* To select between Periodic TTC and ETC*/
#define CONTROLLER_ETC

/*! \brief Output frequency when system generates a signal*/
uint8_t freq = 2;

/*! \brief Direction of the output signal*/
bool    dir  = 0; // 0 = CW or count-up (Count-up would start from BOT (0x0000) value)=, 1 = CCW or count down (Count down would start from TOP value (4*lineCount - 1))

//unsigned char Counter_OverFlows=0;
//unsigned char LED = 0xFF;
//bit LED_Flag=1;

/*! \brief Defines the clock division for the timer used.
 *         if changed both defines NEED to be changed correspondingly.
 */
#define CLOCK_DIV_bm  TC_CLKSEL_DIV64_gc
#define CLOCK_DIV     64
#define PI 3.1416F

/*! \brief Number of lines in the quadrature encoder. */
uint8_t lineCount = 30;

/*! \brief Global frequency variable. */
uint16_t captureFreq  = 0;
uint16_t calcFreq     = 0;
uint16_t calcRPM      = 0;
/*! \brief Declare your global variables here */
unsigned int NumOfRev = 0;
unsigned char Counter_OverFlows=0;
unsigned char LED = 0xFF;
bit LED_Flag=1;
unsigned char CountMode = 1;  // Count mode for HCTL2032 Quad Decoder
unsigned char counter_state1 = 0x00; // Counter state i.e. is it normal, overflow or underflow
unsigned char counter_state2 = 0x00;

//unsigned long int Pos_Count_HCTL_1 = 0x00000000;
//unsigned long int Pos_Count_HCTL_2 = 0x00000000;
//unsigned long int Pos_Count_HCTL_3 = 0x00000000;
//unsigned long int Pos_Count_HCTL_4 = 0x00000000;

signed long int Pos_Count_HCTL_1 = 0x00000000;
signed long int Pos_Count_HCTL_2 = 0x00000000;
signed long int Pos_Count_HCTL_3 = 0x00000000;
signed long int Pos_Count_HCTL_4 = 0x00000000;

unsigned long int Temp_Pos_Count_HCTL_1 = 0x00000000;
unsigned long int Temp_Pos_Count_HCTL_2 = 0x00000000;
unsigned long int Temp_Pos_Count_HCTL_3 = 0x00000000;
unsigned long int Temp_Pos_Count_HCTL_4 = 0x00000000;

/***** Estimation Variables and Constants *****/
float h = 0.01; // 10 ms sampling Time
//const float h = 0.0250; // Uncomment for periodic Controller
const float Td = 0.03; // Derivative Prediction Horizon
const float Ti = 0.1; // Integration Time
const float Tt1 = 0.05; // Tracking Time for Trolley
const float Tt2 = 0.05; // Tracking Time for Arm
const char N = 10; // Low Pass PF Parameter for Derivative Action
const char N1 = 5;//5; // Low Pass PF Parameter for Derivative Action for Payload
//const float PI = 3.1416;
float ad = 0;
float bd = 0;
float ad1 = 0;
float bd1 = 0;
float bi1 = 0;
float bi2 = 0;
float a01 = 0;
float a02 = 0;

/***** System Parameters *****/
const float r_x = 0.0379127; // Trolley Pully Radius
const unsigned int PPR1 = 4096; // Encoder Pulses Per Revolution for Encoder 1 (Payload X Angle)
const unsigned int PPR2 = 4096; // Encoder Pulses Per Revolution for Encoder 2 (Payload Y Angle)
const unsigned int PPR3 = 4096; // Encoder Pulses Per Revolution for Encoder 3 (Trolley Position)
const unsigned int PPR4 = 4000; // Encoder Pulses Per Revolution for Encoder 4 (Arm Position)
const unsigned int PPR5 = 4096; // Encoder Pulses Per Revolution for Encoder 5 (Lift Line Length)

/****** Controller Parameters/Gains ******/
//const float K11 = -5.7015; //Trolley Position Gain
//const float K12 = 1.0;//4.3408;//0.3;//4.3408;//0.3;//0.2;//1.0;//2.0;//0.2;//4.3408; // Trollery Speed Gain ,,, it is too high and creating some problems especially when speed estimate is crude. May work well when using EKF but not with crude estimate!!
//const float K13 = -3.0;//-4.2;//-2.9673;//-3.5055;//-2.9673;//-5.0;//-3.5055;// // X-Angle Position Gain 
//const float K14 = -0.1498; // X-Angle Speed Gain
//const float K15 = 2.5317;//2.4317;//2.4317;//2.8317;//1.1;//2.5317; // Integral State (x_{a1}) Gain  , For real crane, 1.1
//const float K21 = -2.7525; // Arm Position Gain
//const float K22 = 0.1138;//1.7064;//0.1138; // Arm Speed Gain
//const float K23 = 2.8;//3.0;//3.8;//4.2;//2.5477;//3.5477; // Y-Angle Position Gain
//const float K24 = 0.2278;//0.1716; // Y-Angle Speed Gain
//const float K25 = 1.1358;//0.8;//2.0;//1.1358; //1.75;//1.0;//1.75;//1.4358;//2.0;//1.4358;//1.7358;//1.4358;//1.1358; // Integral State (x_{a2}) Gain

// To Be Used with Euler based estimates
const float K11 = -5.7015; //Trolley Position Gain
const float K12 = 1.0;//4.3408;//0.3;//4.3408;//0.3;//0.2;//1.0;//2.0;//0.2;//4.3408; // Trollery Speed Gain ,,, it is too high and creating some problems especially when speed estimate is crude. May work well when using EKF but not with crude estimate!!
const float K13 = -3.5055;//-2.9673;//-5.0;//-3.5055;// // X-Angle Position Gain 
const float K14 = -0.1498; // X-Angle Speed Gain
const float K15 = 2.5317;//2.4317;//2.4317;//2.8317;//1.1;//2.5317; // Integral State (x_{a1}) Gain  , For real crane, 1.1
const float K21 = -2.7525; // Arm Position Gain
const float K22 = 0.1138;//1.7064;//0.1138; // Arm Speed Gain
const float K23 = 3.5477; // Y-Angle Position Gain
const float K24 = 0.1716; // Y-Angle Speed Gain
const float K25 = 1.4358;//1.1358;//1.75;//1.0;//1.75;//1.4358;//2.0;//1.4358;//1.7358;//1.4358;//1.1358; // Integral State (x_{a2}) Gain

/***** System States *****/
float x_1 = 0.0;
float x_2 = 0.0;
float x_3 = 0.0;
float x_4 = 0.0;
float x_a1 = 0.0;
float x_5 = 0.0;
float x_6 = 0.0;
float x_7 = 0.0;
float x_8 = 0.0;
float x_a2 = 0.0;

float x_1_old = 0.0; // x_1 at last time step
float x_2_old = 0.0; // x_2 at last time step
float x_3_old = 0.0; // x_3 at last time step
float x_4_old = 0.0; // x_4 at last time step
float x_5_old = 0.0; // x_5 at last time step
float x_6_old = 0.0; // x_6 at last time step
float x_7_old = 0.0; // x_7 at last time step
float x_8_old = 0.0; // x_8 at last time step
float x_a1_old = 0.0; // x_a1 at last time step
float x_a2_old = 0.0; // x_a2 at last time step
float Phi = 0.0; // \Phi(k-1)=\frac{1}{1+0.0138x_1(k-1)^2}



/*** System States at Last Trigger ***/

float x_1_tk = 0.0;
float x_2_tk = 0.0;
float x_3_tk = 0.0;
float x_4_tk = 0.0;
float x_a1_tk = 0.0;
float x_5_tk = 0.0;
float x_6_tk = 0.0;
float x_7_tk = 0.0;
float x_8_tk = 0.0;
float x_a2_tk = 0.0;

/********** Event Generation Rule (EGR) Variables **********/
/** Measurement Error Norm**/
float norm_e = 0.0;
/** Shifted State **/
float norm_epsilon = 0.0;

/** EGR Parameters **/
// For Model Based Estimates
const float sigma = 0.007;//0.007;//0.004;//0.0004;//0.0000001;//0.00005;//0.0008;//0.035;//0.075;//0.035;//0.0175;//0.075;//0.0080;//0.075;
const float delta = 0.005;//0.005;//0.0001;//0.0;//0.005;//0.0001;//0.001;//0.01;//0.0005;//0.005;//0.001;//0.005; //0.01;
const float taumina = 0.04;

//// For Euler Based Estimates
//const float sigma = 0.075;//0.007;//0.004;//0.0004;//0.0000001;//0.00005;//0.0008;//0.035;//0.075;//0.035;//0.0175;//0.075;//0.0080;//0.075;
//const float delta = 0.005;//0.0001;//0.001;//0.01;//0.0005;//0.005;//0.001;//0.005; //0.01;
//const float taumina = 0.04;

//Weighting Matrix
const float w_1 = 1; // Weight for x_1
const float w_2 = 1;  // Weight for x_2
const float w_3 = 0.0001;//0.01;//0.1;//0.001;//0.0001;//0.00001;//0.0001;//0.005;//0.5;//1;  // Weight for x_3
const float w_4 = 0.0001;//0.01;//0.1;//0.001;//0.0001;//0.00001;//0.0001;//0.005;//1;  // Weight for x_4
const float w_5 = 1;       // Weight for x_a1
const float w_6 = 1;      // Weight for x_5
const float w_7 = 1;     // Weight for x_6
const float w_8 = 0.00002;//0.01;//0.1;//0.001;//0.00002;//0.000001;//0.00002;//0.0002;//0.02;//0.2;//1;  // Weight for x_7
const float w_9 = 0.00002;//0.01;//0.1;//0.001;//0.00002;//0.000001;//0.00002;//0.0002;//0.5;//0.002;//0.02;//0.2;//1;  // Weight for x_8
const float w_10 = 1;      // Weight for x_a2

/**** Reference Signals ****/
float r_1 = 0.0;//0.3;
float r_2 = 0.0;//PI;
/*** Homing after encoder reset ***/
float home_x = 0.25; // bring the trolley to this position initially
float home_theta = PI*0.5; // bring the Arm to this position initially

/*** Control Signals ***/
// Saturated Control Signals
float u_1 = 0.0;
float u_2 = 0.0;
float u_1_old = 0.0; // Control Signal for trolley at previous step
float u_2_old = 0.0; // Control Signal for trolley at previous step


// Computed Control Signals
float v_1 = 0.0;
float v_2 = 0.0;

// Start Flag
bit start_flag = 0;

//Switch States
bit SW0_pressed = 1;
bit SW1_pressed = 1;
bit SW2_pressed = 1;
bit SW3_pressed = 1;
bit SW4_pressed = 1;

// Decision Variable to Select Between Periodic and and ETC 
bit Periodic_ON = 0; // Select ETC initially

// Number of Times ERG is violated
unsigned long iterations = 0;
unsigned long iterations_old = 0;
bit first_transmission = 1;

// Parameter For Reference Signal Generation
unsigned long count_ref = 0;
bit flag_ref = 0; // signal to integrator that new reference command is available
  

// System Clocks initialization
void system_clocks_init(void)
{
unsigned char n,s;

// Optimize for speed
#pragma optsize- 
// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
#asm("cli")

// Internal 2 MHz RC oscillator initialization
// Enable the internal 2 MHz RC oscillator
OSC.CTRL|=OSC_RC2MEN_bm;

// System Clock prescaler A division factor: 1
// System Clock prescalers B & C division factors: B:1, C:1
// ClkPer4: 32000,000 kHz
// ClkPer2: 32000,000 kHz
// ClkPer:  32000,000 kHz
// ClkCPU:  32000,000 kHz
n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
    CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
CCP=CCP_IOREG_gc;
CLK.PSCTRL=n;

// Disable the autocalibration of the internal 2 MHz RC oscillator
DFLLRC2M.CTRL&= ~DFLL_ENABLE_bm;

// Wait for the internal 2 MHz RC oscillator to stabilize
while ((OSC.STATUS & OSC_RC2MRDY_bm)==0);

// Select the system clock source: 2 MHz Internal RC Osc.
n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC2M_gc;
CCP=CCP_IOREG_gc;
CLK.CTRL=n;


// Disable the unused oscillators: 32 MHz, 32 kHz, external clock/crystal oscillator, PLL
OSC.CTRL&= ~(OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);

// Peripheral Clock output: Disabled
PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;

// Restore interrupts enabled/disabled state
SREG=s;
// Restore optimization for size if needed
#pragma optsize_default
}

// System Clocks initialization
void system_clocks_init_32mhz(void)
{
unsigned char n,s;

// Optimize for speed
#pragma optsize- 
// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
#asm("cli")

// Internal 32 kHz RC oscillator initialization
// Enable the internal 32 kHz RC oscillator
OSC.CTRL|=OSC_RC32KEN_bm;
// Wait for the internal 32 kHz RC oscillator to stabilize
while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);

// Internal 32 MHz RC oscillator initialization
// Enable the internal 32 MHz RC oscillator
OSC.CTRL|=OSC_RC32MEN_bm;

// System Clock prescaler A division factor: 1
// System Clock prescalers B & C division factors: B:1, C:1
// ClkPer4: 32000.000 kHz
// ClkPer2: 32000.000 kHz
// ClkPer:  32000.000 kHz
// ClkCPU:  32000.000 kHz
n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
CCP=CCP_IOREG_gc;
CLK.PSCTRL=n;

// Internal 32 MHz RC osc. calibration reference clock source: 32.768 kHz Internal Osc.
OSC.DFLLCTRL&= ~(OSC_RC32MCREF_bm | OSC_RC2MCREF_bm);
// Enable the autocalibration of the internal 32 MHz RC oscillator
DFLLRC32M.CTRL|=DFLL_ENABLE_bm;

// Wait for the internal 32 MHz RC oscillator to stabilize
while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);

// Select the system clock source: 32 MHz Internal RC Osc.
n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
CCP=CCP_IOREG_gc;
CLK.CTRL=n;

// Disable the unused oscillators: 2 MHz, external clock/crystal oscillator, PLL
OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);

// Peripheral Clock output: Disabled
PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;

// Restore interrupts enabled/disabled state
SREG=s;
// Restore optimization for size if needed
#pragma optsize_default
}

// Disable a Timer/Counter type 0
void tc0_disable(TC0_t *ptc)
{
// Timer/Counter off
ptc->CTRLA=(ptc->CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_OFF_gc;
// Issue a reset command
ptc->CTRLFSET=TC_CMD_RESET_gc;
}

// Disable an USART
void usart_disable(USART_t *pu)
{
// Rx and Tx are off
pu->CTRLB=0;
// Ensure that all interrupts generated by the USART are off
pu->CTRLA=0;
}

// USARTF0 initialization
void usartf0_init(void)
{
// Note: the correct PORTF direction for the RxD, TxD and XCK signals
// is configured in the ports_init function

// Transmitter is enabled
// Set TxD=1
PORTF.OUTSET=0x08;

// Communication mode: Asynchronous USART
// Data bits: 8
// Stop bits: 1
// Parity: Disabled
USARTF0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

// Receive complete interrupt: Disabled
// Transmit complete interrupt: Disabled
// Data register empty interrupt: Disabled
USARTF0.CTRLA=(USARTF0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
    USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;

// If We are using 32MHz Clock
#ifdef SYSTEM_CLOCK_32MHZ && SYSTEM_CLOCK_32MHZ_115200
// Required Baud rate: 115200
// Real Baud Rate: 115211.5 (x1 Mode), Error: 0.0 %
USARTF0.BAUDCTRLA=0x2E;
USARTF0.BAUDCTRLB=((0x09 << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x08;
#elif  SYSTEM_CLOCK_32MHZ
// Required Baud rate: 19200
// Real Baud Rate: 19196.2, Error: 0.0 %
USARTF0.BAUDCTRLA=0xE5;
USARTF0.BAUDCTRLB=((0x0B << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x0C;
#else
// If we are using 2MHz Clock
// Required Baud rate: 19200
// Real Baud Rate: 19196.2 (x2 Mode), Error: 0.0 %
USARTF0.BAUDCTRLA=0x03;
USARTF0.BAUDCTRLB=((0x09 << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x06;
#endif

// Required Baud rate: 115200
// Real Baud Rate: 115211.5 (x1 Mode), Error: 0.0 %
//USARTF0.BAUDCTRLA=0x2E;
//USARTF0.BAUDCTRLB=((0x09 << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x08;

#ifdef SYSTEM_CLOCK_32MHZ
// Receiver: Off
// Transmitter: On
// Double transmission speed mode: Off
// Multi-processor communication mode: Off
USARTF0.CTRLB=(USARTF0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_TXEN_bm;
#else   
// Receiver: Off
// Transmitter: On
// Double transmission speed mode: On
// Multi-processor communication mode: Off
USARTF0.CTRLB=(USARTF0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_TXEN_bm | USART_CLK2X_bm;
#endif        
}

// Write a character to the USARTF0 Transmitter
#pragma used+
void putchar_usartf0(char c)
{
while ((USARTF0.STATUS & USART_DREIF_bm) == 0);
USARTF0.DATA=c;
}
#pragma used-

// Timer/Counter TCC0 initialization
void tcf0_init(void)
{
unsigned char s;
unsigned char n;

// Note: the correct PORTC direction for the Compare Channels outputs
// is configured in the ports_init function

// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
#asm("cli")

// Disable and reset the timer/counter just to be sure
tc0_disable(&TCF0);
// Clock source: Peripheral Clock/64
TCF0.CTRLA=(TCF0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV1024_gc;
// Mode: Normal Operation, Overflow Int./Event on TOP
// Compare/Capture on channel A: Off
// Compare/Capture on channel B: Off
// Compare/Capture on channel C: Off
// Compare/Capture on channel D: Off
TCF0.CTRLB=(TCF0.CTRLB & (~(TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE_gm))) |
    TC_WGMODE_NORMAL_gc;

// Capture event source: None
// Capture event action: None
TCF0.CTRLD=(TCF0.CTRLD & (~(TC0_EVACT_gm | TC0_EVSEL_gm))) |
    TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

// Overflow interrupt: Low Level
// Error interrupt: Disabled
TCF0.INTCTRLA=(TCF0.INTCTRLA & (~(TC0_ERRINTLVL_gm | TC0_OVFINTLVL_gm))) |
    TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_LO_gc;

// Compare/Capture channel A interrupt: Disabled
// Compare/Capture channel B interrupt: Disabled
// Compare/Capture channel C interrupt: Disabled
// Compare/Capture channel D interrupt: Disabled
TCF0.INTCTRLB=(TCF0.INTCTRLB & (~(TC0_CCDINTLVL_gm | TC0_CCCINTLVL_gm | TC0_CCBINTLVL_gm | TC0_CCAINTLVL_gm))) |
    TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

// High resolution extension: Off
HIRESC.CTRL&= ~HIRES_HREN0_bm;

// Advanced Waveform Extension initialization
// Optimize for speed
#pragma optsize- 
// Disable locking the AWEX configuration registers just to be sure
n=MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
CCP=CCP_IOREG_gc;
MCU.AWEXLOCK=n;
// Restore optimization for size if needed
#pragma optsize_default

// Pattern generation: Off
// Dead time insertion: Off
AWEXC.CTRL&= ~(AWEX_PGM_bm | AWEX_CWCM_bm | AWEX_DTICCDEN_bm | AWEX_DTICCCEN_bm | AWEX_DTICCBEN_bm | AWEX_DTICCAEN_bm);

// Fault protection initialization
// Fault detection on OCD Break detection: On
// Fault detection restart mode: Latched Mode
// Fault detection action: None (Fault protection disabled)
AWEXC.FDCTRL=(AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) |
    AWEX_FDACT_NONE_gc;
// Fault detect events: 
// Event channel 0: Off
// Event channel 1: Off
// Event channel 2: Off
// Event channel 3: Off
// Event channel 4: Off
// Event channel 5: Off
// Event channel 6: Off
// Event channel 7: Off
AWEXC.FDEVMASK=0b00000000;
// Make sure the fault detect flag is cleared
AWEXC.STATUS|=AWEXC.STATUS & AWEX_FDF_bm;

// Clear the interrupt flags
TCF0.INTFLAGS=TCF0.INTFLAGS;
// Set counter register
TCF0.CNT=0x0000;
// Set period register
#ifdef SYSTEM_CLOCK_32MHZ
// if 32 mhz clock is being used
TCF0.PER= 0x8000; //0x0100
#else
// if 2 mhz clock is being used
TCF0.PER= 0x0800; //0x0100
#endif
// Set channel A Compare/Capture register
TCF0.CCA=0x0000;
// Set channel B Compare/Capture register
TCF0.CCB=0x0000;
// Set channel C Compare/Capture register
TCF0.CCC=0x0000;
// Set channel D Compare/Capture register
TCF0.CCD=0x0000;

// Restore interrupts enabled/disabled state
SREG=s;
}

// Timer/counter TCF0 Overflow/Underflow interrupt service routine
interrupt [TCF0_OVF_vect] void tcf0_overflow_isr(void)
{
//write your code here
unsigned long int Pos_Count_HCTL = 0x00000000;
unsigned char MSB = 0;
unsigned char MSB_2nd = 0;
unsigned char MSB_3rd = 0;
unsigned char LSB = 0;
unsigned int Pos_Count = 0;
unsigned char Pos_Count_Low, Pos_Count_Hi;
unsigned Data_Send_Low, Data_Send_Hi;
unsigned char * p;
unsigned char i;
char hello_str[] = "Hello World";
char mystr[8];

//ftoa(12470.547031,2,mystr);
//putsf("The floating point value is: ");
//puts(hello_str);
//puts(mystr);

/* For debugging*/
//unsigned char MSB = 0;
//unsigned char MSB_2nd = 0;
//unsigned char MSB_3rd = 0;
//unsigned char LSB = 0;
//unsigned char MSB_old = 0;
//unsigned char MSB_2nd_old = 0;
//unsigned char MSB_3rd_old = 0;
//unsigned char LSB_old = 0;
//unsigned char MSB_new = 0;
//unsigned char MSB_2nd_new = 0;
//unsigned char MSB_3rd_new = 0;
//unsigned char LSB_new = 0;

/******** ATxmega128A1 Internal Quadrature Decoder  ********/
/* Read and Send Position Count from Internal QDEC of ATxmega */
//Pos_Count = TCC0.CNT;
//Pos_Count += NumOfRev*(4*lineCount - 1);
//Pos_Count_Low = (unsigned char) Pos_Count;
//Pos_Count_Hi = (unsigned char) Pos_Count>>8;
//Data_Send_Low = Pos_Count_Low;
//Data_Send_Hi = Pos_Count_Hi;
//putchar_usartf0(Data_Send_Hi);
//putchar_usartf0(Data_Send_Low);

/******** External Quadrature Decoders using HCTL2032 ********/

/* First HCTL2032: Encoder 1- Read and Send Position Count from QDEC of Second HCTL2032 */
//Pos_Count_HCTL = HCTL2032_Read_Count_Data(&PORTD, &PORTE, false);
/* if we initialized Pos_Count_HCTL = 0x0A0B0C0D then LSB is not sent properly. 
if we change the data from 0D to something else then it works*/
//Pos_Count_HCTL = Pos_Count_HCTL_1;
//LSB = (unsigned char) Pos_Count_HCTL; 
//MSB_3rd = (unsigned char) (Pos_Count_HCTL>>8);
//MSB_2nd = (unsigned char) (Pos_Count_HCTL>>16);
//MSB = (unsigned char) (Pos_Count_HCTL>>24);
//putchar_usartf0(MSB);
//putchar_usartf0(MSB_2nd);
//putchar_usartf0(MSB_3rd);
//putchar_usartf0(LSB);

//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);


/* First HCTL2032: Encoder 2- Read and Send Position Count from QDEC of First HCTL2032 */
//Pos_Count_HCTL = HCTL2032_Read_Count_Data(&PORTD, &PORTE, true);
/* if we initialized Pos_Count_HCTL = 0x0A0B0C0D then LSB is not sent properly. 
if we change the data from 0D to something else then it works*/
//Pos_Count_HCTL = Pos_Count_HCTL_2;
//LSB = (unsigned char) Pos_Count_HCTL; 
//MSB_3rd = (unsigned char) (Pos_Count_HCTL>>8);
//MSB_2nd = (unsigned char) (Pos_Count_HCTL>>16);
//MSB = (unsigned char) (Pos_Count_HCTL>>24);
//putchar_usartf0(MSB);
//putchar_usartf0(MSB_2nd);
//putchar_usartf0(MSB_3rd);
//putchar_usartf0(LSB);

//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);

/* Second HCTL2032: Encoder 3- Read and Send Position Count from QDEC of Second HCTL2032 */
//Pos_Count_HCTL = HCTL2032_Read_Count_Data(&PORTH, &PORTC, false);
/* if we initialized Pos_Count_HCTL = 0x0A0B0C0D then LSB is not sent properly. 
if we change the data from 0D to something else then it works*/
//Pos_Count_HCTL = Pos_Count_HCTL_3;
//LSB = (unsigned char) Pos_Count_HCTL; 
//MSB_3rd = (unsigned char) (Pos_Count_HCTL>>8);
//MSB_2nd = (unsigned char) (Pos_Count_HCTL>>16);
//MSB = (unsigned char) (Pos_Count_HCTL>>24);
//putchar_usartf0(MSB);
//putchar_usartf0(MSB_2nd);
//putchar_usartf0(MSB_3rd);
//putchar_usartf0(LSB);

//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);


/* Second HCTL2032: Encoder 4- Read and Send Position Count from QDEC of Second HCTL2032 */
//Pos_Count_HCTL = HCTL2032_Read_Count_Data(&PORTH, &PORTC, true);
/* if we initialized Pos_Count_HCTL = 0x0A0B0C0D then LSB is not sent properly. 
if we change the data from 0D to something else then it works*/
//Pos_Count_HCTL = Pos_Count_HCTL_4;
//LSB = (unsigned char) Pos_Count_HCTL; 
//MSB_3rd = (unsigned char) (Pos_Count_HCTL>>8);
//MSB_2nd = (unsigned char) (Pos_Count_HCTL>>16);
//MSB = (unsigned char) (Pos_Count_HCTL>>24);
//putchar_usartf0(MSB);
//putchar_usartf0(MSB_2nd);
//putchar_usartf0(MSB_3rd);
//putchar_usartf0(LSB);

//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);


/*** Send State Data in Raw Form ***/

// Put 0xFF to signal mote that we start sending data now
//putchar_usartf0(0xFF);

//// Send Trolley Position
//p = (unsigned char *) & x_1;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Trolley Speed
//p = (unsigned char *) & x_2;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Payload X angle
//p = (unsigned char *) & x_3;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Payload X angle Speed
//p = (unsigned char *) & x_4;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Integrator State 1
//p = (unsigned char *) & x_a1;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Arm Position
//p = (unsigned char *) & x_5;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Arm Speed
//p = (unsigned char *) & x_6;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Payload Y angle
//p = (unsigned char *) & x_7;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Payload Y angle Speed
//p = (unsigned char *) & x_8;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Integrator State 2
//p = (unsigned char *) & x_a2;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Control Signal For Trolley, u_1
//p = (unsigned char *) & u_1;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}
//
//// Send Control Signal For Arm, u_2
//p = (unsigned char *) & u_2;
//for (i=0; i<4; i++)
//{ 
// putchar_usartf0( *p++ ); 
//}

//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//putchar_usartf0(0xFF);
//
//ftoa(x_3,4,mystr);
//putsf("The x_1 value is: ");
//puts(mystr);

//printf("x_1, (Trolley Position): Encoder Counts = %ld => %f m\n\r",Pos_Count_HCTL_3,x_1);
//printf("x_2, (Trolley Speed): \t\t\t      %f m/s\n\r",x_2);
//printf("x_3, (Payload X Angle):Encoder Counts = %ld => %f rad\n\r",Pos_Count_HCTL_1,x_3);
//printf("x_4, (Payload X Angle Speed): \t\t    %f rad/s\n\r",x_4);
//printf("x_a1, (Integrator State 1):\t %f\n\r",x_a1);
//printf("x_5, (Arm Position): Encoder Counts = %ld => %f rad\n\r",Pos_Count_HCTL_4,x_5);
//printf("x_6, (Arm Speed): \t\t\t      %f rad/s\n\r",x_6);
//printf("x_7, (Payload Y Angle):Encoder Counts = %ld => %f rad\n\r",Pos_Count_HCTL_2,x_7);
//printf("x_8, (Payload Y Angle Speed): \t\t     %f rad/s/\n\r",x_8);
//printf("x_a2, (Integrator State 2):\t %f\n\r",x_a2);

//printf("x_1 = %ld => %f m\n\r",Pos_Count_HCTL_3,x_1);
//printf("x_2 = \t     %f m/s\n\r",x_2);
//printf("x_3 = %ld => %f rad\n\r",Pos_Count_HCTL_1,x_3);
//printf("x_4 = \t    %f rad/s\n\r",x_4);
//printf("x_a1 = \t %f\n\r",x_a1);
//printf("x_5 = %ld => %f rad\n\r",Pos_Count_HCTL_4,x_5);
//printf("x_6 = \t      %f rad/s\n\r",x_6);
//printf("x_7 = %ld => %f rad\n\r",Pos_Count_HCTL_2,x_7);
//printf("x_8 = \t     %f rad/s/\n\r",x_8);
//printf("x_a2 = \t %f\n\r",x_a2);
//printf("u_1 = \t %f\n\r",u_1);
//printf("u_2 = \t %f\n\r",u_2);


}


void tcd0_init(void)
{
unsigned char s;
unsigned char n;

// Note: the correct PORTC direction for the Compare Channels outputs
// is configured in the ports_init function

// Save interrupts enabled/disabled state
s=SREG;
// Disable interrupts
#asm("cli")

// Disable and reset the timer/counter just to be sure
tc0_disable(&TCD0);

// Note that we want to set sampling time of 10 ms.
// so these settings are according to that for 2MHz clock
// for 32MHz use Clock/64
#ifdef SYSTEM_CLOCK_32MHZ
// if 32 mhz clock is being used
TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV64_gc; 
//TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV1024_gc;   // temporary change
#else
// if 2 mhz clock is being used. Clock source: Peripheral Clock/4.
TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV4_gc;
#endif

// Overflow interrupt: Low Level
// Error interrupt: Disabled
TCD0.INTCTRLA=(TCD0.INTCTRLA & (~(TC0_ERRINTLVL_gm | TC0_OVFINTLVL_gm))) | TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_LO_gc;

// Mode: Dual Slope PWM Operation, Overflow Int./Event on TOP and Bottom
// Compare/Capture on channel A: Off
// Compare/Capture on channel B: Off
// Compare/Capture on channel C: Off
// Compare/Capture on channel D: Off
TCD0.CTRLB=(TCD0.CTRLB & (~(TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE_gm))) | TC_WGMODE_NORMAL_gc;

// Clear the interrupt flags
TCD0.INTFLAGS=TCD0.INTFLAGS;

// Set counter register
TCD0.CNT=0x0000;
//// Set period register
TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
//TCD0.PER= 0x30D4; // Uncomment For periodic
//// Set channel A Compare/Capture register
TCD0.CCA=0x0000;
// Set channel B Compare/Capture register
TCD0.CCB=0x0000;
// Set channel C Compare/Capture register
TCD0.CCC=0x0000;
// Set channel D Compare/Capture register
TCD0.CCD=0x0000;

// Restore interrupts enabled/disabled state
SREG=s;
}

 //Timer/counter TCD0 overflow/Underflow interrupt service routine
interrupt [TCD0_OVF_vect] void tcd0_overflow_isr(void)
{
/*status registered is not stored automatically when entering ISR and restored when leaving ISR. See pg. 9 in device manual*/

// write your code here
//unsigned int Pos_Count = 0;
//unsigned char Pos_Count_Low, Pos_Count_Hi;
//unsigned Data_Send_Low, Data_Send_Hi;
unsigned char * p;
unsigned char i;

// Measure the control delay
//PORTF.OUTTGL = PIN6_bm;

// To Control Inter-Event Time
iterations +=1;

/*** Payload X Angle Count Data ***/
/***** First HCTL2032: Encoder 1- Read and Send Position Count from QDEC of Second HCTL2032 *****/
Pos_Count_HCTL_1 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, false);
Temp_Pos_Count_HCTL_1 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, false);

/****  Check for Counter Underflow for Payload X Angle ****/
// I am assuming that overflow should never happen for crane which is reasonable to assume as crane has well defined boundries for state space

if(counter_state1 == 0x02)
Pos_Count_HCTL_1 = (signed long)Temp_Pos_Count_HCTL_1 - 4294967295;
else
Pos_Count_HCTL_1 = (signed long)Temp_Pos_Count_HCTL_1; 
// Scale the encoder count between  -2147483648 t0 2147483647
// Pos_Count_HCTL_1 = -2147483648 +  Temp_Pos_Count_HCTL_1;

/*** Payload Y Angle Count Data ***/
/****** First HCTL2032: Encoder 2- Read and Send Position Count from QDEC of First HCTL2032 ******/

Pos_Count_HCTL_2 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, true);
Temp_Pos_Count_HCTL_2 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, true);

/****  Check for Counter Underflow for Payload Y Angle ****/
if(counter_state2 == 0x02)
Pos_Count_HCTL_2 = (signed long)Temp_Pos_Count_HCTL_2 - 4294967295;
else
Pos_Count_HCTL_2 = (signed long)Temp_Pos_Count_HCTL_2;

/*** Trolley Position Count Data ***/
/* Second HCTL2032: Encoder 3- Read and Send Position Count from QDEC of Second HCTL2032 */
Pos_Count_HCTL_3 = HCTL2032_Read_Count_Data(&PORTH, &PORTC, false);

/*** Arm Position Count Data ***/
/* Second HCTL2032: Encoder 4- Read and Send Position Count from QDEC of Second HCTL2032 */
Pos_Count_HCTL_4 = HCTL2032_Read_Count_Data(&PORTH, &PORTC, true);

/********** Trolley and Arm Position Data *************/

/* Calculate Trolley Position 'x_1' in meters */
x_1 = (float)Pos_Count_HCTL_3*2.0*PI*r_x/(float)PPR3;

/* Calculate Arm Position 'x_5' in radians*/
x_5 = (float)Pos_Count_HCTL_4*2.0*PI/(float)PPR4;

/********** Payload Angle Data ***********/

/* Calculate X-Angle 'x_3' in Radians */
x_3 = (float)Pos_Count_HCTL_1*2.0*PI/(float)PPR1;
// Invert Position (see thesis page 41 for description)
//x_3 = -x_3; // not required if we are reading encoders on ED 

/* Calculate Y-Angle 'x_7' in Radians */
x_7 = (float)Pos_Count_HCTL_2*2.0*PI/(float)PPR2;

/*********** Estimate Speed States ***********/
/********* EULER APPROXIMATION **********/
// ad and bd may be different for different states but for time being we are using the same for all. 
// Consider using different in case estimation doesnt work.
/* Trolley Speed m/sec*/
//x_2 = ad1*x_2 + bd1*(x_1 - x_1_old);

/*X-Angle Speed rad/sec*/
//x_4 = ad1*x_4 + bd1*(x_3 - x_3_old);
// Invert Speed (see thesis page 41)
//x_4 = x_4; // not required if we are reading encoders on ED 

/*Arm Speed rad/sec*/
//x_6 = ad*x_6 + bd*(x_5 - x_5_old);

/*Y-Angle Speed rad/sec */
//x_8 = ad1*x_8 + bd1*(x_7 - x_7_old);

/********** MODEL BASED SPEED ESTIMATION ***********/

// Varying Parameter
Phi = 1.0/(1.0+0.0138*x_1_old*x_1_old);
//// Trolley Speed
x_2 = (1.0-41.667*h)*x_2_old+h*(-0.8464*x_3_old+5.8808*u_1_old);
//// Payload X angle speed
x_4 = x_4_old + h*(-69.4450*x_2_old-17.7607*x_3_old+9.8013*u_1_old);
//// Arm Speed
x_6 = (1.0 - 17.3765*h*Phi)*x_6_old + h*Phi*(0.0618*x_1_old*x_7_old+11.912*u_2_old);
//// Payload Y angle speed
x_8 = x_8_old + h*(-16.35*x_7_old-Phi*x_1_old*(-28.9608*x_6_old + 0.1030*x_1_old*x_7_old + 19.8533*u_2_old));


/*** Put Limit on Event Generation ***/
// Reset Payload Angles
// Due to Quantization Effect because Encoders resolution is 0.0015 radians
// Allow for 5mm deviation from zero,, s = r*alpha => alpha = 0.005/0.6
if(fabs(x_3) < 0.0) //  0.0083, 0.005 ,0.0
{
x_3 = 0.0;
x_4 = 0.0;
}
if(fabs(x_7) < 0.0) //  0.0083 , 0.005
{
x_7 = 0.0;
x_8 = 0.0;
//flag_ref = 1;
}

/**** Generate Reference Signal ****/
count_ref +=1;
//if(count_ref == 3000)
if(count_ref == (unsigned long)((float)20/h))
{
r_1 = 0.40;
r_2 = 2.3562; // 135 degree
flag_ref = 1; // signal to integrator that new reference command is available
}
//if(count_ref == 6000)
if(count_ref == (unsigned long)((float)35/h))
{
r_1 = 0.3;
r_2 = home_theta;
flag_ref = 1; // signal to integrator that new reference command is available
}

/*********** Estimate Integrator State ***********/
// bi may be different for x_a1 and x_a2. For time being we are using the same for both x_a1 and x_a2
// Note that bi1 = K15*h , bi2 = K25*h , a01 = h/Tt1 and a02 = h/Tt2
// We are not using Anti-Windup Scheme. May be added later!! We have now added !!
//if(((start_flag == 1) && (fabs(r_1 - x_1) > 0.005*r_1) && (fabs(r_2 - x_2)>0.005*r_2)) || flag_ref )
if(start_flag == 1)
{
if((fabs(r_1 - x_1) > 0.005*r_1) || flag_ref )
x_a1 = x_a1_old + bi1*(r_1 - x_1) + 0.2*(u_1 - v_1); // x_a1_old + bi*(x_1 - r_1);
if((fabs(r_2 - x_2)> 0.005*r_2) || flag_ref ) 
x_a2 = x_a2_old + bi2*(r_2 - x_5) + 0.2*(u_2 - v_2); // x_a2_old + bi*(x_5 - r_2);
}



if(Periodic_ON == 0 && start_flag)    // IF ETC Selected
{
    /*******  Computation of Event Generation Rule (EGR) ********/
    //  norm(e) >= sigma*norm(epsilon) + delta

    // Compute Norms
    // Shifted State Norm
    // x_a1* = -K11*r_1 , x_a2* = -K21*r_2 
    norm_epsilon = sqrt((x_1 - r_1)*(x_1 - r_1) + x_2*x_2 + w_3*x_3*x_3 + w_4*x_4*x_4 + (x_a1 + K11*r_1)*(x_a1 + K11*r_1) + (x_5 - r_2)*(x_5 - r_2) + x_6*x_6 + w_8*x_7*x_7 + w_9*x_8*x_8 + (x_a2 + K21*r_2)*(x_a2 + K21*r_2));

    // Measurement Error Norm, Left Hand Side of EGR 
    norm_e = sqrt((x_1 - x_1_tk)*(x_1 - x_1_tk) + (x_2 - x_2_tk)*(x_2 - x_2_tk) + (x_3 - x_3_tk)*(x_3 - x_3_tk) + (x_4 - x_4_tk)*(x_4 - x_4_tk) + (x_a1 - x_a1_tk)*(x_a1 - x_a1_tk) + (x_5 - x_5_tk)*(x_5 - x_5_tk) + (x_6 - x_6_tk)*(x_6 - x_6_tk) + (x_7 - x_7_tk)*(x_7 - x_7_tk) + (x_8 - x_8_tk)*(x_8 - x_8_tk) + (x_a2 - x_a2_tk)*(x_a2 - x_a2_tk));

    if((norm_e >= (sigma*norm_epsilon + delta)) && start_flag) // uncomment for ETC
    //if(start_flag) // uncomment for periodic TTC
    {
    
        //iterations += 1;
    
        //if(iterations == 3)
        //if(iterations > 0)
        {
        
            /*** Compute the control Signal ***/
            // Controller For Trolley
            v_1 = K11*x_1 + K12*x_2 + K13*(x_3) + K14*(x_4) + x_a1;  // x_3 and x_4 negated due to convention followed for alpha (= -X) angle in thesis
            // Controller For Arm
            v_2 = K21*x_5 + K22*x_6 + K23*x_7 + K24*x_8 + x_a2;

            // Saturate the output control signal
            // Saturate Trolley Control Signal
            if(v_1 > 1.0)
            u_1 = 1.0;
            else if(v_1 < -1.0)
            u_1 = -1.0;
            else
            u_1 = v_1;

            // Saturate Arm Control Signal
            if(v_2 > 1.0)
            u_2 = 1.0;
            else if(v_2 < -1.0)
            u_2 = -1.0;
            else
            u_2 = v_2;

            // Flag for Start of Transmission to measure loop Delay
            //PORTF.OUTTGL = PIN6_bm;
            
            /***** SEND THE DATA OVER SERIAL RS232 *****/

            // SEND DATA IN ASCII
            //printf("u_1 = %f \tx_1 = %f \n\r",u_1,x_1);
            //printf("u_2 = %f \tx_5 = %f \n\r",u_2,x_5);
            
            // Transmit only if it is first transmission or the diference between two consecutive is greater than 30ms
            if(((iterations - iterations_old) >= 3) || first_transmission)
            {
            // SEND CONTROL DATA IN RAW FROM
            //Send Control Signal For Trolley, u_1
            p = (unsigned char *) & u_1;
            for (i=0; i<4; i++)
            { 
             putchar_usartf0( *p++ ); 
            }
            // Send Control Signal For Arm, u_2
            p = (unsigned char *) & u_2;
            for (i=0; i<4; i++)
            { 
             putchar_usartf0( *p++ ); 
            }            
            // SEND REFERENCE DATA IN RAW FROM
            //Send Reference Signal For Trolley, r_1
            p = (unsigned char *) & r_1;
            for (i=0; i<4; i++)
            { 
             putchar_usartf0( *p++ ); 
            }
            // Send Reference Signal For Arm, r_2
            p = (unsigned char *) & r_2;
            for (i=0; i<4; i++)
            { 
             putchar_usartf0( *p++ ); 
            }
            iterations_old = iterations;
            first_transmission = 0;
            // Store the state value at the time of transmission of currrent trigger/event
            x_1_tk = x_1;
            x_2_tk = x_2;
            x_3_tk = x_3;
            x_4_tk = x_4;
            x_a1_tk = x_a1;
            x_5_tk = x_5;
            x_6_tk = x_6;
            x_7_tk = x_7;
            x_8_tk = x_8;
            x_a2_tk = x_a2;
            }



            
                                
        }

    };
    
}

else if(Periodic_ON == 1 && start_flag)  // If periodic Selected
{
    /*** Compute the control Signal ***/
    // Controller For Trolley
    v_1 = K11*x_1 + K12*x_2 + K13*(x_3) + K14*(x_4) + x_a1;  // x_3 and x_4 negated due to convention followed for alpha (= -X) angle in thesis
    // Controller For Arm
    v_2 = K21*x_5 + K22*x_6 + K23*x_7 + K24*x_8 + x_a2;

    // Saturate the output control signal
    // Saturate Trolley Control Signal
    if(v_1 > 1.0)
    u_1 = 1.0;
    else if(v_1 < -1.0)
    u_1 = -1.0;
    else
    u_1 = v_1;

    // Saturate Arm Control Signal
    if(v_2 > 1.0)
    u_2 = 1.0;
    else if(v_2 < -1.0)
    u_2 = -1.0;
    else
    u_2 = v_2;
    
    // Flag for Start of Transmission to measure loop Delay
    //PORTF.OUTTGL = PIN6_bm;

    /***** SEND THE DATA OVER SERIAL RS232 *****/
    
    // Trigger to tell we gonna send the data
    // used just for testing purpose
    PORTF.OUTTGL = PIN6_bm;
    //#asm   
    //nop
    //#endasm
    //PORTF.OUTTGL = PIN6_bm;

    // SEND DATA IN ASCII
    //printf("u_1 = %f \tx_1 = %f \n\r",u_1,x_1);
    //printf("u_2 = %f \tx_5 = %f \n\r",u_2,x_5);

    // SEND CONTROL DATA IN RAW FROM
    //Send Control Signal For Trolley, u_1
    p = (unsigned char *) & u_1;
    for (i=0; i<4; i++)
    { 
     putchar_usartf0( *p++ ); 
    }
    // Send Control Signal For Arm, u_2
    p = (unsigned char *) & u_2;
    for (i=0; i<4; i++)
    { 
     putchar_usartf0( *p++ ); 
    }
     
    
    // SEND REFERENCE DATA IN RAW FROM
    //Send Reference Signal For Trolley, r_1
    p = (unsigned char *) & r_1;
    for (i=0; i<4; i++)
    { 
     putchar_usartf0( *p++ ); 
    }               
    // Send Reference Signal For Arm, r_2
    p = (unsigned char *) & r_2;
    for (i=0; i<4; i++)
    { 
     putchar_usartf0( *p++ ); 
    }

};




/***** Store Current State for use in next (plant) sampling step *******/
x_1_old = x_1;
x_2_old = x_2;
x_3_old = x_3;
x_4_old = x_4;
x_5_old = x_5;
x_6_old = x_6;
x_7_old = x_7;
x_8_old = x_8;
u_1_old = u_1;
u_2_old = u_2;
x_a1_old = x_a1;
x_a2_old = x_a2;

flag_ref = 0;  // Bring the flag down to make it ready for next reference update

//PORTF.OUTTGL = PIN6_bm; 

}

// PORTJ interrupt 0 service routine
interrupt [PORTJ_INT0_vect] void portj_int0_isr(void)
{
// write your code here
unsigned long int temp = 0x00000000;

PORTJ.INTFLAGS |= PORT_INT0IF_bm;
/* First HCTL2032: Encoder 1- Read and Send Position Count from QDEC of Second HCTL2032 */
temp = HCTL2032_Read_Count_Data(&PORTD, &PORTE, false);

if(temp < 500)
counter_state1 = 0x01;  //it is overflow
//NumOfUF =
else
counter_state1 = 0x02; // otherwise it is underflow

//putchar_usartf0(0xAA);
//putchar_usartf0(0xAA);
//putchar_usartf0(0xAA);
//putchar_usartf0(0xAA);
}

// PORTK interrupt 0 service routine
interrupt [PORTK_INT0_vect] void portk_int0_isr(void)
//interrupt [PORTJ_INT1_vect] void portj_int1_isr(void)
{
// write your code here
unsigned char * p;
unsigned char i;
//unsigned long int temp = 0x00000000;
/* First HCTL2032: Encoder 2- Read and Send Position Count from QDEC of First HCTL2032 */
//temp = HCTL2032_Read_Count_Data(&PORTD, &PORTE, true);

//if(temp < 500)
//counter_state2 = 0x01; // it is overflow
//else
//counter_state2 = 0x02; // otherwise it is underflow

// Reset all controller states
u_1 = 0;
u_2 = 0;
v_1 = 0;
v_2 = 0;
x_a1 = 0;
x_a1_old = 0;
x_a1_tk = 0;
x_a2 = 0;
x_a2_old = 0;
x_a2_tk = 0;
start_flag = 0;

//Added Later
u_1_old = 0;
u_2_old = 0;


//printf("STOP, u_1 = %f\t u_2 = %f\n\r",u_1,u_2);
// SEND DATA IN RAW FROM
//Send Control Signal For Trolley, u_1
p = (unsigned char *) & u_1;
for (i=0; i<4; i++)
{ 
putchar_usartf0( *p++ ); 
}

// Send Control Signal For Arm, u_2
p = (unsigned char *) & u_2;
for (i=0; i<4; i++)
{ 
putchar_usartf0( *p++ ); 
}
}

// Ports initialization
void ports_init(void)
{
// PORTF initialization
// OUT register
PORTF.OUT=0x08;
// Bit0: Input
// Bit1: Input
// Bit2: Input
// Bit3: Output
// Bit4: Input
// Bit5: Input
// Bit6: Input
// Bit7: Input
PORTF.DIR=0x08;
// Bit0 Output/Pull configuration: Totempole/No
// Bit0 Input/Sense configuration: Sense both edges
// Bit0 inverted: Off
// Bit0 slew rate limitation: Off
PORTF.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit1 Output/Pull configuration: Totempole/No
// Bit1 Input/Sense configuration: Sense both edges
// Bit1 inverted: Off
// Bit1 slew rate limitation: Off
PORTF.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit2 Output/Pull configuration: Totempole/No
// Bit2 Input/Sense configuration: Sense both edges
// Bit2 inverted: Off
// Bit2 slew rate limitation: Off
PORTF.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit3 Output/Pull configuration: Totempole/No
// Bit3 Input/Sense configuration: Sense both edges
// Bit3 inverted: Off
// Bit3 slew rate limitation: Off
PORTF.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit4 Output/Pull configuration: Totempole/No
// Bit4 Input/Sense configuration: Sense both edges
// Bit4 inverted: Off
// Bit4 slew rate limitation: Off
PORTF.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit5 Output/Pull configuration: Totempole/No
// Bit5 Input/Sense configuration: Sense both edges
// Bit5 inverted: Off
// Bit5 slew rate limitation: Off
PORTF.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit6 Output/Pull configuration: Totempole/No
// Bit6 Input/Sense configuration: Sense both edges
// Bit6 inverted: Off
// Bit6 slew rate limitation: Off
PORTF.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Bit7 Output/Pull configuration: Totempole/No
// Bit7 Input/Sense configuration: Sense both edges
// Bit7 inverted: Off
// Bit7 slew rate limitation: Off
PORTF.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
// Interrupt 0 level: Disabled
// Interrupt 1 level: Disabled
PORTF.INTCTRL=(PORTF.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
    PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
// Bit0 pin change interrupt 0: Off
// Bit1 pin change interrupt 0: Off
// Bit2 pin change interrupt 0: Off
// Bit3 pin change interrupt 0: Off
// Bit4 pin change interrupt 0: Off
// Bit5 pin change interrupt 0: Off
// Bit6 pin change interrupt 0: Off
// Bit7 pin change interrupt 0: Off
PORTF.INT0MASK=0x00;
// Bit0 pin change interrupt 1: Off
// Bit1 pin change interrupt 1: Off
// Bit2 pin change interrupt 1: Off
// Bit3 pin change interrupt 1: Off
// Bit4 pin change interrupt 1: Off
// Bit5 pin change interrupt 1: Off
// Bit6 pin change interrupt 1: Off
// Bit7 pin change interrupt 1: Off
PORTF.INT1MASK=0x00;

// Pin configuration for Switch Port
//Configure all as Input
PORTK.DIRCLR = 0xFF;
//Enable Interrupt 0 on PIN2
PORTK.PIN2CTRL = (PORTK.PIN2CTRL & (~(PORT_ISC_gm | PORT_OPC_gm))) | PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;
/* Mask Interrupt 0 for PIN7*/
PORTK.INT0MASK |= PIN2_bm;
/* Enable the interrupt 0 and mark it as high level*/
PORTK.INTCTRL = (PORTK.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_HI_gc;
}

void estimation_param_init(void)
{
    /* Calculate Estimation Parameters */
    bi1 = K15*h;
    bi2 = K25*h;
    a01 = h/Tt1;   
    a02 = h/Tt2;
    ad = 1.0/(1.0 + (float)N*h);
    bd = (float)N/(1.0 + (float)N*h);
    
    ad1 = 1.0/(1.0 + (float)N1*h);
    bd1 = (float)N1/(1.0 + (float)N1*h);
}




/*! \brief Quadrature decoding example.
 *
 *  This is the main example code demonstrating the setup and configuration
 *  to make the quadrature decoder work. The example use the event system and
 *  a Timer/Counter to decode the signals and the counter value in the
 *  Timer/Counter will reflect the angle of rotation. The direction bit in the
 *  TC reflect the current direction of rotation.
 *
 *  Hardware setup:
 *   - PD0 - QDPH0
 *   - PD1 - QDPH90
 *   - PD2 - QDINDX
 *
 *  Peripherals used:
 *   - PORTD[2:0] signal input.
 *   - Event channel 0: quadrature signal.
 *   - Event channel 1: index signal.
 *   - TCC0: Quadrature counter.
 */
 
//int main( void )
void main( void )
{
    unsigned char * p;
    unsigned char i;
    // System clocks initialization
    #ifdef SYSTEM_CLOCK_32MHZ
    system_clocks_init_32mhz();
    #else
    system_clocks_init();
    #endif
    // Disable and reset the timer/counter just to be sure
    //tc0_disable(&TCC0);
    
    PORTJ.DIRSET = EN_max3002;
    PORTJ.OUTSET = EN_max3002;
    

    
    // USARTF0 initialization
    usartf0_init();


    // Timer/Counter TCF0 initialization
    //tcf0_init();
    tcd0_init();
    
    // Port F initialization for serial transmission
    ports_init();
    
    // Set Pin6 as output for signalling start of transmisson of packet
    // this is meant to compute the loop delay
    PORTF.DIRSET = PIN6_bm;
    PORTF.OUTCLR = PIN6_bm; // set it zero initially
    
    /* Calculate Estimation Parameters */
    estimation_param_init();
//    bi1 = K15*h;
//    bi2 = K25*h;
//    a01 = h/Tt1;   
//    a02 = h/Tt2;
//    ad = 1.0/(1.0 + (float)N*h);
//    bd = (float)N/(1.0 + (float)N*h);

 

    /* Setup PORTD with pin 0 as input for QDPH0, dont invert IO pins.
     *
     * Setup event channel 0, pin 0 on PORTD as input, don't enable index.
     * if index used then state 00 is set as the index reset state.
     *
     * Setup TCC0 with Event channel 0 and lineCount.
     */
//    QDEC_Total_Setup(&PORTD,                    /*PORT_t * qPort*/
//                     0,                         /*uint8_t qPin*/
//                     false,                     /*bool invIO*/
//                     0,                         /*uint8_t qEvMux*/
//                     EVSYS_CHMUX_PORTD_PIN0_gc, /*EVSYS_CHMUX_t qPinInput*/
//                     true,                     /*bool useIndex*/
//                     EVSYS_QDIRM_00_gc,         /*EVSYS_QDIRM_t qIndexState*/
//                     &TCC0,                     /*TC0_t * qTimer*/
//                     TC_EVSEL_CH0_gc,           /*TC_EVSEL_t qEventChannel*/
//                     lineCount);                /*uint8_t lineCount*/ 
    /* Setup HCTL2032 Quadrature Decoder for Encoder 1 and 2 */
    
    HCTL2032_Total_Setup(&PORTD,                        /* The port to use for 8 bit data.*/    
                      &PORTE,                           /*The port to use for controlling HCTL2032.*/
                      &PORTJ,                           /*The Port which would read the signals from HCTL2032. These signals are kind of reporting signals
                                                        which tells about internal state of HCTL2032 like overflow/underflow, count updated etc*/ 
                      CountMode);                       /*Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x.*/
     
     /* Setup HCTL2032 Quadrature Decoder for Encoder 3 and 4 */
     
    HCTL2032_Total_Setup(&PORTH,                        /* The port to use for 8 bit data.*/    
                      &PORTC,                           /*The port to use for controlling HCTL2032.*/
                      &PORTK,                           /*The Port which would read the signals from HCTL2032. These signals are kind of reporting signals
                                                        which tells about internal state of HCTL2032 like overflow/underflow, count updated etc*/ 
                      CountMode);                       /*Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x.*/
                
    /* Setup TCD0 with Event channel 2 for same pin as QDPH0 and clk div 64. */
    // This setup the counter for speed measurement
    //QDEC_TC_Freq_Setup(&TCD0, TC_EVSEL_CH2_gc, EVSYS_CHMUX_PORTD_PIN0_gc, CLOCK_DIV_bm);

#ifdef GENERATE_TEST_SIGNAL
    /* Initialize and start outputting quadrature signal.*/
    // TCE0 is being used to generate quadrature signal
    generate_qdec_signal(&PORTE, lineCount, freq, dir);

    /* Enable low level and medium level interrupt.*/
    PMIC.CTRL |= (PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);
    //PMIC.CTRL |= PMIC_LOLVLEN_bm;
    /* Enable global interrupts.*/
    sei();
#endif

    /* Enable low level and medium level interrupt.*/
    PMIC.CTRL |= (PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);
    /* Enable global interrupts.*/
    sei();
        
   
    /* Display the frequency of rotation on LEDs */
while (1) {

       
    // Place your code here
                          
    //PORTK.DIRCLR = PIN0_bm;
    // SWITCH INTERFACE
    PORTK.DIRCLR = 0xFF;
    // START THE CRANE OPERATION WHEN SW0 IS PRESSED
    if((~PORTK.IN) & PIN7_bm)
    {
        r_1 = home_x;//0.25;//0.3;
        r_2 = home_theta;//PI*0.5;
        count_ref = 0;
        first_transmission = 1; // this is first transmission
        // RESET INTEGRATOR STATE
        x_a1 = 0.0;
        x_a1_old = 0.0;
        x_a1_tk = 0.0;
        x_a2 = 0.0;
        x_a2_old = 0.0;
        x_a2_tk = 0.0;
        x_1_old = 0;
        x_2_old = 0;
        x_3_old = 0;
        x_4_old = 0;
        x_5_old = 0;
        x_6_old = 0;
        x_7_old = 0;
        x_8_old = 0;

        u_1 = 0;
        u_2 = 0;
        u_1_old = 0;
        u_2_old = 0;
        v_1 = 0;
        v_2 = 0;
        TCD0.CNT = 0x0000; // Reset the counter
        TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
        h = 0.01;
        estimation_param_init(); // update estimation parameters
        // START CALCULATING INTEGRATOR STATE
        start_flag = 1;
        PORTF.OUTCLR = PIN6_bm;
        HCTL2032_Reset_Counter(&PORTE);
        HCTL2032_Reset_Counter(&PORTC);      
            
    }
            
    /**** Manual Controller ****/
    // Move Trolley Forward
    // SWx_pressed is being used to conditionally send the control data on serial link. It is important otherwise
    // if we pressed the button continuously then we would be sending same u continuously which is not required!        
    if((~PORTK.IN) & PIN0_bm)
    {
        start_flag = 0;
        u_1 = 0.5;
        u_2 = 0;
        if(SW0_pressed)
        {
        //printf("%f\n\r",u_1);
        //printf("%f\n\r",u_2);
        // SEND DATA IN RAW FROM
        //Send Control Signal For Trolley, u_1
        p = (unsigned char *) & u_1;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }

        // Send Control Signal For Arm, u_2
        p = (unsigned char *) & u_2;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }
        }
        // if SW is being continuously pressed then don't send control repeateadly
        SW0_pressed = 0;
    }
    else
    {
    	SW0_pressed = 1;
    }
            
    if((~PORTK.IN) & PIN1_bm)
    {
        start_flag = 0;
        u_1 = -0.5;
        u_2 = 0;
        if(SW1_pressed)
        {
        //        printf("%f\n\r",u_1);
        //        printf("%f\n\r",u_2);

        // SEND DATA IN RAW FROM
        //Send Control Signal For Trolley, u_1
        p = (unsigned char *) & u_1;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }

        // Send Control Signal For Arm, u_2
        p = (unsigned char *) & u_2;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }
        }
        // if SW is being continuously pressed then don't send control repeateadly
        SW1_pressed = 0;
    }
    else
    {
    	SW1_pressed = 1;
    }
            
    if((~PORTK.IN) & PIN3_bm)
    {
        start_flag = 0;
        u_1 = 0;
        u_2 = 0.5;
        if(SW3_pressed)
        {
        //        printf("%f\n\r",u_1);
        //        printf("%f\n\r",u_2);
        // SEND DATA IN RAW FROM
        //Send Control Signal For Trolley, u_1
        p = (unsigned char *) & u_1;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }
                                
        // Send Control Signal For Arm, u_2
        p = (unsigned char *) & u_2;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }
        }
        // if SW is being continuously pressed then don't send control repeateadly
        SW3_pressed = 0;
    }
    else
    {
    	SW3_pressed = 1;
    }
            
    if((~PORTK.IN) & PIN4_bm)
    {
        start_flag = 0;
        u_1 = 0;
        u_2 = -0.5;
        if(SW4_pressed)
        {
        //           printf("%f\n\r",u_1);
        //           printf("%f\n\r",u_2);
        // SEND DATA IN RAW FROM
        //Send Control Signal For Trolley, u_1
        p = (unsigned char *) & u_1;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }

        // Send Control Signal For Arm, u_2
        p = (unsigned char *) & u_2;
        for (i=0; i<4; i++)
        { 
        putchar_usartf0( *p++ ); 
        }
        }
        // if SW is being continuously pressed then don't send control repeateadly
        SW4_pressed = 0;
    }
    else
    {
    	SW4_pressed = 1;

    }
    
    if((~PORTK.IN) & PIN5_bm)
    {
        Periodic_ON = 0; // If SW5 is pressed Turn On ETC
        start_flag = 1;
        PORTF.OUTCLR = PIN6_bm;
        TCD0.CNT = 0x0000; // Reset the counter
        TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
        h = 0.01; // 10 ms sampling time
        /*Calculate Estimation Parameters for ETC*/
        estimation_param_init();

        
    }
    else if((~PORTK.IN) & PIN6_bm)
    {
    	Periodic_ON = 1; // If SW6 is pressed, Turn On Periodic Controller
        start_flag = 1;
        PORTF.OUTCLR = PIN6_bm;
        r_1 = home_x;//0.25;//0.3;
        r_2 = home_theta;//PI*0.5;
        count_ref = 0;
        TCD0.CNT = 0x0000; // Reset the counter
        //TCD0.PER= 0x30D4; // 25ms period
        TCD0.PER = 0x3A98; // 30 ms period
        //h = 0.025; // 25 ms sampling time
        h = 0.030; // 30 ms sampling time
        /*Calculate Estimation Parameters for TTC*/
        estimation_param_init();

    }
    
}
            
   
}