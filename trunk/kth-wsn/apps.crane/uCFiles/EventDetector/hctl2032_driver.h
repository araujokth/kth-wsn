/* This file has been prepared for Doxygen automatic documentation generation. */
/*! \file *********************************************************************
 *
 * \brief  The XMEGA HCTL2032 Quadrature Decoder driver header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the HCTL2032 Quadrature Decoder.
 *
 *      The driver is not intended for size and/or speed critical code. The
 *      driver is intended for rapid prototyping and documentation purposes for
 *      getting started with the HCTL2032 Quadrature Decoder. For size and/or 
 *      speed critical code, it is recommended to copy the  function contents 
 *      directly into your application instead of making a function call.
 *
 * \par Application note:
 *      See NetCon Wiki Page
 *      HCTL2032 Datasheet        
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
 * Copyright (c) 2010, KTH-Royal Institute of Technology, Reglerteknik. All rights reserved.
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

#ifndef HCTL2032_DRIVER_H
#define HCTL2032_DRIVER_H


#include "avr_compiler.h"

/*********** Control Port Pin Assignment **************
 * Note: no comments allowed in front of #define
 *****************************************************/
 /* SEL2 Signal which alongwith SEL1 select the byte to read from internal 32-bit register of HCTL2032 */
#define SEL2 PIN0_bm 
#define SEL2_bp PIN0_bp

/* Signal to select which axis data to be read*/
#define XNY PIN1_bm   
#define XNY_bp PIN1_bp


/* EN1 and EN2 signals select the counting mode (1x, 2x or 4x) */
#define EN2 PIN2_bm
#define EN2_bp PIN2_bm

#define EN1 PIN3_bm 
#define EN1_bp PIN3_bp

/* Active low Enable signal control the tri-state output buffer. OEN alongwith SEL1 and SEL2 also 
controls the loading of data into Position latch at falling edge of clock. OEN high allows data 
latch to be loaded with new counter data at falling edge of clock but when OEN is low then further 
loading of data from counter to latch is stopped to make sure that during read operation stable data 
belonging to same time instant is read*/
#define OEN PIN4_bm 
#define OEN_bp PIN4_bp

#define SEL1 PIN5_bm
#define SEL1_bp PIN5_bp

/* Reset signal for x-axis */
#define RSTNx PIN6_bm 
#define RSTNx_bp PIN6_bp

/* Reset signal for y-axis */
#define RSTNy PIN7_bm 
#define RSTNy_bp PIN7_bp



/*********** Signal Port Pin Assignment **************
 *                                                   *
 *****************************************************/

/* This LSTTL-compatible output allows the user to determine whether the IC is counting up or down and is intended to be 
used with the CNTDEC and CNTCAS outputs. The proper signal U (high level) or D/ (low level) will be present before the 
rising edge of the CNTDEC and CNTCAS outputs. */

#define EN_max3002 PIN0_bm
#define EN_max3002_bp PIN0_bp

#define UDx PIN2_bm
#define UDx_bp PIN2_bm

#define UDy PIN3_bm 
#define UDy_bp PIN3_bp

/*A pulse is presented on this LSTTL-compatible output when the HCTL-2032 / 2032-SC internal counter
 overflows or underflows. The rising edge on this waveform may be used to trigger an external counter.*/
/*
#define CNTCASx PIN4_bm 
#define CNTCASx_bp PIN4_bp

#define CNTCASy PIN5_bm
#define CNTCASy_bp PIN5_bp*/


#define CNTCASx PIN2_bm 
#define CNTCASx_bp PIN2_bp

#define CNTCASy PIN2_bm
#define CNTCASy_bp PIN2_bp


/*A pulse is presented on these LSTTL-compatible output when the quadrature decoder (4x/2x/1x) has 
detected a state transition. CNTDECx is for 1st axis and CNTDECy is for 2nd axis.*/

#define CNTDECy PIN6_bm 
#define CNTDECy_bp PIN6_bp

#define CNTDECx PIN7_bm 
#define CNTDECx_bp PIN7_bp


/* Prototyping of functions. */

#pragma used+

bool HCTL2032_Total_Setup(PORT_t * dPort,
                      PORT_t * cPort,
                      PORT_t * sPort,
                      unsigned char CountMode);

bool HCTL2032_Set_Count_Mode(PORT_t * cPort,unsigned char CountMode);

unsigned long int HCTL2032_Read_Count_Data(PORT_t * dPort, PORT_t * cPort, bool axis);

void HCTL2032_Reset_Counter(PORT_t * cPort);

void HCTL2032_Get_Direction(PORT_t * sPort, bool axis);                     

#pragma used-

#pragma library hctl2032_driver.lib

#endif /* HCTL2032_DRIVER_H */