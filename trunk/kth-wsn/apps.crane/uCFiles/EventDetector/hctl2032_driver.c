/* This file has been prepared for Doxygen automatic documentation generation. */
/*! \file *********************************************************************
 *
 * \brief  The XMEGA HCTL2032 Quadrature Decoder driver source file.
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
 
 #include "hctl2032_driver.h"
 
 /*! \brief Wrapperfunction to set up all parameters for the quadrature decoder.
 *
 *  This function combines the following functions for a total setup:
 *  QDEC_Port_Setup, QDEC_EVSYS_Setup and QDEC_TC_Dec_Setup.
 *
 * \param dPort         The port to use for 8 bit data.
 * \param cPort         The port to use for controlling HCTL2032.
 
 * \param sPort         The Port which would read the signals from HCTL2032. These signals are kind of reporting signals
                        which tells about internal state of HCTL2032 like overflow/underflow, count updated etc 
 * \param CountMode     Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x.
 * \return bool         True if setup was ok, false if any errors.
*/

bool HCTL2032_Total_Setup(PORT_t * dPort,
                      PORT_t * cPort,
                      PORT_t * sPort,
                      unsigned char CountMode)
{
	/* Set the Data port as input */
    dPort->DIRCLR = 0xFF;
    //PORTCFG.MPCMASK = 0xFF;
    //dPort->PIN0CTRL = 0x18;//0x08; // Totem-pole inputs with pull-ups. 0x08 => Totem pole with Bus-keeper
    /* Set the Control Port as output*/
    cPort->DIRSET = 0xFF;
    
    /* Set the Signal Port as input (PIN2 to PIN7)*/
    sPort->DIRCLR = 0xFE;

    
    /* Set the Signal Port PIN0 as output (PIN0). This will be used for controlling EN pin of MAX3002*/
    sPort->DIRSET = EN_max3002;
    /*Enable the voltage level translator (MAX3002)*/
    sPort->OUTSET = EN_max3002;
    
//    /* Set the PIN4, PIN5 of status port for external interrupt on RISING edge
//    sPort->PIN0CTRL & ~PORT_ISC_gm clears the old pin configuration before setting new one using PORT_ISC_RISING_gc*/
//    PORTCFG.MPCMASK = (0x03<<CNTCASx_bp);        
//    sPort->PIN4CTRL = (sPort->PIN4CTRL & ~PORT_ISC_gm) | PORT_ISC_RISING_gc;
//    /* Mask Interrupt 0 for CNTCASx pin*/
//    sPort->INT0MASK |= CNTCASx;
//    /* Enable the interrupt 0 and mark it as low level*/
//    sPort->INTCTRL = (sPort->INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_HI_gc;
//    /* Mask Interrupt 1 for CNTCASy pin*/
//    sPort->INT1MASK |= CNTCASy;
//    /* Enable the interrupt 1 and mark it as low level*/
//    sPort->INTCTRL = (sPort->INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_HI_gc | PORT_INT0LVL_OFF_gc;
    
    /*********** Manual Setting for Interrupt for CNTCAS signals, for testing purpose ************/
    
//    PORTJ.DIRCLR = 0xFE;
//    PORTK.DIRCLR = 0xFE;    
//        /* Set the PIN4, PIN5 of status port for external interrupt on RISING edge
//    sPort->PIN0CTRL & ~PORT_ISC_gm clears the old pin configuration before setting new one using PORT_ISC_RISING_gc*/
//    //PORTCFG.MPCMASK = (0x03<<CNTCASx);        
//    PORTJ.PIN2CTRL = (PORTJ.PIN2CTRL & (~(PORT_ISC_gm | PORT_OPC_gm))) | PORT_ISC_RISING_gc | PORT_OPC_PULLDOWN_gc;
//    //PORTJ.PIN2CTRL = (PORTJ.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc | PORT_INVEN_bm;
//    /* Mask Interrupt 0 for CNTCASx pin*/
//    PORTJ.INT0MASK |= CNTCASx;
//    /* Enable the interrupt 0 and mark it as high level*/
//    PORTJ.INTCTRL = (PORTJ.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_HI_gc;
//    
//    // pin configuration
//    PORTK.PIN2CTRL = (PORTK.PIN2CTRL & (~(PORT_ISC_gm | PORT_OPC_gm))) | PORT_ISC_RISING_gc | PORT_OPC_PULLDOWN_gc;
//    //PORTK.PIN2CTRL = (PORTK.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc | PORT_INVEN_bm;
//    /* Mask Interrupt 0 for CNTCASy pin*/
//    PORTK.INT0MASK |= CNTCASy;
//    /* Enable the interrupt 0 and mark it as high level*/
//    PORTK.INTCTRL = (PORTK.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_HI_gc;  
           
    /* Reset the internal registers of HCTL2032 for both axis*/
    cPort->OUTCLR = (RSTNx | RSTNy);
    #asm
    nop
    #endasm
    cPort->OUTSET = (RSTNx | RSTNy);
        
    /* Disable the output buffer of HCTL2032 and keep it in high state*/
    cPort->OUTSET = OEN;
    
    /* Select the x-axis initially*/
    cPort->OUTCLR = XNY;
    
    if( !HCTL2032_Set_Count_Mode(cPort,CountMode) )
        return false;
    
   	return true;
}

/*! \brief This function setup the HCTL2032 decoder for proper count mode.
 *  \param CountMode Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x
*/

bool HCTL2032_Set_Count_Mode(PORT_t * cPort,unsigned char CountMode)
{
    switch (CountMode)
    {
    /*Illegal Count Mode*/
    case 1:
    /*4x Count Mode*/
    cPort->OUTSET = EN1;
    cPort->OUTCLR = EN2;
    break;
    case 2:
    /*2x Count Mode*/
    cPort->OUTCLR = EN1;
    cPort->OUTSET = EN2;
    break;
    case 3:
    /*1x Count Mode*/
    cPort->OUTSET = EN1;
    cPort->OUTSET = EN2;
    break;
    default:
    return false;
    }
    
    return true;
}

/*! \brief This function read the current position count available in position latch. The data will be read through 
 *  8-bit data port MSB (Most Significant Byte) first. 
 *  \param dPort     The port to use with data port of HCTL2032.
 *  \param cPort     The port to use for control signals of HCTL2032.
 *  \param axis      Which axis data to be read. 0 = x-axis, 1 = y-axis
*/

unsigned long int HCTL2032_Read_Count_Data(PORT_t * dPort, PORT_t * cPort, bool axis)
{
unsigned long int result = 0;
unsigned char MSB = 0;
unsigned char MSB_2nd = 0;
unsigned char MSB_3rd = 0;
unsigned char LSB = 0;
unsigned char MSB_old = 0;
unsigned char MSB_2nd_old = 0;
unsigned char MSB_3rd_old = 0;
unsigned char LSB_old = 0;
unsigned char MSB_new = 0;
unsigned char MSB_2nd_new = 0;
unsigned char MSB_3rd_new = 0;
unsigned char LSB_new = 0;

/* Give little bit delay before changing axis*/
#asm
nop
#endasm

/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
#endasm

if(axis)                // select y-axis
cPort->OUTSET = XNY;
else
cPort->OUTCLR = XNY;    // select x-axis

dPort->DIRCLR = 0xFF;

/* MSB-MSB_2nd-MSB_3rd-LSB*/

/* Read the MSB*/

cPort->OUTCLR = SEL1;
cPort->OUTSET = SEL2;
cPort->OUTCLR = OEN;
/*Give delay just to ensure the setup timmings*/
#asm
nop
nop
#endasm

/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm
MSB = dPort->IN;
MSB = dPort->IN; // Read twice for ensuring data integrity
//Get_MSB:
//MSB_old = dPort->IN;
//MSB_new = dPort->IN;
//if(MSB_old == MSB_new)
//    MSB = MSB_new;
//else
//    goto Get_MSB;
    

/*wait one clock cycle before going for next data byte*/
#asm
nop
#endasm

/*Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm

/* Read the 2nd MSB*/
cPort->OUTSET = SEL1;
cPort->OUTSET = SEL2;
/*Give delay just to ensure the setup timmings*/
#asm
nop
nop
#endasm
/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm
MSB_2nd = dPort->IN;
MSB_2nd = dPort->IN; // Read twice for ensuring data integrity
//Get_MSB_2nd: 
//MSB_2nd_old = dPort->IN;
//MSB_2nd_new = dPort->IN;
//if(MSB_2nd_old == MSB_2nd_new)
//    MSB_2nd = MSB_2nd_new;
//else
//    goto Get_MSB_2nd;
#asm
nop
#endasm

/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm

/* Read the 3rd MSB*/
cPort->OUTCLR = SEL1;
cPort->OUTCLR = SEL2;
/*Give delay just to ensure the setup timmings*/
#asm
nop
nop
#endasm
/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm
MSB_3rd = dPort->IN;
MSB_3rd = dPort->IN; // Read twice for ensuring data integrity
//Get_MSB_3rd: 
//MSB_3rd_old = dPort->IN;
//MSB_3rd_new = dPort->IN;
//if(MSB_3rd_old == MSB_3rd_new)
//    MSB_3rd = MSB_3rd_new;
//else
//    goto Get_MSB_3rd;
#asm
nop
#endasm

/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm

/* Read the LSB*/
cPort->OUTSET = SEL1;
cPort->OUTCLR = SEL2;
/*Give delay just to ensure the setup timmings*/
#asm
nop
nop
#endasm
/* Extra Delay: Comment this when the driver is used for high speed board. this bloc of delay is added 
to meet timming and drive requirements for MAX3002 when driving from ATSTK600 board. The
is due to long signal lines*/
#asm
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
#endasm
LSB = dPort->IN;
LSB = dPort->IN; // Read twice for ensuring data integrity
//Get_LSB: 
//LSB_old = dPort->IN;
//LSB_new = dPort->IN;
//if(LSB_old == LSB_new)
//    LSB = LSB_new;
//else
//    goto Get_LSB;
#asm
nop
#endasm

/* Disble the data bus. Bring it to High Z state*/
cPort->OUTSET = OEN;

//MSB = dPort->IN;
//MSB_2nd = dPort->IN;
//MSB_3rd = dPort->IN;
//LSB = dPort->IN;
/*Form 32-bit data frame*/
result = ((unsigned long int)MSB)<<24;
result |= ((unsigned long int)MSB_2nd)<<16;
result |= ((unsigned long int)MSB_3rd)<<8;
result |= (unsigned long int)LSB;

return result;

}

/*! \brief This function reset the internal registers of HCTL2032. This function will be primarily used 
 *   for homing of the system.
 * \param cPort     The port to use for control signals of HCTL2032.
*/

void HCTL2032_Reset_Counter(PORT_t * cPort)
{
    /* Reset the internal registers of HCTL2032 for both axis*/
    cPort->OUTCLR = (RSTNx | RSTNy);
    #asm
    nop
    #endasm
    cPort->OUTSET = (RSTNx | RSTNy);
    
    
    /* Disable the output buffer of HCTL2032 and keep it in high state*/
    cPort->OUTSET = OEN;
    
    /* Select the x-axis initially*/
    cPort->OUTCLR = XNY;
}

/*! \brief This function enable the external interrupt on the pins connected to CNTDEC (x or y) signals whenever requested/required by user. 
    We dont want to be interrupted all the time so we would enable external interrupts only when required. In the main source file,
    the ISRs INT0 for hardware pin interrupts must be defined.
    
 * \param sPort     The port with which monitoring signals from HCTL2032 are attached.
 * \param axis      Which axis to be tested for direction. 0 = x-axis, 1 = y-axis
*/

void HCTL2032_Get_Direction(PORT_t * sPort, bool axis)
{
 unsigned char s;
 
 /* Store the status register*/
 s = SREG;
 
 /*Disable global interrupts*/
 #asm("cli")
 
  /* Set PIN2 and PIN3 as input*/
 sPort->DIRCLR = UDy | UDx;
 
 /* Enable interrupts on rising edge of CNTDEC signal*/
 
 if(axis) // y-axis
 { 
 PORTCFG.MPCMASK = (0x01<<CNTDECy);
 sPort->PIN6CTRL = (sPort->PIN7CTRL & ~PORT_ISC_gm) | PORT_ISC_RISING_gc;
 /* Mask Interrupt 0 for CNTDECy pin*/
 sPort->INT0MASK |= CNTDECy;
 /* Enable the interrupt 0 and mark it as low level*/
 sPort->INTCTRL = (sPort->INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_LO_gc;  
 }
 else   // x-axis
 { 
 PORTCFG.MPCMASK = (0x01<<CNTDECx);
 sPort->PIN7CTRL = (sPort->PIN6CTRL & ~PORT_ISC_gm) | PORT_ISC_RISING_gc;
 /* Mask Interrupt 0 for CNTDECx pin*/
 sPort->INT0MASK |= CNTDECx;
 /* Enable the interrupt 0 and mark it as low level*/
 sPort->INTCTRL = (sPort->INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_LO_gc;
 }

 SREG = s;
}