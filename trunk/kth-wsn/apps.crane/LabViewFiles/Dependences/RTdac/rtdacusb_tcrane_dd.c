/*
 * RTDACUSB_TCRANE_DD.C
 *
 *   S-Function for the RT-DAC/USB device
 *   All I/O functions as a single device block
 *
 *  Copyright (c) 2004 by 2K/InTeCo
 *  All Rights Reserved
 */

 ////////////////////////////////////
 //
 // Inputs:               / size / comment
 //      PWMPrescaler     /    1 / 0..63
 //      PWMWidth         /    3 / -1..+1 [Z Y X]
 //      EncoderReset     /    5 / [YAngle XAngle Z Y X]
 //      EncoderAutoReset /    3 / [Z Y X]
 //      RailLimit        /    3 / [Z Y X] [bit*64]
 //      RailLimitFlag    /    3 / [Z Y X]
 //      ThermFlag        /    3 / [Z Y X]
 //      TmrCntReset      /    1
 //
 // Outputs:              / size / comment
 //      Status           /    2 / [LogicVersion ErrorCode]
 //      PWMPrescaler     /    1 / 0..63
 //      PWMWidth         /    3 / -1..+1 [Z Y X]
 //      EncoderCounter   /    5 / [YAngle XAngle Z Y X] [bit]
 //      EncoderReset     /    5 / [YAngle XAngle Z Y X]
 //      EncoderAutoReset /    3 / [Z Y X]
 //      RailLimit        /    3 / [Z Y X] [bit*64]
 //      RailLimitFlag    /    3 / [Z Y X]
 //      LimitSwitch      /    3 / [Z Y X]
 //      ThermFlag        /    3 / [Z Y X]
 //      ThermStatus      /    3 / [Z Y X]
 //      TmrCntCounter    /    1
 //      TmrCntReset      /    1

#define S_FUNCTION_NAME rtdacusb_tcrane_dd
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <conio.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>

#include "windows.h"

#ifdef __WATCOMC__
  #include "ftd2xx.h"
  #include "Watcom2FTD2XX.h"
  #include "Watcom2FTD2XX.c"
  int LoadDLLStatus;
#endif  /* __WATCOMC__ */
#include "rtdacusb_tcrane.c"

// Configuration of the RT-DAC/USB device
#ifndef MATLAB_MEX_FILE
  static RTDACUSBBufferType RTDACUSBBuffer; 
#endif


/* Input Arguments */
#define SAMPLE_TIME_ARG  (ssGetSFcnParam(S,0))
#define NUMBER_OF_ARGS   (1)
#define NSAMPLE_TIMES    (1)

#define SAMPLE_TIME      ((real_T) mxGetPr(SAMPLE_TIME_ARG)[0])


int NoOfDetectedUSBDevices;

int RetVal;

static void mdlInitializeSizes(SimStruct *S) {
   #ifdef MATLAB_MEX_FILE
    // mexEvalString( "startio;" );
   #endif
    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 8)) return;
    ssSetInputPortWidth(S,  0, 1);               // PWMPrescaler     /    1 / 0..63
    ssSetInputPortDirectFeedThrough(S,  0, 0);
    ssSetInputPortWidth(S,  1, 3);               // PWMWidth         /    3 / -1..+1 [Z Y X]
    ssSetInputPortDirectFeedThrough(S,  1, 0);
    ssSetInputPortWidth(S,  2, 5);               // EncoderReset     /    5 / [YAngle XAngle Z Y X]
    ssSetInputPortDirectFeedThrough(S,  2, 0);
    ssSetInputPortWidth(S,  3, 3);               // EncoderAutoReset /    3 / [Z Y X]
    ssSetInputPortDirectFeedThrough(S,  3, 0);
    ssSetInputPortWidth(S,  4, 3);               // RailLimit        /    3 / [Z Y X] [bit*64]
    ssSetInputPortDirectFeedThrough(S,  4, 0);
    ssSetInputPortWidth(S,  5, 3);               // RailLimitFlag    /    3 / [Z Y X]
    ssSetInputPortDirectFeedThrough(S,  5, 0);
    ssSetInputPortWidth(S,  6, 3);               // ThermFlag        /    3 / [Z Y X]
    ssSetInputPortDirectFeedThrough(S,  6, 0);
    ssSetInputPortWidth(S,  7, 1);               // TmrCntReset      /    1
    ssSetInputPortDirectFeedThrough(S,  7, 0);

    if (!ssSetNumOutputPorts(S, 13)) return;
    ssSetOutputPortWidth(S,  0,  2 );            // Status           /    1 / [LogicVersion ErrorCode]
    ssSetOutputPortWidth(S,  1,  1 );            // PWMPrescaler     /    1 / 0..63
    ssSetOutputPortWidth(S,  2,  3 );            // PWMWidth         /    3 / -1..+1 [Z Y X]
    ssSetOutputPortWidth(S,  3,  5 );            // EncoderCounter   /    5 / [YAngle XAngle Z Y X] [bit]
    ssSetOutputPortWidth(S,  4,  5 );            // EncoderReset     /    3 / [YAngle XAngle Z Y X]
    ssSetOutputPortWidth(S,  5,  3 );            // EncoderAutoReset /    3 / [Z Y X]
    ssSetOutputPortWidth(S,  6,  3 );            // RailLimit        /    3 / [Z Y X] [bit*64]
    ssSetOutputPortWidth(S,  7,  3 );            // RailLimitFlag    /    3 / [Z Y X]
    ssSetOutputPortWidth(S,  8,  3 );            // LimitSwitch      /    3 / [Z Y X]
    ssSetOutputPortWidth(S,  9,  3 );            // ThermStatus      /    3 / [Z Y X]
    ssSetOutputPortWidth(S, 10,  3 );            // ThermFlag        /    3 / [Z Y X]
    ssSetOutputPortWidth(S, 11,  1 );            // TmrCntCounter    /    1
    ssSetOutputPortWidth(S, 12,  1 );            // TmrCntReset      /    1

    ssSetNumSampleTimes(S, NSAMPLE_TIMES );

    /* Take care when specifying exception free code */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}
 
 
/* Function to initialize sample times */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0);
    
#ifndef MATLAB_MEX_FILE
#ifdef __WATCOMC__
    LoadDLLStatus = LoadFTD2XXLibrary( );
    if(LoadDLLStatus != 0 ) return;
#endif  /* __WATCOMC__ */
    NoOfDetectedUSBDevices = USBOpen( );
    CommandRead_TCrane( &RTDACUSBBuffer );
#endif
}
 
/* Function to compute outputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE
    int i;
    double AuxPWM;
    
    InputRealPtrsType uPtrsPWMPrescaler     = ssGetInputPortRealSignalPtrs(S,  0);
    InputRealPtrsType uPtrsPWMWidth         = ssGetInputPortRealSignalPtrs(S,  1);
    InputRealPtrsType uPtrsEncoderReset     = ssGetInputPortRealSignalPtrs(S,  2);
    InputRealPtrsType uPtrsEncoderAutoReset = ssGetInputPortRealSignalPtrs(S,  3);
    InputRealPtrsType uPtrsRailLimit        = ssGetInputPortRealSignalPtrs(S,  4);
    InputRealPtrsType uPtrsRailLimitFlag    = ssGetInputPortRealSignalPtrs(S,  5);
    InputRealPtrsType uPtrsThermFlag        = ssGetInputPortRealSignalPtrs(S,  6);
    InputRealPtrsType uPtrsTmrCntReset      = ssGetInputPortRealSignalPtrs(S,  7);
    
    real_T *yStatus           = ssGetOutputPortRealSignal(S,  0);
    real_T *yPWPrescaler      = ssGetOutputPortRealSignal(S,  1);
    real_T *yPWMWidth         = ssGetOutputPortRealSignal(S,  2);
    real_T *yEncoderCounter   = ssGetOutputPortRealSignal(S,  3);
    real_T *yEncoderReset     = ssGetOutputPortRealSignal(S,  4);
    real_T *EncoderAutoReset  = ssGetOutputPortRealSignal(S,  5);
    real_T *RailLimit         = ssGetOutputPortRealSignal(S,  6);
    real_T *RailLimitFlag     = ssGetOutputPortRealSignal(S,  7);
    real_T *LimitSwitch       = ssGetOutputPortRealSignal(S,  8);
    real_T *ThermStatus       = ssGetOutputPortRealSignal(S,  9);
    real_T *ThermFlag         = ssGetOutputPortRealSignal(S, 10);
    real_T *yTmrCntCounter    = ssGetOutputPortRealSignal(S, 11);
    real_T *yTmrCntReset      = ssGetOutputPortRealSignal(S, 12);
   
    // Inpur signals
    RTDACUSBBuffer.PWMDivider = (int)(*uPtrsPWMPrescaler[0]);
    if( RTDACUSBBuffer.PWMDivider > 63 ) RTDACUSBBuffer.PWMDivider = 63;
    if( RTDACUSBBuffer.PWMDivider <  0 ) RTDACUSBBuffer.PWMDivider = 0;
    for(i=0;i<3;i++) {
      AuxPWM = (*uPtrsPWMWidth[i]); 
      if( AuxPWM > +1.0 ) AuxPWM =  1.0;
      if( AuxPWM < -1.0 ) AuxPWM = -1.0;
      RTDACUSBBuffer.PWMWidth[i] = (int)(1023*fabs(AuxPWM));
      if( AuxPWM == 0 ) RTDACUSBBuffer.PWMBrake[i] = 1; else RTDACUSBBuffer.PWMBrake[i] = 0; 
      if( AuxPWM <  0 ) RTDACUSBBuffer.PWMDir[i] = 1;   else RTDACUSBBuffer.PWMDir[i]   = 0; 
    }
    for(i=0;i<5;i++) 
      RTDACUSBBuffer.EncoderReset[i] = (int)(*uPtrsEncoderReset[i]); 
    for(i=0;i<3;i++) {
      RTDACUSBBuffer.PWMThermFlag[i] = (int)(*uPtrsThermFlag[i]); 
      RTDACUSBBuffer.Limit[i]        = (int)(*uPtrsRailLimit[i]); 
      RTDACUSBBuffer.LimitFlag[i]    = (int)(*uPtrsRailLimitFlag[i]); 
    }
    
    RTDACUSBBuffer.TmrCnt[0].Reset = (int)(*uPtrsTmrCntReset[0]); 
   
#ifdef __WATCOMC__
    if(LoadDLLStatus == 0 ) {
#endif  
    CommandSend_TCrane( &RTDACUSBBuffer );
    RetVal = CommandRead_TCrane( &RTDACUSBBuffer );
//    RetVal = CommandSendAndRead_TCrane( &RTDACUSBBuffer );
#ifdef __WATCOMC__
    }
#endif  

    // Output Signals
    yStatus[0]       = RTDACUSBBuffer.LogicVersion;
    yStatus[1]       = RetVal;
    yPWPrescaler[0] = RTDACUSBBuffer.PWMDivider;
    for(i=0;i<3;i++) {
      yPWMWidth[i] = RTDACUSBBuffer.PWMWidth[i]/1023.0;
      if( RTDACUSBBuffer.PWMDir[i] == 1 ) yPWMWidth[i] = -yPWMWidth[i];
    }    
    for(i=0;i<5;i++) {
      yEncoderReset[i]     = RTDACUSBBuffer.EncoderReset[i]; 
      yEncoderCounter[i]   = RTDACUSBBuffer.Encoder[i];
    }
    for(i=0;i<3;i++) {
      EncoderAutoReset[i] = RTDACUSBBuffer.EncoderAutoResetFlag[i]; 
      RailLimit[i]        = RTDACUSBBuffer.Limit[i];
      RailLimitFlag[i]    = RTDACUSBBuffer.LimitFlag[i];
      LimitSwitch[i]      = RTDACUSBBuffer.LimitActive[i];
      ThermFlag[i]        = RTDACUSBBuffer.PWMThermFlag[i];
      ThermStatus[i]      = RTDACUSBBuffer.PWMTherm[i];
    }
    yTmrCntReset[0]   = RTDACUSBBuffer.TmrCnt[0].Reset; 
    yTmrCntCounter[0] = RTDACUSBBuffer.TmrCnt[0].Counter; 
    
#endif    
}



/* Function to perform cleanup at execution termination */
static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
#ifdef __WATCOMC__
   if(LoadDLLStatus == 0 ) 
#endif

    CommandRead_TCrane( &RTDACUSBBuffer );
    RTDACUSBBuffer.PWMWidth[0] = RTDACUSBBuffer.PWMWidth[1] = RTDACUSBBuffer.PWMWidth[2] = 0;
    CommandSend_TCrane( &RTDACUSBBuffer );

   USBClose( );
#ifdef __WATCOMC__
   if(LoadDLLStatus == 0 ) 
     FreeFTD2XXLibrary( );
#endif  /* __WATCOMC__ */
#endif
}

#ifdef MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file? */
#include "simulink.c"        /* MEX-File interface mechanism */
#else
#include "cg_sfun.h"    /* Code generation registration function */
#endif