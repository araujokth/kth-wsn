/*

sfun_fetch.c

Based on The MathWorks, Inc's sfuntmpl_basic.c

João Felipe Faria
2010-08-03

*/



/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  sfun_fetch
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include <stdio.h>
#include <stdlib.h>
#include "simstruc.h"

#include "sfsource.h"

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    /*ssSetInputPortWidth(S, 0, 1);*/
    /*ssSetInputPortRequiredContiguous(S, 0, true); /direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    /*ssSetInputPortDirectFeedThrough(S, 0, 1);*/

    if (!ssSetNumOutputPorts(S, 21)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortWidth(S, 4, 1);
    ssSetOutputPortWidth(S, 5, 1);
    ssSetOutputPortWidth(S, 6, 1);
    ssSetOutputPortWidth(S, 7, 1);
    ssSetOutputPortWidth(S, 8, 1);
    ssSetOutputPortWidth(S, 9, 1);
    ssSetOutputPortWidth(S, 10, 1);
    ssSetOutputPortWidth(S, 11, 1);
    ssSetOutputPortWidth(S, 12, 1);
    ssSetOutputPortWidth(S, 13, 1);
    ssSetOutputPortWidth(S, 14, 1);
    ssSetOutputPortWidth(S, 15, 1);
    ssSetOutputPortWidth(S, 16, 1);
    ssSetOutputPortWidth(S, 17, 1);
    ssSetOutputPortWidth(S, 18, 1);
    ssSetOutputPortWidth(S, 19, 1);
    ssSetOutputPortWidth(S, 20, 1);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#undef MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    char host[] = "localhost";
	int port = 9002;
    int len, i, fd, payloadLen;
    
    real_T       *Y11 = ssGetOutputPortSignal(S,0);
    real_T       *Y12 = ssGetOutputPortSignal(S,1);
    real_T       *Y21 = ssGetOutputPortSignal(S,2);
    real_T       *Y22 = ssGetOutputPortSignal(S,3);
    real_T       *S1 = ssGetOutputPortSignal(S,4);
    real_T       *S2 = ssGetOutputPortSignal(S,5);
    real_T       *S3 = ssGetOutputPortSignal(S,6);
    real_T       *U1 = ssGetOutputPortSignal(S,7);
    real_T       *U2 = ssGetOutputPortSignal(S,8);
    real_T       *TIME = ssGetOutputPortSignal(S,9);
    real_T       *ERR = ssGetOutputPortSignal(S,10);
    real_T       *I1 = ssGetOutputPortSignal(S,11);
    real_T       *I2 = ssGetOutputPortSignal(S,12);
    real_T       *y11_init = ssGetOutputPortSignal(S,13);
    real_T       *y12_init = ssGetOutputPortSignal(S,14);
    real_T       *y21_init = ssGetOutputPortSignal(S,15);
    real_T       *y22_init = ssGetOutputPortSignal(S,16);
    real_T       *u1_init = ssGetOutputPortSignal(S,17);
    real_T       *u2_init = ssGetOutputPortSignal(S,18);
    real_T       *i1_init = ssGetOutputPortSignal(S,19);
    real_T       *i2_init = ssGetOutputPortSignal(S,20);
    
    fd = open_sf_source(host, port);
    const unsigned char *packet = read_sf_packet(fd, &len);
    float i1, i2;
    float *p1, *p2;
    
    payloadLen = (int) packet[5];
    
    if ( payloadLen == 50 ) {
    
        Y11[0] = (((int) packet[8])<<8) + (int) packet[9];
        Y12[0] = (((int) packet[10])<<8) + (int) packet[11];
        Y21[0] = (((int) packet[12])<<8) + (int) packet[13];
        Y22[0] = (((int) packet[14])<<8) + (int) packet[15];
        S1[0] = (((int) packet[16])<<8) + (int) packet[17];
        S2[0] = (((int) packet[18])<<8) + (int) packet[19];
        S3[0] = (((int) packet[20])<<8) + (int) packet[21];
        U1[0] = (((int) packet[22])<<8) + (int) packet[23];
        U2[0] = (((int) packet[24])<<8) + (int) packet[25];
        TIME[0] = (((long) packet[26]) << 24) + (((long) packet[27]) << 16) + (((long) packet[28]) << 8) + (long) packet[29];
        /*I1[0] = (((int) packet[30])<<8) + (int) packet[31];
        I2[0] = (((int) packet[32])<<8) + (int) packet[33];*/
        p1 = (float *) &packet[30];
        I1[0] = (real_T) *p1; 
        p2 = (float *) &packet[34];
        I2[0] = (real_T) *p2;
        ERR[0] = 1;
        y11_init[0] = (((int) packet[38])<<8) + (int) packet[39];
        y12_init[0] = (((int) packet[40])<<8) + (int) packet[41];
        y21_init[0] = (((int) packet[42])<<8) + (int) packet[43];
        y22_init[0] = (((int) packet[44])<<8) + (int) packet[45];
        u1_init[0] = (((int) packet[46])<<8) + (int) packet[47];
        u2_init[0] = (((int) packet[48])<<8) + (int) packet[49];
        p1 = (float *) &packet[50];
        i1_init[0] = (real_T) *p1; 
        p2 = (float *) &packet[54];
        i2_init[0] = (real_T) *p2;
        
        
    } else {
        
        Y11[0] = 0;
        Y12[0] = 0;
        Y21[0] = 0;
        Y22[0] = 0;
        S1[0] = 0;
        S2[0] = 0;
        S3[0] = 0;
        U1[0] = 0;
        U2[0] = 0;
        I1[0] = 0;
        I2[0] = 0;
        TIME[0] = 0;
        ERR[0] = 0;
        
    }
    
    close(fd);
}



#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
