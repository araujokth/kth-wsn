/*

sfun_send.c

Based on The MathWorks, Inc's sfuntmpl_basic.c

João Felipe Faria
2010-09-09

*/



/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  sfun_send
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

    if (!ssSetNumInputPorts(S, 10)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortRequiredContiguous(S, 2, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 3, 1);
    ssSetInputPortRequiredContiguous(S, 3, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 4, 1);
    ssSetInputPortRequiredContiguous(S, 4, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 5, 1);
    ssSetInputPortRequiredContiguous(S, 5, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 6, 1);
    ssSetInputPortRequiredContiguous(S, 6, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 7, 1);
    ssSetInputPortRequiredContiguous(S, 7, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 8, 1);
    ssSetInputPortRequiredContiguous(S, 8, true); /*direct input signal access*/
    ssSetInputPortWidth(S, 9, 1);
    ssSetInputPortRequiredContiguous(S, 9, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);

    if (!ssSetNumOutputPorts(S, 0)) return;
    /*ssSetOutputPortWidth(S, 0, 1);*/

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
    int len, i, fd;
    char packet[100];
    char input_value[8];
    
    const real_T *BI = (const real_T *) ssGetInputPortSignal(S,0);
    const real_T *T1 = (const real_T *) ssGetInputPortSignal(S,1);
    const real_T *T2 = (const real_T *) ssGetInputPortSignal(S,2);
    const real_T *T3 = (const real_T *) ssGetInputPortSignal(S,3);
    const real_T *T4 = (const real_T *) ssGetInputPortSignal(S,4);
    const real_T *T5 = (const real_T *) ssGetInputPortSignal(S,5);
    const real_T *T6 = (const real_T *) ssGetInputPortSignal(S,6);
    const real_T *T7 = (const real_T *) ssGetInputPortSignal(S,7);
    const real_T *INIT = (const real_T *) ssGetInputPortSignal(S,8);
    const real_T *OK = (const real_T *) ssGetInputPortSignal(S,9);
    
    packet[0] = 0x00;
    packet[1] = 0xff;
    packet[2] = 0xff;
    packet[3] = 0xff;
    packet[4] = 0xff;
    packet[5] = (char) sizeof(char)*8; /* payload length */
    packet[6] = 0x00;
    packet[7] = 0x06;
    
    if( INIT[0] == 0 ) {
        
        input_value[0] = (char) BI[0];
        input_value[1] = (char) T1[0];
        input_value[2] = (char) T2[0];
        input_value[3] = (char) T3[0];
        input_value[4] = (char) T4[0];
        input_value[5] = (char) T5[0];
        input_value[6] = (char) T6[0];
        input_value[7] = (char) T7[0];
        
    } else {
        
        input_value[0] = (char) 0xff;
        input_value[1] = (char) 0xff;
        input_value[2] = (char) 0xff;
        input_value[3] = (char) 0xff;
        input_value[4] = (char) 0xff;
        input_value[5] = (char) 0xff;
        input_value[6] = (char) 0xff;
        input_value[7] = (char) 0xff;
        
    }
    
    memcpy(packet + 8, input_value, packet[5]);
    
    if(OK[0]==1) {
        fd = open_sf_source(host, port);
        write_sf_packet(fd, (void *) packet, 16);
        close(fd);
    }
    
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
