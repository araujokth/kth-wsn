/*
 * This file is not available for use in any application other than as a
 * MATLAB(R) MEX file for use with the Simulink(R) product.
 */

/*
 * sensor_block_acc.c
 *
 * Real-Time Workshop code generation for Simulink model "sensor_block_acc.mdl".
 *
 * Model Version              : 1.591
 * Real-Time Workshop version : 7.4  (R2009b)  29-Jun-2009
 * C source code generated on : Fri Feb  4 15:08:01 2011
 *
 * Target selection: accel.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#include <math.h>
#include "sensor_block_acc.h"
#include "sensor_block_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat                     S-Function
#define AccDefine1                     Accelerator_S-Function

/* Outputs for root system: '<Root>' */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  {
    real_T B_7_0_0;
    if (ssIsSampleHit(S, 1, tid)) {
      B_7_0_0 = ((real_T)((D_Work *) ssGetRootDWork(S))->clockTickCounter <
                 ((Parameters *) ssGetDefaultParam(S))->P_2) && (((D_Work *)
        ssGetRootDWork(S))->clockTickCounter >= 0) ? ((Parameters *)
        ssGetDefaultParam(S))->P_0 : 0.0;
      if ((real_T)((D_Work *) ssGetRootDWork(S))->clockTickCounter >=
          ((Parameters *) ssGetDefaultParam(S))->P_1 - 1.0) {
        ((D_Work *) ssGetRootDWork(S))->clockTickCounter = 0;
      } else {
        ((D_Work *) ssGetRootDWork(S))->clockTickCounter = ((D_Work *)
          ssGetRootDWork(S))->clockTickCounter + 1;
      }

      if (rt_ZCFcn(RISING_ZERO_CROSSING,&((PrevZCSigStates *)
            _ssGetPrevZCSigState(S))->TriggeredSubsystem_Trig_ZCE,
                   (B_7_0_0)) != NO_ZCEVENT) {
        ((BlockIO *) _ssGetBlockIO(S))->B_5_0_0 = ((Parameters *)
          ssGetDefaultParam(S))->P_4;

        /* M-S-Function: '<S5>/Level-2 M-file S-Function' */
        /* Call into Simulink */
        ssCallAccelRunBlock(S, 5, 1, SS_CALL_MDL_OUTPUTS);

        /* Level2 S-Function Block: '<S7>/B_4_0' (stateflow) */
        /* Call into Simulink for MEX-version of S-function */
        ssCallAccelRunBlock(S, 4, 0, SS_CALL_MDL_OUTPUTS);

        /* Update for M-S-Function: '<S5>/Level-2 M-file S-Function' */

        /* Call into Simulink */
        ssCallAccelRunBlock(S, 5, 1, SS_CALL_MDL_UPDATE);
        ((D_Work *) ssGetRootDWork(S))->TriggeredSubsystem_SubsysRanBC = 4;
      }

      /* Level2 S-Function Block: '<S4>/B_3_0' (stateflow) */
      /* Call into Simulink for MEX-version of S-function */
      ssCallAccelRunBlock(S, 3, 0, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/actuator_delay' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 3, SS_CALL_MDL_OUTPUTS);

      /* Level2 S-Function Block: '<S1>/B_0_0' (stateflow) */
      /* Call into Simulink for MEX-version of S-function */
      ssCallAccelRunBlock(S, 0, 0, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/actuator_errors' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 5, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/actuator_reliability' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 6, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/actuator_value' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 7, SS_CALL_MDL_OUTPUTS);

      /* Level2 S-Function Block: '<S3>/B_2_0' (stateflow) */
      /* Call into Simulink for MEX-version of S-function */
      ssCallAccelRunBlock(S, 2, 0, SS_CALL_MDL_OUTPUTS);

      /* Level2 S-Function Block: '<S2>/B_1_0' (stateflow) */
      /* Call into Simulink for MEX-version of S-function */
      ssCallAccelRunBlock(S, 1, 0, SS_CALL_MDL_OUTPUTS);
      ((BlockIO *) _ssGetBlockIO(S))->B_6_0_0[0] = ((BlockIO *) _ssGetBlockIO(S))
        ->B_3_0_1;
      ((BlockIO *) _ssGetBlockIO(S))->B_6_0_0[1] = ((BlockIO *) _ssGetBlockIO(S))
        ->B_3_0_4;
      ((BlockIO *) _ssGetBlockIO(S))->B_6_0_0[2] = ((BlockIO *) _ssGetBlockIO(S))
        ->B_3_0_7;

      /* Level2 S-Function Block: '<S6>/B_6_0' (stateflow) */
      /* Call into Simulink for MEX-version of S-function */
      ssCallAccelRunBlock(S, 6, 1, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/delay' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 11, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/errors' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 12, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/reliability' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 13, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/theta_delay' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 14, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/theta_errors' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 15, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/theta_reliability' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 16, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/theta_value' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 17, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/xc_delay' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 18, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/xc_errors' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 19, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/xc_reliability' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 20, SS_CALL_MDL_OUTPUTS);

      /* Scope: '<Root>/xc_value' */

      /* Call into Simulink for Scope */
      ssCallAccelRunBlock(S, 7, 21, SS_CALL_MDL_OUTPUTS);
    }
  }
}

/* Update for root system: '<Root>' */
#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid)
{
  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Function to initialize sizes */
static void mdlInitializeSizes(SimStruct *S)
{
  /* checksum */
  ssSetChecksumVal(S, 0, 2478266878U);
  ssSetChecksumVal(S, 1, 1940382330U);
  ssSetChecksumVal(S, 2, 442266481U);
  ssSetChecksumVal(S, 3, 692836452U);

  /* options */
  ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

  /* Accelerator check memory map size match for DWork */
  if (ssGetSizeofDWork(S) != sizeof(D_Work)) {
    ssSetErrorStatus(S,"Unexpected error: Internal DWork sizes do "
                     "not match for accelerator mex file.");
  }

  /* Accelerator check memory map size match for BlockIO */
  if (ssGetSizeofGlobalBlockIO(S) != sizeof(BlockIO)) {
    ssSetErrorStatus(S,"Unexpected error: Internal BlockIO sizes do "
                     "not match for accelerator mex file.");
  }

  /* model parameters */
  _ssSetDefaultParam(S, (real_T *) &rtDefaultParameters);

  /* non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* register function-calls */
  {
    SimStruct *childS;
    SysOutputFcn *callSysFcns;

    /* Level2 S-Function Block: '<S1>/B_0_0' (stateflow) */
    childS = ssGetSFunction(S, 0);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);

    /* Level2 S-Function Block: '<S2>/B_1_0' (stateflow) */
    childS = ssGetSFunction(S, 1);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);

    /* Level2 S-Function Block: '<S3>/B_2_0' (stateflow) */
    childS = ssGetSFunction(S, 2);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);

    /* Level2 S-Function Block: '<S4>/B_3_0' (stateflow) */
    childS = ssGetSFunction(S, 3);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);

    /* Level2 S-Function Block: '<S7>/B_4_0' (stateflow) */
    childS = ssGetSFunction(S, 4);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);

    /* Level2 S-Function Block: '<S6>/B_6_0' (stateflow) */
    childS = ssGetSFunction(S, 5);
    callSysFcns = ssGetCallSystemOutputFcnList(childS);

    /* Unconnected function-call */
    callSysFcns[3 + 0] = (SysOutputFcn) (NULL);
  }
}

/* Empty mdlInitializeSampleTimes function (never called) */
static void mdlInitializeSampleTimes(SimStruct *S)
{
}

/* Empty mdlTerminate function (never called) */
static void mdlTerminate(SimStruct *S)
{
}

/* MATLAB MEX Glue */
#include "simulink.c"
