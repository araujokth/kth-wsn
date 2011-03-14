/*
 * sensor_block_acc.h
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
#ifndef RTW_HEADER_sensor_block_acc_h_
#define RTW_HEADER_sensor_block_acc_h_
#ifndef sensor_block_acc_COMMON_INCLUDES_
# define sensor_block_acc_COMMON_INCLUDES_
#include <stdlib.h>
#include <stddef.h>
#define S_FUNCTION_NAME                simulink_only_sfcn
#define S_FUNCTION_LEVEL               2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_zcfcn.h"
#include "rt_nonfinite.h"
#endif                                 /* sensor_block_acc_COMMON_INCLUDES_ */

#include "sensor_block_acc_types.h"

/* Block signals (auto storage) */
typedef struct {
  real_T B_6_1_1;                      /* '<Root>/getGlobalStadistics' */
  real_T B_6_1_2;                      /* '<Root>/getGlobalStadistics' */
  real_T B_6_1_3;                      /* '<Root>/getGlobalStadistics' */
  real_T B_5_0_0;                      /* '<S5>/Constant' */
  real_T B_3_0_2;                      /* '<Root>/SplitSensors' */
  real_T B_3_0_3;                      /* '<Root>/SplitSensors' */
  real_T B_3_0_5;                      /* '<Root>/SplitSensors' */
  real_T B_3_0_6;                      /* '<Root>/SplitSensors' */
  real_T B_3_0_8;                      /* '<Root>/SplitSensors' */
  real_T B_3_0_9;                      /* '<Root>/SplitSensors' */
  real_T B_2_0_1;                      /* '<Root>/GetStadisticsXc' */
  real_T B_2_0_2;                      /* '<Root>/GetStadisticsXc' */
  real_T B_1_0_1;                      /* '<Root>/GetStadisticsTheta' */
  real_T B_1_0_2;                      /* '<Root>/GetStadisticsTheta' */
  real_T B_0_0_1;                      /* '<Root>/GetStadisticsActuator' */
  real_T B_0_0_2;                      /* '<Root>/GetStadisticsActuator' */
  uint32_T B_4_0_1;                    /* '<S5>/Embedded MATLAB Function' */
  uint16_T B_4_0_2;                    /* '<S5>/Embedded MATLAB Function' */
  uint16_T B_4_0_3;                    /* '<S5>/Embedded MATLAB Function' */
  uint16_T B_4_0_4;                    /* '<S5>/Embedded MATLAB Function' */
  uint16_T B_4_0_5;                    /* '<S5>/Embedded MATLAB Function' */
  uint16_T B_4_0_6;                    /* '<S5>/Embedded MATLAB Function' */
  uint8_T B_5_1_0[26];                 /* '<S5>/Level-2 M-file S-Function' */
  boolean_T B_6_0_0[3];                /* '<Root>/getGlobalStadistics' */
  boolean_T B_3_0_1;                   /* '<Root>/SplitSensors' */
  boolean_T B_3_0_4;                   /* '<Root>/SplitSensors' */
  boolean_T B_3_0_7;                   /* '<Root>/SplitSensors' */
  char pad_B_3_0_7[2];
} BlockIO;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Level2MfileSFunction_x1;      /* '<S5>/Level-2 M-file S-Function' */
  struct {
    void *LoggedData;
  } actuator_delay_PWORK;              /* '<Root>/actuator_delay' */

  struct {
    void *LoggedData;
  } actuator_errors_PWORK;             /* '<Root>/actuator_errors' */

  struct {
    void *LoggedData;
  } actuator_reliability_PWORK;        /* '<Root>/actuator_reliability' */

  struct {
    void *LoggedData;
  } actuator_value_PWORK;              /* '<Root>/actuator_value' */

  struct {
    void *LoggedData;
  } delay_PWORK;                       /* '<Root>/delay' */

  struct {
    void *LoggedData;
  } errors_PWORK;                      /* '<Root>/errors' */

  struct {
    void *LoggedData;
  } reliability_PWORK;                 /* '<Root>/reliability' */

  struct {
    void *LoggedData;
  } theta_delay_PWORK;                 /* '<Root>/theta_delay' */

  struct {
    void *LoggedData;
  } theta_errors_PWORK;                /* '<Root>/theta_errors' */

  struct {
    void *LoggedData;
  } theta_reliability_PWORK;           /* '<Root>/theta_reliability' */

  struct {
    void *LoggedData;
  } theta_value_PWORK;                 /* '<Root>/theta_value' */

  struct {
    void *LoggedData;
  } xc_delay_PWORK;                    /* '<Root>/xc_delay' */

  struct {
    void *LoggedData;
  } xc_errors_PWORK;                   /* '<Root>/xc_errors' */

  struct {
    void *LoggedData;
  } xc_reliability_PWORK;              /* '<Root>/xc_reliability' */

  struct {
    void *LoggedData;
  } xc_value_PWORK;                    /* '<Root>/xc_value' */

  void *Level2MfileSFunction_PWORK[3]; /* '<S5>/Level-2 M-file S-Function' */
  int32_T clockTickCounter;            /* '<Root>/Pulse Generator' */
  int8_T TriggeredSubsystem_SubsysRanBC;/* '<Root>/Triggered Subsystem' */
  char pad_TriggeredSubsystem_SubsysRanBC[3];
} D_Work;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState TriggeredSubsystem_Trig_ZCE;/* '<Root>/Triggered Subsystem' */
} PrevZCSigStates;

/* Parameters (auto storage) */
struct Parameters_ {
  real_T P_0;                          /* Expression: 1
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T P_1;                          /* Expression: 2
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T P_2;                          /* Expression: 1
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T P_3;                          /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T P_4;                          /* Expression: 1
                                        * Referenced by: '<S5>/Constant'
                                        */
};

extern Parameters rtDefaultParameters; /* parameters */

#endif                                 /* RTW_HEADER_sensor_block_acc_h_ */
