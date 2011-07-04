/*
 * USB_TCrane_PIDxyz_data.c
 *
 * Real-Time Workshop code generation for Simulink model "USB_TCrane_PIDxyz.mdl".
 *
 * Model Version              : 1.22
 * Real-Time Workshop version : 7.0  (R2007b)  02-Aug-2007
 * C source code generated on : Tue Feb 01 10:11:10 2011
 */

#include "USB_TCrane_PIDxyz.h"
#include "USB_TCrane_PIDxyz_private.h"

/* Block parameters (auto storage) */
Parameters_USB_TCrane_PIDxyz USB_TCrane_PIDxyz_P = {
  /*  RTDAC_USB3DCraneDeviceDriver_P1 : '<S5>/RTDAC_USB  3D Crane Device Driver'
   */
  { 1.0, 1.0 },
  0.01,                                /* RTDAC_USB3DCraneDeviceDriver__g : '<S5>/RTDAC_USB  3D Crane Device Driver'
                                        */

  /*  Gain_Gain : '<S6>/Gain'
   */
  { 5.8157E-005, 0.00157, 9.9484E-005, 1.5339807878856412E-003,
    1.5339807878856412E-003 },
  0.24,                                /* Constant2_Value : '<Root>/Constant2'
                                        */
  0.144,                               /* SignalGenerator5_Amplitude : '<Root>/Signal Generator5'
                                        */
  0.075,                               /* SignalGenerator5_Frequency : '<Root>/Signal Generator5'
                                        */
  5.0,                                 /* Kp_Gain : '<S2>/Kp'
                                        */
  0.0,                                 /* Integrator_IC : '<S2>/Integrator'
                                        */
  1.0,                                 /* Integrator_UpperSat : '<S2>/Integrator'
                                        */
  -1.0,                                /* Integrator_LowerSat : '<S2>/Integrator'
                                        */
  0.0,                                 /* Memory_X0 : '<S2>/Memory'
                                        */
  0.0,                                 /* Kd1_Gain : '<S2>/Kd1'
                                        */
  1.0,                                 /* Saturation1_UpperSat : '<Root>/Saturation1'
                                        */
  -1.0,                                /* Saturation1_LowerSat : '<Root>/Saturation1'
                                        */
  0.3,                                 /* Constant1_Value : '<Root>/Constant1'
                                        */
  0.18,                                /* SignalGenerator4_Amplitude : '<Root>/Signal Generator4'
                                        */
  0.065,                               /* SignalGenerator4_Frequency : '<Root>/Signal Generator4'
                                        */
  5.0,                                 /* Kp_Gain_c : '<S3>/Kp'
                                        */
  0.0,                                 /* Integrator_IC_k : '<S3>/Integrator'
                                        */
  1.0,                                 /* Integrator_UpperSat_o : '<S3>/Integrator'
                                        */
  -1.0,                                /* Integrator_LowerSat_o : '<S3>/Integrator'
                                        */
  0.0,                                 /* Memory_X0_k : '<S3>/Memory'
                                        */
  0.0,                                 /* Kd1_Gain_k : '<S3>/Kd1'
                                        */
  1.0,                                 /* Saturation3_UpperSat : '<Root>/Saturation3'
                                        */
  -1.0,                                /* Saturation3_LowerSat : '<Root>/Saturation3'
                                        */
  0.25,                                /* Constant_Value : '<Root>/Constant'
                                        */
  0.15,                                /* SignalGenerator3_Amplitude : '<Root>/Signal Generator3'
                                        */
  0.075,                               /* SignalGenerator3_Frequency : '<Root>/Signal Generator3'
                                        */
  5.0,                                 /* Kp_Gain_m : '<S4>/Kp'
                                        */
  0.0,                                 /* Integrator_IC_ki : '<S4>/Integrator'
                                        */
  1.0,                                 /* Integrator_UpperSat_p : '<S4>/Integrator'
                                        */
  -1.0,                                /* Integrator_LowerSat_h : '<S4>/Integrator'
                                        */
  0.0,                                 /* Memory_X0_e : '<S4>/Memory'
                                        */
  0.0,                                 /* Kd1_Gain_kh : '<S4>/Kd1'
                                        */
  1.0,                                 /* Saturation4_UpperSat : '<Root>/Saturation4'
                                        */
  -1.0,                                /* Saturation4_LowerSat : '<Root>/Saturation4'
                                        */
  1.0,                                 /* Ki_Gain : '<S2>/Ki'
                                        */
  1.0,                                 /* Ki_Gain_a : '<S3>/Ki'
                                        */
  1.0,                                 /* Ki_Gain_f : '<S4>/Ki'
                                        */
  0.5,                                 /* Switch_Threshold : '<S6>/Switch'
                                        */

  /*  Constant2_Value_p : '<S6>/Constant2'
   */
  { 1.0, 1.0, 1.0 },
  1.0,                                 /* Constant9_Value : '<S7>/Constant9'
                                        */

  /*  Constant1_Value_j : '<S8>/Constant1'
   */
  { 1.0, 1.0, 1.0 },

  /*  Constant4_Value : '<S8>/Constant4'
   */
  { 134.0, 31.0, 126.0 },

  /*  Constant1_Value_o : '<S9>/Constant1'
   */
  { 1.0, 1.0, 1.0 },
  0.0,                                 /* Constant4_Value_o : '<S10>/Constant4'
                                        */
  2.5E-005,                            /* Gain_Gain_k : '<S10>/Gain'
                                        */
  0.0,                                 /* Memory_X0_d : '<S10>/Memory'
                                        */
  0.0,                                 /* Accelerate1_Value : '<Root>/Accelerate1'
                                        */
  1.0,                                 /* Accelerate_Value : '<Root>/Accelerate'
                                        */

  /*  Constant1_Value_l : '<S6>/Constant1'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0 },

  /*  Constant10_Value : '<S6>/Constant10'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },
  1U,                                  /* Constant_Value_e : '<S1>/Constant'
                                        */
  1U                                   /* SwitchControl_Threshold : '<S1>/SwitchControl'
                                        */
};
