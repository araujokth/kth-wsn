/*
 * USB_TCrane_PIDxyz.c
 *
 * Real-Time Workshop code generation for Simulink model "USB_TCrane_PIDxyz.mdl".
 *
 * Model Version              : 1.22
 * Real-Time Workshop version : 7.0  (R2007b)  02-Aug-2007
 * C source code generated on : Tue Feb 01 10:11:10 2011
 */

#include "USB_TCrane_PIDxyz.h"
#include "USB_TCrane_PIDxyz_private.h"

/* Block signals (auto storage) */
BlockIO_USB_TCrane_PIDxyz USB_TCrane_PIDxyz_B;

/* Continuous states */
ContinuousStates_USB_TCrane_PIDxyz USB_TCrane_PIDxyz_X;

/* Block states (auto storage) */
D_Work_USB_TCrane_PIDxyz USB_TCrane_PIDxyz_DWork;

/* Real-time model */
RT_MODEL_USB_TCrane_PIDxyz USB_TCrane_PIDxyz_M_;
RT_MODEL_USB_TCrane_PIDxyz *USB_TCrane_PIDxyz_M = &USB_TCrane_PIDxyz_M_;

/* This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  USB_TCrane_PIDxyz_derivatives();
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; i++) {
    *x += h * f0[i];
    x++;
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void USB_TCrane_PIDxyz_output(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_Kp;
  real_T rtb_Sum;
  real_T rtb_Sum_g;

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(USB_TCrane_PIDxyz_M)) {
    USB_TCrane_PIDxyz_M->Timing.t[0] = rtsiGetT(&USB_TCrane_PIDxyz_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&USB_TCrane_PIDxyz_M->solverInfo,
                          ((USB_TCrane_PIDxyz_M->Timing.clockTick0+1)*
      USB_TCrane_PIDxyz_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  {
    int32_T i;
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      /* Level2 S-Function Block: '<S5>/RTDAC_USB  3D Crane Device Driver' (rtdacusb_tcrane_dd) */
      {
        SimStruct *rts = USB_TCrane_PIDxyz_M->childSfunctions[0];
        sfcnOutputs(rts, 1);
      }

      for (i = 0; i < 5; i++) {
        /* Gain: '<S6>/Gain' */
        USB_TCrane_PIDxyz_B.Gain[i] = USB_TCrane_PIDxyz_P.Gain_Gain[i] *
          USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o4[i];
      }
    }

    /* SignalGenerator: '<Root>/Signal Generator5' */
    {
      real_T phase = USB_TCrane_PIDxyz_P.SignalGenerator5_Frequency*
        USB_TCrane_PIDxyz_M->Timing.t[0];
      phase = phase-floor(phase);
      rtb_Sum_g = ( phase >= 0.5 ) ?
        USB_TCrane_PIDxyz_P.SignalGenerator5_Amplitude :
        -USB_TCrane_PIDxyz_P.SignalGenerator5_Amplitude;
    }

    /* Sum: '<Root>/Sum8' incorporates:
     *  Constant: '<Root>/Constant2'
     *  Sum: '<Root>/Sum10'
     */
    USB_TCrane_PIDxyz_B.Sum8 = (USB_TCrane_PIDxyz_P.Constant2_Value + rtb_Sum_g)
      - USB_TCrane_PIDxyz_B.Gain[0];

    /* Gain: '<S2>/Kp' */
    rtb_Kp = USB_TCrane_PIDxyz_P.Kp_Gain * USB_TCrane_PIDxyz_B.Sum8;

    /* Integrator: '<S2>/Integrator' */
    if (USB_TCrane_PIDxyz_X.Integrator_CSTATE >=
        USB_TCrane_PIDxyz_P.Integrator_UpperSat ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE =
        USB_TCrane_PIDxyz_P.Integrator_UpperSat;
    } else if (USB_TCrane_PIDxyz_X.Integrator_CSTATE <=
               USB_TCrane_PIDxyz_P.Integrator_LowerSat ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE =
        USB_TCrane_PIDxyz_P.Integrator_LowerSat;
    }

    rtb_Sum_g = USB_TCrane_PIDxyz_X.Integrator_CSTATE;
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S2>/Memory' */
      USB_TCrane_PIDxyz_B.Memory = USB_TCrane_PIDxyz_DWork.Memory_PreviousInput;
    }

    /* Sum: '<S2>/Sum' incorporates:
     *  Gain: '<S2>/Kd1'
     *  Sum: '<S2>/Sum1'
     */
    rtb_Sum_g = (USB_TCrane_PIDxyz_B.Sum8 - USB_TCrane_PIDxyz_B.Memory) *
      USB_TCrane_PIDxyz_P.Kd1_Gain + (rtb_Kp + rtb_Sum_g);

    /* Saturate: '<Root>/Saturation1' */
    USB_TCrane_PIDxyz_B.Saturation1 = rt_SATURATE(rtb_Sum_g,
      USB_TCrane_PIDxyz_P.Saturation1_LowerSat,
      USB_TCrane_PIDxyz_P.Saturation1_UpperSat);

    /* SignalGenerator: '<Root>/Signal Generator4' */
    {
      real_T phase = USB_TCrane_PIDxyz_P.SignalGenerator4_Frequency*
        USB_TCrane_PIDxyz_M->Timing.t[0];
      phase = phase-floor(phase);
      rtb_Sum_g = ( phase >= 0.5 ) ?
        USB_TCrane_PIDxyz_P.SignalGenerator4_Amplitude :
        -USB_TCrane_PIDxyz_P.SignalGenerator4_Amplitude;
    }

    /* Sum: '<Root>/Sum9' incorporates:
     *  Constant: '<Root>/Constant1'
     *  Sum: '<Root>/Sum7'
     */
    USB_TCrane_PIDxyz_B.Sum9 = (USB_TCrane_PIDxyz_P.Constant1_Value + rtb_Sum_g)
      - USB_TCrane_PIDxyz_B.Gain[1];

    /* Gain: '<S3>/Kp' */
    rtb_Kp = USB_TCrane_PIDxyz_P.Kp_Gain_c * USB_TCrane_PIDxyz_B.Sum9;

    /* Integrator: '<S3>/Integrator' */
    if (USB_TCrane_PIDxyz_X.Integrator_CSTATE_p >=
        USB_TCrane_PIDxyz_P.Integrator_UpperSat_o ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE_p =
        USB_TCrane_PIDxyz_P.Integrator_UpperSat_o;
    } else if (USB_TCrane_PIDxyz_X.Integrator_CSTATE_p <=
               USB_TCrane_PIDxyz_P.Integrator_LowerSat_o ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE_p =
        USB_TCrane_PIDxyz_P.Integrator_LowerSat_o;
    }

    rtb_Sum_g = USB_TCrane_PIDxyz_X.Integrator_CSTATE_p;
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S3>/Memory' */
      USB_TCrane_PIDxyz_B.Memory_p =
        USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_c;
    }

    /* Sum: '<S3>/Sum' incorporates:
     *  Gain: '<S3>/Kd1'
     *  Sum: '<S3>/Sum1'
     */
    rtb_Sum_g = (USB_TCrane_PIDxyz_B.Sum9 - USB_TCrane_PIDxyz_B.Memory_p) *
      USB_TCrane_PIDxyz_P.Kd1_Gain_k + (rtb_Kp + rtb_Sum_g);

    /* Saturate: '<Root>/Saturation3' */
    USB_TCrane_PIDxyz_B.Saturation3 = rt_SATURATE(rtb_Sum_g,
      USB_TCrane_PIDxyz_P.Saturation3_LowerSat,
      USB_TCrane_PIDxyz_P.Saturation3_UpperSat);

    /* SignalGenerator: '<Root>/Signal Generator3' */
    {
      real_T phase = USB_TCrane_PIDxyz_P.SignalGenerator3_Frequency*
        USB_TCrane_PIDxyz_M->Timing.t[0];
      phase = phase-floor(phase);
      rtb_Sum_g = ( phase >= 0.5 ) ?
        USB_TCrane_PIDxyz_P.SignalGenerator3_Amplitude :
        -USB_TCrane_PIDxyz_P.SignalGenerator3_Amplitude;
    }

    /* Sum: '<Root>/Sum6' incorporates:
     *  Constant: '<Root>/Constant'
     *  Sum: '<Root>/Sum5'
     */
    USB_TCrane_PIDxyz_B.Sum6 = (USB_TCrane_PIDxyz_P.Constant_Value + rtb_Sum_g)
      - USB_TCrane_PIDxyz_B.Gain[2];

    /* Gain: '<S4>/Kp' */
    rtb_Kp = USB_TCrane_PIDxyz_P.Kp_Gain_m * USB_TCrane_PIDxyz_B.Sum6;

    /* Integrator: '<S4>/Integrator' */
    if (USB_TCrane_PIDxyz_X.Integrator_CSTATE_a >=
        USB_TCrane_PIDxyz_P.Integrator_UpperSat_p ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE_a =
        USB_TCrane_PIDxyz_P.Integrator_UpperSat_p;
    } else if (USB_TCrane_PIDxyz_X.Integrator_CSTATE_a <=
               USB_TCrane_PIDxyz_P.Integrator_LowerSat_h ) {
      USB_TCrane_PIDxyz_X.Integrator_CSTATE_a =
        USB_TCrane_PIDxyz_P.Integrator_LowerSat_h;
    }

    rtb_Sum_g = USB_TCrane_PIDxyz_X.Integrator_CSTATE_a;
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S4>/Memory' */
      USB_TCrane_PIDxyz_B.Memory_f =
        USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_j;
    }

    /* Sum: '<S4>/Sum' incorporates:
     *  Gain: '<S4>/Kd1'
     *  Sum: '<S4>/Sum1'
     */
    rtb_Sum_g = (USB_TCrane_PIDxyz_B.Sum6 - USB_TCrane_PIDxyz_B.Memory_f) *
      USB_TCrane_PIDxyz_P.Kd1_Gain_kh + (rtb_Kp + rtb_Sum_g);

    /* Saturate: '<Root>/Saturation4' */
    USB_TCrane_PIDxyz_B.Saturation4 = rt_SATURATE(rtb_Sum_g,
      USB_TCrane_PIDxyz_P.Saturation4_LowerSat,
      USB_TCrane_PIDxyz_P.Saturation4_UpperSat);
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      /* Scope: '<Root>/Scope' */
      if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M)) {
        StructLogVar *svar = (StructLogVar *)
          USB_TCrane_PIDxyz_DWork.Scope_PWORK.LoggedData;
        LogVar *var = svar->signals.values;

        /* time */
        {
          double locTime = USB_TCrane_PIDxyz_M->Timing.t[1];
          rt_UpdateLogVar((LogVar *)svar->time, &locTime, 0);
        }

        /* signals */
        {
          real_T up0[8];
          up0[0] = USB_TCrane_PIDxyz_B.Gain[0];
          up0[1] = USB_TCrane_PIDxyz_B.Saturation1;
          up0[2] = USB_TCrane_PIDxyz_B.Gain[1];
          up0[3] = USB_TCrane_PIDxyz_B.Saturation3;
          up0[4] = USB_TCrane_PIDxyz_B.Gain[2];
          up0[5] = USB_TCrane_PIDxyz_B.Saturation4;
          up0[6] = USB_TCrane_PIDxyz_B.Gain[3];
          up0[7] = USB_TCrane_PIDxyz_B.Gain[4];
          rt_UpdateLogVar((LogVar *)var, up0, 0);
        }
      }

      /* Switch: '<S1>/SwitchControl' incorporates:
       *  Constant: '<Root>/Accelerate'
       *  Constant: '<Root>/Accelerate1'
       *  Constant: '<S1>/Constant'
       */
      if (USB_TCrane_PIDxyz_P.Constant_Value_e >
          USB_TCrane_PIDxyz_P.SwitchControl_Threshold) {
        rtb_Sum = USB_TCrane_PIDxyz_P.Accelerate_Value;
      } else {
        rtb_Sum = USB_TCrane_PIDxyz_P.Accelerate1_Value;
      }
    }

    /* Gain: '<S2>/Ki' */
    USB_TCrane_PIDxyz_B.Ki = USB_TCrane_PIDxyz_P.Ki_Gain *
      USB_TCrane_PIDxyz_B.Sum8;

    /* Gain: '<S3>/Ki' */
    USB_TCrane_PIDxyz_B.Ki_e = USB_TCrane_PIDxyz_P.Ki_Gain_a *
      USB_TCrane_PIDxyz_B.Sum9;

    /* Gain: '<S4>/Ki' */
    USB_TCrane_PIDxyz_B.Ki_n = USB_TCrane_PIDxyz_P.Ki_Gain_f *
      USB_TCrane_PIDxyz_B.Sum6;
    if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
        USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
      for (i = 0; i < 5; i++) {
        /* Switch: '<S6>/Switch' incorporates:
         *  Constant: '<S6>/Constant1'
         *  Constant: '<S6>/Constant10'
         */
        if (rtb_Sum >= USB_TCrane_PIDxyz_P.Switch_Threshold) {
          USB_TCrane_PIDxyz_B.Switch[i] = USB_TCrane_PIDxyz_P.Constant10_Value[i];
        } else {
          USB_TCrane_PIDxyz_B.Switch[i] =
            USB_TCrane_PIDxyz_P.Constant1_Value_l[i];
        }
      }

      /* Gain: '<S10>/Gain' */
      USB_TCrane_PIDxyz_B.Gain_n = USB_TCrane_PIDxyz_P.Gain_Gain_k *
        USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver__a;

      /* Memory: '<S10>/Memory' */
      rtb_Sum = USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_m;

      /* Sum: '<S10>/Sum' */
      rtb_Sum = USB_TCrane_PIDxyz_B.Gain_n - rtb_Sum;
    }
  }

  UNUSED_PARAMETER(tid);
}

/* Model update function */
void USB_TCrane_PIDxyz_update(int_T tid)
{
  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
      USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update for Memory: '<S2>/Memory' */
    USB_TCrane_PIDxyz_DWork.Memory_PreviousInput = USB_TCrane_PIDxyz_B.Sum8;
  }

  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
      USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update for Memory: '<S3>/Memory' */
    USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_c = USB_TCrane_PIDxyz_B.Sum9;
  }

  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
      USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update for Memory: '<S4>/Memory' */
    USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_j = USB_TCrane_PIDxyz_B.Sum6;

    /* Update for Memory: '<S10>/Memory' */
    USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_m = USB_TCrane_PIDxyz_B.Gain_n;
  }

  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M)) {
    rt_ertODEUpdateContinuousStates(&USB_TCrane_PIDxyz_M->solverInfo);
  }

  /* Update absolute time for base rate */
  if (!(++USB_TCrane_PIDxyz_M->Timing.clockTick0))
    ++USB_TCrane_PIDxyz_M->Timing.clockTickH0;
  USB_TCrane_PIDxyz_M->Timing.t[0] = USB_TCrane_PIDxyz_M->Timing.clockTick0 *
    USB_TCrane_PIDxyz_M->Timing.stepSize0 +
    USB_TCrane_PIDxyz_M->Timing.clockTickH0 *
    USB_TCrane_PIDxyz_M->Timing.stepSize0 * 4294967296.0;
  if (rtmIsMajorTimeStep(USB_TCrane_PIDxyz_M) &&
      USB_TCrane_PIDxyz_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    if (!(++USB_TCrane_PIDxyz_M->Timing.clockTick1))
      ++USB_TCrane_PIDxyz_M->Timing.clockTickH1;
    USB_TCrane_PIDxyz_M->Timing.t[1] = USB_TCrane_PIDxyz_M->Timing.clockTick1 *
      USB_TCrane_PIDxyz_M->Timing.stepSize1 +
      USB_TCrane_PIDxyz_M->Timing.clockTickH1 *
      USB_TCrane_PIDxyz_M->Timing.stepSize1 * 4294967296.0;
  }

  UNUSED_PARAMETER(tid);
}

/* Derivatives for root system: '<Root>' */
void USB_TCrane_PIDxyz_derivatives(void)
{
  /* Limited Integrator Block: '<S2>/Integrator' */
  {
    boolean_T lsat;
    boolean_T usat;
    lsat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE <=
            USB_TCrane_PIDxyz_P.Integrator_LowerSat );
    usat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE >=
            USB_TCrane_PIDxyz_P.Integrator_UpperSat );
    if ((!lsat && !usat) ||
        (lsat && (USB_TCrane_PIDxyz_B.Ki > 0)) ||
        (usat && (USB_TCrane_PIDxyz_B.Ki < 0)) ) {
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE =
        USB_TCrane_PIDxyz_B.Ki;
    } else {
      /* in saturation */
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE = 0.0;
    }
  }

  /* Limited Integrator Block: '<S3>/Integrator' */
  {
    boolean_T lsat;
    boolean_T usat;
    lsat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE_p <=
            USB_TCrane_PIDxyz_P.Integrator_LowerSat_o );
    usat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE_p >=
            USB_TCrane_PIDxyz_P.Integrator_UpperSat_o );
    if ((!lsat && !usat) ||
        (lsat && (USB_TCrane_PIDxyz_B.Ki_e > 0)) ||
        (usat && (USB_TCrane_PIDxyz_B.Ki_e < 0)) ) {
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE_p =
        USB_TCrane_PIDxyz_B.Ki_e;
    } else {
      /* in saturation */
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE_p = 0.0;
    }
  }

  /* Limited Integrator Block: '<S4>/Integrator' */
  {
    boolean_T lsat;
    boolean_T usat;
    lsat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE_a <=
            USB_TCrane_PIDxyz_P.Integrator_LowerSat_h );
    usat = ( USB_TCrane_PIDxyz_X.Integrator_CSTATE_a >=
            USB_TCrane_PIDxyz_P.Integrator_UpperSat_p );
    if ((!lsat && !usat) ||
        (lsat && (USB_TCrane_PIDxyz_B.Ki_n > 0)) ||
        (usat && (USB_TCrane_PIDxyz_B.Ki_n < 0)) ) {
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE_a =
        USB_TCrane_PIDxyz_B.Ki_n;
    } else {
      /* in saturation */
      ((StateDerivatives_USB_TCrane_PIDxyz *)
        USB_TCrane_PIDxyz_M->ModelData.derivs)->Integrator_CSTATE_a = 0.0;
    }
  }
}

/* Model initialize function */
void USB_TCrane_PIDxyz_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));    /* initialize real-time model */
  (void) memset((char_T *)USB_TCrane_PIDxyz_M,0,
                sizeof(RT_MODEL_USB_TCrane_PIDxyz));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&USB_TCrane_PIDxyz_M->solverInfo,
                          &USB_TCrane_PIDxyz_M->Timing.simTimeStep);
    rtsiSetTPtr(&USB_TCrane_PIDxyz_M->solverInfo, &rtmGetTPtr
                (USB_TCrane_PIDxyz_M));
    rtsiSetStepSizePtr(&USB_TCrane_PIDxyz_M->solverInfo,
                       &USB_TCrane_PIDxyz_M->Timing.stepSize0);
    rtsiSetdXPtr(&USB_TCrane_PIDxyz_M->solverInfo,
                 &USB_TCrane_PIDxyz_M->ModelData.derivs);
    rtsiSetContStatesPtr(&USB_TCrane_PIDxyz_M->solverInfo,
                         &USB_TCrane_PIDxyz_M->ModelData.contStates);
    rtsiSetNumContStatesPtr(&USB_TCrane_PIDxyz_M->solverInfo,
      &USB_TCrane_PIDxyz_M->Sizes.numContStates);
    rtsiSetErrorStatusPtr(&USB_TCrane_PIDxyz_M->solverInfo, (&rtmGetErrorStatus
      (USB_TCrane_PIDxyz_M)));
    rtsiSetRTModelPtr(&USB_TCrane_PIDxyz_M->solverInfo, USB_TCrane_PIDxyz_M);
  }

  rtsiSetSimTimeStep(&USB_TCrane_PIDxyz_M->solverInfo, MAJOR_TIME_STEP);
  USB_TCrane_PIDxyz_M->ModelData.intgData.f[0] =
    USB_TCrane_PIDxyz_M->ModelData.odeF[0];
  USB_TCrane_PIDxyz_M->ModelData.contStates = ((real_T *) &USB_TCrane_PIDxyz_X);
  rtsiSetSolverData(&USB_TCrane_PIDxyz_M->solverInfo, (void *)
                    &USB_TCrane_PIDxyz_M->ModelData.intgData);
  rtsiSetSolverName(&USB_TCrane_PIDxyz_M->solverInfo,"ode1");
  USB_TCrane_PIDxyz_M->solverInfoPtr = (&USB_TCrane_PIDxyz_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = USB_TCrane_PIDxyz_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    USB_TCrane_PIDxyz_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    USB_TCrane_PIDxyz_M->Timing.sampleTimes =
      (&USB_TCrane_PIDxyz_M->Timing.sampleTimesArray[0]);
    USB_TCrane_PIDxyz_M->Timing.offsetTimes =
      (&USB_TCrane_PIDxyz_M->Timing.offsetTimesArray[0]);

    /* task periods */
    USB_TCrane_PIDxyz_M->Timing.sampleTimes[0] = (0.0);
    USB_TCrane_PIDxyz_M->Timing.sampleTimes[1] = (0.01);

    /* task offsets */
    USB_TCrane_PIDxyz_M->Timing.offsetTimes[0] = (0.0);
    USB_TCrane_PIDxyz_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(USB_TCrane_PIDxyz_M, &USB_TCrane_PIDxyz_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = USB_TCrane_PIDxyz_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    USB_TCrane_PIDxyz_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(USB_TCrane_PIDxyz_M, 100.0);
  USB_TCrane_PIDxyz_M->Timing.stepSize0 = 0.01;
  USB_TCrane_PIDxyz_M->Timing.stepSize1 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    USB_TCrane_PIDxyz_M->rtwLogInfo = &rt_DataLoggingInfo;
    rtliSetLogFormat(USB_TCrane_PIDxyz_M->rtwLogInfo, 0);
    rtliSetLogMaxRows(USB_TCrane_PIDxyz_M->rtwLogInfo, 1000);
    rtliSetLogDecimation(USB_TCrane_PIDxyz_M->rtwLogInfo, 1);
    rtliSetLogVarNameModifier(USB_TCrane_PIDxyz_M->rtwLogInfo, "rt_");
    rtliSetLogT(USB_TCrane_PIDxyz_M->rtwLogInfo, "");
    rtliSetLogX(USB_TCrane_PIDxyz_M->rtwLogInfo, "");
    rtliSetLogXFinal(USB_TCrane_PIDxyz_M->rtwLogInfo, "");
    rtliSetSigLog(USB_TCrane_PIDxyz_M->rtwLogInfo, "");
    rtliSetLogXSignalInfo(USB_TCrane_PIDxyz_M->rtwLogInfo, NULL);
    rtliSetLogXSignalPtrs(USB_TCrane_PIDxyz_M->rtwLogInfo, NULL);
    rtliSetLogY(USB_TCrane_PIDxyz_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(USB_TCrane_PIDxyz_M->rtwLogInfo, NULL);
    rtliSetLogYSignalPtrs(USB_TCrane_PIDxyz_M->rtwLogInfo, NULL);
  }

  USB_TCrane_PIDxyz_M->solverInfoPtr = (&USB_TCrane_PIDxyz_M->solverInfo);
  USB_TCrane_PIDxyz_M->Timing.stepSize = (0.01);
  rtsiSetFixedStepSize(&USB_TCrane_PIDxyz_M->solverInfo, 0.01);
  rtsiSetSolverMode(&USB_TCrane_PIDxyz_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  USB_TCrane_PIDxyz_M->ModelData.blockIO = ((void *) &USB_TCrane_PIDxyz_B);

  {
    int_T i;
    void *pVoidBlockIORegion;
    pVoidBlockIORegion = (void *)
      (&USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o1[0]);
    for (i = 0; i < 59; i++) {
      ((real_T*)pVoidBlockIORegion)[i] = 0.0;
    }
  }

  /* parameters */
  USB_TCrane_PIDxyz_M->ModelData.defaultParam = ((real_T *) &USB_TCrane_PIDxyz_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &USB_TCrane_PIDxyz_X;
    USB_TCrane_PIDxyz_M->ModelData.contStates = (x);
    (void) memset((char_T *)x,0,
                  sizeof(ContinuousStates_USB_TCrane_PIDxyz));
  }

  /* states (dwork) */
  USB_TCrane_PIDxyz_M->Work.dwork = ((void *) &USB_TCrane_PIDxyz_DWork);
  (void) memset((char_T *) &USB_TCrane_PIDxyz_DWork,0,
                sizeof(D_Work_USB_TCrane_PIDxyz));

  {
    real_T *dwork_ptr = (real_T *) &USB_TCrane_PIDxyz_DWork.Memory_PreviousInput;
    dwork_ptr[0] = 0.0;
    dwork_ptr[1] = 0.0;
    dwork_ptr[2] = 0.0;
    dwork_ptr[3] = 0.0;
  }

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &USB_TCrane_PIDxyz_M->NonInlinedSFcns.sfcnInfo;
    USB_TCrane_PIDxyz_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(USB_TCrane_PIDxyz_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo,
      &USB_TCrane_PIDxyz_M->Sizes.numSampTimes);
    rtssSetTPtrPtr(sfcnInfo, &rtmGetTPtr(USB_TCrane_PIDxyz_M));
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(USB_TCrane_PIDxyz_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(USB_TCrane_PIDxyz_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput
      (USB_TCrane_PIDxyz_M));
    rtssSetStepSizePtr(sfcnInfo, &USB_TCrane_PIDxyz_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(USB_TCrane_PIDxyz_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &USB_TCrane_PIDxyz_M->ModelData.derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo,
      &USB_TCrane_PIDxyz_M->ModelData.zCCacheNeedsReset);
    rtssSetBlkStateChangePtr(sfcnInfo,
      &USB_TCrane_PIDxyz_M->ModelData.blkStateChange);
    rtssSetSampleHitsPtr(sfcnInfo, &USB_TCrane_PIDxyz_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &USB_TCrane_PIDxyz_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &USB_TCrane_PIDxyz_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &USB_TCrane_PIDxyz_M->solverInfoPtr);
  }

  USB_TCrane_PIDxyz_M->Sizes.numSFcns = (1);

  /* register each child */
  {
    (void) memset((void *)&USB_TCrane_PIDxyz_M->NonInlinedSFcns.childSFunctions
                  [0],0,
                  1*sizeof(SimStruct));
    USB_TCrane_PIDxyz_M->childSfunctions =
      (&USB_TCrane_PIDxyz_M->NonInlinedSFcns.childSFunctionPtrs[0]);
    USB_TCrane_PIDxyz_M->childSfunctions[0] =
      (&USB_TCrane_PIDxyz_M->NonInlinedSFcns.childSFunctions[0]);

    /* Level2 S-Function Block: USB_TCrane_PIDxyz/<S5>/RTDAC_USB  3D Crane Device Driver (rtdacusb_tcrane_dd) */
    {
      SimStruct *rts = USB_TCrane_PIDxyz_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset((void*)sfcnPeriod,0,
                    sizeof(time_T)*1);
      (void) memset((void*)sfcnOffset,0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      /* Set up the mdlInfo pointer */
      {
        ssSetBlkInfo2Ptr(rts, &USB_TCrane_PIDxyz_M->NonInlinedSFcns.blkInfo2[0]);
        ssSetRTWSfcnInfo(rts, USB_TCrane_PIDxyz_M->sfcnInfo);
      }

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &USB_TCrane_PIDxyz_M->NonInlinedSFcns.methods2[0]);
      }

      /* inputs */
      {
        _ssSetNumInputPorts(rts, 8);
        ssSetPortInfoForInputs(rts,
          &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.inputPortInfo[0]);

        /* port 0 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs0;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant9_Value;
          ssSetInputPortSignalPtrs(rts, 0, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 0, 1);
          ssSetInputPortWidth(rts, 0, 1);
        }

        /* port 1 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs1;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_B.Saturation1;
          sfcnUPtrs[1] = &USB_TCrane_PIDxyz_B.Saturation3;
          sfcnUPtrs[2] = &USB_TCrane_PIDxyz_B.Saturation4;
          ssSetInputPortSignalPtrs(rts, 1, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 1, 1);
          ssSetInputPortWidth(rts, 1, 3);
        }

        /* port 2 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs2;

          {
            int_T i1;
            const real_T *u2 = USB_TCrane_PIDxyz_B.Switch;
            for (i1=0; i1 < 5; i1++) {
              sfcnUPtrs[i1] = &u2[i1];
            }
          }

          ssSetInputPortSignalPtrs(rts, 2, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 2, 1);
          ssSetInputPortWidth(rts, 2, 5);
        }

        /* port 3 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs3;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant2_Value_p[0];
          sfcnUPtrs[1] = &USB_TCrane_PIDxyz_P.Constant2_Value_p[1];
          sfcnUPtrs[2] = &USB_TCrane_PIDxyz_P.Constant2_Value_p[2];
          ssSetInputPortSignalPtrs(rts, 3, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 3, 1);
          ssSetInputPortWidth(rts, 3, 3);
        }

        /* port 4 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs4;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant4_Value[0];
          sfcnUPtrs[1] = &USB_TCrane_PIDxyz_P.Constant4_Value[1];
          sfcnUPtrs[2] = &USB_TCrane_PIDxyz_P.Constant4_Value[2];
          ssSetInputPortSignalPtrs(rts, 4, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 4, 1);
          ssSetInputPortWidth(rts, 4, 3);
        }

        /* port 5 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs5;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant1_Value_j[0];
          sfcnUPtrs[1] = &USB_TCrane_PIDxyz_P.Constant1_Value_j[1];
          sfcnUPtrs[2] = &USB_TCrane_PIDxyz_P.Constant1_Value_j[2];
          ssSetInputPortSignalPtrs(rts, 5, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 5, 1);
          ssSetInputPortWidth(rts, 5, 3);
        }

        /* port 6 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs6;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant1_Value_o[0];
          sfcnUPtrs[1] = &USB_TCrane_PIDxyz_P.Constant1_Value_o[1];
          sfcnUPtrs[2] = &USB_TCrane_PIDxyz_P.Constant1_Value_o[2];
          ssSetInputPortSignalPtrs(rts, 6, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 6, 1);
          ssSetInputPortWidth(rts, 6, 3);
        }

        /* port 7 */
        {
          real_T const **sfcnUPtrs = (real_T const **)
            &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.UPtrs7;
          sfcnUPtrs[0] = &USB_TCrane_PIDxyz_P.Constant4_Value_o;
          ssSetInputPortSignalPtrs(rts, 7, (InputPtrsType)&sfcnUPtrs[0]);
          _ssSetInputPortNumDimensions(rts, 7, 1);
          ssSetInputPortWidth(rts, 7, 1);
        }
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 13);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 2);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidth(rts, 1, 1);
          ssSetOutputPortSignal(rts, 1, ((real_T *)
            &USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o2));
        }

        /* port 2 */
        {
          _ssSetOutputPortNumDimensions(rts, 2, 1);
          ssSetOutputPortWidth(rts, 2, 3);
          ssSetOutputPortSignal(rts, 2, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o3));
        }

        /* port 3 */
        {
          _ssSetOutputPortNumDimensions(rts, 3, 1);
          ssSetOutputPortWidth(rts, 3, 5);
          ssSetOutputPortSignal(rts, 3, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o4));
        }

        /* port 4 */
        {
          _ssSetOutputPortNumDimensions(rts, 4, 1);
          ssSetOutputPortWidth(rts, 4, 5);
          ssSetOutputPortSignal(rts, 4, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o5));
        }

        /* port 5 */
        {
          _ssSetOutputPortNumDimensions(rts, 5, 1);
          ssSetOutputPortWidth(rts, 5, 3);
          ssSetOutputPortSignal(rts, 5, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o6));
        }

        /* port 6 */
        {
          _ssSetOutputPortNumDimensions(rts, 6, 1);
          ssSetOutputPortWidth(rts, 6, 3);
          ssSetOutputPortSignal(rts, 6, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o7));
        }

        /* port 7 */
        {
          _ssSetOutputPortNumDimensions(rts, 7, 1);
          ssSetOutputPortWidth(rts, 7, 3);
          ssSetOutputPortSignal(rts, 7, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o8));
        }

        /* port 8 */
        {
          _ssSetOutputPortNumDimensions(rts, 8, 1);
          ssSetOutputPortWidth(rts, 8, 3);
          ssSetOutputPortSignal(rts, 8, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver_o9));
        }

        /* port 9 */
        {
          _ssSetOutputPortNumDimensions(rts, 9, 1);
          ssSetOutputPortWidth(rts, 9, 3);
          ssSetOutputPortSignal(rts, 9, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver__p));
        }

        /* port 10 */
        {
          _ssSetOutputPortNumDimensions(rts, 10, 1);
          ssSetOutputPortWidth(rts, 10, 3);
          ssSetOutputPortSignal(rts, 10, ((real_T *)
            USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver__m));
        }

        /* port 11 */
        {
          _ssSetOutputPortNumDimensions(rts, 11, 1);
          ssSetOutputPortWidth(rts, 11, 1);
          ssSetOutputPortSignal(rts, 11, ((real_T *)
            &USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver__a));
        }

        /* port 12 */
        {
          _ssSetOutputPortNumDimensions(rts, 12, 1);
          ssSetOutputPortWidth(rts, 12, 1);
          ssSetOutputPortSignal(rts, 12, ((real_T *)
            &USB_TCrane_PIDxyz_B.RTDAC_USB3DCraneDeviceDriver__f));
        }
      }

      /* path info */
      ssSetModelName(rts, "RTDAC_USB \n3D Crane Device Driver");
      ssSetPath(rts,
                "USB_TCrane_PIDxyz/RT-DAC//USB Tower Crane Driver/RTDAC_USB  3D Crane Device Driver");
      ssSetRTModel(rts,USB_TCrane_PIDxyz_M);
      ssSetParentSS(rts, NULL);
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &USB_TCrane_PIDxyz_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 1);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)
                       &USB_TCrane_PIDxyz_P.RTDAC_USB3DCraneDeviceDriver_P1[0]);
      }

      /* registration */
      rtdacusb_tcrane_dd(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.01);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 1;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetInputPortConnected(rts, 0, 1);
      _ssSetInputPortConnected(rts, 1, 1);
      _ssSetInputPortConnected(rts, 2, 1);
      _ssSetInputPortConnected(rts, 3, 1);
      _ssSetInputPortConnected(rts, 4, 1);
      _ssSetInputPortConnected(rts, 5, 1);
      _ssSetInputPortConnected(rts, 6, 1);
      _ssSetInputPortConnected(rts, 7, 1);
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortConnected(rts, 2, 1);
      _ssSetOutputPortConnected(rts, 3, 1);
      _ssSetOutputPortConnected(rts, 4, 1);
      _ssSetOutputPortConnected(rts, 5, 1);
      _ssSetOutputPortConnected(rts, 6, 1);
      _ssSetOutputPortConnected(rts, 7, 1);
      _ssSetOutputPortConnected(rts, 8, 1);
      _ssSetOutputPortConnected(rts, 9, 1);
      _ssSetOutputPortConnected(rts, 10, 1);
      _ssSetOutputPortConnected(rts, 11, 1);
      _ssSetOutputPortConnected(rts, 12, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);
      _ssSetOutputPortBeingMerged(rts, 2, 0);
      _ssSetOutputPortBeingMerged(rts, 3, 0);
      _ssSetOutputPortBeingMerged(rts, 4, 0);
      _ssSetOutputPortBeingMerged(rts, 5, 0);
      _ssSetOutputPortBeingMerged(rts, 6, 0);
      _ssSetOutputPortBeingMerged(rts, 7, 0);
      _ssSetOutputPortBeingMerged(rts, 8, 0);
      _ssSetOutputPortBeingMerged(rts, 9, 0);
      _ssSetOutputPortBeingMerged(rts, 10, 0);
      _ssSetOutputPortBeingMerged(rts, 11, 0);
      _ssSetOutputPortBeingMerged(rts, 12, 0);

      /* Update the BufferDstPort flags for each input port */
      ssSetInputPortBufferDstPort(rts, 0, -1);
      ssSetInputPortBufferDstPort(rts, 1, -1);
      ssSetInputPortBufferDstPort(rts, 2, -1);
      ssSetInputPortBufferDstPort(rts, 3, -1);
      ssSetInputPortBufferDstPort(rts, 4, -1);
      ssSetInputPortBufferDstPort(rts, 5, -1);
      ssSetInputPortBufferDstPort(rts, 6, -1);
      ssSetInputPortBufferDstPort(rts, 7, -1);
    }
  }
}

/* Model terminate function */
void USB_TCrane_PIDxyz_terminate(void)
{
  /* Level2 S-Function Block: '<S5>/RTDAC_USB  3D Crane Device Driver' (rtdacusb_tcrane_dd) */
  {
    SimStruct *rts = USB_TCrane_PIDxyz_M->childSfunctions[0];
    sfcnTerminate(rts);
  }
}

/*========================================================================*
 * Start of GRT compatible call interface                                 *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  USB_TCrane_PIDxyz_output(tid);
}

void MdlUpdate(int_T tid)
{
  USB_TCrane_PIDxyz_update(tid);
}

void MdlInitializeSizes(void)
{
  USB_TCrane_PIDxyz_M->Sizes.numContStates = (3);/* Number of continuous states */
  USB_TCrane_PIDxyz_M->Sizes.numY = (0);/* Number of model outputs */
  USB_TCrane_PIDxyz_M->Sizes.numU = (0);/* Number of model inputs */
  USB_TCrane_PIDxyz_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  USB_TCrane_PIDxyz_M->Sizes.numSampTimes = (2);/* Number of sample times */
  USB_TCrane_PIDxyz_M->Sizes.numBlocks = (60);/* Number of blocks */
  USB_TCrane_PIDxyz_M->Sizes.numBlockIO = (28);/* Number of block outputs */
  USB_TCrane_PIDxyz_M->Sizes.numBlockPrms = (75);/* Sum of parameter "widths" */
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
  /* Limited Integrator Block: '<S2>/Integrator' */
  USB_TCrane_PIDxyz_X.Integrator_CSTATE = USB_TCrane_PIDxyz_P.Integrator_IC;

  /* InitializeConditions for Memory: '<S2>/Memory' */
  USB_TCrane_PIDxyz_DWork.Memory_PreviousInput = USB_TCrane_PIDxyz_P.Memory_X0;

  /* Limited Integrator Block: '<S3>/Integrator' */
  USB_TCrane_PIDxyz_X.Integrator_CSTATE_p = USB_TCrane_PIDxyz_P.Integrator_IC_k;

  /* InitializeConditions for Memory: '<S3>/Memory' */
  USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_c =
    USB_TCrane_PIDxyz_P.Memory_X0_k;

  /* Limited Integrator Block: '<S4>/Integrator' */
  USB_TCrane_PIDxyz_X.Integrator_CSTATE_a = USB_TCrane_PIDxyz_P.Integrator_IC_ki;

  /* InitializeConditions for Memory: '<S4>/Memory' */
  USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_j =
    USB_TCrane_PIDxyz_P.Memory_X0_e;

  /* InitializeConditions for Memory: '<S10>/Memory' */
  USB_TCrane_PIDxyz_DWork.Memory_PreviousInput_m =
    USB_TCrane_PIDxyz_P.Memory_X0_d;
}

void MdlStart(void)
{
  /* Scope Block: '<Root>/Scope' */
  {
    RTWLogSignalInfo rt_ScopeSignalInfo;
    static int_T rt_ScopeSignalWidths[] = { 8 };

    static int_T rt_ScopeSignalNumDimensions[] = { 1 };

    static int_T rt_ScopeSignalDimensions[] = { 8 };

    static const char_T *rt_ScopeSignalLabels[] = { "" };

    static char_T rt_ScopeSignalTitles[] = "";
    static int_T rt_ScopeSignalTitleLengths[] = { 0 };

    static boolean_T rt_ScopeSignalIsVarDims[] = { 0 };

    static int_T rt_ScopeSignalPlotStyles[] = { 1, 0, 1, 0, 1, 0, 1, 1 };

    BuiltInDTypeId dTypes[1] = {
      SS_DOUBLE
    };

    static char_T rt_ScopeBlockName[] = "USB_TCrane_PIDxyz/Scope";
    rt_ScopeSignalInfo.numSignals = 1;
    rt_ScopeSignalInfo.numCols = rt_ScopeSignalWidths;
    rt_ScopeSignalInfo.numDims = rt_ScopeSignalNumDimensions;
    rt_ScopeSignalInfo.dims = rt_ScopeSignalDimensions;
    rt_ScopeSignalInfo.isVarDims = rt_ScopeSignalIsVarDims;
    rt_ScopeSignalInfo.currSigDims = NULL;
    rt_ScopeSignalInfo.dataTypes = dTypes;
    rt_ScopeSignalInfo.complexSignals = NULL;
    rt_ScopeSignalInfo.frameData = NULL;
    rt_ScopeSignalInfo.labels.cptr = rt_ScopeSignalLabels;
    rt_ScopeSignalInfo.titles = rt_ScopeSignalTitles;
    rt_ScopeSignalInfo.titleLengths = rt_ScopeSignalTitleLengths;
    rt_ScopeSignalInfo.plotStyles = rt_ScopeSignalPlotStyles;
    rt_ScopeSignalInfo.blockNames.cptr = NULL;
    rt_ScopeSignalInfo.stateNames.cptr = NULL;
    rt_ScopeSignalInfo.crossMdlRef = NULL;
    rt_ScopeSignalInfo.dataTypeConvert = NULL;
    USB_TCrane_PIDxyz_DWork.Scope_PWORK.LoggedData = rt_CreateStructLogVar(
      USB_TCrane_PIDxyz_M->rtwLogInfo,
      rtmGetTFinal(USB_TCrane_PIDxyz_M),
      USB_TCrane_PIDxyz_M->Timing.stepSize0,
      (&rtmGetErrorStatus(USB_TCrane_PIDxyz_M)),
      "SD",
      1,
      0,
      1,
      0.01,
      &rt_ScopeSignalInfo,
      rt_ScopeBlockName);
    if (USB_TCrane_PIDxyz_DWork.Scope_PWORK.LoggedData == NULL)
      return;
  }

  MdlInitialize();
}

RT_MODEL_USB_TCrane_PIDxyz *USB_TCrane_PIDxyz(void)
{
  USB_TCrane_PIDxyz_initialize(1);
  return USB_TCrane_PIDxyz_M;
}

void MdlTerminate(void)
{
  USB_TCrane_PIDxyz_terminate();
}

/*========================================================================*
 * End of GRT compatible call interface                                   *
 *========================================================================*/
#include <stdio.h>

/* Final time from "Simulation Parameters"   */
real_T finaltime = 100.0;

////////////////////////////////////////////////
//
//  Return compilation date and time
//
char *GetDateAndTime( void )
{
  static char AuxStr[ 128 ];
  sprintf( AuxStr, "%s %s", __DATE__, __TIME__ );
  return( AuxStr );
}
