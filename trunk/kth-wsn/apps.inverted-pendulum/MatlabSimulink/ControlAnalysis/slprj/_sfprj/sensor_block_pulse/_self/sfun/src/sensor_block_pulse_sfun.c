/* Include files */

#include "sensor_block_pulse_sfun.h"
#include "c1_sensor_block_pulse.h"
#include "c2_sensor_block_pulse.h"
#include "c3_sensor_block_pulse.h"
#include "c4_sensor_block_pulse.h"
#include "c5_sensor_block_pulse.h"
#include "c6_sensor_block_pulse.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
int32_T _sfEvent_;
uint32_T _sensor_block_pulseMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void sensor_block_pulse_initializer(void)
{
  _sfEvent_ = CALL_EVENT;
}

void sensor_block_pulse_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_sensor_block_pulse_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_sensor_block_pulse_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_sensor_block_pulse_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4202500840U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2014665614U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2326734559U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2007282736U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(604683477U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(677452846U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3972413636U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(987058571U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c1_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c2_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c3_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c4_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c5_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
          sf_c6_sensor_block_pulse_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1202817889U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2707286010U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(801213713U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(289536825U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3875767634U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2848304718U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3001230660U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(238240916U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_sensor_block_pulse_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  if (nrhs<2 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern mxArray *sf_c1_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c1_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     case 2:
      {
        extern mxArray *sf_c2_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c2_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     case 3:
      {
        extern mxArray *sf_c3_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c3_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     case 4:
      {
        extern mxArray *sf_c4_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c4_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     case 5:
      {
        extern mxArray *sf_c5_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c5_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     case 6:
      {
        extern mxArray *sf_c6_sensor_block_pulse_get_autoinheritance_info(void);
        plhs[0] = sf_c6_sensor_block_pulse_get_autoinheritance_info();
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_sensor_block_pulse_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_sensor_block_pulse_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_sensor_block_pulse_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

void sensor_block_pulse_debug_initialize(void)
{
  _sensor_block_pulseMachineNumber_ = sf_debug_initialize_machine(
    "sensor_block_pulse","sfun",0,6,0,0,0);
  sf_debug_set_machine_event_thresholds(_sensor_block_pulseMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_sensor_block_pulseMachineNumber_,0);
}

void sensor_block_pulse_register_exported_symbols(SimStruct* S)
{
}
