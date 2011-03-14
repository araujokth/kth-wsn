#ifndef __c1_sensor_block_pulse_h__
#define __c1_sensor_block_pulse_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  char *context;
  char *name;
  char *dominantType;
  char *resolved;
  uint32_T fileLength;
  uint32_T fileTime1;
  uint32_T fileTime2;
} c1_ResolvedFunctionInfo;

typedef struct {
  real_T c1_errors_window[50];
  real_T c1_item;
  real_T c1_pckTotalTrack[50];
  SimStruct *S;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  uint16_T c1_prevErrors;
  uint16_T c1_prevTotalPackets;
  boolean_T c1_doneDoubleBufferReInit;
  boolean_T c1_errors_window_not_empty;
  boolean_T c1_isStable;
  boolean_T c1_item_not_empty;
  boolean_T c1_pckTotalTrack_not_empty;
  boolean_T c1_prevErrors_not_empty;
  boolean_T c1_prevTotalPackets_not_empty;
  uint8_T c1_is_active_c1_sensor_block_pulse;
  ChartInfoStruct chartInfo;
} SFc1_sensor_block_pulseInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_sensor_block_pulse_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c1_sensor_block_pulse_get_check_sum(mxArray *plhs[]);
extern void c1_sensor_block_pulse_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
