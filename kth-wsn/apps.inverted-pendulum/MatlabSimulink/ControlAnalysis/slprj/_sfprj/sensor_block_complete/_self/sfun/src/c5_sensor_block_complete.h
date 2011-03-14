#ifndef __c5_sensor_block_complete_h__
#define __c5_sensor_block_complete_h__

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
} c5_ResolvedFunctionInfo;

typedef struct {
  real_T c5_errors_window[50];
  real_T c5_item;
  real_T c5_pckTotalTrack[50];
  SimStruct *S;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  uint16_T c5_prevErrors;
  uint16_T c5_prevTotalPackets;
  boolean_T c5_doneDoubleBufferReInit;
  boolean_T c5_errors_window_not_empty;
  boolean_T c5_isStable;
  boolean_T c5_item_not_empty;
  boolean_T c5_pckTotalTrack_not_empty;
  boolean_T c5_prevErrors_not_empty;
  boolean_T c5_prevTotalPackets_not_empty;
  uint8_T c5_is_active_c5_sensor_block_complete;
  ChartInfoStruct chartInfo;
} SFc5_sensor_block_completeInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c5_sensor_block_complete_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c5_sensor_block_complete_get_check_sum(mxArray *plhs[]);
extern void c5_sensor_block_complete_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
