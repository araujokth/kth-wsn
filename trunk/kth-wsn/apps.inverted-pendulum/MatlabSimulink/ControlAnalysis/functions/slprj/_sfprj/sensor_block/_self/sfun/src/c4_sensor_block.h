#ifndef __c4_sensor_block_h__
#define __c4_sensor_block_h__

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
} c4_ResolvedFunctionInfo;

typedef struct {
  real_T c4_errors_window[100];
  real_T c4_item;
  real_T c4_pckTotalTrack[100];
  SimStruct *S;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  uint16_T c4_prevTotalPackets;
  boolean_T c4_doneDoubleBufferReInit;
  boolean_T c4_errors_window_not_empty;
  boolean_T c4_isStable;
  boolean_T c4_item_not_empty;
  boolean_T c4_pckTotalTrack_not_empty;
  boolean_T c4_prevTotalPackets_not_empty;
  uint8_T c4_is_active_c4_sensor_block;
  ChartInfoStruct chartInfo;
} SFc4_sensor_blockInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c4_sensor_block_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_sensor_block_get_check_sum(mxArray *plhs[]);
extern void c4_sensor_block_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
