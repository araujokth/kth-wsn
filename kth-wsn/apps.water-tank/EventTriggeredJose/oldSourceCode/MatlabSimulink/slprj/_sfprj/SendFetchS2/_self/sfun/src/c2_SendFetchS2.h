#ifndef __c2_SendFetchS2_h__
#define __c2_SendFetchS2_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_smQvuE1V6fJPmdrAbGjhegG
#define struct_smQvuE1V6fJPmdrAbGjhegG

typedef struct smQvuE1V6fJPmdrAbGjhegG {
  real_T x[3];
} c2_smQvuE1V6fJPmdrAbGjhegG;

#else

typedef struct smQvuE1V6fJPmdrAbGjhegG c2_smQvuE1V6fJPmdrAbGjhegG;

#endif

typedef struct {
  const char *context;
  const char *name;
  const char *dominantType;
  const char *resolved;
  uint32_T fileLength;
  uint32_T fileTime1;
  uint32_T fileTime2;
} c2_ResolvedFunctionInfo;

typedef struct {
  SimStruct *S;
  uint32_T c2_twister_state[625];
  uint32_T c2_v4_state;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  boolean_T c2_doneDoubleBufferReInit;
  boolean_T c2_isStable;
  boolean_T c2_method_not_empty;
  boolean_T c2_twister_state_not_empty;
  boolean_T c2_v4_state_not_empty;
  uint8_T c2_is_active_c2_SendFetchS2;
  uint8_T c2_method;
  ChartInfoStruct chartInfo;
} SFc2_SendFetchS2InstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_SendFetchS2_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_SendFetchS2_get_check_sum(mxArray *plhs[]);
extern void c2_SendFetchS2_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
