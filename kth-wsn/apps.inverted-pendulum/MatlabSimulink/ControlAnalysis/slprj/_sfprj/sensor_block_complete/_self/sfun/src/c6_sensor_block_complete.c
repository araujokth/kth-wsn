/* Include files */

#include "blascompat32.h"
#include "sensor_block_complete_sfun.h"
#include "c6_sensor_block_complete.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_complete_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c6_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc6_sensor_block_completeInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c6_sensor_block_complete(void);
static void initialize_params_c6_sensor_block_complete(void);
static void enable_c6_sensor_block_complete(void);
static void disable_c6_sensor_block_complete(void);
static void c6_update_debugger_state_c6_sensor_block_complete(void);
static const mxArray *get_sim_state_c6_sensor_block_complete(void);
static void set_sim_state_c6_sensor_block_complete(const mxArray *c6_st);
static void finalize_c6_sensor_block_complete(void);
static void sf_c6_sensor_block_complete(void);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static real_T c6_sum(real_T c6_x[3]);
static void c6_eml_warning(void);
static const mxArray *c6_sf_marshall(void *c6_chartInstance, void *c6_u);
static const mxArray *c6_b_sf_marshall(void *c6_chartInstance, void *c6_u);
static const mxArray *c6_c_sf_marshall(void *c6_chartInstance, void *c6_u);
static const mxArray *c6_d_sf_marshall(void *c6_chartInstance, void *c6_u);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[35]);
static const mxArray *c6_e_sf_marshall(void *c6_chartInstance, void *c6_u);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c6_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c6_checkLinks_not_empty = false;
  chartInstance.c6_idx_not_empty = false;
  chartInstance.c6_is_active_c6_sensor_block_complete = 0U;
}

static void initialize_params_c6_sensor_block_complete(void)
{
}

static void enable_c6_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c6_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c6_update_debugger_state_c6_sensor_block_complete(void)
{
}

static const mxArray *get_sim_state_c6_sensor_block_complete(void)
{
  const mxArray *c6_st = NULL;
  const mxArray *c6_y = NULL;
  real_T c6_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_d_y = NULL;
  int32_T c6_i0;
  real_T c6_d_u[3];
  const mxArray *c6_e_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_f_y = NULL;
  uint8_T c6_f_u;
  const mxArray *c6_g_y = NULL;
  real_T *c6_delay;
  real_T *c6_errors;
  real_T *c6_reliability;
  c6_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c6_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c6_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(6));
  c6_u = *c6_delay;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_b_u = *c6_errors;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c6_y, 1, c6_c_y);
  c6_c_u = *c6_reliability;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c6_y, 2, c6_d_y);
  for (c6_i0 = 0; c6_i0 < 3; c6_i0 = c6_i0 + 1) {
    c6_d_u[c6_i0] = chartInstance.c6_checkLinks[c6_i0];
  }

  c6_e_y = NULL;
  if (!chartInstance.c6_checkLinks_not_empty) {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_d_u, 0, 0U, 1U, 0U, 1, 3));
  }

  sf_mex_setcell(c6_y, 3, c6_e_y);
  c6_e_u = chartInstance.c6_idx;
  c6_f_y = NULL;
  if (!chartInstance.c6_idx_not_empty) {
    sf_mex_assign(&c6_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c6_y, 4, c6_f_y);
  c6_f_u = chartInstance.c6_is_active_c6_sensor_block_complete;
  c6_g_y = NULL;
  sf_mex_assign(&c6_g_y, sf_mex_create("y", &c6_f_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c6_y, 5, c6_g_y);
  sf_mex_assign(&c6_st, c6_y);
  return c6_st;
}

static void set_sim_state_c6_sensor_block_complete(const mxArray *c6_st)
{
  const mxArray *c6_u;
  const mxArray *c6_delay;
  real_T c6_d0;
  real_T c6_y;
  const mxArray *c6_b_delay;
  real_T c6_d1;
  real_T c6_b_y;
  const mxArray *c6_c_delay;
  real_T c6_d2;
  real_T c6_c_y;
  const mxArray *c6_b_checkLinks;
  real_T c6_dv0[3];
  int32_T c6_i1;
  real_T c6_d_y[3];
  int32_T c6_i2;
  const mxArray *c6_b_idx;
  real_T c6_d3;
  real_T c6_e_y;
  const mxArray *c6_b_is_active_c6_sensor_block_complete;
  uint8_T c6_u0;
  uint8_T c6_f_y;
  real_T *c6_d_delay;
  real_T *c6_errors;
  real_T *c6_reliability;
  c6_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c6_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c6_d_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  chartInstance.c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  c6_delay = sf_mex_dup(sf_mex_getcell(c6_u, 0));
  sf_mex_import("delay", sf_mex_dup(c6_delay), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_delay);
  *c6_d_delay = c6_y;
  c6_b_delay = sf_mex_dup(sf_mex_getcell(c6_u, 1));
  sf_mex_import("errors", sf_mex_dup(c6_b_delay), &c6_d1, 1, 0, 0U, 0, 0U, 0);
  c6_b_y = c6_d1;
  sf_mex_destroy(&c6_b_delay);
  *c6_errors = c6_b_y;
  c6_c_delay = sf_mex_dup(sf_mex_getcell(c6_u, 2));
  sf_mex_import("reliability", sf_mex_dup(c6_c_delay), &c6_d2, 1, 0, 0U, 0, 0U,
                0);
  c6_c_y = c6_d2;
  sf_mex_destroy(&c6_c_delay);
  *c6_reliability = c6_c_y;
  c6_b_checkLinks = sf_mex_dup(sf_mex_getcell(c6_u, 3));
  if (mxIsEmpty(c6_b_checkLinks)) {
    chartInstance.c6_checkLinks_not_empty = false;
  } else {
    chartInstance.c6_checkLinks_not_empty = true;
    sf_mex_import("checkLinks", sf_mex_dup(c6_b_checkLinks), &c6_dv0, 1, 0, 0U,
                  1, 0U, 1, 3);
    for (c6_i1 = 0; c6_i1 < 3; c6_i1 = c6_i1 + 1) {
      c6_d_y[c6_i1] = c6_dv0[c6_i1];
    }
  }

  sf_mex_destroy(&c6_b_checkLinks);
  for (c6_i2 = 0; c6_i2 < 3; c6_i2 = c6_i2 + 1) {
    chartInstance.c6_checkLinks[c6_i2] = c6_d_y[c6_i2];
  }

  c6_b_idx = sf_mex_dup(sf_mex_getcell(c6_u, 4));
  if (mxIsEmpty(c6_b_idx)) {
    chartInstance.c6_idx_not_empty = false;
  } else {
    chartInstance.c6_idx_not_empty = true;
    sf_mex_import("idx", sf_mex_dup(c6_b_idx), &c6_d3, 1, 0, 0U, 0, 0U, 0);
    c6_e_y = c6_d3;
  }

  sf_mex_destroy(&c6_b_idx);
  chartInstance.c6_idx = c6_e_y;
  c6_b_is_active_c6_sensor_block_complete = sf_mex_dup(sf_mex_getcell(c6_u, 5));
  sf_mex_import("is_active_c6_sensor_block_complete", sf_mex_dup
                (c6_b_is_active_c6_sensor_block_complete), &c6_u0, 1, 3, 0U, 0,
                0U, 0);
  c6_f_y = c6_u0;
  sf_mex_destroy(&c6_b_is_active_c6_sensor_block_complete);
  chartInstance.c6_is_active_c6_sensor_block_complete = c6_f_y;
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_sensor_block_complete();
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_sensor_block_complete(void)
{
}

static void sf_c6_sensor_block_complete(void)
{
  int32_T c6_i3;
  int32_T c6_previousEvent;
  real_T c6_xc_errors;
  real_T c6_xc_delay;
  real_T c6_xc_reliability;
  real_T c6_theta_errors;
  real_T c6_theta_delay;
  real_T c6_theta_reliability;
  real_T c6_actuator_errors;
  real_T c6_actuator_delay;
  real_T c6_actuator_reliability;
  int32_T c6_i4;
  boolean_T c6_update[3];
  real_T c6_nargout = 3.0;
  real_T c6_nargin = 10.0;
  real_T c6_i;
  real_T c6_LEARNING_WINDOW;
  real_T c6_delay;
  real_T c6_reliability;
  real_T c6_errors;
  int32_T c6_i5;
  real_T c6_b_i;
  real_T c6_A;
  int32_T c6_i6;
  real_T c6_dv1[3];
  real_T c6_B;
  real_T c6_x;
  real_T c6_y;
  real_T c6_b_x;
  real_T c6_b_y;
  real_T c6_c_x;
  real_T c6_c_y;
  real_T c6_b_A;
  int32_T c6_i7;
  real_T c6_dv2[3];
  real_T c6_b_B;
  real_T c6_d_x;
  real_T c6_d_y;
  real_T c6_e_x;
  real_T c6_e_y;
  real_T c6_f_x;
  real_T c6_f_y;
  real_T *c6_b_xc_errors;
  real_T *c6_b_xc_delay;
  real_T *c6_b_xc_reliability;
  real_T *c6_b_errors;
  real_T *c6_b_reliability;
  real_T *c6_b_theta_errors;
  real_T *c6_b_theta_delay;
  real_T *c6_b_theta_reliability;
  real_T *c6_b_actuator_errors;
  real_T *c6_b_actuator_delay;
  real_T *c6_b_actuator_reliability;
  real_T *c6_b_delay;
  boolean_T (*c6_b_update)[3];
  c6_b_theta_reliability = (real_T *)ssGetInputPortSignal(chartInstance.S, 5);
  c6_b_xc_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 1);
  c6_b_actuator_reliability = (real_T *)ssGetInputPortSignal(chartInstance.S, 8);
  c6_b_theta_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 4);
  c6_b_theta_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 3);
  c6_b_actuator_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 6);
  c6_b_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c6_b_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c6_b_xc_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c6_b_xc_reliability = (real_T *)ssGetInputPortSignal(chartInstance.S, 2);
  c6_b_actuator_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 7);
  c6_b_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c6_b_update = (boolean_T (*)[3])ssGetInputPortSignal(chartInstance.S, 9);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,5);
  _SFD_DATA_RANGE_CHECK(*c6_b_xc_errors, 0U);
  _SFD_DATA_RANGE_CHECK(*c6_b_xc_delay, 1U);
  _SFD_DATA_RANGE_CHECK(*c6_b_xc_reliability, 2U);
  _SFD_DATA_RANGE_CHECK(*c6_b_errors, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_b_reliability, 4U);
  _SFD_DATA_RANGE_CHECK(*c6_b_theta_errors, 5U);
  _SFD_DATA_RANGE_CHECK(*c6_b_theta_delay, 6U);
  _SFD_DATA_RANGE_CHECK(*c6_b_theta_reliability, 7U);
  _SFD_DATA_RANGE_CHECK(*c6_b_actuator_errors, 8U);
  _SFD_DATA_RANGE_CHECK(*c6_b_actuator_delay, 9U);
  _SFD_DATA_RANGE_CHECK(*c6_b_actuator_reliability, 10U);
  _SFD_DATA_RANGE_CHECK(*c6_b_delay, 11U);
  for (c6_i3 = 0; c6_i3 < 3; c6_i3 = c6_i3 + 1) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c6_b_update)[c6_i3], 12U);
  }

  c6_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,5);
  c6_xc_errors = *c6_b_xc_errors;
  c6_xc_delay = *c6_b_xc_delay;
  c6_xc_reliability = *c6_b_xc_reliability;
  c6_theta_errors = *c6_b_theta_errors;
  c6_theta_delay = *c6_b_theta_delay;
  c6_theta_reliability = *c6_b_theta_reliability;
  c6_actuator_errors = *c6_b_actuator_errors;
  c6_actuator_delay = *c6_b_actuator_delay;
  c6_actuator_reliability = *c6_b_actuator_reliability;
  for (c6_i4 = 0; c6_i4 < 3; c6_i4 = c6_i4 + 1) {
    c6_update[c6_i4] = (*c6_b_update)[c6_i4];
  }

  sf_debug_symbol_scope_push(19U, 0U);
  sf_debug_symbol_scope_add("idx", &chartInstance.c6_idx, c6_d_sf_marshall);
  sf_debug_symbol_scope_add("checkLinks", &chartInstance.c6_checkLinks,
    c6_c_sf_marshall);
  sf_debug_symbol_scope_add("nargout", &c6_nargout, c6_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c6_nargin, c6_sf_marshall);
  sf_debug_symbol_scope_add("i", &c6_i, c6_sf_marshall);
  sf_debug_symbol_scope_add("LEARNING_WINDOW", &c6_LEARNING_WINDOW,
    c6_sf_marshall);
  sf_debug_symbol_scope_add("delay", &c6_delay, c6_sf_marshall);
  sf_debug_symbol_scope_add("reliability", &c6_reliability, c6_sf_marshall);
  sf_debug_symbol_scope_add("errors", &c6_errors, c6_sf_marshall);
  sf_debug_symbol_scope_add("update", &c6_update, c6_b_sf_marshall);
  sf_debug_symbol_scope_add("actuator_reliability", &c6_actuator_reliability,
    c6_sf_marshall);
  sf_debug_symbol_scope_add("actuator_delay", &c6_actuator_delay, c6_sf_marshall);
  sf_debug_symbol_scope_add("actuator_errors", &c6_actuator_errors,
    c6_sf_marshall);
  sf_debug_symbol_scope_add("theta_reliability", &c6_theta_reliability,
    c6_sf_marshall);
  sf_debug_symbol_scope_add("theta_delay", &c6_theta_delay, c6_sf_marshall);
  sf_debug_symbol_scope_add("theta_errors", &c6_theta_errors, c6_sf_marshall);
  sf_debug_symbol_scope_add("xc_reliability", &c6_xc_reliability, c6_sf_marshall);
  sf_debug_symbol_scope_add("xc_delay", &c6_xc_delay, c6_sf_marshall);
  sf_debug_symbol_scope_add("xc_errors", &c6_xc_errors, c6_sf_marshall);
  CV_EML_FCN(0, 0);

  /*   */
  _SFD_EML_CALL(0,8);
  c6_LEARNING_WINDOW = 50.0;

  /* % detect how many links do we  */
  _SFD_EML_CALL(0,13);
  if (CV_EML_IF(0, 0, !chartInstance.c6_checkLinks_not_empty)) {
    _SFD_EML_CALL(0,14);
    for (c6_i5 = 0; c6_i5 < 3; c6_i5 = c6_i5 + 1) {
      chartInstance.c6_checkLinks[c6_i5] = 0.0;
    }

    chartInstance.c6_checkLinks_not_empty = true;
    _SFD_EML_CALL(0,15);
    chartInstance.c6_idx = 1.0;
    chartInstance.c6_idx_not_empty = true;
  }

  _SFD_EML_CALL(0,17);
  if (CV_EML_IF(0, 1, chartInstance.c6_idx < c6_LEARNING_WINDOW)) {
    c6_i = 1.0;
    c6_b_i = 1.0;
    while (c6_b_i <= 3.0) {
      c6_i = c6_b_i;
      CV_EML_FOR(0, 0, 1);
      _SFD_EML_CALL(0,19);
      if (CV_EML_IF(0, 2, (real_T)c6_update[_SFD_EML_ARRAY_BOUNDS_CHECK("update",
            (int32_T)_SFD_INTEGER_CHECK("i", c6_i), 1, 3, 1, 0) - 1]
                    == 1.0)) {
        _SFD_EML_CALL(0,20);
        chartInstance.c6_checkLinks[_SFD_EML_ARRAY_BOUNDS_CHECK("checkLinks",
          (int32_T)_SFD_INTEGER_CHECK("i", c6_i), 1, 3, 1, 0) - 1] = 1.0;
      }

      c6_b_i = c6_b_i + 1.0;
      sf_mex_listen_for_ctrl_c(chartInstance.S);
    }

    CV_EML_FOR(0, 0, 0);
    _SFD_EML_CALL(0,23);
    chartInstance.c6_idx = chartInstance.c6_idx + 1.0;
  }

  _SFD_EML_CALL(0,26);
  c6_errors = (c6_xc_errors + c6_theta_errors) + c6_actuator_errors;
  _SFD_EML_CALL(0,27);
  c6_A = (c6_xc_reliability + c6_theta_reliability) + c6_actuator_reliability;
  for (c6_i6 = 0; c6_i6 < 3; c6_i6 = c6_i6 + 1) {
    c6_dv1[c6_i6] = chartInstance.c6_checkLinks[c6_i6];
  }

  c6_B = c6_sum(c6_dv1);
  c6_x = c6_A;
  c6_y = c6_B;
  if (c6_y == 0.0) {
    c6_eml_warning();
  }

  c6_b_x = c6_x;
  c6_b_y = c6_y;
  c6_c_x = c6_b_x;
  c6_c_y = c6_b_y;
  c6_reliability = c6_c_x / c6_c_y;
  _SFD_EML_CALL(0,28);
  c6_b_A = (c6_xc_delay + c6_theta_delay) + c6_actuator_delay;
  for (c6_i7 = 0; c6_i7 < 3; c6_i7 = c6_i7 + 1) {
    c6_dv2[c6_i7] = chartInstance.c6_checkLinks[c6_i7];
  }

  c6_b_B = c6_sum(c6_dv2);
  c6_d_x = c6_b_A;
  c6_d_y = c6_b_B;
  if (c6_d_y == 0.0) {
    c6_eml_warning();
  }

  c6_e_x = c6_d_x;
  c6_e_y = c6_d_y;
  c6_f_x = c6_e_x;
  c6_f_y = c6_e_y;
  c6_delay = c6_f_x / c6_f_y;
  _SFD_EML_CALL(0,-28);
  sf_debug_symbol_scope_pop();
  *c6_b_errors = c6_errors;
  *c6_b_reliability = c6_reliability;
  *c6_b_delay = c6_delay;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,5);
  _sfEvent_ = c6_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_block_completeMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber
    );
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
}

static real_T c6_sum(real_T c6_x[3])
{
  real_T c6_y;
  int32_T c6_i8;
  real_T c6_b_x[3];
  real_T c6_k;
  real_T c6_b_k;
  for (c6_i8 = 0; c6_i8 < 3; c6_i8 = c6_i8 + 1) {
    c6_b_x[c6_i8] = c6_x[c6_i8];
  }

  c6_y = c6_b_x[0];
  for (c6_k = 2.0; c6_k <= 3.0; c6_k = c6_k + 1.0) {
    c6_b_k = c6_k;
    c6_y = c6_y + c6_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("x", (int32_T)
      _SFD_INTEGER_CHECK("k", c6_b_k), 1, 3, 1, 0) - 1];
  }

  return c6_y;
}

static void c6_eml_warning(void)
{
  int32_T c6_i9;
  static char_T c6_cv0[15] = { 'D', 'i', 'v', 'i', 'd', 'e', ' ', 'b', 'y', ' ',
    'z', 'e', 'r', 'o', '.' };

  char_T c6_u[15];
  const mxArray *c6_y = NULL;
  int32_T c6_i10;
  static char_T c6_cv1[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'd', 'i', 'v',
    'i', 'd', 'e', 'B', 'y', 'Z', 'e', 'r', 'o' };

  char_T c6_b_u[19];
  const mxArray *c6_b_y = NULL;
  for (c6_i9 = 0; c6_i9 < 15; c6_i9 = c6_i9 + 1) {
    c6_u[c6_i9] = c6_cv0[c6_i9];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 10, 0U, 1U, 0U, 2, 1, 15));
  for (c6_i10 = 0; c6_i10 < 19; c6_i10 = c6_i10 + 1) {
    c6_b_u[c6_i10] = c6_cv1[c6_i10];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 10, 0U, 1U, 0U, 2, 1, 19));
  sf_mex_call_debug("warning", 0U, 2U, 14, c6_b_y, 14, c6_y);
}

static const mxArray *c6_sf_marshall(void *c6_chartInstance, void *c6_u)
{
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  c6_y = NULL;
  c6_b_u = *((real_T *)c6_u);
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c6_y, c6_b_y);
  return c6_y;
}

static const mxArray *c6_b_sf_marshall(void *c6_chartInstance, void *c6_u)
{
  const mxArray *c6_y = NULL;
  int32_T c6_i11;
  boolean_T c6_b_u[3];
  const mxArray *c6_b_y = NULL;
  c6_y = NULL;
  for (c6_i11 = 0; c6_i11 < 3; c6_i11 = c6_i11 + 1) {
    c6_b_u[c6_i11] = (*((boolean_T (*)[3])c6_u))[c6_i11];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 11, 0U, 1U, 0U, 1, 3));
  sf_mex_assign(&c6_y, c6_b_y);
  return c6_y;
}

static const mxArray *c6_c_sf_marshall(void *c6_chartInstance, void *c6_u)
{
  const mxArray *c6_y = NULL;
  int32_T c6_i12;
  real_T c6_b_u[3];
  const mxArray *c6_b_y = NULL;
  c6_y = NULL;
  for (c6_i12 = 0; c6_i12 < 3; c6_i12 = c6_i12 + 1) {
    c6_b_u[c6_i12] = (*((real_T (*)[3])c6_u))[c6_i12];
  }

  c6_b_y = NULL;
  if (!chartInstance.c6_checkLinks_not_empty) {
    sf_mex_assign(&c6_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 1U, 0U, 1, 3));
  }

  sf_mex_assign(&c6_y, c6_b_y);
  return c6_y;
}

static const mxArray *c6_d_sf_marshall(void *c6_chartInstance, void *c6_u)
{
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  c6_y = NULL;
  c6_b_u = *((real_T *)c6_u);
  c6_b_y = NULL;
  if (!chartInstance.c6_idx_not_empty) {
    sf_mex_assign(&c6_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c6_y, c6_b_y);
  return c6_y;
}

const mxArray *sf_c6_sensor_block_complete_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_ResolvedFunctionInfo c6_info[35];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i13;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 35));
  for (c6_i13 = 0; c6_i13 < 35; c6_i13 = c6_i13 + 1) {
    c6_r0 = &c6_info[c6_i13];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context",
                    "nameCaptureInfo", c6_i13);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name",
                    "nameCaptureInfo", c6_i13);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c6_i13);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c6_i13);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c6_i13);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c6_i13
                    );
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c6_i13
                    );
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[35])
{
  c6_info[0].context = "";
  c6_info[0].name = "length";
  c6_info[0].dominantType = "logical";
  c6_info[0].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[0].fileLength = 326U;
  c6_info[0].fileTime1 = 1226598875U;
  c6_info[0].fileTime2 = 0U;
  c6_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[1].name = "nargin";
  c6_info[1].dominantType = "";
  c6_info[1].resolved = "[B]nargin";
  c6_info[1].fileLength = 0U;
  c6_info[1].fileTime1 = 0U;
  c6_info[1].fileTime2 = 0U;
  c6_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[2].name = "eq";
  c6_info[2].dominantType = "double";
  c6_info[2].resolved = "[B]eq";
  c6_info[2].fileLength = 0U;
  c6_info[2].fileTime1 = 0U;
  c6_info[2].fileTime2 = 0U;
  c6_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[3].name = "size";
  c6_info[3].dominantType = "logical";
  c6_info[3].resolved = "[B]size";
  c6_info[3].fileLength = 0U;
  c6_info[3].fileTime1 = 0U;
  c6_info[3].fileTime2 = 0U;
  c6_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[4].name = "gt";
  c6_info[4].dominantType = "double";
  c6_info[4].resolved = "[B]gt";
  c6_info[4].fileLength = 0U;
  c6_info[4].fileTime1 = 0U;
  c6_info[4].fileTime2 = 0U;
  c6_info[5].context = "";
  c6_info[5].name = "zeros";
  c6_info[5].dominantType = "double";
  c6_info[5].resolved = "[B]zeros";
  c6_info[5].fileLength = 0U;
  c6_info[5].fileTime1 = 0U;
  c6_info[5].fileTime2 = 0U;
  c6_info[6].context = "";
  c6_info[6].name = "lt";
  c6_info[6].dominantType = "double";
  c6_info[6].resolved = "[B]lt";
  c6_info[6].fileLength = 0U;
  c6_info[6].fileTime1 = 0U;
  c6_info[6].fileTime2 = 0U;
  c6_info[7].context = "";
  c6_info[7].name = "plus";
  c6_info[7].dominantType = "double";
  c6_info[7].resolved = "[B]plus";
  c6_info[7].fileLength = 0U;
  c6_info[7].fileTime1 = 0U;
  c6_info[7].fileTime2 = 0U;
  c6_info[8].context = "";
  c6_info[8].name = "sum";
  c6_info[8].dominantType = "double";
  c6_info[8].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[8].fileLength = 3965U;
  c6_info[8].fileTime1 = 1240283566U;
  c6_info[8].fileTime2 = 0U;
  c6_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[9].name = "isa";
  c6_info[9].dominantType = "double";
  c6_info[9].resolved = "[B]isa";
  c6_info[9].fileLength = 0U;
  c6_info[9].fileTime1 = 0U;
  c6_info[9].fileTime2 = 0U;
  c6_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[10].name = "isequal";
  c6_info[10].dominantType = "double";
  c6_info[10].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[10].fileLength = 180U;
  c6_info[10].fileTime1 = 1226598871U;
  c6_info[10].fileTime2 = 0U;
  c6_info[11].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[11].name = "false";
  c6_info[11].dominantType = "";
  c6_info[11].resolved = "[B]false";
  c6_info[11].fileLength = 0U;
  c6_info[11].fileTime1 = 0U;
  c6_info[11].fileTime2 = 0U;
  c6_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c6_info[12].name = "eml_isequal_core";
  c6_info[12].dominantType = "double";
  c6_info[12].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[12].fileLength = 3981U;
  c6_info[12].fileTime1 = 1236278872U;
  c6_info[12].fileTime2 = 0U;
  c6_info[13].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[13].name = "ge";
  c6_info[13].dominantType = "double";
  c6_info[13].resolved = "[B]ge";
  c6_info[13].fileLength = 0U;
  c6_info[13].fileTime1 = 0U;
  c6_info[13].fileTime2 = 0U;
  c6_info[14].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[14].name = "isscalar";
  c6_info[14].dominantType = "logical";
  c6_info[14].resolved = "[B]isscalar";
  c6_info[14].fileLength = 0U;
  c6_info[14].fileTime1 = 0U;
  c6_info[14].fileTime2 = 0U;
  c6_info[15].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[15].name = "islogical";
  c6_info[15].dominantType = "logical";
  c6_info[15].resolved = "[B]islogical";
  c6_info[15].fileLength = 0U;
  c6_info[15].fileTime1 = 0U;
  c6_info[15].fileTime2 = 0U;
  c6_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[16].name = "minus";
  c6_info[16].dominantType = "double";
  c6_info[16].resolved = "[B]minus";
  c6_info[16].fileLength = 0U;
  c6_info[16].fileTime1 = 0U;
  c6_info[16].fileTime2 = 0U;
  c6_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[17].name = "isnumeric";
  c6_info[17].dominantType = "double";
  c6_info[17].resolved = "[B]isnumeric";
  c6_info[17].fileLength = 0U;
  c6_info[17].fileTime1 = 0U;
  c6_info[17].fileTime2 = 0U;
  c6_info[18].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c6_info[18].name = "ndims";
  c6_info[18].dominantType = "double";
  c6_info[18].resolved = "[B]ndims";
  c6_info[18].fileLength = 0U;
  c6_info[18].fileTime1 = 0U;
  c6_info[18].fileTime2 = 0U;
  c6_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c6_info[19].name = "ne";
  c6_info[19].dominantType = "double";
  c6_info[19].resolved = "[B]ne";
  c6_info[19].fileLength = 0U;
  c6_info[19].fileTime1 = 0U;
  c6_info[19].fileTime2 = 0U;
  c6_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c6_info[20].name = "true";
  c6_info[20].dominantType = "";
  c6_info[20].resolved = "[B]true";
  c6_info[20].fileLength = 0U;
  c6_info[20].fileTime1 = 0U;
  c6_info[20].fileTime2 = 0U;
  c6_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c6_info[21].name = "not";
  c6_info[21].dominantType = "logical";
  c6_info[21].resolved = "[B]not";
  c6_info[21].fileLength = 0U;
  c6_info[21].fileTime1 = 0U;
  c6_info[21].fileTime2 = 0U;
  c6_info[22].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c6_info[22].name = "eml_const_nonsingleton_dim";
  c6_info[22].dominantType = "double";
  c6_info[22].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c6_info[22].fileLength = 1473U;
  c6_info[22].fileTime1 = 1240283603U;
  c6_info[22].fileTime2 = 0U;
  c6_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m/sumwork";
  c6_info[23].name = "cast";
  c6_info[23].dominantType = "double";
  c6_info[23].resolved = "[B]cast";
  c6_info[23].fileLength = 0U;
  c6_info[23].fileTime1 = 0U;
  c6_info[23].fileTime2 = 0U;
  c6_info[24].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m/sumwork";
  c6_info[24].name = "eml_scalar_eg";
  c6_info[24].dominantType = "double";
  c6_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[24].fileLength = 3068U;
  c6_info[24].fileTime1 = 1240283610U;
  c6_info[24].fileTime2 = 0U;
  c6_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[25].name = "isstruct";
  c6_info[25].dominantType = "double";
  c6_info[25].resolved = "[B]isstruct";
  c6_info[25].fileLength = 0U;
  c6_info[25].fileTime1 = 0U;
  c6_info[25].fileTime2 = 0U;
  c6_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c6_info[26].name = "class";
  c6_info[26].dominantType = "double";
  c6_info[26].resolved = "[B]class";
  c6_info[26].fileLength = 0U;
  c6_info[26].fileTime1 = 0U;
  c6_info[26].fileTime2 = 0U;
  c6_info[27].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/allreal";
  c6_info[27].name = "isreal";
  c6_info[27].dominantType = "double";
  c6_info[27].resolved = "[B]isreal";
  c6_info[27].fileLength = 0U;
  c6_info[27].fileTime1 = 0U;
  c6_info[27].fileTime2 = 0U;
  c6_info[28].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m/sumwork";
  c6_info[28].name = "isempty";
  c6_info[28].dominantType = "double";
  c6_info[28].resolved = "[B]isempty";
  c6_info[28].fileLength = 0U;
  c6_info[28].fileTime1 = 0U;
  c6_info[28].fileTime2 = 0U;
  c6_info[29].context = "";
  c6_info[29].name = "mrdivide";
  c6_info[29].dominantType = "double";
  c6_info[29].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c6_info[29].fileLength = 800U;
  c6_info[29].fileTime1 = 1238455891U;
  c6_info[29].fileTime2 = 0U;
  c6_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c6_info[30].name = "rdivide";
  c6_info[30].dominantType = "double";
  c6_info[30].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[30].fileLength = 620U;
  c6_info[30].fileTime1 = 1213948366U;
  c6_info[30].fileTime2 = 0U;
  c6_info[31].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[31].name = "strcmp";
  c6_info[31].dominantType = "char";
  c6_info[31].resolved = "[B]strcmp";
  c6_info[31].fileLength = 0U;
  c6_info[31].fileTime1 = 0U;
  c6_info[31].fileTime2 = 0U;
  c6_info[32].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[32].name = "eml_warning";
  c6_info[32].dominantType = "char";
  c6_info[32].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[32].fileLength = 262U;
  c6_info[32].fileTime1 = 1236278878U;
  c6_info[32].fileTime2 = 0U;
  c6_info[33].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[33].name = "eml_div";
  c6_info[33].dominantType = "double";
  c6_info[33].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[33].fileLength = 4269U;
  c6_info[33].fileTime1 = 1228115426U;
  c6_info[33].fileTime2 = 0U;
  c6_info[34].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[34].name = "isinteger";
  c6_info[34].dominantType = "double";
  c6_info[34].resolved = "[B]isinteger";
  c6_info[34].fileLength = 0U;
  c6_info[34].fileTime1 = 0U;
  c6_info[34].fileTime2 = 0U;
}

static const mxArray *c6_e_sf_marshall(void *c6_chartInstance, void *c6_u)
{
  const mxArray *c6_y = NULL;
  boolean_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  c6_y = NULL;
  c6_b_u = *((boolean_T *)c6_u);
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c6_y, c6_b_y);
  return c6_y;
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c6_sensor_block_complete_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2492325598U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(173232413U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3959453165U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2709635817U);
}

mxArray *sf_c6_sensor_block_complete_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(642279922U);
    pr[1] = (double)(100038741U);
    pr[2] = (double)(1023814575U);
    pr[3] = (double)(2342274758U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,10,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c6_sensor_block_complete(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[32],T\"delay\",},{M[1],M[10],T\"errors\",},{M[1],M[11],T\"reliability\",},{M[4],M[0],T\"checkLinks\",S'l','i','p'{{M1x2[269 279],M[0],}}},{M[4],M[0],T\"idx\",S'l','i','p'{{M1x2[280 283],M[0],}}},{M[8],M[0],T\"is_active_c6_sensor_block_complete\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_sensor_block_complete_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_sensor_block_completeMachineNumber_,
           6,
           1,
           1,
           13,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance.chartNumber),
           &(chartInstance.instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_sensor_block_completeMachineNumber_,
            chartInstance.chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_sensor_block_completeMachineNumber_,chartInstance.chartNumber,1);
          sf_debug_set_chart_event_thresholds
            (_sensor_block_completeMachineNumber_,
             chartInstance.chartNumber,
             0,
             0,
             0);
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_errors",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(1,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_delay",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_reliability",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(3,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"errors",
                              0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(4,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "reliability",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(5,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_errors",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(6,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_delay",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(7,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_reliability",0,(MexFcnForType)
                              c6_sf_marshall);
          _SFD_SET_DATA_PROPS(8,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_errors",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(9,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_delay",0,(MexFcnForType)c6_sf_marshall);
          _SFD_SET_DATA_PROPS(10,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_reliability",0,(MexFcnForType)
                              c6_sf_marshall);
          _SFD_SET_DATA_PROPS(11,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"delay",
                              0,(MexFcnForType)c6_sf_marshall);

          {
            unsigned int dimVector[1];
            dimVector[0]= 3;
            _SFD_SET_DATA_PROPS(12,1,1,0,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"update",0,(MexFcnForType)c6_b_sf_marshall);
          }

          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of EML Model Coverage */
        _SFD_CV_INIT_EML(0,1,3,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,756);
        _SFD_CV_INIT_EML_IF(0,0,318,340,-1,401);
        _SFD_CV_INIT_EML_IF(0,1,402,426,-1,544);
        _SFD_CV_INIT_EML_IF(0,2,462,479,-1,518);
        _SFD_CV_INIT_EML_FOR(0,0,429,456,524);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T *c6_xc_errors;
          real_T *c6_xc_delay;
          real_T *c6_xc_reliability;
          real_T *c6_errors;
          real_T *c6_reliability;
          real_T *c6_theta_errors;
          real_T *c6_theta_delay;
          real_T *c6_theta_reliability;
          real_T *c6_actuator_errors;
          real_T *c6_actuator_delay;
          real_T *c6_actuator_reliability;
          real_T *c6_delay;
          boolean_T (*c6_update)[3];
          c6_theta_reliability = (real_T *)ssGetInputPortSignal(chartInstance.S,
            5);
          c6_xc_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 1);
          c6_actuator_reliability = (real_T *)ssGetInputPortSignal
            (chartInstance.S, 8);
          c6_theta_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 4);
          c6_theta_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 3);
          c6_actuator_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 6);
          c6_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          c6_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          c6_xc_errors = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
          c6_xc_reliability = (real_T *)ssGetInputPortSignal(chartInstance.S, 2);
          c6_actuator_delay = (real_T *)ssGetInputPortSignal(chartInstance.S, 7);
          c6_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
          c6_update = (boolean_T (*)[3])ssGetInputPortSignal(chartInstance.S, 9);
          _SFD_SET_DATA_VALUE_PTR(0U, c6_xc_errors);
          _SFD_SET_DATA_VALUE_PTR(1U, c6_xc_delay);
          _SFD_SET_DATA_VALUE_PTR(2U, c6_xc_reliability);
          _SFD_SET_DATA_VALUE_PTR(3U, c6_errors);
          _SFD_SET_DATA_VALUE_PTR(4U, c6_reliability);
          _SFD_SET_DATA_VALUE_PTR(5U, c6_theta_errors);
          _SFD_SET_DATA_VALUE_PTR(6U, c6_theta_delay);
          _SFD_SET_DATA_VALUE_PTR(7U, c6_theta_reliability);
          _SFD_SET_DATA_VALUE_PTR(8U, c6_actuator_errors);
          _SFD_SET_DATA_VALUE_PTR(9U, c6_actuator_delay);
          _SFD_SET_DATA_VALUE_PTR(10U, c6_actuator_reliability);
          _SFD_SET_DATA_VALUE_PTR(11U, c6_delay);
          _SFD_SET_DATA_VALUE_PTR(12U, c6_update);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_sensor_block_completeMachineNumber_,chartInstance.chartNumber,
         chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c6_sensor_block_complete(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c6_sensor_block_complete();
  initialize_c6_sensor_block_complete();
}

static void sf_opaque_enable_c6_sensor_block_complete(void *chartInstanceVar)
{
  enable_c6_sensor_block_complete();
}

static void sf_opaque_disable_c6_sensor_block_complete(void *chartInstanceVar)
{
  disable_c6_sensor_block_complete();
}

static void sf_opaque_gateway_c6_sensor_block_complete(void *chartInstanceVar)
{
  sf_c6_sensor_block_complete();
}

static mxArray* sf_opaque_get_sim_state_c6_sensor_block_complete(void
  *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c6_sensor_block_complete();
  return st;
}

static void sf_opaque_set_sim_state_c6_sensor_block_complete(void
  *chartInstanceVar, const mxArray *st)
{
  set_sim_state_c6_sensor_block_complete(sf_mex_dup(st));
}

static void sf_opaque_terminate_c6_sensor_block_complete(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c6_sensor_block_complete();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_sensor_block_complete(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_sensor_block_complete();
  }
}

static void mdlSetWorkWidths_c6_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block_complete",
      "sensor_block_complete",6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block_complete",
                "sensor_block_complete",6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block_complete",
      "sensor_block_complete",6,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block_complete",
        "sensor_block_complete",6,10);
      sf_mark_chart_reusable_outputs(S,"sensor_block_complete",
        "sensor_block_complete",6,3);
    }

    sf_set_rtw_dwork_info(S,"sensor_block_complete","sensor_block_complete",6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(3711451906U));
  ssSetChecksum1(S,(168717876U));
  ssSetChecksum2(S,(3820674990U));
  ssSetChecksum3(S,(1079899775U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c6_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block_complete", "sensor_block_complete",
      6);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_sensor_block_complete(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_sensor_block_complete;
  chartInstance.chartInfo.initializeChart =
    sf_opaque_initialize_c6_sensor_block_complete;
  chartInstance.chartInfo.terminateChart =
    sf_opaque_terminate_c6_sensor_block_complete;
  chartInstance.chartInfo.enableChart =
    sf_opaque_enable_c6_sensor_block_complete;
  chartInstance.chartInfo.disableChart =
    sf_opaque_disable_c6_sensor_block_complete;
  chartInstance.chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_sensor_block_complete;
  chartInstance.chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_sensor_block_complete;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_sensor_block_complete;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c6_sensor_block_complete;
  chartInstance.chartInfo.mdlStart = mdlStart_c6_sensor_block_complete;
  chartInstance.chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_sensor_block_complete;
  chartInstance.chartInfo.extModeExec = NULL;
  chartInstance.chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance.chartInfo.storeCurrentConfiguration = NULL;
  chartInstance.S = S;
  ssSetUserData(S,(void *)(&(chartInstance.chartInfo)));/* register the chart instance with simstruct */
  if (!sim_mode_is_rtw_gen(S)) {
    init_dsm_address_info();
  }

  chart_debug_initialization(S,1);
}

void c6_sensor_block_complete_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_sensor_block_complete(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_sensor_block_complete_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
