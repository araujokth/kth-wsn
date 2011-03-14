/* Include files */

#include "blascompat32.h"
#include "sensor_block_sfun.h"
#include "c1_sensor_block.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c1_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc1_sensor_blockInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c1_sensor_block(void);
static void initialize_params_c1_sensor_block(void);
static void enable_c1_sensor_block(void);
static void disable_c1_sensor_block(void);
static void c1_update_debugger_state_c1_sensor_block(void);
static const mxArray *get_sim_state_c1_sensor_block(void);
static void set_sim_state_c1_sensor_block(const mxArray *c1_st);
static void finalize_c1_sensor_block(void);
static void sf_c1_sensor_block(void);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static void c1_eml_scalar_eg(void);
static real_T c1_sum(real_T c1_x[100]);
static void c1_eml_warning(void);
static const mxArray *c1_sf_marshall(void *c1_chartInstance, void *c1_u);
static const mxArray *c1_b_sf_marshall(void *c1_chartInstance, void *c1_u);
static const mxArray *c1_c_sf_marshall(void *c1_chartInstance, void *c1_u);
static const mxArray *c1_d_sf_marshall(void *c1_chartInstance, void *c1_u);
static const mxArray *c1_e_sf_marshall(void *c1_chartInstance, void *c1_u);
static const mxArray *c1_f_sf_marshall(void *c1_chartInstance, void *c1_u);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[48]);
static const mxArray *c1_g_sf_marshall(void *c1_chartInstance, void *c1_u);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c1_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c1_item_not_empty = false;
  chartInstance.c1_errors_window_not_empty = false;
  chartInstance.c1_pckTotalTrack_not_empty = false;
  chartInstance.c1_prevTotalPackets_not_empty = false;
  chartInstance.c1_is_active_c1_sensor_block = 0U;
}

static void initialize_params_c1_sensor_block(void)
{
}

static void enable_c1_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c1_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c1_update_debugger_state_c1_sensor_block(void)
{
}

static const mxArray *get_sim_state_c1_sensor_block(void)
{
  const mxArray *c1_st = NULL;
  const mxArray *c1_y = NULL;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  int32_T c1_i0;
  real_T c1_c_u[100];
  const mxArray *c1_d_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  int32_T c1_i1;
  real_T c1_e_u[100];
  const mxArray *c1_f_y = NULL;
  uint16_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  uint8_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T *c1_errors;
  real_T *c1_reliability;
  c1_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c1_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(7));
  c1_u = *c1_errors;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_u = *c1_reliability;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c1_y, 1, c1_c_y);
  for (c1_i0 = 0; c1_i0 < 100; c1_i0 = c1_i0 + 1) {
    c1_c_u[c1_i0] = chartInstance.c1_errors_window[c1_i0];
  }

  c1_d_y = NULL;
  if (!chartInstance.c1_errors_window_not_empty) {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 1U, 0U, 1, 100));
  }

  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_u = chartInstance.c1_item;
  c1_e_y = NULL;
  if (!chartInstance.c1_item_not_empty) {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c1_y, 3, c1_e_y);
  for (c1_i1 = 0; c1_i1 < 100; c1_i1 = c1_i1 + 1) {
    c1_e_u[c1_i1] = chartInstance.c1_pckTotalTrack[c1_i1];
  }

  c1_f_y = NULL;
  if (!chartInstance.c1_pckTotalTrack_not_empty) {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 1U, 0U, 1, 100));
  }

  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_u = chartInstance.c1_prevTotalPackets;
  c1_g_y = NULL;
  if (!chartInstance.c1_prevTotalPackets_not_empty) {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_u = chartInstance.c1_is_active_c1_sensor_block;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c1_y, 6, c1_h_y);
  sf_mex_assign(&c1_st, c1_y);
  return c1_st;
}

static void set_sim_state_c1_sensor_block(const mxArray *c1_st)
{
  const mxArray *c1_u;
  const mxArray *c1_errors;
  real_T c1_d0;
  real_T c1_y;
  const mxArray *c1_b_errors;
  real_T c1_d1;
  real_T c1_b_y;
  const mxArray *c1_b_errors_window;
  real_T c1_dv0[100];
  int32_T c1_i2;
  real_T c1_c_y[100];
  int32_T c1_i3;
  const mxArray *c1_b_item;
  real_T c1_d2;
  real_T c1_d_y;
  const mxArray *c1_b_pckTotalTrack;
  real_T c1_dv1[100];
  int32_T c1_i4;
  real_T c1_e_y[100];
  int32_T c1_i5;
  const mxArray *c1_b_prevTotalPackets;
  uint16_T c1_u0;
  uint16_T c1_f_y;
  const mxArray *c1_b_is_active_c1_sensor_block;
  uint8_T c1_u1;
  uint8_T c1_g_y;
  real_T *c1_c_errors;
  real_T *c1_reliability;
  c1_c_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c1_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  chartInstance.c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_errors = sf_mex_dup(sf_mex_getcell(c1_u, 0));
  sf_mex_import("errors", sf_mex_dup(c1_errors), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_errors);
  *c1_c_errors = c1_y;
  c1_b_errors = sf_mex_dup(sf_mex_getcell(c1_u, 1));
  sf_mex_import("reliability", sf_mex_dup(c1_b_errors), &c1_d1, 1, 0, 0U, 0, 0U,
                0);
  c1_b_y = c1_d1;
  sf_mex_destroy(&c1_b_errors);
  *c1_reliability = c1_b_y;
  c1_b_errors_window = sf_mex_dup(sf_mex_getcell(c1_u, 2));
  if (mxIsEmpty(c1_b_errors_window)) {
    chartInstance.c1_errors_window_not_empty = false;
  } else {
    chartInstance.c1_errors_window_not_empty = true;
    sf_mex_import("errors_window", sf_mex_dup(c1_b_errors_window), &c1_dv0, 1, 0,
                  0U, 1, 0U, 1, 100);
    for (c1_i2 = 0; c1_i2 < 100; c1_i2 = c1_i2 + 1) {
      c1_c_y[c1_i2] = c1_dv0[c1_i2];
    }
  }

  sf_mex_destroy(&c1_b_errors_window);
  for (c1_i3 = 0; c1_i3 < 100; c1_i3 = c1_i3 + 1) {
    chartInstance.c1_errors_window[c1_i3] = c1_c_y[c1_i3];
  }

  c1_b_item = sf_mex_dup(sf_mex_getcell(c1_u, 3));
  if (mxIsEmpty(c1_b_item)) {
    chartInstance.c1_item_not_empty = false;
  } else {
    chartInstance.c1_item_not_empty = true;
    sf_mex_import("item", sf_mex_dup(c1_b_item), &c1_d2, 1, 0, 0U, 0, 0U, 0);
    c1_d_y = c1_d2;
  }

  sf_mex_destroy(&c1_b_item);
  chartInstance.c1_item = c1_d_y;
  c1_b_pckTotalTrack = sf_mex_dup(sf_mex_getcell(c1_u, 4));
  if (mxIsEmpty(c1_b_pckTotalTrack)) {
    chartInstance.c1_pckTotalTrack_not_empty = false;
  } else {
    chartInstance.c1_pckTotalTrack_not_empty = true;
    sf_mex_import("pckTotalTrack", sf_mex_dup(c1_b_pckTotalTrack), &c1_dv1, 1, 0,
                  0U, 1, 0U, 1, 100);
    for (c1_i4 = 0; c1_i4 < 100; c1_i4 = c1_i4 + 1) {
      c1_e_y[c1_i4] = c1_dv1[c1_i4];
    }
  }

  sf_mex_destroy(&c1_b_pckTotalTrack);
  for (c1_i5 = 0; c1_i5 < 100; c1_i5 = c1_i5 + 1) {
    chartInstance.c1_pckTotalTrack[c1_i5] = c1_e_y[c1_i5];
  }

  c1_b_prevTotalPackets = sf_mex_dup(sf_mex_getcell(c1_u, 5));
  if (mxIsEmpty(c1_b_prevTotalPackets)) {
    chartInstance.c1_prevTotalPackets_not_empty = false;
  } else {
    chartInstance.c1_prevTotalPackets_not_empty = true;
    sf_mex_import("prevTotalPackets", sf_mex_dup(c1_b_prevTotalPackets), &c1_u0,
                  1, 5, 0U, 0, 0U, 0);
    c1_f_y = c1_u0;
  }

  sf_mex_destroy(&c1_b_prevTotalPackets);
  chartInstance.c1_prevTotalPackets = c1_f_y;
  c1_b_is_active_c1_sensor_block = sf_mex_dup(sf_mex_getcell(c1_u, 6));
  sf_mex_import("is_active_c1_sensor_block", sf_mex_dup
                (c1_b_is_active_c1_sensor_block), &c1_u1, 1, 3, 0U, 0, 0U, 0);
  c1_g_y = c1_u1;
  sf_mex_destroy(&c1_b_is_active_c1_sensor_block);
  chartInstance.c1_is_active_c1_sensor_block = c1_g_y;
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_sensor_block();
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_sensor_block(void)
{
}

static void sf_c1_sensor_block(void)
{
  int32_T c1_previousEvent;
  real_T c1_update;
  uint16_T c1_pck_success;
  uint16_T c1_pck_total;
  real_T c1_nargout = 2.0;
  real_T c1_nargin = 3.0;
  real_T c1_currentItem;
  real_T c1_SLIDING_WINDOW;
  real_T c1_reliability;
  real_T c1_errors;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_x;
  real_T c1_xk;
  real_T c1_b_x;
  real_T c1_c_x;
  real_T c1_d_x;
  uint32_T c1_u2;
  uint32_T c1_u3;
  uint32_T c1_u4;
  uint16_T c1_u5;
  uint32_T c1_u6;
  uint32_T c1_u7;
  uint32_T c1_u8;
  uint16_T c1_u9;
  uint32_T c1_u10;
  uint32_T c1_u11;
  uint32_T c1_u12;
  uint16_T c1_u13;
  real_T c1_d3;
  real_T c1_d4;
  real_T c1_d5;
  uint16_T c1_u14;
  int32_T c1_i8;
  real_T c1_dv2[100];
  real_T c1_A;
  int32_T c1_i9;
  real_T c1_dv3[100];
  real_T c1_B;
  real_T c1_e_x;
  real_T c1_y;
  real_T c1_f_x;
  real_T c1_b_y;
  real_T c1_g_x;
  real_T c1_c_y;
  real_T c1_d_y;
  real_T *c1_b_reliability;
  real_T *c1_b_errors;
  uint16_T *c1_b_pck_total;
  uint16_T *c1_b_pck_success;
  real_T *c1_b_update;
  c1_b_pck_success = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
  c1_b_update = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c1_b_pck_total = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
  c1_b_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c1_b_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,0);
  _SFD_DATA_RANGE_CHECK(*c1_b_update, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_b_pck_success, 1U);
  _SFD_DATA_RANGE_CHECK((real_T)*c1_b_pck_total, 2U);
  _SFD_DATA_RANGE_CHECK(*c1_b_errors, 3U);
  _SFD_DATA_RANGE_CHECK(*c1_b_reliability, 4U);
  c1_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  c1_update = *c1_b_update;
  c1_pck_success = *c1_b_pck_success;
  c1_pck_total = *c1_b_pck_total;
  sf_debug_symbol_scope_push(13U, 0U);
  sf_debug_symbol_scope_add("prevTotalPackets",
    &chartInstance.c1_prevTotalPackets, c1_f_sf_marshall);
  sf_debug_symbol_scope_add("pckTotalTrack", &chartInstance.c1_pckTotalTrack,
    c1_e_sf_marshall);
  sf_debug_symbol_scope_add("errors_window", &chartInstance.c1_errors_window,
    c1_d_sf_marshall);
  sf_debug_symbol_scope_add("item", &chartInstance.c1_item, c1_c_sf_marshall);
  sf_debug_symbol_scope_add("nargout", &c1_nargout, c1_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c1_nargin, c1_sf_marshall);
  sf_debug_symbol_scope_add("currentItem", &c1_currentItem, c1_sf_marshall);
  sf_debug_symbol_scope_add("SLIDING_WINDOW", &c1_SLIDING_WINDOW, c1_sf_marshall);
  sf_debug_symbol_scope_add("reliability", &c1_reliability, c1_sf_marshall);
  sf_debug_symbol_scope_add("errors", &c1_errors, c1_sf_marshall);
  sf_debug_symbol_scope_add("pck_total", &c1_pck_total, c1_b_sf_marshall);
  sf_debug_symbol_scope_add("pck_success", &c1_pck_success, c1_b_sf_marshall);
  sf_debug_symbol_scope_add("update", &c1_update, c1_sf_marshall);
  CV_EML_FCN(0, 0);

  /*   */
  _SFD_EML_CALL(0,7);
  c1_SLIDING_WINDOW = 100.0;

  /*  First initialization of the variables */
  _SFD_EML_CALL(0,10);
  if (CV_EML_IF(0, 0, !chartInstance.c1_item_not_empty)) {
    _SFD_EML_CALL(0,11);
    for (c1_i6 = 0; c1_i6 < 100; c1_i6 = c1_i6 + 1) {
      chartInstance.c1_errors_window[c1_i6] = 0.0;
    }

    chartInstance.c1_errors_window_not_empty = true;
    _SFD_EML_CALL(0,12);
    for (c1_i7 = 0; c1_i7 < 100; c1_i7 = c1_i7 + 1) {
      chartInstance.c1_pckTotalTrack[c1_i7] = 0.0;
    }

    chartInstance.c1_pckTotalTrack_not_empty = true;
    _SFD_EML_CALL(0,13);
    chartInstance.c1_item = 1.0;
    chartInstance.c1_item_not_empty = true;
  }

  _SFD_EML_CALL(0,15);
  if (CV_EML_IF(0, 1, c1_update == 1.0)) {
    _SFD_EML_CALL(0,16);
    c1_x = chartInstance.c1_item + 1.0;
    c1_eml_scalar_eg();
    c1_xk = c1_x;
    c1_b_x = c1_xk;
    c1_eml_scalar_eg();
    c1_c_x = c1_b_x / 101.0;
    c1_d_x = c1_c_x;
    c1_d_x = muDoubleScalarFloor(c1_d_x);
    c1_currentItem = c1_b_x - c1_d_x * 101.0;
    _SFD_EML_CALL(0,17);
    if (CV_EML_IF(0, 2, c1_currentItem == 0.0)) {
      _SFD_EML_CALL(0,17);
      c1_currentItem = 1.0;
    }

    /*  It could be that the number of received packets */
    /*  is not the total packets sent, so we need */
    /*  to keep track of it. */
    _SFD_EML_CALL(0,21);
    if (CV_EML_COND(0, 0, (real_T)c1_pck_total == 0.0)) {
    } else if (CV_EML_COND(0, 1, !chartInstance.c1_prevTotalPackets_not_empty))
    {
    } else {
      CV_EML_MCDC(0, 0, false);
      CV_EML_IF(0, 3, false);
      _SFD_EML_CALL(0,25);
      c1_u2 = (uint32_T)c1_pck_total;
      c1_u3 = c1_u2 - (uint32_T)chartInstance.c1_prevTotalPackets;
      if (c1_u3 > c1_u2) {
        c1_u3 = 0U;
      }

      c1_u4 = c1_u3;
      if (c1_u4 > 65535U) {
        c1_u5 = MAX_uint16_T;
      } else {
        c1_u5 = (uint16_T)c1_u4;
      }

      chartInstance.c1_pckTotalTrack[_SFD_EML_ARRAY_BOUNDS_CHECK("pckTotalTrack",
        (int32_T)_SFD_INTEGER_CHECK("currentItem",
        c1_currentItem), 1, 100, 1, 0) - 1] = (real_T)c1_u5;
      goto label_1;
    }

    CV_EML_MCDC(0, 0, true);
    CV_EML_IF(0, 3, true);
    _SFD_EML_CALL(0,22);
    chartInstance.c1_pckTotalTrack[_SFD_EML_ARRAY_BOUNDS_CHECK("pckTotalTrack",
      (int32_T)_SFD_INTEGER_CHECK("currentItem",
      c1_currentItem), 1, 100, 1, 0) - 1] = 1.0;
    _SFD_EML_CALL(0,23);
    chartInstance.c1_prevTotalPackets = c1_pck_total;
    chartInstance.c1_prevTotalPackets_not_empty = true;
   label_1:
    ;

    /*  if the previous errors in less than the actual value */
    /*  it means that we get a need error */
    _SFD_EML_CALL(0,30);
    c1_u6 = (uint32_T)c1_pck_total;
    c1_u7 = c1_u6 - (uint32_T)c1_pck_success;
    if (c1_u7 > c1_u6) {
      c1_u7 = 0U;
    }

    c1_u8 = c1_u7;
    if (c1_u8 > 65535U) {
      c1_u9 = MAX_uint16_T;
    } else {
      c1_u9 = (uint16_T)c1_u8;
    }

    if (CV_EML_IF(0, 4,
                  chartInstance.c1_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "errors_window", (int32_T)_SFD_INTEGER_CHECK("item",
           chartInstance.c1_item), 1, 100, 1, 0) - 1] < (real_T)c1_u9)) {
      _SFD_EML_CALL(0,31);
      c1_u10 = (uint32_T)c1_pck_total;
      c1_u11 = c1_u10 - (uint32_T)c1_pck_success;
      if (c1_u11 > c1_u10) {
        c1_u11 = 0U;
      }

      c1_u12 = c1_u11;
      if (c1_u12 > 65535U) {
        c1_u13 = MAX_uint16_T;
      } else {
        c1_u13 = (uint16_T)c1_u12;
      }

      c1_d3 = (real_T)c1_u13 -
        chartInstance.c1_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "errors_window", (int32_T)_SFD_INTEGER_CHECK(
        "item", chartInstance.c1_item), 1, 100, 1, 0) - 1];
      c1_d4 = c1_d3;
      c1_d4 = c1_d4 < 0.0 ? muDoubleScalarCeil(c1_d4 - 0.5) :
        muDoubleScalarFloor(c1_d4 + 0.5);
      c1_d5 = c1_d4;
      if (c1_d5 < 65536.0) {
        if (c1_d5 >= 0.0) {
          c1_u14 = (uint16_T)c1_d5;
        } else {
          c1_u14 = 0U;
        }
      } else if (c1_d5 >= 65536.0) {
        c1_u14 = MAX_uint16_T;
      } else {
        c1_u14 = 0U;
      }

      chartInstance.c1_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK("errors_window",
        (int32_T)_SFD_INTEGER_CHECK("currentItem",
        c1_currentItem), 1, 100, 1, 0) - 1] = (real_T)c1_u14;
    } else {
      _SFD_EML_CALL(0,33);
      chartInstance.c1_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK("errors_window",
        (int32_T)_SFD_INTEGER_CHECK("currentItem",
        c1_currentItem), 1, 100, 1, 0) - 1] = 0.0;
    }

    _SFD_EML_CALL(0,36);
    chartInstance.c1_item = c1_currentItem;
  }

  /*  */
  /*  Map the outputs */
  /*  */
  _SFD_EML_CALL(0,42);
  c1_errors = chartInstance.c1_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
    "errors_window", (int32_T)_SFD_INTEGER_CHECK("item",
    chartInstance.c1_item), 1, 100, 1, 0) - 1];
  _SFD_EML_CALL(0,43);
  for (c1_i8 = 0; c1_i8 < 100; c1_i8 = c1_i8 + 1) {
    c1_dv2[c1_i8] = chartInstance.c1_errors_window[c1_i8];
  }

  c1_A = c1_sum(c1_dv2);
  for (c1_i9 = 0; c1_i9 < 100; c1_i9 = c1_i9 + 1) {
    c1_dv3[c1_i9] = chartInstance.c1_pckTotalTrack[c1_i9];
  }

  c1_B = c1_sum(c1_dv3);
  c1_e_x = c1_A;
  c1_y = c1_B;
  if (c1_y == 0.0) {
    c1_eml_warning();
  }

  c1_f_x = c1_e_x;
  c1_b_y = c1_y;
  c1_g_x = c1_f_x;
  c1_c_y = c1_b_y;
  c1_d_y = c1_g_x / c1_c_y;
  c1_reliability = 1.0 - c1_d_y;
  _SFD_EML_CALL(0,-43);
  sf_debug_symbol_scope_pop();
  *c1_b_errors = c1_errors;
  *c1_b_reliability = c1_reliability;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
  _sfEvent_ = c1_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_blockMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber);
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static void c1_eml_scalar_eg(void)
{
}

static real_T c1_sum(real_T c1_x[100])
{
  real_T c1_y;
  int32_T c1_i10;
  real_T c1_b_x[100];
  real_T c1_k;
  real_T c1_b_k;
  for (c1_i10 = 0; c1_i10 < 100; c1_i10 = c1_i10 + 1) {
    c1_b_x[c1_i10] = c1_x[c1_i10];
  }

  c1_y = c1_b_x[0];
  for (c1_k = 2.0; c1_k <= 100.0; c1_k = c1_k + 1.0) {
    c1_b_k = c1_k;
    c1_y = c1_y + c1_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("x", (int32_T)
      _SFD_INTEGER_CHECK("k", c1_b_k), 1, 100, 1, 0) - 1];
  }

  return c1_y;
}

static void c1_eml_warning(void)
{
  int32_T c1_i11;
  static char_T c1_cv0[15] = { 'D', 'i', 'v', 'i', 'd', 'e', ' ', 'b', 'y', ' ',
    'z', 'e', 'r', 'o', '.' };

  char_T c1_u[15];
  const mxArray *c1_y = NULL;
  int32_T c1_i12;
  static char_T c1_cv1[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'd', 'i', 'v',
    'i', 'd', 'e', 'B', 'y', 'Z', 'e', 'r', 'o' };

  char_T c1_b_u[19];
  const mxArray *c1_b_y = NULL;
  for (c1_i11 = 0; c1_i11 < 15; c1_i11 = c1_i11 + 1) {
    c1_u[c1_i11] = c1_cv0[c1_i11];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 10, 0U, 1U, 0U, 2, 1, 15));
  for (c1_i12 = 0; c1_i12 < 19; c1_i12 = c1_i12 + 1) {
    c1_b_u[c1_i12] = c1_cv1[c1_i12];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 10, 0U, 1U, 0U, 2, 1, 19));
  sf_mex_call_debug("warning", 0U, 2U, 14, c1_b_y, 14, c1_y);
}

static const mxArray *c1_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  c1_b_u = *((real_T *)c1_u);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static const mxArray *c1_b_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  uint16_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  c1_b_u = *((uint16_T *)c1_u);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static const mxArray *c1_c_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  c1_b_u = *((real_T *)c1_u);
  c1_b_y = NULL;
  if (!chartInstance.c1_item_not_empty) {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static const mxArray *c1_d_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  int32_T c1_i13;
  real_T c1_b_u[100];
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  for (c1_i13 = 0; c1_i13 < 100; c1_i13 = c1_i13 + 1) {
    c1_b_u[c1_i13] = (*((real_T (*)[100])c1_u))[c1_i13];
  }

  c1_b_y = NULL;
  if (!chartInstance.c1_errors_window_not_empty) {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 1U, 0U, 1, 100));
  }

  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static const mxArray *c1_e_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  int32_T c1_i14;
  real_T c1_b_u[100];
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  for (c1_i14 = 0; c1_i14 < 100; c1_i14 = c1_i14 + 1) {
    c1_b_u[c1_i14] = (*((real_T (*)[100])c1_u))[c1_i14];
  }

  c1_b_y = NULL;
  if (!chartInstance.c1_pckTotalTrack_not_empty) {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 1U, 0U, 1, 100));
  }

  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static const mxArray *c1_f_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  uint16_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  c1_b_u = *((uint16_T *)c1_u);
  c1_b_y = NULL;
  if (!chartInstance.c1_prevTotalPackets_not_empty) {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

const mxArray *sf_c1_sensor_block_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_ResolvedFunctionInfo c1_info[48];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i15;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 48));
  for (c1_i15 = 0; c1_i15 < 48; c1_i15 = c1_i15 + 1) {
    c1_r0 = &c1_info[c1_i15];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context",
                    "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name",
                    "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c1_i15
                    );
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c1_i15
                    );
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[48])
{
  c1_info[0].context = "";
  c1_info[0].name = "zeros";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved = "[B]zeros";
  c1_info[0].fileLength = 0U;
  c1_info[0].fileTime1 = 0U;
  c1_info[0].fileTime2 = 0U;
  c1_info[1].context = "";
  c1_info[1].name = "eq";
  c1_info[1].dominantType = "double";
  c1_info[1].resolved = "[B]eq";
  c1_info[1].fileLength = 0U;
  c1_info[1].fileTime1 = 0U;
  c1_info[1].fileTime2 = 0U;
  c1_info[2].context = "";
  c1_info[2].name = "plus";
  c1_info[2].dominantType = "double";
  c1_info[2].resolved = "[B]plus";
  c1_info[2].fileLength = 0U;
  c1_info[2].fileTime1 = 0U;
  c1_info[2].fileTime2 = 0U;
  c1_info[3].context = "";
  c1_info[3].name = "mod";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[3].fileLength = 960U;
  c1_info[3].fileTime1 = 1228115389U;
  c1_info[3].fileTime2 = 0U;
  c1_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[4].name = "nargin";
  c1_info[4].dominantType = "";
  c1_info[4].resolved = "[B]nargin";
  c1_info[4].fileLength = 0U;
  c1_info[4].fileTime1 = 0U;
  c1_info[4].fileTime2 = 0U;
  c1_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[5].name = "gt";
  c1_info[5].dominantType = "double";
  c1_info[5].resolved = "[B]gt";
  c1_info[5].fileLength = 0U;
  c1_info[5].fileTime1 = 0U;
  c1_info[5].fileTime2 = 0U;
  c1_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[6].name = "isreal";
  c1_info[6].dominantType = "double";
  c1_info[6].resolved = "[B]isreal";
  c1_info[6].fileLength = 0U;
  c1_info[6].fileTime1 = 0U;
  c1_info[6].fileTime2 = 0U;
  c1_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[7].name = "isa";
  c1_info[7].dominantType = "double";
  c1_info[7].resolved = "[B]isa";
  c1_info[7].fileLength = 0U;
  c1_info[7].fileTime1 = 0U;
  c1_info[7].fileTime2 = 0U;
  c1_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[8].name = "class";
  c1_info[8].dominantType = "double";
  c1_info[8].resolved = "[B]class";
  c1_info[8].fileLength = 0U;
  c1_info[8].fileTime1 = 0U;
  c1_info[8].fileTime2 = 0U;
  c1_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[9].name = "eml_scalar_eg";
  c1_info[9].dominantType = "double";
  c1_info[9].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[9].fileLength = 3068U;
  c1_info[9].fileTime1 = 1240283610U;
  c1_info[9].fileTime2 = 0U;
  c1_info[10].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c1_info[10].name = "false";
  c1_info[10].dominantType = "";
  c1_info[10].resolved = "[B]false";
  c1_info[10].fileLength = 0U;
  c1_info[10].fileTime1 = 0U;
  c1_info[10].fileTime2 = 0U;
  c1_info[11].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[11].name = "isstruct";
  c1_info[11].dominantType = "double";
  c1_info[11].resolved = "[B]isstruct";
  c1_info[11].fileLength = 0U;
  c1_info[11].fileTime1 = 0U;
  c1_info[11].fileTime2 = 0U;
  c1_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c1_info[12].name = "cast";
  c1_info[12].dominantType = "double";
  c1_info[12].resolved = "[B]cast";
  c1_info[12].fileLength = 0U;
  c1_info[12].fileTime1 = 0U;
  c1_info[12].fileTime2 = 0U;
  c1_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[13].name = "eml_scalexp_alloc";
  c1_info[13].dominantType = "double";
  c1_info[13].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[13].fileLength = 808U;
  c1_info[13].fileTime1 = 1230516299U;
  c1_info[13].fileTime2 = 0U;
  c1_info[14].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[14].name = "minus";
  c1_info[14].dominantType = "double";
  c1_info[14].resolved = "[B]minus";
  c1_info[14].fileLength = 0U;
  c1_info[14].fileTime1 = 0U;
  c1_info[14].fileTime2 = 0U;
  c1_info[15].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[15].name = "isscalar";
  c1_info[15].dominantType = "double";
  c1_info[15].resolved = "[B]isscalar";
  c1_info[15].fileLength = 0U;
  c1_info[15].fileTime1 = 0U;
  c1_info[15].fileTime2 = 0U;
  c1_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[16].name = "not";
  c1_info[16].dominantType = "logical";
  c1_info[16].resolved = "[B]not";
  c1_info[16].fileLength = 0U;
  c1_info[16].fileTime1 = 0U;
  c1_info[16].fileTime2 = 0U;
  c1_info[17].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c1_info[17].name = "eml_scalar_mod";
  c1_info[17].dominantType = "double";
  c1_info[17].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[17].fileLength = 1103U;
  c1_info[17].fileTime1 = 1228115385U;
  c1_info[17].fileTime2 = 0U;
  c1_info[18].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[18].name = "isinteger";
  c1_info[18].dominantType = "double";
  c1_info[18].resolved = "[B]isinteger";
  c1_info[18].fileLength = 0U;
  c1_info[18].fileTime1 = 0U;
  c1_info[18].fileTime2 = 0U;
  c1_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[19].name = "eml_scalar_floor";
  c1_info[19].dominantType = "double";
  c1_info[19].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c1_info[19].fileLength = 260U;
  c1_info[19].fileTime1 = 1209352390U;
  c1_info[19].fileTime2 = 0U;
  c1_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[20].name = "times";
  c1_info[20].dominantType = "double";
  c1_info[20].resolved = "[B]times";
  c1_info[20].fileLength = 0U;
  c1_info[20].fileTime1 = 0U;
  c1_info[20].fileTime2 = 0U;
  c1_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[21].name = "eml_scalar_round";
  c1_info[21].dominantType = "double";
  c1_info[21].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c1_info[21].fileLength = 523U;
  c1_info[21].fileTime1 = 1203469608U;
  c1_info[21].fileTime2 = 0U;
  c1_info[22].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c1_info[22].name = "isfloat";
  c1_info[22].dominantType = "double";
  c1_info[22].resolved = "[B]isfloat";
  c1_info[22].fileLength = 0U;
  c1_info[22].fileTime1 = 0U;
  c1_info[22].fileTime2 = 0U;
  c1_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m/scalar_round";
  c1_info[23].name = "lt";
  c1_info[23].dominantType = "double";
  c1_info[23].resolved = "[B]lt";
  c1_info[23].fileLength = 0U;
  c1_info[23].fileTime1 = 0U;
  c1_info[23].fileTime2 = 0U;
  c1_info[24].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[24].name = "eml_scalar_abs";
  c1_info[24].dominantType = "double";
  c1_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[24].fileLength = 461U;
  c1_info[24].fileTime1 = 1203469560U;
  c1_info[24].fileTime2 = 0U;
  c1_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[25].name = "ischar";
  c1_info[25].dominantType = "double";
  c1_info[25].resolved = "[B]ischar";
  c1_info[25].fileLength = 0U;
  c1_info[25].fileTime1 = 0U;
  c1_info[25].fileTime2 = 0U;
  c1_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c1_info[26].name = "islogical";
  c1_info[26].dominantType = "double";
  c1_info[26].resolved = "[B]islogical";
  c1_info[26].fileLength = 0U;
  c1_info[26].fileTime1 = 0U;
  c1_info[26].fileTime2 = 0U;
  c1_info[27].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[27].name = "eps";
  c1_info[27].dominantType = "char";
  c1_info[27].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[27].fileLength = 1337U;
  c1_info[27].fileTime1 = 1228115399U;
  c1_info[27].fileTime2 = 0U;
  c1_info[28].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[28].name = "eml_is_float_class";
  c1_info[28].dominantType = "char";
  c1_info[28].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[28].fileLength = 226U;
  c1_info[28].fileTime1 = 1197872041U;
  c1_info[28].fileTime2 = 0U;
  c1_info[29].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[29].name = "strcmp";
  c1_info[29].dominantType = "char";
  c1_info[29].resolved = "[B]strcmp";
  c1_info[29].fileLength = 0U;
  c1_info[29].fileTime1 = 0U;
  c1_info[29].fileTime2 = 0U;
  c1_info[30].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c1_info[30].name = "uminus";
  c1_info[30].dominantType = "double";
  c1_info[30].resolved = "[B]uminus";
  c1_info[30].fileLength = 0U;
  c1_info[30].fileTime1 = 0U;
  c1_info[30].fileTime2 = 0U;
  c1_info[31].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[31].name = "mtimes";
  c1_info[31].dominantType = "double";
  c1_info[31].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[31].fileLength = 3302U;
  c1_info[31].fileTime1 = 1242772494U;
  c1_info[31].fileTime2 = 0U;
  c1_info[32].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[32].name = "size";
  c1_info[32].dominantType = "double";
  c1_info[32].resolved = "[B]size";
  c1_info[32].fileLength = 0U;
  c1_info[32].fileTime1 = 0U;
  c1_info[32].fileTime2 = 0U;
  c1_info[33].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c1_info[33].name = "le";
  c1_info[33].dominantType = "double";
  c1_info[33].resolved = "[B]le";
  c1_info[33].fileLength = 0U;
  c1_info[33].fileTime1 = 0U;
  c1_info[33].fileTime2 = 0U;
  c1_info[34].context = "";
  c1_info[34].name = "sum";
  c1_info[34].dominantType = "double";
  c1_info[34].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c1_info[34].fileLength = 3965U;
  c1_info[34].fileTime1 = 1240283566U;
  c1_info[34].fileTime2 = 0U;
  c1_info[35].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c1_info[35].name = "isequal";
  c1_info[35].dominantType = "double";
  c1_info[35].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c1_info[35].fileLength = 180U;
  c1_info[35].fileTime1 = 1226598871U;
  c1_info[35].fileTime2 = 0U;
  c1_info[36].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c1_info[36].name = "eml_isequal_core";
  c1_info[36].dominantType = "double";
  c1_info[36].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c1_info[36].fileLength = 3981U;
  c1_info[36].fileTime1 = 1236278872U;
  c1_info[36].fileTime2 = 0U;
  c1_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c1_info[37].name = "ge";
  c1_info[37].dominantType = "double";
  c1_info[37].resolved = "[B]ge";
  c1_info[37].fileLength = 0U;
  c1_info[37].fileTime1 = 0U;
  c1_info[37].fileTime2 = 0U;
  c1_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c1_info[38].name = "isnumeric";
  c1_info[38].dominantType = "double";
  c1_info[38].resolved = "[B]isnumeric";
  c1_info[38].fileLength = 0U;
  c1_info[38].fileTime1 = 0U;
  c1_info[38].fileTime2 = 0U;
  c1_info[39].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c1_info[39].name = "ndims";
  c1_info[39].dominantType = "double";
  c1_info[39].resolved = "[B]ndims";
  c1_info[39].fileLength = 0U;
  c1_info[39].fileTime1 = 0U;
  c1_info[39].fileTime2 = 0U;
  c1_info[40].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c1_info[40].name = "ne";
  c1_info[40].dominantType = "double";
  c1_info[40].resolved = "[B]ne";
  c1_info[40].fileLength = 0U;
  c1_info[40].fileTime1 = 0U;
  c1_info[40].fileTime2 = 0U;
  c1_info[41].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c1_info[41].name = "true";
  c1_info[41].dominantType = "";
  c1_info[41].resolved = "[B]true";
  c1_info[41].fileLength = 0U;
  c1_info[41].fileTime1 = 0U;
  c1_info[41].fileTime2 = 0U;
  c1_info[42].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c1_info[42].name = "eml_const_nonsingleton_dim";
  c1_info[42].dominantType = "double";
  c1_info[42].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c1_info[42].fileLength = 1473U;
  c1_info[42].fileTime1 = 1240283603U;
  c1_info[42].fileTime2 = 0U;
  c1_info[43].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m/sumwork";
  c1_info[43].name = "isempty";
  c1_info[43].dominantType = "double";
  c1_info[43].resolved = "[B]isempty";
  c1_info[43].fileLength = 0U;
  c1_info[43].fileTime1 = 0U;
  c1_info[43].fileTime2 = 0U;
  c1_info[44].context = "";
  c1_info[44].name = "mrdivide";
  c1_info[44].dominantType = "double";
  c1_info[44].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c1_info[44].fileLength = 800U;
  c1_info[44].fileTime1 = 1238455891U;
  c1_info[44].fileTime2 = 0U;
  c1_info[45].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c1_info[45].name = "rdivide";
  c1_info[45].dominantType = "double";
  c1_info[45].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[45].fileLength = 620U;
  c1_info[45].fileTime1 = 1213948366U;
  c1_info[45].fileTime2 = 0U;
  c1_info[46].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[46].name = "eml_warning";
  c1_info[46].dominantType = "char";
  c1_info[46].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c1_info[46].fileLength = 262U;
  c1_info[46].fileTime1 = 1236278878U;
  c1_info[46].fileTime2 = 0U;
  c1_info[47].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c1_info[47].name = "eml_div";
  c1_info[47].dominantType = "double";
  c1_info[47].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c1_info[47].fileLength = 4269U;
  c1_info[47].fileTime1 = 1228115426U;
  c1_info[47].fileTime2 = 0U;
}

static const mxArray *c1_g_sf_marshall(void *c1_chartInstance, void *c1_u)
{
  const mxArray *c1_y = NULL;
  boolean_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  c1_y = NULL;
  c1_b_u = *((boolean_T *)c1_u);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c1_y, c1_b_y);
  return c1_y;
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c1_sensor_block_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2570896686U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3065183488U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3111739103U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3109027375U);
}

mxArray *sf_c1_sensor_block_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(3087150661U);
    pr[1] = (double)(909338684U);
    pr[2] = (double)(696488969U);
    pr[3] = (double)(3532587422U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c1_sensor_block(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[10],T\"errors\",},{M[1],M[11],T\"reliability\",},{M[4],M[0],T\"errors_window\",S'l','i','p'{{M1x2[110 123],M[0],}}},{M[4],M[0],T\"item\",S'l','i','p'{{M1x2[124 128],M[0],}}},{M[4],M[0],T\"pckTotalTrack\",S'l','i','p'{{M1x2[129 142],M[0],}}},{M[4],M[0],T\"prevTotalPackets\",S'l','i','p'{{M1x2[143 159],M[0],}}},{M[8],M[0],T\"is_active_c1_sensor_block\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_sensor_block_get_check_sum(&mxChecksum);
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
          (_sensor_blockMachineNumber_,
           1,
           1,
           1,
           5,
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
          init_script_number_translation(_sensor_blockMachineNumber_,
            chartInstance.chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_sensor_blockMachineNumber_,chartInstance.chartNumber,1);
          sf_debug_set_chart_event_thresholds(_sensor_blockMachineNumber_,
            chartInstance.chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"update",
                              0,(MexFcnForType)c1_sf_marshall);
          _SFD_SET_DATA_PROPS(1,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_success",0,(MexFcnForType)c1_b_sf_marshall);
          _SFD_SET_DATA_PROPS(2,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_total",0,(MexFcnForType)c1_b_sf_marshall);
          _SFD_SET_DATA_PROPS(3,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"errors",
                              0,(MexFcnForType)c1_sf_marshall);
          _SFD_SET_DATA_PROPS(4,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "reliability",0,(MexFcnForType)c1_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,5,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1246);
        _SFD_CV_INIT_EML_IF(0,0,226,242,-1,352);
        _SFD_CV_INIT_EML_IF(0,1,353,369,-1,1137);
        _SFD_CV_INIT_EML_IF(0,2,425,444,-1,465);
        _SFD_CV_INIT_EML_IF(0,3,599,647,730,809);
        _SFD_CV_INIT_EML_IF(0,4,914,964,1055,1107);

        {
          static int condStart[] = { 603, 621 };

          static int condEnd[] = { 617, 646 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,0,603,646,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          real_T *c1_update;
          uint16_T *c1_pck_success;
          uint16_T *c1_pck_total;
          real_T *c1_errors;
          real_T *c1_reliability;
          c1_pck_success = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
          c1_update = (real_T *)ssGetInputPortSignal(chartInstance.S, 0);
          c1_pck_total = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
          c1_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          c1_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          _SFD_SET_DATA_VALUE_PTR(0U, c1_update);
          _SFD_SET_DATA_VALUE_PTR(1U, c1_pck_success);
          _SFD_SET_DATA_VALUE_PTR(2U, c1_pck_total);
          _SFD_SET_DATA_VALUE_PTR(3U, c1_errors);
          _SFD_SET_DATA_VALUE_PTR(4U, c1_reliability);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_sensor_blockMachineNumber_,
        chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c1_sensor_block(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c1_sensor_block();
  initialize_c1_sensor_block();
}

static void sf_opaque_enable_c1_sensor_block(void *chartInstanceVar)
{
  enable_c1_sensor_block();
}

static void sf_opaque_disable_c1_sensor_block(void *chartInstanceVar)
{
  disable_c1_sensor_block();
}

static void sf_opaque_gateway_c1_sensor_block(void *chartInstanceVar)
{
  sf_c1_sensor_block();
}

static mxArray* sf_opaque_get_sim_state_c1_sensor_block(void *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c1_sensor_block();
  return st;
}

static void sf_opaque_set_sim_state_c1_sensor_block(void *chartInstanceVar,
  const mxArray *st)
{
  set_sim_state_c1_sensor_block(sf_mex_dup(st));
}

static void sf_opaque_terminate_c1_sensor_block(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c1_sensor_block();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_sensor_block(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_sensor_block();
  }
}

static void mdlSetWorkWidths_c1_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block","sensor_block",1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block","sensor_block",1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block",
      "sensor_block",1,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block","sensor_block",1,3);
      sf_mark_chart_reusable_outputs(S,"sensor_block","sensor_block",1,2);
    }

    sf_set_rtw_dwork_info(S,"sensor_block","sensor_block",1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1136137151U));
  ssSetChecksum1(S,(1460375593U));
  ssSetChecksum2(S,(978552910U));
  ssSetChecksum3(S,(4089779848U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block", "sensor_block",1);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_sensor_block(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_opaque_gateway_c1_sensor_block;
  chartInstance.chartInfo.initializeChart = sf_opaque_initialize_c1_sensor_block;
  chartInstance.chartInfo.terminateChart = sf_opaque_terminate_c1_sensor_block;
  chartInstance.chartInfo.enableChart = sf_opaque_enable_c1_sensor_block;
  chartInstance.chartInfo.disableChart = sf_opaque_disable_c1_sensor_block;
  chartInstance.chartInfo.getSimState = sf_opaque_get_sim_state_c1_sensor_block;
  chartInstance.chartInfo.setSimState = sf_opaque_set_sim_state_c1_sensor_block;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_sensor_block;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c1_sensor_block;
  chartInstance.chartInfo.mdlStart = mdlStart_c1_sensor_block;
  chartInstance.chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_sensor_block;
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

void c1_sensor_block_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_sensor_block(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_sensor_block(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_sensor_block(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_sensor_block_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
