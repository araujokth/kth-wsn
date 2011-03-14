/* Include files */

#include "blascompat32.h"
#include "sensor_block_complete_sfun.h"
#include "c4_sensor_block_complete.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_complete_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c4_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc4_sensor_block_completeInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c4_sensor_block_complete(void);
static void initialize_params_c4_sensor_block_complete(void);
static void enable_c4_sensor_block_complete(void);
static void disable_c4_sensor_block_complete(void);
static void c4_update_debugger_state_c4_sensor_block_complete(void);
static const mxArray *get_sim_state_c4_sensor_block_complete(void);
static void set_sim_state_c4_sensor_block_complete(const mxArray *c4_st);
static void finalize_c4_sensor_block_complete(void);
static void sf_c4_sensor_block_complete(void);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static void c4_eml_scalar_eg(void);
static real_T c4_sum(real_T c4_x[50]);
static void c4_eml_warning(void);
static const mxArray *c4_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_b_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_c_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_d_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_e_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_f_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_g_sf_marshall(void *c4_chartInstance, void *c4_u);
static const mxArray *c4_h_sf_marshall(void *c4_chartInstance, void *c4_u);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[48]);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c4_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c4_item_not_empty = false;
  chartInstance.c4_errors_window_not_empty = false;
  chartInstance.c4_pckTotalTrack_not_empty = false;
  chartInstance.c4_prevErrors_not_empty = false;
  chartInstance.c4_prevTotalPackets_not_empty = false;
  chartInstance.c4_is_active_c4_sensor_block_complete = 0U;
}

static void initialize_params_c4_sensor_block_complete(void)
{
}

static void enable_c4_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c4_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c4_update_debugger_state_c4_sensor_block_complete(void)
{
}

static const mxArray *get_sim_state_c4_sensor_block_complete(void)
{
  const mxArray *c4_st = NULL;
  const mxArray *c4_y = NULL;
  real_T c4_u;
  const mxArray *c4_b_y = NULL;
  real_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  int32_T c4_i0;
  real_T c4_c_u[50];
  const mxArray *c4_d_y = NULL;
  real_T c4_d_u;
  const mxArray *c4_e_y = NULL;
  int32_T c4_i1;
  real_T c4_e_u[50];
  const mxArray *c4_f_y = NULL;
  uint16_T c4_f_u;
  const mxArray *c4_g_y = NULL;
  uint16_T c4_g_u;
  const mxArray *c4_h_y = NULL;
  uint8_T c4_h_u;
  const mxArray *c4_i_y = NULL;
  real_T *c4_errors;
  real_T *c4_reliability;
  c4_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c4_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(8));
  c4_u = *c4_errors;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_b_u = *c4_reliability;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c4_y, 1, c4_c_y);
  for (c4_i0 = 0; c4_i0 < 50; c4_i0 = c4_i0 + 1) {
    c4_c_u[c4_i0] = chartInstance.c4_errors_window[c4_i0];
  }

  c4_d_y = NULL;
  if (!chartInstance.c4_errors_window_not_empty) {
    sf_mex_assign(&c4_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_c_u, 0, 0U, 1U, 0U, 1, 50));
  }

  sf_mex_setcell(c4_y, 2, c4_d_y);
  c4_d_u = chartInstance.c4_item;
  c4_e_y = NULL;
  if (!chartInstance.c4_item_not_empty) {
    sf_mex_assign(&c4_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_e_y, sf_mex_create("y", &c4_d_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c4_y, 3, c4_e_y);
  for (c4_i1 = 0; c4_i1 < 50; c4_i1 = c4_i1 + 1) {
    c4_e_u[c4_i1] = chartInstance.c4_pckTotalTrack[c4_i1];
  }

  c4_f_y = NULL;
  if (!chartInstance.c4_pckTotalTrack_not_empty) {
    sf_mex_assign(&c4_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_f_y, sf_mex_create("y", &c4_e_u, 0, 0U, 1U, 0U, 1, 50));
  }

  sf_mex_setcell(c4_y, 4, c4_f_y);
  c4_f_u = chartInstance.c4_prevErrors;
  c4_g_y = NULL;
  if (!chartInstance.c4_prevErrors_not_empty) {
    sf_mex_assign(&c4_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_g_y, sf_mex_create("y", &c4_f_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c4_y, 5, c4_g_y);
  c4_g_u = chartInstance.c4_prevTotalPackets;
  c4_h_y = NULL;
  if (!chartInstance.c4_prevTotalPackets_not_empty) {
    sf_mex_assign(&c4_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_h_y, sf_mex_create("y", &c4_g_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c4_y, 6, c4_h_y);
  c4_h_u = chartInstance.c4_is_active_c4_sensor_block_complete;
  c4_i_y = NULL;
  sf_mex_assign(&c4_i_y, sf_mex_create("y", &c4_h_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c4_y, 7, c4_i_y);
  sf_mex_assign(&c4_st, c4_y);
  return c4_st;
}

static void set_sim_state_c4_sensor_block_complete(const mxArray *c4_st)
{
  const mxArray *c4_u;
  const mxArray *c4_errors;
  real_T c4_d0;
  real_T c4_y;
  const mxArray *c4_b_errors;
  real_T c4_d1;
  real_T c4_b_y;
  const mxArray *c4_b_errors_window;
  real_T c4_dv0[50];
  int32_T c4_i2;
  real_T c4_c_y[50];
  int32_T c4_i3;
  const mxArray *c4_b_item;
  real_T c4_d2;
  real_T c4_d_y;
  const mxArray *c4_b_pckTotalTrack;
  real_T c4_dv1[50];
  int32_T c4_i4;
  real_T c4_e_y[50];
  int32_T c4_i5;
  const mxArray *c4_b_prevErrors;
  uint16_T c4_u0;
  uint16_T c4_f_y;
  const mxArray *c4_b_prevTotalPackets;
  uint16_T c4_u1;
  uint16_T c4_g_y;
  const mxArray *c4_b_is_active_c4_sensor_block_complete;
  uint8_T c4_u2;
  uint8_T c4_h_y;
  real_T *c4_c_errors;
  real_T *c4_reliability;
  c4_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c4_c_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  chartInstance.c4_doneDoubleBufferReInit = true;
  c4_u = sf_mex_dup(c4_st);
  c4_errors = sf_mex_dup(sf_mex_getcell(c4_u, 0));
  sf_mex_import("errors", sf_mex_dup(c4_errors), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_errors);
  *c4_c_errors = c4_y;
  c4_b_errors = sf_mex_dup(sf_mex_getcell(c4_u, 1));
  sf_mex_import("reliability", sf_mex_dup(c4_b_errors), &c4_d1, 1, 0, 0U, 0, 0U,
                0);
  c4_b_y = c4_d1;
  sf_mex_destroy(&c4_b_errors);
  *c4_reliability = c4_b_y;
  c4_b_errors_window = sf_mex_dup(sf_mex_getcell(c4_u, 2));
  if (mxIsEmpty(c4_b_errors_window)) {
    chartInstance.c4_errors_window_not_empty = false;
  } else {
    chartInstance.c4_errors_window_not_empty = true;
    sf_mex_import("errors_window", sf_mex_dup(c4_b_errors_window), &c4_dv0, 1, 0,
                  0U, 1, 0U, 1, 50);
    for (c4_i2 = 0; c4_i2 < 50; c4_i2 = c4_i2 + 1) {
      c4_c_y[c4_i2] = c4_dv0[c4_i2];
    }
  }

  sf_mex_destroy(&c4_b_errors_window);
  for (c4_i3 = 0; c4_i3 < 50; c4_i3 = c4_i3 + 1) {
    chartInstance.c4_errors_window[c4_i3] = c4_c_y[c4_i3];
  }

  c4_b_item = sf_mex_dup(sf_mex_getcell(c4_u, 3));
  if (mxIsEmpty(c4_b_item)) {
    chartInstance.c4_item_not_empty = false;
  } else {
    chartInstance.c4_item_not_empty = true;
    sf_mex_import("item", sf_mex_dup(c4_b_item), &c4_d2, 1, 0, 0U, 0, 0U, 0);
    c4_d_y = c4_d2;
  }

  sf_mex_destroy(&c4_b_item);
  chartInstance.c4_item = c4_d_y;
  c4_b_pckTotalTrack = sf_mex_dup(sf_mex_getcell(c4_u, 4));
  if (mxIsEmpty(c4_b_pckTotalTrack)) {
    chartInstance.c4_pckTotalTrack_not_empty = false;
  } else {
    chartInstance.c4_pckTotalTrack_not_empty = true;
    sf_mex_import("pckTotalTrack", sf_mex_dup(c4_b_pckTotalTrack), &c4_dv1, 1, 0,
                  0U, 1, 0U, 1, 50);
    for (c4_i4 = 0; c4_i4 < 50; c4_i4 = c4_i4 + 1) {
      c4_e_y[c4_i4] = c4_dv1[c4_i4];
    }
  }

  sf_mex_destroy(&c4_b_pckTotalTrack);
  for (c4_i5 = 0; c4_i5 < 50; c4_i5 = c4_i5 + 1) {
    chartInstance.c4_pckTotalTrack[c4_i5] = c4_e_y[c4_i5];
  }

  c4_b_prevErrors = sf_mex_dup(sf_mex_getcell(c4_u, 5));
  if (mxIsEmpty(c4_b_prevErrors)) {
    chartInstance.c4_prevErrors_not_empty = false;
  } else {
    chartInstance.c4_prevErrors_not_empty = true;
    sf_mex_import("prevErrors", sf_mex_dup(c4_b_prevErrors), &c4_u0, 1, 5, 0U, 0,
                  0U, 0);
    c4_f_y = c4_u0;
  }

  sf_mex_destroy(&c4_b_prevErrors);
  chartInstance.c4_prevErrors = c4_f_y;
  c4_b_prevTotalPackets = sf_mex_dup(sf_mex_getcell(c4_u, 6));
  if (mxIsEmpty(c4_b_prevTotalPackets)) {
    chartInstance.c4_prevTotalPackets_not_empty = false;
  } else {
    chartInstance.c4_prevTotalPackets_not_empty = true;
    sf_mex_import("prevTotalPackets", sf_mex_dup(c4_b_prevTotalPackets), &c4_u1,
                  1, 5, 0U, 0, 0U, 0);
    c4_g_y = c4_u1;
  }

  sf_mex_destroy(&c4_b_prevTotalPackets);
  chartInstance.c4_prevTotalPackets = c4_g_y;
  c4_b_is_active_c4_sensor_block_complete = sf_mex_dup(sf_mex_getcell(c4_u, 7));
  sf_mex_import("is_active_c4_sensor_block_complete", sf_mex_dup
                (c4_b_is_active_c4_sensor_block_complete), &c4_u2, 1, 3, 0U, 0,
                0U, 0);
  c4_h_y = c4_u2;
  sf_mex_destroy(&c4_b_is_active_c4_sensor_block_complete);
  chartInstance.c4_is_active_c4_sensor_block_complete = c4_h_y;
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_sensor_block_complete();
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_sensor_block_complete(void)
{
}

static void sf_c4_sensor_block_complete(void)
{
  int32_T c4_previousEvent;
  boolean_T c4_update;
  uint16_T c4_pck_success;
  uint16_T c4_pck_total;
  real_T c4_nargout = 2.0;
  real_T c4_nargin = 3.0;
  real_T c4_currentItem;
  boolean_T c4_newPacket;
  real_T c4_SLIDING_WINDOW;
  real_T c4_reliability;
  real_T c4_errors;
  int32_T c4_i6;
  int32_T c4_i7;
  uint32_T c4_u3;
  uint32_T c4_u4;
  uint32_T c4_u5;
  uint16_T c4_u6;
  real_T c4_x;
  real_T c4_xk;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_d_x;
  uint32_T c4_u7;
  uint32_T c4_u8;
  uint32_T c4_u9;
  uint16_T c4_u10;
  uint32_T c4_u11;
  uint32_T c4_u12;
  uint32_T c4_u13;
  uint16_T c4_u14;
  uint32_T c4_u15;
  uint32_T c4_u16;
  uint32_T c4_u17;
  uint16_T c4_u18;
  uint32_T c4_u19;
  uint32_T c4_u20;
  uint32_T c4_u21;
  uint16_T c4_u22;
  int32_T c4_i8;
  real_T c4_dv2[50];
  int32_T c4_i9;
  real_T c4_dv3[50];
  real_T c4_A;
  int32_T c4_i10;
  real_T c4_dv4[50];
  real_T c4_B;
  real_T c4_e_x;
  real_T c4_y;
  real_T c4_f_x;
  real_T c4_b_y;
  real_T c4_g_x;
  real_T c4_c_y;
  real_T c4_d_y;
  real_T *c4_b_reliability;
  real_T *c4_b_errors;
  uint16_T *c4_b_pck_total;
  uint16_T *c4_b_pck_success;
  boolean_T *c4_b_update;
  c4_b_pck_success = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
  c4_b_update = (boolean_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c4_b_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c4_b_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c4_b_pck_total = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,3);
  _SFD_DATA_RANGE_CHECK((real_T)*c4_b_update, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c4_b_pck_success, 1U);
  _SFD_DATA_RANGE_CHECK((real_T)*c4_b_pck_total, 2U);
  _SFD_DATA_RANGE_CHECK(*c4_b_errors, 3U);
  _SFD_DATA_RANGE_CHECK(*c4_b_reliability, 4U);
  c4_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,3);
  c4_update = *c4_b_update;
  c4_pck_success = *c4_b_pck_success;
  c4_pck_total = *c4_b_pck_total;
  sf_debug_symbol_scope_push(15U, 0U);
  sf_debug_symbol_scope_add("prevTotalPackets",
    &chartInstance.c4_prevTotalPackets, c4_h_sf_marshall);
  sf_debug_symbol_scope_add("prevErrors", &chartInstance.c4_prevErrors,
    c4_g_sf_marshall);
  sf_debug_symbol_scope_add("pckTotalTrack", &chartInstance.c4_pckTotalTrack,
    c4_f_sf_marshall);
  sf_debug_symbol_scope_add("errors_window", &chartInstance.c4_errors_window,
    c4_e_sf_marshall);
  sf_debug_symbol_scope_add("item", &chartInstance.c4_item, c4_d_sf_marshall);
  sf_debug_symbol_scope_add("nargout", &c4_nargout, c4_c_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c4_nargin, c4_c_sf_marshall);
  sf_debug_symbol_scope_add("currentItem", &c4_currentItem, c4_c_sf_marshall);
  sf_debug_symbol_scope_add("newPacket", &c4_newPacket, c4_sf_marshall);
  sf_debug_symbol_scope_add("SLIDING_WINDOW", &c4_SLIDING_WINDOW,
    c4_c_sf_marshall);
  sf_debug_symbol_scope_add("reliability", &c4_reliability, c4_c_sf_marshall);
  sf_debug_symbol_scope_add("errors", &c4_errors, c4_c_sf_marshall);
  sf_debug_symbol_scope_add("pck_total", &c4_pck_total, c4_b_sf_marshall);
  sf_debug_symbol_scope_add("pck_success", &c4_pck_success, c4_b_sf_marshall);
  sf_debug_symbol_scope_add("update", &c4_update, c4_sf_marshall);
  CV_EML_FCN(0, 0);

  /*   */
  /*     %vector that contains the number of errors */
  /*                     %index of the vector */
  /*            % vector that contains the number of total packets */
  /*         % previous number of total packets to check if we have received a new one */
  /*  previous number of errors */
  _SFD_EML_CALL(0,11);
  c4_SLIDING_WINDOW = 50.0;

  /*  First initialization of the variables */
  _SFD_EML_CALL(0,14);
  if (CV_EML_IF(0, 0, !chartInstance.c4_item_not_empty)) {
    _SFD_EML_CALL(0,15);
    for (c4_i6 = 0; c4_i6 < 50; c4_i6 = c4_i6 + 1) {
      chartInstance.c4_errors_window[c4_i6] = 0.0;
    }

    chartInstance.c4_errors_window_not_empty = true;
    _SFD_EML_CALL(0,16);
    for (c4_i7 = 0; c4_i7 < 50; c4_i7 = c4_i7 + 1) {
      chartInstance.c4_pckTotalTrack[c4_i7] = 0.0;
    }

    chartInstance.c4_pckTotalTrack_not_empty = true;
    _SFD_EML_CALL(0,17);
    chartInstance.c4_item = 1.0;
    chartInstance.c4_item_not_empty = true;
    _SFD_EML_CALL(0,18);
    c4_u3 = (uint32_T)c4_pck_total;
    c4_u4 = c4_u3 - (uint32_T)c4_pck_success;
    if (c4_u4 > c4_u3) {
      c4_u4 = 0U;
    }

    c4_u5 = c4_u4;
    if (c4_u5 > 65535U) {
      c4_u6 = MAX_uint16_T;
    } else {
      c4_u6 = (uint16_T)c4_u5;
    }

    chartInstance.c4_prevErrors = c4_u6;
    chartInstance.c4_prevErrors_not_empty = true;
    _SFD_EML_CALL(0,19);
    chartInstance.c4_prevTotalPackets = c4_pck_total;
    chartInstance.c4_prevTotalPackets_not_empty = true;
  }

  _SFD_EML_CALL(0,23);
  c4_newPacket = (chartInstance.c4_prevTotalPackets != c4_pck_total);
  _SFD_EML_CALL(0,25);
  if (CV_EML_COND(0, 0, c4_update)) {
    if (CV_EML_COND(0, 1, c4_newPacket)) {
      CV_EML_MCDC(0, 0, true);
      CV_EML_IF(0, 1, true);
      _SFD_EML_CALL(0,26);
      c4_x = chartInstance.c4_item + 1.0;
      c4_eml_scalar_eg();
      c4_xk = c4_x;
      c4_b_x = c4_xk;
      c4_eml_scalar_eg();
      c4_c_x = c4_b_x / 51.0;
      c4_d_x = c4_c_x;
      c4_d_x = muDoubleScalarFloor(c4_d_x);
      c4_currentItem = c4_b_x - c4_d_x * 51.0;
      _SFD_EML_CALL(0,27);
      if (CV_EML_IF(0, 2, c4_currentItem == 0.0)) {
        _SFD_EML_CALL(0,28);
        c4_currentItem = 1.0;
      }

      /*  It could be that the number of received packets */
      /*  is not the total packets sent, so we need */
      /*  to keep track of it. */
      _SFD_EML_CALL(0,34);
      c4_u7 = (uint32_T)c4_pck_total;
      c4_u8 = c4_u7 - (uint32_T)chartInstance.c4_prevTotalPackets;
      if (c4_u8 > c4_u7) {
        c4_u8 = 0U;
      }

      c4_u9 = c4_u8;
      if (c4_u9 > 65535U) {
        c4_u10 = MAX_uint16_T;
      } else {
        c4_u10 = (uint16_T)c4_u9;
      }

      chartInstance.c4_pckTotalTrack[_SFD_EML_ARRAY_BOUNDS_CHECK("pckTotalTrack",
        (int32_T)_SFD_INTEGER_CHECK("currentItem",
        c4_currentItem), 1, 50, 1, 0) - 1] = (real_T)c4_u10;
      _SFD_EML_CALL(0,35);
      if (CV_EML_IF(0, 3,
                    chartInstance.c4_pckTotalTrack[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "pckTotalTrack", (int32_T)_SFD_INTEGER_CHECK(
             "currentItem", c4_currentItem), 1, 50, 1, 0) - 1] <= 0.0)) {
        _SFD_EML_CALL(0,36);
        chartInstance.c4_pckTotalTrack[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "pckTotalTrack", (int32_T)_SFD_INTEGER_CHECK("currentItem",
          c4_currentItem), 1, 50, 1, 0) - 1] = 1.0;
      }

      /* when we restart the counters */
      /*  if the previous errors in less than the actual value */
      /*  it means that we get a need error */
      _SFD_EML_CALL(0,42);
      c4_u11 = (uint32_T)c4_pck_total;
      c4_u12 = c4_u11 - (uint32_T)c4_pck_success;
      if (c4_u12 > c4_u11) {
        c4_u12 = 0U;
      }

      c4_u13 = c4_u12;
      if (c4_u13 > 65535U) {
        c4_u14 = MAX_uint16_T;
      } else {
        c4_u14 = (uint16_T)c4_u13;
      }

      c4_u15 = (uint32_T)c4_u14;
      c4_u16 = c4_u15 - (uint32_T)chartInstance.c4_prevErrors;
      if (c4_u16 > c4_u15) {
        c4_u16 = 0U;
      }

      c4_u17 = c4_u16;
      if (c4_u17 > 65535U) {
        c4_u18 = MAX_uint16_T;
      } else {
        c4_u18 = (uint16_T)c4_u17;
      }

      chartInstance.c4_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK("errors_window",
        (int32_T)_SFD_INTEGER_CHECK("currentItem",
        c4_currentItem), 1, 50, 1, 0) - 1] = (real_T)c4_u18;

      /* %in case.... */
      _SFD_EML_CALL(0,44);
      if (CV_EML_IF(0, 4,
                    chartInstance.c4_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
            "errors_window", (int32_T)_SFD_INTEGER_CHECK(
             "currentItem", c4_currentItem), 1, 50, 1, 0) - 1] < 0.0)) {
        _SFD_EML_CALL(0,45);
        chartInstance.c4_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "errors_window", (int32_T)_SFD_INTEGER_CHECK("currentItem",
          c4_currentItem), 1, 50, 1, 0) - 1] = 0.0;
      }

      /* when we restart the counters */
      /*  update with the current data */
      _SFD_EML_CALL(0,50);
      c4_u19 = (uint32_T)c4_pck_total;
      c4_u20 = c4_u19 - (uint32_T)c4_pck_success;
      if (c4_u20 > c4_u19) {
        c4_u20 = 0U;
      }

      c4_u21 = c4_u20;
      if (c4_u21 > 65535U) {
        c4_u22 = MAX_uint16_T;
      } else {
        c4_u22 = (uint16_T)c4_u21;
      }

      chartInstance.c4_prevErrors = c4_u22;
      _SFD_EML_CALL(0,51);
      chartInstance.c4_prevTotalPackets = c4_pck_total;
      _SFD_EML_CALL(0,52);
      chartInstance.c4_item = c4_currentItem;
      goto label_1;
    }
  }

  CV_EML_MCDC(0, 0, false);
  CV_EML_IF(0, 1, false);
 label_1:
  ;

  /*  */
  /*  Map the outputs */
  _SFD_EML_CALL(0,57);
  c4_errors = chartInstance.c4_errors_window[_SFD_EML_ARRAY_BOUNDS_CHECK(
    "errors_window", (int32_T)_SFD_INTEGER_CHECK("item",
    chartInstance.c4_item), 1, 50, 1, 0) - 1];
  _SFD_EML_CALL(0,58);
  for (c4_i8 = 0; c4_i8 < 50; c4_i8 = c4_i8 + 1) {
    c4_dv2[c4_i8] = chartInstance.c4_pckTotalTrack[c4_i8];
  }

  if (CV_EML_IF(0, 5, c4_sum(c4_dv2) == 0.0)) {
    _SFD_EML_CALL(0,59);
    c4_reliability = 0.0;
  } else {
    _SFD_EML_CALL(0,61);
    for (c4_i9 = 0; c4_i9 < 50; c4_i9 = c4_i9 + 1) {
      c4_dv3[c4_i9] = chartInstance.c4_errors_window[c4_i9];
    }

    c4_A = c4_sum(c4_dv3);
    for (c4_i10 = 0; c4_i10 < 50; c4_i10 = c4_i10 + 1) {
      c4_dv4[c4_i10] = chartInstance.c4_pckTotalTrack[c4_i10];
    }

    c4_B = c4_sum(c4_dv4);
    c4_e_x = c4_A;
    c4_y = c4_B;
    if (c4_y == 0.0) {
      c4_eml_warning();
    }

    c4_f_x = c4_e_x;
    c4_b_y = c4_y;
    c4_g_x = c4_f_x;
    c4_c_y = c4_b_y;
    c4_d_y = c4_g_x / c4_c_y;
    c4_reliability = 1.0 - c4_d_y;
  }

  _SFD_EML_CALL(0,-61);
  sf_debug_symbol_scope_pop();
  *c4_b_errors = c4_errors;
  *c4_b_reliability = c4_reliability;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,3);
  _sfEvent_ = c4_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_block_completeMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber
    );
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static void c4_eml_scalar_eg(void)
{
}

static real_T c4_sum(real_T c4_x[50])
{
  real_T c4_y;
  int32_T c4_i11;
  real_T c4_b_x[50];
  real_T c4_k;
  real_T c4_b_k;
  for (c4_i11 = 0; c4_i11 < 50; c4_i11 = c4_i11 + 1) {
    c4_b_x[c4_i11] = c4_x[c4_i11];
  }

  c4_y = c4_b_x[0];
  for (c4_k = 2.0; c4_k <= 50.0; c4_k = c4_k + 1.0) {
    c4_b_k = c4_k;
    c4_y = c4_y + c4_b_x[_SFD_EML_ARRAY_BOUNDS_CHECK("x", (int32_T)
      _SFD_INTEGER_CHECK("k", c4_b_k), 1, 50, 1, 0) - 1];
  }

  return c4_y;
}

static void c4_eml_warning(void)
{
  int32_T c4_i12;
  static char_T c4_cv0[15] = { 'D', 'i', 'v', 'i', 'd', 'e', ' ', 'b', 'y', ' ',
    'z', 'e', 'r', 'o', '.' };

  char_T c4_u[15];
  const mxArray *c4_y = NULL;
  int32_T c4_i13;
  static char_T c4_cv1[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'd', 'i', 'v',
    'i', 'd', 'e', 'B', 'y', 'Z', 'e', 'r', 'o' };

  char_T c4_b_u[19];
  const mxArray *c4_b_y = NULL;
  for (c4_i12 = 0; c4_i12 < 15; c4_i12 = c4_i12 + 1) {
    c4_u[c4_i12] = c4_cv0[c4_i12];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 10, 0U, 1U, 0U, 2, 1, 15));
  for (c4_i13 = 0; c4_i13 < 19; c4_i13 = c4_i13 + 1) {
    c4_b_u[c4_i13] = c4_cv1[c4_i13];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 10, 0U, 1U, 0U, 2, 1, 19));
  sf_mex_call_debug("warning", 0U, 2U, 14, c4_b_y, 14, c4_y);
}

static const mxArray *c4_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  boolean_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((boolean_T *)c4_u);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_b_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  uint16_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((uint16_T *)c4_u);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_c_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  real_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((real_T *)c4_u);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_d_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  real_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((real_T *)c4_u);
  c4_b_y = NULL;
  if (!chartInstance.c4_item_not_empty) {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_e_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  int32_T c4_i14;
  real_T c4_b_u[50];
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  for (c4_i14 = 0; c4_i14 < 50; c4_i14 = c4_i14 + 1) {
    c4_b_u[c4_i14] = (*((real_T (*)[50])c4_u))[c4_i14];
  }

  c4_b_y = NULL;
  if (!chartInstance.c4_errors_window_not_empty) {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 50));
  }

  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_f_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  int32_T c4_i15;
  real_T c4_b_u[50];
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  for (c4_i15 = 0; c4_i15 < 50; c4_i15 = c4_i15 + 1) {
    c4_b_u[c4_i15] = (*((real_T (*)[50])c4_u))[c4_i15];
  }

  c4_b_y = NULL;
  if (!chartInstance.c4_pckTotalTrack_not_empty) {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 0, 0U, 1U, 0U, 1, 50));
  }

  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_g_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  uint16_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((uint16_T *)c4_u);
  c4_b_y = NULL;
  if (!chartInstance.c4_prevErrors_not_empty) {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

static const mxArray *c4_h_sf_marshall(void *c4_chartInstance, void *c4_u)
{
  const mxArray *c4_y = NULL;
  uint16_T c4_b_u;
  const mxArray *c4_b_y = NULL;
  c4_y = NULL;
  c4_b_u = *((uint16_T *)c4_u);
  c4_b_y = NULL;
  if (!chartInstance.c4_prevTotalPackets_not_empty) {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_b_u, 5, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c4_y, c4_b_y);
  return c4_y;
}

const mxArray *sf_c4_sensor_block_complete_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_ResolvedFunctionInfo c4_info[48];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i16;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 48));
  for (c4_i16 = 0; c4_i16 < 48; c4_i16 = c4_i16 + 1) {
    c4_r0 = &c4_info[c4_i16];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context",
                    "nameCaptureInfo", c4_i16);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name",
                    "nameCaptureInfo", c4_i16);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c4_i16);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c4_i16);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c4_i16);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c4_i16
                    );
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c4_i16
                    );
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[48])
{
  c4_info[0].context = "";
  c4_info[0].name = "zeros";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved = "[B]zeros";
  c4_info[0].fileLength = 0U;
  c4_info[0].fileTime1 = 0U;
  c4_info[0].fileTime2 = 0U;
  c4_info[1].context = "";
  c4_info[1].name = "minus";
  c4_info[1].dominantType = "uint16";
  c4_info[1].resolved = "[B]minus";
  c4_info[1].fileLength = 0U;
  c4_info[1].fileTime1 = 0U;
  c4_info[1].fileTime2 = 0U;
  c4_info[2].context = "";
  c4_info[2].name = "ne";
  c4_info[2].dominantType = "uint16";
  c4_info[2].resolved = "[B]ne";
  c4_info[2].fileLength = 0U;
  c4_info[2].fileTime1 = 0U;
  c4_info[2].fileTime2 = 0U;
  c4_info[3].context = "";
  c4_info[3].name = "plus";
  c4_info[3].dominantType = "double";
  c4_info[3].resolved = "[B]plus";
  c4_info[3].fileLength = 0U;
  c4_info[3].fileTime1 = 0U;
  c4_info[3].fileTime2 = 0U;
  c4_info[4].context = "";
  c4_info[4].name = "mod";
  c4_info[4].dominantType = "double";
  c4_info[4].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[4].fileLength = 960U;
  c4_info[4].fileTime1 = 1228115389U;
  c4_info[4].fileTime2 = 0U;
  c4_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[5].name = "nargin";
  c4_info[5].dominantType = "";
  c4_info[5].resolved = "[B]nargin";
  c4_info[5].fileLength = 0U;
  c4_info[5].fileTime1 = 0U;
  c4_info[5].fileTime2 = 0U;
  c4_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[6].name = "gt";
  c4_info[6].dominantType = "double";
  c4_info[6].resolved = "[B]gt";
  c4_info[6].fileLength = 0U;
  c4_info[6].fileTime1 = 0U;
  c4_info[6].fileTime2 = 0U;
  c4_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[7].name = "isreal";
  c4_info[7].dominantType = "double";
  c4_info[7].resolved = "[B]isreal";
  c4_info[7].fileLength = 0U;
  c4_info[7].fileTime1 = 0U;
  c4_info[7].fileTime2 = 0U;
  c4_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[8].name = "isa";
  c4_info[8].dominantType = "double";
  c4_info[8].resolved = "[B]isa";
  c4_info[8].fileLength = 0U;
  c4_info[8].fileTime1 = 0U;
  c4_info[8].fileTime2 = 0U;
  c4_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[9].name = "class";
  c4_info[9].dominantType = "double";
  c4_info[9].resolved = "[B]class";
  c4_info[9].fileLength = 0U;
  c4_info[9].fileTime1 = 0U;
  c4_info[9].fileTime2 = 0U;
  c4_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[10].name = "eml_scalar_eg";
  c4_info[10].dominantType = "double";
  c4_info[10].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[10].fileLength = 3068U;
  c4_info[10].fileTime1 = 1240283610U;
  c4_info[10].fileTime2 = 0U;
  c4_info[11].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c4_info[11].name = "false";
  c4_info[11].dominantType = "";
  c4_info[11].resolved = "[B]false";
  c4_info[11].fileLength = 0U;
  c4_info[11].fileTime1 = 0U;
  c4_info[11].fileTime2 = 0U;
  c4_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[12].name = "isstruct";
  c4_info[12].dominantType = "double";
  c4_info[12].resolved = "[B]isstruct";
  c4_info[12].fileLength = 0U;
  c4_info[12].fileTime1 = 0U;
  c4_info[12].fileTime2 = 0U;
  c4_info[13].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c4_info[13].name = "eq";
  c4_info[13].dominantType = "double";
  c4_info[13].resolved = "[B]eq";
  c4_info[13].fileLength = 0U;
  c4_info[13].fileTime1 = 0U;
  c4_info[13].fileTime2 = 0U;
  c4_info[14].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c4_info[14].name = "cast";
  c4_info[14].dominantType = "double";
  c4_info[14].resolved = "[B]cast";
  c4_info[14].fileLength = 0U;
  c4_info[14].fileTime1 = 0U;
  c4_info[14].fileTime2 = 0U;
  c4_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[15].name = "eml_scalexp_alloc";
  c4_info[15].dominantType = "double";
  c4_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[15].fileLength = 808U;
  c4_info[15].fileTime1 = 1230516299U;
  c4_info[15].fileTime2 = 0U;
  c4_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[16].name = "isscalar";
  c4_info[16].dominantType = "double";
  c4_info[16].resolved = "[B]isscalar";
  c4_info[16].fileLength = 0U;
  c4_info[16].fileTime1 = 0U;
  c4_info[16].fileTime2 = 0U;
  c4_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[17].name = "not";
  c4_info[17].dominantType = "logical";
  c4_info[17].resolved = "[B]not";
  c4_info[17].fileLength = 0U;
  c4_info[17].fileTime1 = 0U;
  c4_info[17].fileTime2 = 0U;
  c4_info[18].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c4_info[18].name = "eml_scalar_mod";
  c4_info[18].dominantType = "double";
  c4_info[18].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[18].fileLength = 1103U;
  c4_info[18].fileTime1 = 1228115385U;
  c4_info[18].fileTime2 = 0U;
  c4_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[19].name = "isinteger";
  c4_info[19].dominantType = "double";
  c4_info[19].resolved = "[B]isinteger";
  c4_info[19].fileLength = 0U;
  c4_info[19].fileTime1 = 0U;
  c4_info[19].fileTime2 = 0U;
  c4_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[20].name = "eml_scalar_floor";
  c4_info[20].dominantType = "double";
  c4_info[20].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c4_info[20].fileLength = 260U;
  c4_info[20].fileTime1 = 1209352390U;
  c4_info[20].fileTime2 = 0U;
  c4_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[21].name = "times";
  c4_info[21].dominantType = "double";
  c4_info[21].resolved = "[B]times";
  c4_info[21].fileLength = 0U;
  c4_info[21].fileTime1 = 0U;
  c4_info[21].fileTime2 = 0U;
  c4_info[22].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[22].name = "eml_scalar_round";
  c4_info[22].dominantType = "double";
  c4_info[22].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c4_info[22].fileLength = 523U;
  c4_info[22].fileTime1 = 1203469608U;
  c4_info[22].fileTime2 = 0U;
  c4_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c4_info[23].name = "isfloat";
  c4_info[23].dominantType = "double";
  c4_info[23].resolved = "[B]isfloat";
  c4_info[23].fileLength = 0U;
  c4_info[23].fileTime1 = 0U;
  c4_info[23].fileTime2 = 0U;
  c4_info[24].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m/scalar_round";
  c4_info[24].name = "lt";
  c4_info[24].dominantType = "double";
  c4_info[24].resolved = "[B]lt";
  c4_info[24].fileLength = 0U;
  c4_info[24].fileTime1 = 0U;
  c4_info[24].fileTime2 = 0U;
  c4_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[25].name = "eml_scalar_abs";
  c4_info[25].dominantType = "double";
  c4_info[25].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[25].fileLength = 461U;
  c4_info[25].fileTime1 = 1203469560U;
  c4_info[25].fileTime2 = 0U;
  c4_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[26].name = "ischar";
  c4_info[26].dominantType = "double";
  c4_info[26].resolved = "[B]ischar";
  c4_info[26].fileLength = 0U;
  c4_info[26].fileTime1 = 0U;
  c4_info[26].fileTime2 = 0U;
  c4_info[27].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[27].name = "islogical";
  c4_info[27].dominantType = "double";
  c4_info[27].resolved = "[B]islogical";
  c4_info[27].fileLength = 0U;
  c4_info[27].fileTime1 = 0U;
  c4_info[27].fileTime2 = 0U;
  c4_info[28].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[28].name = "eps";
  c4_info[28].dominantType = "char";
  c4_info[28].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[28].fileLength = 1337U;
  c4_info[28].fileTime1 = 1228115399U;
  c4_info[28].fileTime2 = 0U;
  c4_info[29].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[29].name = "eml_is_float_class";
  c4_info[29].dominantType = "char";
  c4_info[29].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c4_info[29].fileLength = 226U;
  c4_info[29].fileTime1 = 1197872041U;
  c4_info[29].fileTime2 = 0U;
  c4_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c4_info[30].name = "strcmp";
  c4_info[30].dominantType = "char";
  c4_info[30].resolved = "[B]strcmp";
  c4_info[30].fileLength = 0U;
  c4_info[30].fileTime1 = 0U;
  c4_info[30].fileTime2 = 0U;
  c4_info[31].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[31].name = "uminus";
  c4_info[31].dominantType = "double";
  c4_info[31].resolved = "[B]uminus";
  c4_info[31].fileLength = 0U;
  c4_info[31].fileTime1 = 0U;
  c4_info[31].fileTime2 = 0U;
  c4_info[32].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[32].name = "mtimes";
  c4_info[32].dominantType = "double";
  c4_info[32].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[32].fileLength = 3302U;
  c4_info[32].fileTime1 = 1242772494U;
  c4_info[32].fileTime2 = 0U;
  c4_info[33].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[33].name = "size";
  c4_info[33].dominantType = "double";
  c4_info[33].resolved = "[B]size";
  c4_info[33].fileLength = 0U;
  c4_info[33].fileTime1 = 0U;
  c4_info[33].fileTime2 = 0U;
  c4_info[34].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c4_info[34].name = "le";
  c4_info[34].dominantType = "double";
  c4_info[34].resolved = "[B]le";
  c4_info[34].fileLength = 0U;
  c4_info[34].fileTime1 = 0U;
  c4_info[34].fileTime2 = 0U;
  c4_info[35].context = "";
  c4_info[35].name = "sum";
  c4_info[35].dominantType = "double";
  c4_info[35].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c4_info[35].fileLength = 3965U;
  c4_info[35].fileTime1 = 1240283566U;
  c4_info[35].fileTime2 = 0U;
  c4_info[36].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c4_info[36].name = "isequal";
  c4_info[36].dominantType = "double";
  c4_info[36].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c4_info[36].fileLength = 180U;
  c4_info[36].fileTime1 = 1226598871U;
  c4_info[36].fileTime2 = 0U;
  c4_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c4_info[37].name = "eml_isequal_core";
  c4_info[37].dominantType = "double";
  c4_info[37].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c4_info[37].fileLength = 3981U;
  c4_info[37].fileTime1 = 1236278872U;
  c4_info[37].fileTime2 = 0U;
  c4_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c4_info[38].name = "ge";
  c4_info[38].dominantType = "double";
  c4_info[38].resolved = "[B]ge";
  c4_info[38].fileLength = 0U;
  c4_info[38].fileTime1 = 0U;
  c4_info[38].fileTime2 = 0U;
  c4_info[39].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c4_info[39].name = "isnumeric";
  c4_info[39].dominantType = "double";
  c4_info[39].resolved = "[B]isnumeric";
  c4_info[39].fileLength = 0U;
  c4_info[39].fileTime1 = 0U;
  c4_info[39].fileTime2 = 0U;
  c4_info[40].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c4_info[40].name = "ndims";
  c4_info[40].dominantType = "double";
  c4_info[40].resolved = "[B]ndims";
  c4_info[40].fileLength = 0U;
  c4_info[40].fileTime1 = 0U;
  c4_info[40].fileTime2 = 0U;
  c4_info[41].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c4_info[41].name = "true";
  c4_info[41].dominantType = "";
  c4_info[41].resolved = "[B]true";
  c4_info[41].fileLength = 0U;
  c4_info[41].fileTime1 = 0U;
  c4_info[41].fileTime2 = 0U;
  c4_info[42].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c4_info[42].name = "eml_const_nonsingleton_dim";
  c4_info[42].dominantType = "double";
  c4_info[42].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c4_info[42].fileLength = 1473U;
  c4_info[42].fileTime1 = 1240283603U;
  c4_info[42].fileTime2 = 0U;
  c4_info[43].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m/sumwork";
  c4_info[43].name = "isempty";
  c4_info[43].dominantType = "double";
  c4_info[43].resolved = "[B]isempty";
  c4_info[43].fileLength = 0U;
  c4_info[43].fileTime1 = 0U;
  c4_info[43].fileTime2 = 0U;
  c4_info[44].context = "";
  c4_info[44].name = "mrdivide";
  c4_info[44].dominantType = "double";
  c4_info[44].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c4_info[44].fileLength = 800U;
  c4_info[44].fileTime1 = 1238455891U;
  c4_info[44].fileTime2 = 0U;
  c4_info[45].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c4_info[45].name = "rdivide";
  c4_info[45].dominantType = "double";
  c4_info[45].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[45].fileLength = 620U;
  c4_info[45].fileTime1 = 1213948366U;
  c4_info[45].fileTime2 = 0U;
  c4_info[46].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[46].name = "eml_warning";
  c4_info[46].dominantType = "char";
  c4_info[46].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c4_info[46].fileLength = 262U;
  c4_info[46].fileTime1 = 1236278878U;
  c4_info[46].fileTime2 = 0U;
  c4_info[47].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[47].name = "eml_div";
  c4_info[47].dominantType = "double";
  c4_info[47].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[47].fileLength = 4269U;
  c4_info[47].fileTime1 = 1228115426U;
  c4_info[47].fileTime2 = 0U;
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c4_sensor_block_complete_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(457895153U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1578097804U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1626785135U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(479747602U);
}

mxArray *sf_c4_sensor_block_complete_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(448429804U);
    pr[1] = (double)(3977999137U);
    pr[2] = (double)(3034862524U);
    pr[3] = (double)(3583747326U);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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

static mxArray *sf_get_sim_state_info_c4_sensor_block_complete(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x8'type','srcId','name','auxInfo'{{M[1],M[10],T\"errors\",},{M[1],M[11],T\"reliability\",},{M[4],M[0],T\"errors_window\",S'l','i','p'{{M1x2[110 123],M[0],}}},{M[4],M[0],T\"item\",S'l','i','p'{{M1x2[178 182],M[0],}}},{M[4],M[0],T\"pckTotalTrack\",S'l','i','p'{{M1x2[231 244],M[0],}}},{M[4],M[0],T\"prevErrors\",S'l','i','p'{{M1x2[420 430],M[0],}}},{M[4],M[0],T\"prevTotalPackets\",S'l','i','p'{{M1x2[314 330],M[0],}}},{M[8],M[0],T\"is_active_c4_sensor_block_complete\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 8, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_sensor_block_complete_get_check_sum(&mxChecksum);
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
           4,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,"update",0,
                              (MexFcnForType)c4_sf_marshall);
          _SFD_SET_DATA_PROPS(1,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_success",0,(MexFcnForType)c4_b_sf_marshall);
          _SFD_SET_DATA_PROPS(2,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_total",0,(MexFcnForType)c4_b_sf_marshall);
          _SFD_SET_DATA_PROPS(3,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"errors",
                              0,(MexFcnForType)c4_c_sf_marshall);
          _SFD_SET_DATA_PROPS(4,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "reliability",0,(MexFcnForType)c4_c_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,6,0,0,0,2,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1874);
        _SFD_CV_INIT_EML_IF(0,0,539,555,-1,741);
        _SFD_CV_INIT_EML_IF(0,1,793,817,-1,1704);
        _SFD_CV_INIT_EML_IF(0,2,873,892,-1,925);
        _SFD_CV_INIT_EML_IF(0,3,1127,1161,-1,1209);
        _SFD_CV_INIT_EML_IF(0,4,1444,1477,-1,1525);
        _SFD_CV_INIT_EML_IF(0,5,1756,1782,1805,1874);

        {
          static int condStart[] = { 797, 807 };

          static int condEnd[] = { 803, 816 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,0,797,816,2,0,&(condStart[0]),&(condEnd[0]),3,
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
          boolean_T *c4_update;
          uint16_T *c4_pck_success;
          uint16_T *c4_pck_total;
          real_T *c4_errors;
          real_T *c4_reliability;
          c4_pck_success = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
          c4_update = (boolean_T *)ssGetInputPortSignal(chartInstance.S, 0);
          c4_reliability = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          c4_errors = (real_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          c4_pck_total = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
          _SFD_SET_DATA_VALUE_PTR(0U, c4_update);
          _SFD_SET_DATA_VALUE_PTR(1U, c4_pck_success);
          _SFD_SET_DATA_VALUE_PTR(2U, c4_pck_total);
          _SFD_SET_DATA_VALUE_PTR(3U, c4_errors);
          _SFD_SET_DATA_VALUE_PTR(4U, c4_reliability);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_sensor_block_completeMachineNumber_,chartInstance.chartNumber,
         chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c4_sensor_block_complete(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c4_sensor_block_complete();
  initialize_c4_sensor_block_complete();
}

static void sf_opaque_enable_c4_sensor_block_complete(void *chartInstanceVar)
{
  enable_c4_sensor_block_complete();
}

static void sf_opaque_disable_c4_sensor_block_complete(void *chartInstanceVar)
{
  disable_c4_sensor_block_complete();
}

static void sf_opaque_gateway_c4_sensor_block_complete(void *chartInstanceVar)
{
  sf_c4_sensor_block_complete();
}

static mxArray* sf_opaque_get_sim_state_c4_sensor_block_complete(void
  *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c4_sensor_block_complete();
  return st;
}

static void sf_opaque_set_sim_state_c4_sensor_block_complete(void
  *chartInstanceVar, const mxArray *st)
{
  set_sim_state_c4_sensor_block_complete(sf_mex_dup(st));
}

static void sf_opaque_terminate_c4_sensor_block_complete(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c4_sensor_block_complete();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_sensor_block_complete(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_sensor_block_complete();
  }
}

static void mdlSetWorkWidths_c4_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block_complete",
      "sensor_block_complete",4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block_complete",
                "sensor_block_complete",4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block_complete",
      "sensor_block_complete",4,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block_complete",
        "sensor_block_complete",4,3);
      sf_mark_chart_reusable_outputs(S,"sensor_block_complete",
        "sensor_block_complete",4,2);
    }

    sf_set_rtw_dwork_info(S,"sensor_block_complete","sensor_block_complete",4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(431461523U));
  ssSetChecksum1(S,(2075549274U));
  ssSetChecksum2(S,(4147509733U));
  ssSetChecksum3(S,(2799492282U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c4_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block_complete", "sensor_block_complete",
      4);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_sensor_block_complete(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_sensor_block_complete;
  chartInstance.chartInfo.initializeChart =
    sf_opaque_initialize_c4_sensor_block_complete;
  chartInstance.chartInfo.terminateChart =
    sf_opaque_terminate_c4_sensor_block_complete;
  chartInstance.chartInfo.enableChart =
    sf_opaque_enable_c4_sensor_block_complete;
  chartInstance.chartInfo.disableChart =
    sf_opaque_disable_c4_sensor_block_complete;
  chartInstance.chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_sensor_block_complete;
  chartInstance.chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_sensor_block_complete;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_sensor_block_complete;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c4_sensor_block_complete;
  chartInstance.chartInfo.mdlStart = mdlStart_c4_sensor_block_complete;
  chartInstance.chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_sensor_block_complete;
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

void c4_sensor_block_complete_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_sensor_block_complete(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_sensor_block_complete_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
