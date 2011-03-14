/* Include files */

#include "blascompat32.h"
#include "SendFetchS_sfun.h"
#include "c2_SendFetchS.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SendFetchS_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c2_debug_family_names[13] = { "UPDATE_INTERVAL", "NUMBER_WT",
  "j", "wtId", "i", "nargin", "nargout", "y", "instantU", "u", "y_l1", "y_l2",
  "currentFrame" };

/* Function Declarations */
static void initialize_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance);
static void initialize_params_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance);
static void enable_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance);
static void disable_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *
  chartInstance);
static void set_sim_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance);
static void sf_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static void c2_eml_scalar_eg(SFc2_SendFetchSInstanceStruct *chartInstance);
static real_T c2_mod(SFc2_SendFetchSInstanceStruct *chartInstance, real_T c2_x,
                     real_T c2_y);
static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_f_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static const mxArray *c2_g_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[46]);
static const mxArray *c2_h_sf_marshall(void *chartInstanceVoid, void *c2_b_u);
static void c2_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_instantU, const char_T *c2_name, uint16_T c2_y[10]);
static real_T c2_b_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_currentFrame, const char_T *c2_name);
static void c2_c_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_u, const char_T *c2_name, real_T c2_y[200]);
static void c2_d_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_y_l1, const char_T *c2_name, real_T c2_y[200]);
static void c2_e_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_y_l2, const char_T *c2_name, real_T c2_y[200]);
static uint8_T c2_f_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_SendFetchS, const char_T
  *c2_name);
static void init_dsm_address_info(SFc2_SendFetchSInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_u_not_empty = FALSE;
  chartInstance->c2_y_l1_not_empty = FALSE;
  chartInstance->c2_y_l2_not_empty = FALSE;
  chartInstance->c2_currentFrame_not_empty = FALSE;
  chartInstance->c2_is_active_c2_SendFetchS = 0U;
}

static void initialize_params_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance)
{
}

static void enable_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *
  chartInstance)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  uint16_T c2_hoistedGlobal[10];
  int32_T c2_i1;
  uint16_T c2_b_u[10];
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_c_y = NULL;
  int32_T c2_i2;
  real_T c2_c_hoistedGlobal[200];
  int32_T c2_i3;
  real_T c2_d_u[200];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i4;
  real_T c2_d_hoistedGlobal[200];
  int32_T c2_i5;
  real_T c2_e_u[200];
  const mxArray *c2_e_y = NULL;
  int32_T c2_i6;
  real_T c2_e_hoistedGlobal[200];
  int32_T c2_i7;
  real_T c2_f_u[200];
  const mxArray *c2_f_y = NULL;
  uint8_T c2_f_hoistedGlobal;
  uint8_T c2_g_u;
  const mxArray *c2_g_y = NULL;
  uint16_T (*c2_instantU)[10];
  c2_instantU = (uint16_T (*)[10])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(6));
  for (c2_i0 = 0; c2_i0 < 10; c2_i0 = c2_i0 + 1) {
    c2_hoistedGlobal[c2_i0] = (*c2_instantU)[c2_i0];
  }

  for (c2_i1 = 0; c2_i1 < 10; c2_i1 = c2_i1 + 1) {
    c2_b_u[c2_i1] = c2_hoistedGlobal[c2_i1];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 5, 0U, 1U, 0U, 2, 1, 10));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = chartInstance->c2_currentFrame;
  c2_c_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  if (!chartInstance->c2_currentFrame_not_empty) {
    sf_mex_assign(&c2_c_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c2_y, 1, c2_c_y);
  for (c2_i2 = 0; c2_i2 < 200; c2_i2 = c2_i2 + 1) {
    c2_c_hoistedGlobal[c2_i2] = chartInstance->c2_u[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 200; c2_i3 = c2_i3 + 1) {
    c2_d_u[c2_i3] = c2_c_hoistedGlobal[c2_i3];
  }

  c2_d_y = NULL;
  if (!chartInstance->c2_u_not_empty) {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_d_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_setcell(c2_y, 2, c2_d_y);
  for (c2_i4 = 0; c2_i4 < 200; c2_i4 = c2_i4 + 1) {
    c2_d_hoistedGlobal[c2_i4] = chartInstance->c2_y_l1[c2_i4];
  }

  for (c2_i5 = 0; c2_i5 < 200; c2_i5 = c2_i5 + 1) {
    c2_e_u[c2_i5] = c2_d_hoistedGlobal[c2_i5];
  }

  c2_e_y = NULL;
  if (!chartInstance->c2_y_l1_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_e_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i6 = 0; c2_i6 < 200; c2_i6 = c2_i6 + 1) {
    c2_e_hoistedGlobal[c2_i6] = chartInstance->c2_y_l2[c2_i6];
  }

  for (c2_i7 = 0; c2_i7 < 200; c2_i7 = c2_i7 + 1) {
    c2_f_u[c2_i7] = c2_e_hoistedGlobal[c2_i7];
  }

  c2_f_y = NULL;
  if (!chartInstance->c2_y_l2_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_f_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = chartInstance->c2_is_active_c2_SendFetchS;
  c2_g_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_g_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 5, c2_g_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_SendFetchS(SFc2_SendFetchSInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_b_u;
  uint16_T c2_uv0[10];
  int32_T c2_i8;
  real_T c2_dv0[200];
  int32_T c2_i9;
  real_T c2_dv1[200];
  int32_T c2_i10;
  real_T c2_dv2[200];
  int32_T c2_i11;
  uint16_T (*c2_instantU)[10];
  c2_instantU = (uint16_T (*)[10])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_b_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_b_u, 0)),
                      "instantU", c2_uv0);
  for (c2_i8 = 0; c2_i8 < 10; c2_i8 = c2_i8 + 1) {
    (*c2_instantU)[c2_i8] = c2_uv0[c2_i8];
  }

  chartInstance->c2_currentFrame = c2_b_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_b_u, 1)), "currentFrame");
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_b_u, 2)),
                        "u", c2_dv0);
  for (c2_i9 = 0; c2_i9 < 200; c2_i9 = c2_i9 + 1) {
    chartInstance->c2_u[c2_i9] = c2_dv0[c2_i9];
  }

  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_b_u, 3)),
                        "y_l1", c2_dv1);
  for (c2_i10 = 0; c2_i10 < 200; c2_i10 = c2_i10 + 1) {
    chartInstance->c2_y_l1[c2_i10] = c2_dv1[c2_i10];
  }

  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_b_u, 4)),
                        "y_l2", c2_dv2);
  for (c2_i11 = 0; c2_i11 < 200; c2_i11 = c2_i11 + 1) {
    chartInstance->c2_y_l2[c2_i11] = c2_dv2[c2_i11];
  }

  chartInstance->c2_is_active_c2_SendFetchS = c2_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_b_u, 5)),
     "is_active_c2_SendFetchS");
  sf_mex_destroy(&c2_b_u);
  c2_update_debugger_state_c2_SendFetchS(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance)
{
}

static void sf_c2_SendFetchS(SFc2_SendFetchSInstanceStruct *chartInstance)
{
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_previousEvent;
  int32_T c2_i14;
  uint16_T c2_hoistedGlobal[20];
  int32_T c2_i15;
  uint16_T c2_y[20];
  uint32_T c2_debug_family_var_map[13];
  static const char *c2_sv0[13] = { "UPDATE_INTERVAL", "NUMBER_WT", "j", "wtId",
    "i", "nargin", "nargout", "y", "instantU", "u",
    "y_l1", "y_l2", "currentFrame" };

  real_T c2_UPDATE_INTERVAL;
  real_T c2_NUMBER_WT;
  real_T c2_j;
  real_T c2_wtId;
  real_T c2_i;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  uint16_T c2_instantU[10];
  int32_T c2_i16;
  int32_T c2_i17;
  int32_T c2_i18;
  real_T c2_b_j;
  int32_T c2_i19;
  real_T c2_b_hoistedGlobal[200];
  int32_T c2_c_j;
  int32_T c2_i20;
  real_T c2_a[10];
  int32_T c2_i21;
  real_T c2_b_y[10];
  real_T c2_b;
  real_T c2_c_y;
  int32_T c2_d_j;
  int32_T c2_i22;
  real_T c2_b_i;
  int32_T c2_i23;
  int32_T c2_i24;
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_d2;
  uint16_T c2_u0;
  int32_T c2_i25;
  uint16_T (*c2_b_instantU)[10];
  uint16_T (*c2_d_y)[20];
  c2_d_y = (uint16_T (*)[20])ssGetInputPortSignal(chartInstance->S, 0);
  c2_b_instantU = (uint16_T (*)[10])ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,0);
  for (c2_i12 = 0; c2_i12 < 10; c2_i12 = c2_i12 + 1) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_b_instantU)[c2_i12], 0U);
  }

  for (c2_i13 = 0; c2_i13 < 20; c2_i13 = c2_i13 + 1) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_d_y)[c2_i13], 1U);
  }

  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  for (c2_i14 = 0; c2_i14 < 20; c2_i14 = c2_i14 + 1) {
    c2_hoistedGlobal[c2_i14] = (*c2_d_y)[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 20; c2_i15 = c2_i15 + 1) {
    c2_y[c2_i15] = c2_hoistedGlobal[c2_i15];
  }

  sf_debug_symbol_scope_push_eml(0U, 13U, 13U, c2_sv0, c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_UPDATE_INTERVAL, c2_g_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c2_NUMBER_WT, c2_g_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c2_j, c2_g_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c2_wtId, c2_g_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c2_i, c2_g_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c2_nargin, c2_g_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c2_nargout, c2_g_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c2_y, c2_f_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c2_instantU, c2_e_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&chartInstance->c2_u, c2_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&chartInstance->c2_y_l1, c2_c_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&chartInstance->c2_y_l2, c2_b_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&chartInstance->c2_currentFrame, c2_sf_marshall,
    12U);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0,3);
  c2_UPDATE_INTERVAL = 20.0;
  _SFD_EML_CALL(0,4);
  c2_NUMBER_WT = 10.0;
  _SFD_EML_CALL(0,7);
  if (CV_EML_IF(0, 0, !chartInstance->c2_u_not_empty)) {
    _SFD_EML_CALL(0,8);
    for (c2_i16 = 0; c2_i16 < 200; c2_i16 = c2_i16 + 1) {
      chartInstance->c2_y_l1[c2_i16] = 0.0;
    }

    chartInstance->c2_y_l1_not_empty = TRUE;
    _SFD_EML_CALL(0,9);
    for (c2_i17 = 0; c2_i17 < 200; c2_i17 = c2_i17 + 1) {
      chartInstance->c2_y_l2[c2_i17] = 0.0;
    }

    chartInstance->c2_y_l2_not_empty = TRUE;
    _SFD_EML_CALL(0,10);
    for (c2_i18 = 0; c2_i18 < 200; c2_i18 = c2_i18 + 1) {
      chartInstance->c2_u[c2_i18] = 1.0;
    }

    chartInstance->c2_u_not_empty = TRUE;

    /*  init the control voltages */
    c2_j = 1.0;
    c2_b_j = 1.0;
    while (c2_b_j <= 20.0) {
      c2_j = c2_b_j;
      CV_EML_FOR(0, 0, 1);
      _SFD_EML_CALL(0,12);
      for (c2_i19 = 0; c2_i19 < 200; c2_i19 = c2_i19 + 1) {
        c2_b_hoistedGlobal[c2_i19] = chartInstance->c2_u[c2_i19];
      }

      c2_c_j = _SFD_EML_ARRAY_BOUNDS_CHECK("u", (int32_T)_SFD_INTEGER_CHECK("j",
        c2_j), 1, 20, 1, 0) - 1;
      for (c2_i20 = 0; c2_i20 < 10; c2_i20 = c2_i20 + 1) {
        c2_a[c2_i20] = c2_b_hoistedGlobal[c2_c_j + 20 * c2_i20];
      }

      for (c2_i21 = 0; c2_i21 < 10; c2_i21 = c2_i21 + 1) {
        c2_b_y[c2_i21] = c2_a[c2_i21] * 1024.0;
      }

      c2_b = c2_j;
      c2_c_y = 512.0 * c2_b;
      c2_d_j = _SFD_EML_ARRAY_BOUNDS_CHECK("u", (int32_T)_SFD_INTEGER_CHECK("j",
        c2_j), 1, 20, 1, 0) - 1;
      for (c2_i22 = 0; c2_i22 < 10; c2_i22 = c2_i22 + 1) {
        chartInstance->c2_u[c2_d_j + 20 * c2_i22] = c2_b_y[c2_i22] + c2_c_y;
      }

      c2_b_j = c2_b_j + 1.0;
      sf_mex_listen_for_ctrl_c(chartInstance->S);
    }

    CV_EML_FOR(0, 0, 0);
    _SFD_EML_CALL(0,14);
    chartInstance->c2_currentFrame = 1.0;
    chartInstance->c2_currentFrame_not_empty = TRUE;
  }

  /*  the payload is as follows */
  /*  payload = [wt1_l1 wt1_l2 wt2_l1 wt2_l2 ..... wtN_l1 wtN_l2] */
  /* % update the values in the matrices */
  _SFD_EML_CALL(0,22);
  c2_wtId = 1.0;
  c2_i = 1.0;
  c2_b_i = 1.0;
  while (c2_b_i <= 20.0) {
    c2_i = c2_b_i;
    CV_EML_FOR(0, 1, 1);
    _SFD_EML_CALL(0,24);
    if (CV_EML_IF(0, 1, c2_i <= c2_NUMBER_WT)) {
      _SFD_EML_CALL(0,25);
      chartInstance->c2_y_l1[(_SFD_EML_ARRAY_BOUNDS_CHECK("y_l1", (int32_T)
        _SFD_INTEGER_CHECK("currentFrame", chartInstance->
                           c2_currentFrame), 1, 20, 1, 0) - 1) + 20 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("y_l1", (int32_T)_SFD_INTEGER_CHECK("wtId",
           c2_wtId), 1, 10
          , 2, 0) - 1)] = (real_T)c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("y", (int32_T)
        _SFD_INTEGER_CHECK("i", c2_i), 1, 20, 1, 0) - 1];
    } else {
      _SFD_EML_CALL(0,27);
      chartInstance->c2_y_l2[(_SFD_EML_ARRAY_BOUNDS_CHECK("y_l2", (int32_T)
        _SFD_INTEGER_CHECK("currentFrame", chartInstance->
                           c2_currentFrame), 1, 20, 1, 0) - 1) + 20 *
        (_SFD_EML_ARRAY_BOUNDS_CHECK("y_l2", (int32_T)_SFD_INTEGER_CHECK("wtId",
           c2_wtId), 1, 10
          , 2, 0) - 1)] = (real_T)c2_y[_SFD_EML_ARRAY_BOUNDS_CHECK("y", (int32_T)
        _SFD_INTEGER_CHECK("i", c2_i), 1, 20, 1, 0) - 1];
    }

    _SFD_EML_CALL(0,29);
    c2_wtId = c2_mod(chartInstance, c2_wtId + 1.0, 10.0);
    _SFD_EML_CALL(0,30);
    if (CV_EML_IF(0, 2, c2_wtId == 0.0)) {
      _SFD_EML_CALL(0,31);
      c2_wtId = 1.0;
    }

    c2_b_i = c2_b_i + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0);

  /* % update the u matrix with the new values */
  _SFD_EML_CALL(0,36);
  CV_EML_IF(0, 3, chartInstance->c2_currentFrame == c2_UPDATE_INTERVAL + 1.0);
  _SFD_EML_CALL(0,40);
  c2_i23 = _SFD_EML_ARRAY_BOUNDS_CHECK("u", (int32_T)_SFD_INTEGER_CHECK(
    "currentFrame", chartInstance->c2_currentFrame), 1, 20, 1, 0)
    - 1;
  for (c2_i24 = 0; c2_i24 < 10; c2_i24 = c2_i24 + 1) {
    c2_d0 = chartInstance->c2_u[c2_i23 + 20 * c2_i24];
    c2_d1 = c2_d0;
    c2_d1 = c2_d1 < 0.0 ? muDoubleScalarCeil(c2_d1 - 0.5) : muDoubleScalarFloor
      (c2_d1 + 0.5);
    c2_d2 = c2_d1;
    if (c2_d2 < 65536.0) {
      if (c2_d2 >= 0.0) {
        c2_u0 = (uint16_T)c2_d2;
      } else {
        c2_u0 = 0U;
      }
    } else if (c2_d2 >= 65536.0) {
      c2_u0 = MAX_uint16_T;
    } else {
      c2_u0 = 0U;
    }

    c2_instantU[c2_i24] = c2_u0;
  }

  _SFD_EML_CALL(0,41);
  chartInstance->c2_currentFrame = c2_mod(chartInstance,
    chartInstance->c2_currentFrame + 1.0, 21.0);
  _SFD_EML_CALL(0,42);
  if (CV_EML_IF(0, 4, chartInstance->c2_currentFrame == 0.0)) {
    _SFD_EML_CALL(0,43);
    chartInstance->c2_currentFrame = 1.0;
  }

  _SFD_EML_CALL(0,-43);
  sf_debug_symbol_scope_pop();
  for (c2_i25 = 0; c2_i25 < 10; c2_i25 = c2_i25 + 1) {
    (*c2_b_instantU)[c2_i25] = c2_instantU[c2_i25];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_SendFetchSMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static void c2_eml_scalar_eg(SFc2_SendFetchSInstanceStruct *chartInstance)
{
}

static real_T c2_mod(SFc2_SendFetchSInstanceStruct *chartInstance, real_T c2_x,
                     real_T c2_y)
{
  real_T c2_r;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_c_y;
  real_T c2_k_x;
  real_T c2_d_y;
  real_T c2_b;
  real_T c2_e_y;
  real_T c2_l_x;
  real_T c2_m_x;
  c2_eml_scalar_eg(chartInstance);
  c2_xk = c2_x;
  c2_yk = c2_y;
  c2_b_x = c2_xk;
  c2_b_y = c2_yk;
  c2_eml_scalar_eg(chartInstance);
  if (c2_b_y == 0.0) {
    return c2_b_x;
  } else {
    c2_c_x = c2_b_y;
    c2_d_x = c2_c_x;
    c2_d_x = muDoubleScalarFloor(c2_d_x);
    if (c2_b_y == c2_d_x) {
      c2_e_x = c2_b_x / c2_b_y;
      c2_f_x = c2_e_x;
      c2_f_x = muDoubleScalarFloor(c2_f_x);
      return c2_b_x - c2_f_x * c2_b_y;
    } else {
      c2_r = c2_b_x / c2_b_y;
      c2_g_x = c2_r;
      c2_h_x = c2_g_x;
      c2_i_x = c2_h_x;
      if (c2_i_x < 0.0) {
        c2_h_x = muDoubleScalarCeil(c2_i_x - 0.5);
      } else {
        c2_h_x = muDoubleScalarFloor(c2_i_x + 0.5);
      }

      c2_j_x = c2_r - c2_h_x;
      c2_c_y = muDoubleScalarAbs(c2_j_x);
      c2_k_x = c2_r;
      c2_d_y = muDoubleScalarAbs(c2_k_x);
      c2_b = c2_d_y;
      c2_e_y = 2.2204460492503131E-16 * c2_b;
      if (c2_c_y <= c2_e_y) {
        return 0.0;
      } else {
        c2_l_x = c2_r;
        c2_m_x = c2_l_x;
        c2_m_x = muDoubleScalarFloor(c2_m_x);
        return (c2_r - c2_m_x) * c2_b_y;
      }
    }
  }
}

static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_c_u = *((real_T *)c2_b_u);
  c2_b_y = NULL;
  if (!chartInstance->c2_currentFrame_not_empty) {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_c_u[200];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_i26 = 0;
  for (c2_i27 = 0; c2_i27 < 10; c2_i27 = c2_i27 + 1) {
    for (c2_i28 = 0; c2_i28 < 20; c2_i28 = c2_i28 + 1) {
      c2_c_u[c2_i28 + c2_i26] = (*((real_T (*)[200])c2_b_u))[c2_i28 + c2_i26];
    }

    c2_i26 = c2_i26 + 20;
  }

  c2_b_y = NULL;
  if (!chartInstance->c2_y_l2_not_empty) {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_c_u[200];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_i29 = 0;
  for (c2_i30 = 0; c2_i30 < 10; c2_i30 = c2_i30 + 1) {
    for (c2_i31 = 0; c2_i31 < 20; c2_i31 = c2_i31 + 1) {
      c2_c_u[c2_i31 + c2_i29] = (*((real_T (*)[200])c2_b_u))[c2_i31 + c2_i29];
    }

    c2_i29 = c2_i29 + 20;
  }

  c2_b_y = NULL;
  if (!chartInstance->c2_y_l1_not_empty) {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i32;
  int32_T c2_i33;
  int32_T c2_i34;
  real_T c2_c_u[200];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_i32 = 0;
  for (c2_i33 = 0; c2_i33 < 10; c2_i33 = c2_i33 + 1) {
    for (c2_i34 = 0; c2_i34 < 20; c2_i34 = c2_i34 + 1) {
      c2_c_u[c2_i34 + c2_i32] = (*((real_T (*)[200])c2_b_u))[c2_i34 + c2_i32];
    }

    c2_i32 = c2_i32 + 20;
  }

  c2_b_y = NULL;
  if (!chartInstance->c2_u_not_empty) {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 0, 0U, 1U, 0U, 2, 20, 10));
  }

  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i35;
  uint16_T c2_c_u[10];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i35 = 0; c2_i35 < 10; c2_i35 = c2_i35 + 1) {
    c2_c_u[c2_i35] = (*((uint16_T (*)[10])c2_b_u))[c2_i35];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 5, 0U, 1U, 0U, 2, 1, 10));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_f_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i36;
  uint16_T c2_c_u[20];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i36 = 0; c2_i36 < 20; c2_i36 = c2_i36 + 1) {
    c2_c_u[c2_i36] = (*((uint16_T (*)[20])c2_b_u))[c2_i36];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 5, 0U, 1U, 0U, 1, 20));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_g_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_c_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_c_u = *((real_T *)c2_b_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_SendFetchS_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[46];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i37;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 46));
  for (c2_i37 = 0; c2_i37 < 46; c2_i37 = c2_i37 + 1) {
    c2_r0 = &c2_info[c2_i37];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i37);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i37);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i37);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i37);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i37);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i37
                    );
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i37
                    );
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[46])
{
  c2_info[0].context = "";
  c2_info[0].name = "length";
  c2_info[0].dominantType = "uint16";
  c2_info[0].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[0].fileLength = 326U;
  c2_info[0].fileTime1 = 1226577274U;
  c2_info[0].fileTime2 = 0U;
  c2_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[1].name = "nargin";
  c2_info[1].dominantType = "";
  c2_info[1].resolved = "[B]nargin";
  c2_info[1].fileLength = 0U;
  c2_info[1].fileTime1 = 0U;
  c2_info[1].fileTime2 = 0U;
  c2_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[2].name = "eq";
  c2_info[2].dominantType = "double";
  c2_info[2].resolved = "[B]eq";
  c2_info[2].fileLength = 0U;
  c2_info[2].fileTime1 = 0U;
  c2_info[2].fileTime2 = 0U;
  c2_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[3].name = "size";
  c2_info[3].dominantType = "uint16";
  c2_info[3].resolved = "[B]size";
  c2_info[3].fileLength = 0U;
  c2_info[3].fileTime1 = 0U;
  c2_info[3].fileTime2 = 0U;
  c2_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[4].name = "gt";
  c2_info[4].dominantType = "double";
  c2_info[4].resolved = "[B]gt";
  c2_info[4].fileLength = 0U;
  c2_info[4].fileTime1 = 0U;
  c2_info[4].fileTime2 = 0U;
  c2_info[5].context = "";
  c2_info[5].name = "mrdivide";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[5].fileLength = 800U;
  c2_info[5].fileTime1 = 1238434290U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[6].name = "ge";
  c2_info[6].dominantType = "double";
  c2_info[6].resolved = "[B]ge";
  c2_info[6].fileLength = 0U;
  c2_info[6].fileTime1 = 0U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[7].name = "isscalar";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[B]isscalar";
  c2_info[7].fileLength = 0U;
  c2_info[7].fileTime1 = 0U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[8].name = "rdivide";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[8].fileLength = 403U;
  c2_info[8].fileTime1 = 1244735552U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[9].name = "isa";
  c2_info[9].dominantType = "double";
  c2_info[9].resolved = "[B]isa";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[10].name = "eml_div";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[10].fileLength = 4269U;
  c2_info[10].fileTime1 = 1228093826U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[11].name = "isinteger";
  c2_info[11].dominantType = "double";
  c2_info[11].resolved = "[B]isinteger";
  c2_info[11].fileLength = 0U;
  c2_info[11].fileTime1 = 0U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c2_info[12].name = "isreal";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved = "[B]isreal";
  c2_info[12].fileLength = 0U;
  c2_info[12].fileTime1 = 0U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context = "";
  c2_info[13].name = "zeros";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[B]zeros";
  c2_info[13].fileLength = 0U;
  c2_info[13].fileTime1 = 0U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context = "";
  c2_info[14].name = "ones";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved = "[B]ones";
  c2_info[14].fileLength = 0U;
  c2_info[14].fileTime1 = 0U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context = "";
  c2_info[15].name = "mpower";
  c2_info[15].dominantType = "double";
  c2_info[15].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[15].fileLength = 3710U;
  c2_info[15].fileTime1 = 1238434288U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[16].name = "ndims";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved = "[B]ndims";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[17].name = "power";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[17].fileLength = 5380U;
  c2_info[17].fileTime1 = 1228093898U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[18].name = "eml_scalar_eg";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[18].fileLength = 3068U;
  c2_info[18].fileTime1 = 1240262010U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c2_info[19].name = "false";
  c2_info[19].dominantType = "";
  c2_info[19].resolved = "[B]false";
  c2_info[19].fileLength = 0U;
  c2_info[19].fileTime1 = 0U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[20].name = "isstruct";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[B]isstruct";
  c2_info[20].fileLength = 0U;
  c2_info[20].fileTime1 = 0U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[21].name = "class";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved = "[B]class";
  c2_info[21].fileLength = 0U;
  c2_info[21].fileTime1 = 0U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[22].name = "cast";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved = "[B]cast";
  c2_info[22].fileLength = 0U;
  c2_info[22].fileTime1 = 0U;
  c2_info[22].fileTime2 = 0U;
  c2_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[23].name = "plus";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved = "[B]plus";
  c2_info[23].fileLength = 0U;
  c2_info[23].fileTime1 = 0U;
  c2_info[23].fileTime2 = 0U;
  c2_info[24].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[24].name = "eml_scalexp_alloc";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[24].fileLength = 808U;
  c2_info[24].fileTime1 = 1230494698U;
  c2_info[24].fileTime2 = 0U;
  c2_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[25].name = "minus";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved = "[B]minus";
  c2_info[25].fileLength = 0U;
  c2_info[25].fileTime1 = 0U;
  c2_info[25].fileTime2 = 0U;
  c2_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[26].name = "not";
  c2_info[26].dominantType = "logical";
  c2_info[26].resolved = "[B]not";
  c2_info[26].fileLength = 0U;
  c2_info[26].fileTime1 = 0U;
  c2_info[26].fileTime2 = 0U;
  c2_info[27].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[27].name = "lt";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved = "[B]lt";
  c2_info[27].fileLength = 0U;
  c2_info[27].fileTime1 = 0U;
  c2_info[27].fileTime2 = 0U;
  c2_info[28].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[28].name = "eml_scalar_floor";
  c2_info[28].dominantType = "double";
  c2_info[28].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[28].fileLength = 260U;
  c2_info[28].fileTime1 = 1209330790U;
  c2_info[28].fileTime2 = 0U;
  c2_info[29].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[29].name = "ne";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved = "[B]ne";
  c2_info[29].fileLength = 0U;
  c2_info[29].fileTime1 = 0U;
  c2_info[29].fileTime2 = 0U;
  c2_info[30].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[30].name = "eml_error";
  c2_info[30].dominantType = "char";
  c2_info[30].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[30].fileLength = 315U;
  c2_info[30].fileTime1 = 1213926744U;
  c2_info[30].fileTime2 = 0U;
  c2_info[31].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[31].name = "strcmp";
  c2_info[31].dominantType = "char";
  c2_info[31].resolved = "[B]strcmp";
  c2_info[31].fileLength = 0U;
  c2_info[31].fileTime1 = 0U;
  c2_info[31].fileTime2 = 0U;
  c2_info[32].context = "";
  c2_info[32].name = "mtimes";
  c2_info[32].dominantType = "double";
  c2_info[32].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[32].fileLength = 3425U;
  c2_info[32].fileTime1 = 1250672766U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context = "";
  c2_info[33].name = "le";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved = "[B]le";
  c2_info[33].fileLength = 0U;
  c2_info[33].fileTime1 = 0U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context = "";
  c2_info[34].name = "mod";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c2_info[34].fileLength = 960U;
  c2_info[34].fileTime1 = 1228093788U;
  c2_info[34].fileTime2 = 0U;
  c2_info[35].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/mod.m";
  c2_info[35].name = "eml_scalar_mod";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c2_info[35].fileLength = 1103U;
  c2_info[35].fileTime1 = 1228093784U;
  c2_info[35].fileTime2 = 0U;
  c2_info[36].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c2_info[36].name = "times";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved = "[B]times";
  c2_info[36].fileLength = 0U;
  c2_info[36].fileTime1 = 0U;
  c2_info[36].fileTime2 = 0U;
  c2_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c2_info[37].name = "eml_scalar_round";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c2_info[37].fileLength = 523U;
  c2_info[37].fileTime1 = 1203448008U;
  c2_info[37].fileTime2 = 0U;
  c2_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_round.m";
  c2_info[38].name = "isfloat";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved = "[B]isfloat";
  c2_info[38].fileLength = 0U;
  c2_info[38].fileTime1 = 0U;
  c2_info[38].fileTime2 = 0U;
  c2_info[39].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c2_info[39].name = "eml_scalar_abs";
  c2_info[39].dominantType = "double";
  c2_info[39].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[39].fileLength = 461U;
  c2_info[39].fileTime1 = 1203447960U;
  c2_info[39].fileTime2 = 0U;
  c2_info[40].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[40].name = "ischar";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved = "[B]ischar";
  c2_info[40].fileLength = 0U;
  c2_info[40].fileTime1 = 0U;
  c2_info[40].fileTime2 = 0U;
  c2_info[41].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[41].name = "islogical";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved = "[B]islogical";
  c2_info[41].fileLength = 0U;
  c2_info[41].fileTime1 = 0U;
  c2_info[41].fileTime2 = 0U;
  c2_info[42].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_mod.m";
  c2_info[42].name = "eps";
  c2_info[42].dominantType = "char";
  c2_info[42].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[42].fileLength = 1331U;
  c2_info[42].fileTime1 = 1246283386U;
  c2_info[42].fileTime2 = 0U;
  c2_info[43].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[43].name = "eml_is_float_class";
  c2_info[43].dominantType = "char";
  c2_info[43].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[43].fileLength = 226U;
  c2_info[43].fileTime1 = 1197850440U;
  c2_info[43].fileTime2 = 0U;
  c2_info[44].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[44].name = "uminus";
  c2_info[44].dominantType = "double";
  c2_info[44].resolved = "[B]uminus";
  c2_info[44].fileLength = 0U;
  c2_info[44].fileTime1 = 0U;
  c2_info[44].fileTime2 = 0U;
  c2_info[45].context = "";
  c2_info[45].name = "uint16";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved = "[B]uint16";
  c2_info[45].fileLength = 0U;
  c2_info[45].fileTime1 = 0U;
  c2_info[45].fileTime2 = 0U;
}

static const mxArray *c2_h_sf_marshall(void *chartInstanceVoid, void *c2_b_u)
{
  const mxArray *c2_y = NULL;
  boolean_T c2_c_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_c_u = *((boolean_T *)c2_b_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_c_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static void c2_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_instantU, const char_T *c2_name,
  uint16_T c2_y[10])
{
  uint16_T c2_uv1[10];
  int32_T c2_i38;
  sf_mex_import(c2_name, sf_mex_dup(c2_instantU), &c2_uv1, 1, 5, 0U, 1, 0U, 2, 1,
                10);
  for (c2_i38 = 0; c2_i38 < 10; c2_i38 = c2_i38 + 1) {
    c2_y[c2_i38] = c2_uv1[c2_i38];
  }

  sf_mex_destroy(&c2_instantU);
}

static real_T c2_b_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_currentFrame, const char_T *
  c2_name)
{
  real_T c2_y;
  real_T c2_d3;
  if (mxIsEmpty(c2_b_currentFrame)) {
    chartInstance->c2_currentFrame_not_empty = FALSE;
  } else {
    chartInstance->c2_currentFrame_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_currentFrame), &c2_d3, 1, 0, 0U, 0,
                  0U, 0);
    c2_y = c2_d3;
  }

  sf_mex_destroy(&c2_b_currentFrame);
  return c2_y;
}

static void c2_c_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_u, const char_T *c2_name, real_T
  c2_y[200])
{
  real_T c2_dv3[200];
  int32_T c2_i39;
  if (mxIsEmpty(c2_b_u)) {
    chartInstance->c2_u_not_empty = FALSE;
  } else {
    chartInstance->c2_u_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_u), &c2_dv3, 1, 0, 0U, 1, 0U, 2, 20,
                  10);
    for (c2_i39 = 0; c2_i39 < 200; c2_i39 = c2_i39 + 1) {
      c2_y[c2_i39] = c2_dv3[c2_i39];
    }
  }

  sf_mex_destroy(&c2_b_u);
}

static void c2_d_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_y_l1, const char_T *c2_name,
  real_T c2_y[200])
{
  real_T c2_dv4[200];
  int32_T c2_i40;
  if (mxIsEmpty(c2_b_y_l1)) {
    chartInstance->c2_y_l1_not_empty = FALSE;
  } else {
    chartInstance->c2_y_l1_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_y_l1), &c2_dv4, 1, 0, 0U, 1, 0U, 2,
                  20, 10);
    for (c2_i40 = 0; c2_i40 < 200; c2_i40 = c2_i40 + 1) {
      c2_y[c2_i40] = c2_dv4[c2_i40];
    }
  }

  sf_mex_destroy(&c2_b_y_l1);
}

static void c2_e_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct *chartInstance,
  const mxArray *c2_b_y_l2, const char_T *c2_name,
  real_T c2_y[200])
{
  real_T c2_dv5[200];
  int32_T c2_i41;
  if (mxIsEmpty(c2_b_y_l2)) {
    chartInstance->c2_y_l2_not_empty = FALSE;
  } else {
    chartInstance->c2_y_l2_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_y_l2), &c2_dv5, 1, 0, 0U, 1, 0U, 2,
                  20, 10);
    for (c2_i41 = 0; c2_i41 < 200; c2_i41 = c2_i41 + 1) {
      c2_y[c2_i41] = c2_dv5[c2_i41];
    }
  }

  sf_mex_destroy(&c2_b_y_l2);
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_SendFetchSInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_SendFetchS,
  const char_T *c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u1;
  sf_mex_import(c2_name, sf_mex_dup(c2_b_is_active_c2_SendFetchS), &c2_u1, 1, 3,
                0U, 0, 0U, 0);
  c2_y = c2_u1;
  sf_mex_destroy(&c2_b_is_active_c2_SendFetchS);
  return c2_y;
}

static void init_dsm_address_info(SFc2_SendFetchSInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_SendFetchS_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(122179366U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(413473613U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(267836173U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4061010392U);
}

mxArray *sf_c2_SendFetchS_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(982199574U);
    pr[1] = (double)(1292537511U);
    pr[2] = (double)(2768007490U);
    pr[3] = (double)(1161303472U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(20);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(10);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_SendFetchS(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[4],T\"instantU\",},{M[4],M[0],T\"currentFrame\",S'l','i','p'{{M1x2[99 111],M[0],}}},{M[4],M[0],T\"u\",S'l','i','p'{{M1x2[97 98],M[0],}}},{M[4],M[0],T\"y_l1\",S'l','i','p'{{M1x2[87 91],M[0],}}},{M[4],M[0],T\"y_l2\",S'l','i','p'{{M1x2[92 96],M[0],}}},{M[8],M[0],T\"is_active_c2_SendFetchS\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_SendFetchS_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_SendFetchSInstanceStruct *chartInstance;
    chartInstance = (SFc2_SendFetchSInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_SendFetchSMachineNumber_,
           2,
           1,
           1,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_SendFetchSMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_SendFetchSMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_SendFetchSMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);

          {
            unsigned int dimVector[2];
            dimVector[0]= 1;
            dimVector[1]= 10;
            _SFD_SET_DATA_PROPS(0,2,0,1,SF_UINT16,2,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"instantU",0,(MexFcnForType)
                                c2_e_sf_marshall);
          }

          {
            unsigned int dimVector[1];
            dimVector[0]= 20;
            _SFD_SET_DATA_PROPS(1,1,1,0,SF_UINT16,1,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"y",0,(MexFcnForType)c2_f_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,5,0,0,2,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1017);
        _SFD_CV_INIT_EML_IF(0,0,114,127,-1,397);
        _SFD_CV_INIT_EML_IF(0,1,568,585,630,682);
        _SFD_CV_INIT_EML_IF(0,2,723,735,-1,761);
        _SFD_CV_INIT_EML_IF(0,3,810,847,-1,872);
        _SFD_CV_INIT_EML_IF(0,4,970,990,-1,1016);
        _SFD_CV_INIT_EML_FOR(0,0,300,318,367);
        _SFD_CV_INIT_EML_FOR(0,1,543,561,765);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          uint16_T (*c2_instantU)[10];
          uint16_T (*c2_y)[20];
          c2_y = (uint16_T (*)[20])ssGetInputPortSignal(chartInstance->S, 0);
          c2_instantU = (uint16_T (*)[10])ssGetOutputPortSignal(chartInstance->S,
            1);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_instantU);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_y);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_SendFetchSMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_SendFetchS(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_SendFetchSInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*)
    chartInstanceVar);
  initialize_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_SendFetchS(void *chartInstanceVar)
{
  enable_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_SendFetchS(void *chartInstanceVar)
{
  disable_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_SendFetchS(void *chartInstanceVar)
{
  sf_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c2_SendFetchS(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_SendFetchS
    ((SFc2_SendFetchSInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c2_SendFetchS();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

static void sf_internal_set_sim_state_c2_SendFetchS(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_SendFetchS();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c2_SendFetchS(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_SendFetchS(S);
}

static void sf_opaque_set_sim_state_c2_SendFetchS(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c2_SendFetchS(S, st);
}

static void sf_opaque_terminate_c2_SendFetchS(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_SendFetchSInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_SendFetchS(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_SendFetchS((SFc2_SendFetchSInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_SendFetchS(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("SendFetchS","SendFetchS",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("SendFetchS","SendFetchS",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("SendFetchS","SendFetchS",
      2,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"SendFetchS","SendFetchS",2,1);
      sf_mark_chart_reusable_outputs(S,"SendFetchS","SendFetchS",2,1);
    }

    sf_set_rtw_dwork_info(S,"SendFetchS","SendFetchS",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(1502698007U));
  ssSetChecksum1(S,(2212902653U));
  ssSetChecksum2(S,(535891017U));
  ssSetChecksum3(S,(3460203141U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_SendFetchS(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "SendFetchS", "SendFetchS",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_SendFetchS(SimStruct *S)
{
  SFc2_SendFetchSInstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchSInstanceStruct *)malloc(sizeof
    (SFc2_SendFetchSInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_SendFetchSInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_SendFetchS;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_SendFetchS;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_SendFetchS;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_SendFetchS;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_SendFetchS;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_SendFetchS;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_SendFetchS;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_SendFetchS;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_SendFetchS;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_SendFetchS;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_SendFetchS;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  if (!sim_mode_is_rtw_gen(S)) {
    init_dsm_address_info(chartInstance);
  }

  chart_debug_initialization(S,1);
}

void c2_SendFetchS_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_SendFetchS(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_SendFetchS(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_SendFetchS(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_SendFetchS_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
