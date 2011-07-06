/* Include files */

#include "blascompat32.h"
#include "sensor_block_sfun.h"
#include "c2_sensor_block.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c2_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc2_sensor_blockInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c2_sensor_block(void);
static void initialize_params_c2_sensor_block(void);
static void enable_c2_sensor_block(void);
static void disable_c2_sensor_block(void);
static void c2_update_debugger_state_c2_sensor_block(void);
static const mxArray *get_sim_state_c2_sensor_block(void);
static void set_sim_state_c2_sensor_block(const mxArray *c2_st);
static void finalize_c2_sensor_block(void);
static void sf_c2_sensor_block(void);
static void c2_c2_sensor_block(void);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static real_T c2_length(void);
static uint16_T c2_mrdivide(uint16_T c2_A, real_T c2_B);
static void c2_eml_warning(void);
static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[39]);
static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c2_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c2_is_active_c2_sensor_block = 0U;
}

static void initialize_params_c2_sensor_block(void)
{
}

static void enable_c2_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c2_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c2_update_debugger_state_c2_sensor_block(void)
{
}

static const mxArray *get_sim_state_c2_sensor_block(void)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  uint16_T c2_u;
  const mxArray *c2_b_y = NULL;
  uint16_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint16_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  uint16_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  uint8_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  uint16_T *c2_pck_success;
  uint16_T *c2_pck_total;
  uint16_T *c2_x1;
  uint16_T *c2_x2;
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_x2 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_x1 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(5));
  c2_u = *c2_pck_success;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_u = *c2_pck_total;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_u = *c2_x1;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_u = *c2_x2;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_u = chartInstance.c2_is_active_c2_sensor_block;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 4, c2_f_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_sensor_block(const mxArray *c2_st)
{
  const mxArray *c2_u;
  const mxArray *c2_pck_success;
  uint16_T c2_u0;
  uint16_T c2_y;
  const mxArray *c2_b_pck_success;
  uint16_T c2_u1;
  uint16_T c2_b_y;
  const mxArray *c2_c_pck_success;
  uint16_T c2_u2;
  uint16_T c2_c_y;
  const mxArray *c2_d_pck_success;
  uint16_T c2_u3;
  uint16_T c2_d_y;
  const mxArray *c2_b_is_active_c2_sensor_block;
  uint8_T c2_u4;
  uint8_T c2_e_y;
  uint16_T *c2_e_pck_success;
  uint16_T *c2_pck_total;
  uint16_T *c2_x1;
  uint16_T *c2_x2;
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_e_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_x2 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_x1 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  chartInstance.c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_pck_success = sf_mex_dup(sf_mex_getcell(c2_u, 0));
  sf_mex_import("pck_success", sf_mex_dup(c2_pck_success), &c2_u0, 1, 5, 0U, 0,
                0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_pck_success);
  *c2_e_pck_success = c2_y;
  c2_b_pck_success = sf_mex_dup(sf_mex_getcell(c2_u, 1));
  sf_mex_import("pck_total", sf_mex_dup(c2_b_pck_success), &c2_u1, 1, 5, 0U, 0,
                0U, 0);
  c2_b_y = c2_u1;
  sf_mex_destroy(&c2_b_pck_success);
  *c2_pck_total = c2_b_y;
  c2_c_pck_success = sf_mex_dup(sf_mex_getcell(c2_u, 2));
  sf_mex_import("x1", sf_mex_dup(c2_c_pck_success), &c2_u2, 1, 5, 0U, 0, 0U, 0);
  c2_c_y = c2_u2;
  sf_mex_destroy(&c2_c_pck_success);
  *c2_x1 = c2_c_y;
  c2_d_pck_success = sf_mex_dup(sf_mex_getcell(c2_u, 3));
  sf_mex_import("x2", sf_mex_dup(c2_d_pck_success), &c2_u3, 1, 5, 0U, 0, 0U, 0);
  c2_d_y = c2_u3;
  sf_mex_destroy(&c2_d_pck_success);
  *c2_x2 = c2_d_y;
  c2_b_is_active_c2_sensor_block = sf_mex_dup(sf_mex_getcell(c2_u, 4));
  sf_mex_import("is_active_c2_sensor_block", sf_mex_dup
                (c2_b_is_active_c2_sensor_block), &c2_u4, 1, 3, 0U, 0, 0U, 0);
  c2_e_y = c2_u4;
  sf_mex_destroy(&c2_b_is_active_c2_sensor_block);
  chartInstance.c2_is_active_c2_sensor_block = c2_e_y;
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_sensor_block();
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_sensor_block(void)
{
}

static void sf_c2_sensor_block(void)
{
  int32_T c2_i0;
  int32_T c2_previousEvent;
  uint16_T *c2_x1;
  uint16_T *c2_x2;
  uint16_T *c2_pck_success;
  uint16_T *c2_pck_total;
  uint8_T (*c2_u)[18];
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_x2 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_u = (uint8_T (*)[18])ssGetInputPortSignal(chartInstance.S, 0);
  c2_x1 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,0);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_x1, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_x2, 1U);
  for (c2_i0 = 0; c2_i0 < 18; c2_i0 = c2_i0 + 1) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_u)[c2_i0], 2U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c2_pck_success, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_pck_total, 4U);
  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c2_c2_sensor_block();
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_blockMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber);
}

static void c2_c2_sensor_block(void)
{
  int32_T c2_i1;
  uint8_T c2_u[18];
  real_T c2_nargout = 4.0;
  real_T c2_nargin = 1.0;
  uint16_T c2_delay;
  real_T c2_l;
  uint16_T c2_pck_total;
  uint16_T c2_pck_success;
  uint16_T c2_x2;
  uint16_T c2_x1;
  uint16_T c2_a;
  uint16_T c2_a1;
  uint16_T c2_b_a;
  uint16_T c2_b;
  uint16_T c2_c;
  uint16_T c2_c_a;
  uint16_T c2_b_a1;
  uint16_T c2_d_a;
  uint16_T c2_b_b;
  uint16_T c2_b_c;
  uint16_T c2_e_a;
  uint16_T c2_c_a1;
  uint16_T c2_f_a;
  uint16_T c2_c_b;
  uint16_T c2_g_a;
  uint16_T c2_d_a1;
  uint16_T c2_h_a;
  uint16_T c2_d_b;
  uint16_T c2_i_a;
  uint16_T c2_e_a1;
  uint16_T c2_j_a;
  uint16_T c2_e_b;
  uint16_T *c2_b_pck_total;
  uint16_T *c2_b_pck_success;
  uint16_T *c2_b_x2;
  uint16_T *c2_b_x1;
  uint8_T (*c2_b_u)[18];
  c2_b_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_b_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_b_x2 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_b_u = (uint8_T (*)[18])ssGetInputPortSignal(chartInstance.S, 0);
  c2_b_x1 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  for (c2_i1 = 0; c2_i1 < 18; c2_i1 = c2_i1 + 1) {
    c2_u[c2_i1] = (*c2_b_u)[c2_i1];
  }

  sf_debug_symbol_scope_push(9U, 0U);
  sf_debug_symbol_scope_add("nargout", &c2_nargout, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c2_nargin, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("delay", &c2_delay, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("l", &c2_l, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("pck_total", &c2_pck_total, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("pck_success", &c2_pck_success, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("x2", &c2_x2, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("x1", &c2_x1, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("u", &c2_u, c2_sf_marshall);
  CV_EML_FCN(0, 0);

  /*  02    0f     61 88   e7     34 12     03 00      00 00    00 13    04 e3    0f ff    69    c9     00        c0  2d  40  58  */
  /*  02    0f     61 88   d2     34 12     03 00      00 00    00 00    00 00    07 ff    69    c2     84        32 8d de 02  */
  /*  02    0f     61 88   be     34 12     03 00      00 00    00 00    00 00    07 ff    67    ca     00        c4 f0 8d 03  */
  /*  id| length|        |seqnum|  PANID  | Src adr | Dst addr | xc    | theta  |  u_out  | lqi | rssi| mhrLen | timestamp */
  /*             */
  /*
     02   0f     61 88    9f     34 12     00 00      02 00     00 00     4b      4b         02 04      6b    c2     00       ba 15 28 00  */
  /*
     id| length|        |seqnum|  PANID  | Dest adr | Src addr | xc    | success |  total  | timestamp | lqi | rssi| mhrLen | timestamp */
  /* 00 ff ff 00 00 0a 00 0a 03 47 03 45 00 3e 00 3d 01 e6  */
  _SFD_EML_CALL(0,14);
  c2_l = c2_length();

  /*  */
  /*  packet information */
  /*  */
  /*  Sensors and control voltage */
  /*  */
  /*  xc_sensor=bitor(bitshift(uint16(u(14)),8), uint16(u(15)))*9.2E-5; */
  /*  theta_sensor=bitor(bitshift(uint16(u(12)),8), uint16(u(13))) / 1024.0 * 2 * 3.14159265; */
  /*  u_control = bitor(bitshift(uint16(u(16)),8), uint16(u(17))); */
  /*  u_control = (u_control / 1638) * 4.8 -6; */
  /*  Sensor values */
  _SFD_EML_CALL(0,29);
  c2_a = (uint16_T)c2_u[8];
  c2_a1 = c2_a;
  c2_b_a = (uint16_T)(c2_a1 << 8);
  c2_b = (uint16_T)c2_u[9];
  c2_c = (uint16_T)(c2_b_a | c2_b);
  c2_x1 = c2_mrdivide(c2_c, 8.73932E+01);
  _SFD_EML_CALL(0,30);
  c2_c_a = (uint16_T)c2_u[10];
  c2_b_a1 = c2_c_a;
  c2_d_a = (uint16_T)(c2_b_a1 << 8);
  c2_b_b = (uint16_T)c2_u[11];
  c2_b_c = (uint16_T)(c2_d_a | c2_b_b);
  c2_x2 = c2_mrdivide(c2_b_c, 8.73932E+01);
  _SFD_EML_CALL(0,32);
  c2_e_a = (uint16_T)c2_u[12];
  c2_c_a1 = c2_e_a;
  c2_f_a = (uint16_T)(c2_c_a1 << 8);
  c2_c_b = (uint16_T)c2_u[13];
  c2_pck_total = (uint16_T)(c2_f_a | c2_c_b);
  _SFD_EML_CALL(0,33);
  c2_g_a = (uint16_T)c2_u[14];
  c2_d_a1 = c2_g_a;
  c2_h_a = (uint16_T)(c2_d_a1 << 8);
  c2_d_b = (uint16_T)c2_u[15];
  c2_pck_success = (uint16_T)(c2_h_a | c2_d_b);
  _SFD_EML_CALL(0,34);
  c2_i_a = (uint16_T)c2_u[16];
  c2_e_a1 = c2_i_a;
  c2_j_a = (uint16_T)(c2_e_a1 << 8);
  c2_e_b = (uint16_T)c2_u[17];
  c2_delay = (uint16_T)(c2_j_a | c2_e_b);
  _SFD_EML_CALL(0,-34);
  sf_debug_symbol_scope_pop();
  *c2_b_x1 = c2_x1;
  *c2_b_x2 = c2_x2;
  *c2_b_pck_success = c2_pck_success;
  *c2_b_pck_total = c2_pck_total;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static real_T c2_length(void)
{
  real_T c2_n;
  real_T c2_k;
  real_T c2_b_k;
  int32_T c2_hoistedExpr;
  int32_T c2_b_hoistedExpr;
  int32_T c2_c_hoistedExpr;
  c2_n = 0.0;
  c2_k = 1.0;
 label_1:
  ;
  if (c2_k <= 2.0) {
    c2_b_k = c2_k;
    c2_hoistedExpr = _SFD_EML_ARRAY_BOUNDS_CHECK("s", (int32_T)
      _SFD_INTEGER_CHECK("k", c2_b_k), 1, 2, 1, 0) - 1;
    if (18.0 + -17.0 * (real_T)c2_hoistedExpr == 0.0) {
      return 0.0;
    } else {
      c2_b_hoistedExpr = _SFD_EML_ARRAY_BOUNDS_CHECK("s", (int32_T)
        _SFD_INTEGER_CHECK("k", c2_b_k), 1, 2, 1, 0) - 1;
      if (18.0 + -17.0 * (real_T)c2_b_hoistedExpr > c2_n) {
        c2_c_hoistedExpr = _SFD_EML_ARRAY_BOUNDS_CHECK("s", (int32_T)
          _SFD_INTEGER_CHECK("k", c2_b_k), 1, 2, 1, 0) - 1;
        c2_n = 18.0 + -17.0 * (real_T)c2_c_hoistedExpr;
      }

      c2_k = c2_k + 1.0;
      goto label_1;
    }
  }

  return c2_n;
}

static uint16_T c2_mrdivide(uint16_T c2_A, real_T c2_B)
{
  uint16_T c2_x;
  real_T c2_b_y;
  uint16_T c2_b_x;
  real_T c2_c_y;
  uint16_T c2_c_x;
  real_T c2_d_y;
  uint16_T c2_d_x;
  real_T c2_e_y;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_zk;
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_d2;
  uint16_T c2_u5;
  c2_x = c2_A;
  c2_b_y = c2_B;
  if (c2_b_y == 0.0) {
    c2_eml_warning();
  }

  c2_b_x = c2_x;
  c2_c_y = c2_b_y;
  c2_c_x = c2_b_x;
  c2_d_y = c2_c_y;
  c2_d_x = c2_c_x;
  c2_e_y = c2_d_y;
  c2_xk = (real_T)c2_d_x;
  c2_yk = c2_e_y;
  c2_zk = c2_xk / c2_yk;
  c2_d0 = c2_zk;
  c2_d1 = c2_d0;
  c2_d1 = c2_d1 < 0.0 ? muDoubleScalarCeil(c2_d1 - 0.5) : muDoubleScalarFloor
    (c2_d1 + 0.5);
  c2_d2 = c2_d1;
  if (c2_d2 < 65536.0) {
    if (c2_d2 >= 0.0) {
      c2_u5 = (uint16_T)c2_d2;
    } else {
      c2_u5 = 0U;
    }
  } else if (c2_d2 >= 65536.0) {
    c2_u5 = MAX_uint16_T;
  } else {
    c2_u5 = 0U;
  }

  return c2_u5;
}

static void c2_eml_warning(void)
{
  int32_T c2_i2;
  static char_T c2_cv0[15] = { 'D', 'i', 'v', 'i', 'd', 'e', ' ', 'b', 'y', ' ',
    'z', 'e', 'r', 'o', '.' };

  char_T c2_u[15];
  const mxArray *c2_y = NULL;
  int32_T c2_i3;
  static char_T c2_cv1[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'd', 'i', 'v',
    'i', 'd', 'e', 'B', 'y', 'Z', 'e', 'r', 'o' };

  char_T c2_b_u[19];
  const mxArray *c2_b_y = NULL;
  for (c2_i2 = 0; c2_i2 < 15; c2_i2 = c2_i2 + 1) {
    c2_u[c2_i2] = c2_cv0[c2_i2];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 15));
  for (c2_i3 = 0; c2_i3 < 19; c2_i3 = c2_i3 + 1) {
    c2_b_u[c2_i3] = c2_cv1[c2_i3];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1, 19));
  sf_mex_call_debug("warning", 0U, 2U, 14, c2_b_y, 14, c2_y);
}

static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i4;
  uint8_T c2_b_u[18];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  for (c2_i4 = 0; c2_i4 < 18; c2_i4 = c2_i4 + 1) {
    c2_b_u[c2_i4] = (*((uint8_T (*)[18])c2_u))[c2_i4];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 3, 0U, 1U, 0U, 1, 18));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  uint16_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((uint16_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_sensor_block_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[39];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i5;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 39));
  for (c2_i5 = 0; c2_i5 < 39; c2_i5 = c2_i5 + 1) {
    c2_r0 = &c2_info[c2_i5];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i5);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i5);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[39])
{
  c2_info[0].context = "";
  c2_info[0].name = "length";
  c2_info[0].dominantType = "uint8";
  c2_info[0].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m";
  c2_info[0].fileLength = 326U;
  c2_info[0].fileTime1 = 1226598875U;
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
  c2_info[3].dominantType = "uint8";
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
  c2_info[5].name = "uint16";
  c2_info[5].dominantType = "uint8";
  c2_info[5].resolved = "[B]uint16";
  c2_info[5].fileLength = 0U;
  c2_info[5].fileTime1 = 0U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "";
  c2_info[6].name = "bitshift";
  c2_info[6].dominantType = "uint16";
  c2_info[6].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[6].fileLength = 1889U;
  c2_info[6].fileTime1 = 1228115490U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[7].name = "ge";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[B]ge";
  c2_info[7].fileLength = 0U;
  c2_info[7].fileTime1 = 0U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[8].name = "eml_isa_uint";
  c2_info[8].dominantType = "uint16";
  c2_info[8].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[8].fileLength = 239U;
  c2_info[8].fileTime1 = 1192488392U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c2_info[9].name = "isa";
  c2_info[9].dominantType = "uint16";
  c2_info[9].resolved = "[B]isa";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[10].name = "class";
  c2_info[10].dominantType = "uint16";
  c2_info[10].resolved = "[B]class";
  c2_info[10].fileLength = 0U;
  c2_info[10].fileTime1 = 0U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[11].name = "eml_int_nbits";
  c2_info[11].dominantType = "char";
  c2_info[11].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_nbits.m";
  c2_info[11].fileLength = 467U;
  c2_info[11].fileTime1 = 1192488388U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_nbits.m";
  c2_info[12].name = "uint8";
  c2_info[12].dominantType = "double";
  c2_info[12].resolved = "[B]uint8";
  c2_info[12].fileLength = 0U;
  c2_info[12].fileTime1 = 0U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[13].name = "lt";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[B]lt";
  c2_info[13].fileLength = 0U;
  c2_info[13].fileTime1 = 0U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[14].name = "isreal";
  c2_info[14].dominantType = "uint16";
  c2_info[14].resolved = "[B]isreal";
  c2_info[14].fileLength = 0U;
  c2_info[14].fileTime1 = 0U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[15].name = "eml_scalar_eg";
  c2_info[15].dominantType = "uint16";
  c2_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[15].fileLength = 3068U;
  c2_info[15].fileTime1 = 1240283610U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/any_enums";
  c2_info[16].name = "false";
  c2_info[16].dominantType = "";
  c2_info[16].resolved = "[B]false";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[17].name = "isstruct";
  c2_info[17].dominantType = "uint16";
  c2_info[17].resolved = "[B]isstruct";
  c2_info[17].fileLength = 0U;
  c2_info[17].fileTime1 = 0U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[18].name = "cast";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved = "[B]cast";
  c2_info[18].fileLength = 0U;
  c2_info[18].fileTime1 = 0U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[19].name = "eml_scalexp_alloc";
  c2_info[19].dominantType = "uint16";
  c2_info[19].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[19].fileLength = 808U;
  c2_info[19].fileTime1 = 1230516299U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[20].name = "minus";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[B]minus";
  c2_info[20].fileLength = 0U;
  c2_info[20].fileTime1 = 0U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[21].name = "isscalar";
  c2_info[21].dominantType = "uint16";
  c2_info[21].resolved = "[B]isscalar";
  c2_info[21].fileLength = 0U;
  c2_info[21].fileTime1 = 0U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[22].name = "not";
  c2_info[22].dominantType = "logical";
  c2_info[22].resolved = "[B]not";
  c2_info[22].fileLength = 0U;
  c2_info[22].fileTime1 = 0U;
  c2_info[22].fileTime2 = 0U;
  c2_info[23].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[23].name = "isempty";
  c2_info[23].dominantType = "uint16";
  c2_info[23].resolved = "[B]isempty";
  c2_info[23].fileLength = 0U;
  c2_info[23].fileTime1 = 0U;
  c2_info[23].fileTime2 = 0U;
  c2_info[24].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[24].name = "floor";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[24].fileLength = 332U;
  c2_info[24].fileTime1 = 1203469623U;
  c2_info[24].fileTime2 = 0U;
  c2_info[25].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[25].name = "eml_scalar_floor";
  c2_info[25].dominantType = "double";
  c2_info[25].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[25].fileLength = 260U;
  c2_info[25].fileTime1 = 1209352390U;
  c2_info[25].fileTime2 = 0U;
  c2_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[26].name = "ne";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved = "[B]ne";
  c2_info[26].fileLength = 0U;
  c2_info[26].fileTime1 = 0U;
  c2_info[26].fileTime2 = 0U;
  c2_info[27].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[27].name = "eml_error";
  c2_info[27].dominantType = "char";
  c2_info[27].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[27].fileLength = 315U;
  c2_info[27].fileTime1 = 1213948345U;
  c2_info[27].fileTime2 = 0U;
  c2_info[28].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[28].name = "strcmp";
  c2_info[28].dominantType = "char";
  c2_info[28].resolved = "[B]strcmp";
  c2_info[28].fileLength = 0U;
  c2_info[28].fileTime1 = 0U;
  c2_info[28].fileTime2 = 0U;
  c2_info[29].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitshift.m";
  c2_info[29].name = "uminus";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved = "[B]uminus";
  c2_info[29].fileLength = 0U;
  c2_info[29].fileTime1 = 0U;
  c2_info[29].fileTime2 = 0U;
  c2_info[30].context = "";
  c2_info[30].name = "bitor";
  c2_info[30].dominantType = "uint16";
  c2_info[30].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitor.m";
  c2_info[30].fileLength = 1051U;
  c2_info[30].fileTime1 = 1219774532U;
  c2_info[30].fileTime2 = 0U;
  c2_info[31].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/bitor.m";
  c2_info[31].name = "eml_scalexp_compatible";
  c2_info[31].dominantType = "uint16";
  c2_info[31].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m";
  c2_info[31].fileLength = 568U;
  c2_info[31].fileTime1 = 1228115441U;
  c2_info[31].fileTime2 = 0U;
  c2_info[32].context = "";
  c2_info[32].name = "mrdivide";
  c2_info[32].dominantType = "uint16";
  c2_info[32].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[32].fileLength = 800U;
  c2_info[32].fileTime1 = 1238455891U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c2_info[33].name = "rdivide";
  c2_info[33].dominantType = "uint16";
  c2_info[33].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[33].fileLength = 620U;
  c2_info[33].fileTime1 = 1213948366U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[34].name = "eml_warning";
  c2_info[34].dominantType = "char";
  c2_info[34].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c2_info[34].fileLength = 262U;
  c2_info[34].fileTime1 = 1236278878U;
  c2_info[34].fileTime2 = 0U;
  c2_info[35].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c2_info[35].name = "eml_div";
  c2_info[35].dominantType = "uint16";
  c2_info[35].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[35].fileLength = 4269U;
  c2_info[35].fileTime1 = 1228115426U;
  c2_info[35].fileTime2 = 0U;
  c2_info[36].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[36].name = "isinteger";
  c2_info[36].dominantType = "uint16";
  c2_info[36].resolved = "[B]isinteger";
  c2_info[36].fileLength = 0U;
  c2_info[36].fileTime1 = 0U;
  c2_info[36].fileTime2 = 0U;
  c2_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/zerosum";
  c2_info[37].name = "plus";
  c2_info[37].dominantType = "uint16";
  c2_info[37].resolved = "[B]plus";
  c2_info[37].fileLength = 0U;
  c2_info[37].fileTime1 = 0U;
  c2_info[37].fileTime2 = 0U;
  c2_info[38].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_mixed_integer_rdivide";
  c2_info[38].name = "double";
  c2_info[38].dominantType = "uint16";
  c2_info[38].resolved = "[B]double";
  c2_info[38].fileLength = 0U;
  c2_info[38].fileTime1 = 0U;
  c2_info[38].fileTime2 = 0U;
}

static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  boolean_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((boolean_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c2_sensor_block_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(135860165U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2749856063U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1400252197U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1667919412U);
}

mxArray *sf_c2_sensor_block_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2648412319U);
    pr[1] = (double)(596291724U);
    pr[2] = (double)(2660866296U);
    pr[3] = (double)(284226609U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(18);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
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

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_sensor_block(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[5],T\"pck_success\",},{M[1],M[15],T\"pck_total\",},{M[1],M[13],T\"x1\",},{M[1],M[11],T\"x2\",},{M[8],M[0],T\"is_active_c2_sensor_block\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_sensor_block_get_check_sum(&mxChecksum);
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
           2,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,"x1",0,
                              (MexFcnForType)c2_b_sf_marshall);
          _SFD_SET_DATA_PROPS(1,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,"x2",0,
                              (MexFcnForType)c2_b_sf_marshall);

          {
            unsigned int dimVector[1];
            dimVector[0]= 18;
            _SFD_SET_DATA_PROPS(2,1,1,0,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,1.0,
                                0,"u",0,(MexFcnForType)c2_sf_marshall);
          }

          _SFD_SET_DATA_PROPS(3,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_success",0,(MexFcnForType)c2_b_sf_marshall);
          _SFD_SET_DATA_PROPS(4,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_total",0,(MexFcnForType)c2_b_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1559);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          uint16_T *c2_x1;
          uint16_T *c2_x2;
          uint8_T (*c2_u)[18];
          uint16_T *c2_pck_success;
          uint16_T *c2_pck_total;
          c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
          c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
          c2_x2 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          c2_u = (uint8_T (*)[18])ssGetInputPortSignal(chartInstance.S, 0);
          c2_x1 = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_x1);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_x2);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_u);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_pck_success);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_pck_total);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_sensor_blockMachineNumber_,
        chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_sensor_block(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c2_sensor_block();
  initialize_c2_sensor_block();
}

static void sf_opaque_enable_c2_sensor_block(void *chartInstanceVar)
{
  enable_c2_sensor_block();
}

static void sf_opaque_disable_c2_sensor_block(void *chartInstanceVar)
{
  disable_c2_sensor_block();
}

static void sf_opaque_gateway_c2_sensor_block(void *chartInstanceVar)
{
  sf_c2_sensor_block();
}

static mxArray* sf_opaque_get_sim_state_c2_sensor_block(void *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c2_sensor_block();
  return st;
}

static void sf_opaque_set_sim_state_c2_sensor_block(void *chartInstanceVar,
  const mxArray *st)
{
  set_sim_state_c2_sensor_block(sf_mex_dup(st));
}

static void sf_opaque_terminate_c2_sensor_block(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c2_sensor_block();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_sensor_block(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_sensor_block();
  }
}

static void mdlSetWorkWidths_c2_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block","sensor_block",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block","sensor_block",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block",
      "sensor_block",2,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block","sensor_block",2,1);
      sf_mark_chart_reusable_outputs(S,"sensor_block","sensor_block",2,4);
    }

    sf_set_rtw_dwork_info(S,"sensor_block","sensor_block",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(478948466U));
  ssSetChecksum1(S,(1516221470U));
  ssSetChecksum2(S,(867467429U));
  ssSetChecksum3(S,(1600239102U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block", "sensor_block",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_sensor_block(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_opaque_gateway_c2_sensor_block;
  chartInstance.chartInfo.initializeChart = sf_opaque_initialize_c2_sensor_block;
  chartInstance.chartInfo.terminateChart = sf_opaque_terminate_c2_sensor_block;
  chartInstance.chartInfo.enableChart = sf_opaque_enable_c2_sensor_block;
  chartInstance.chartInfo.disableChart = sf_opaque_disable_c2_sensor_block;
  chartInstance.chartInfo.getSimState = sf_opaque_get_sim_state_c2_sensor_block;
  chartInstance.chartInfo.setSimState = sf_opaque_set_sim_state_c2_sensor_block;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_sensor_block;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c2_sensor_block;
  chartInstance.chartInfo.mdlStart = mdlStart_c2_sensor_block;
  chartInstance.chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_sensor_block;
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

void c2_sensor_block_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_sensor_block(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_sensor_block(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_sensor_block(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_sensor_block_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
