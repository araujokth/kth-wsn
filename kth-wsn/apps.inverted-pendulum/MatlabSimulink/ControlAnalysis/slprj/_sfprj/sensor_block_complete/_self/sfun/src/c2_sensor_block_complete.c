/* Include files */

#include "blascompat32.h"
#include "sensor_block_complete_sfun.h"
#include "c2_sensor_block_complete.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_complete_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c2_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc2_sensor_block_completeInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c2_sensor_block_complete(void);
static void initialize_params_c2_sensor_block_complete(void);
static void enable_c2_sensor_block_complete(void);
static void disable_c2_sensor_block_complete(void);
static void c2_update_debugger_state_c2_sensor_block_complete(void);
static const mxArray *get_sim_state_c2_sensor_block_complete(void);
static void set_sim_state_c2_sensor_block_complete(const mxArray *c2_st);
static void finalize_c2_sensor_block_complete(void);
static void sf_c2_sensor_block_complete(void);
static void c2_c2_sensor_block_complete(void);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u);
static const mxArray *c2_e_sf_marshall(void *c2_chartInstance, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[35]);
static const mxArray *c2_f_sf_marshall(void *c2_chartInstance, void *c2_u);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c2_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c2_is_active_c2_sensor_block_complete = 0U;
}

static void initialize_params_c2_sensor_block_complete(void)
{
}

static void enable_c2_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c2_sensor_block_complete(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c2_update_debugger_state_c2_sensor_block_complete(void)
{
}

static const mxArray *get_sim_state_c2_sensor_block_complete(void)
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
  uint16_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  uint32_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  uint8_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  uint16_T *c2_delay;
  uint16_T *c2_pck_success;
  uint16_T *c2_pck_total;
  uint16_T *c2_sensor_v;
  uint16_T *c2_src_addr;
  uint32_T *c2_timestamp;
  c2_sensor_v = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_src_addr = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_timestamp = (uint32_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c2_delay = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(7));
  c2_u = *c2_delay;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_u = *c2_pck_success;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_u = *c2_pck_total;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_u = *c2_sensor_v;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_u = *c2_src_addr;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 5, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_u = *c2_timestamp;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 7, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_u = chartInstance.c2_is_active_c2_sensor_block_complete;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 6, c2_h_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_sensor_block_complete(const mxArray *c2_st)
{
  const mxArray *c2_u;
  const mxArray *c2_delay;
  uint16_T c2_u0;
  uint16_T c2_y;
  const mxArray *c2_b_delay;
  uint16_T c2_u1;
  uint16_T c2_b_y;
  const mxArray *c2_c_delay;
  uint16_T c2_u2;
  uint16_T c2_c_y;
  const mxArray *c2_d_delay;
  uint16_T c2_u3;
  uint16_T c2_d_y;
  const mxArray *c2_e_delay;
  uint16_T c2_u4;
  uint16_T c2_e_y;
  const mxArray *c2_timestamp;
  uint32_T c2_u5;
  uint32_T c2_f_y;
  const mxArray *c2_b_is_active_c2_sensor_block_complete;
  uint8_T c2_u6;
  uint8_T c2_g_y;
  uint16_T *c2_f_delay;
  uint16_T *c2_pck_success;
  uint16_T *c2_pck_total;
  uint16_T *c2_sensor_v;
  uint16_T *c2_src_addr;
  uint32_T *c2_b_timestamp;
  c2_sensor_v = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_src_addr = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_b_timestamp = (uint32_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c2_f_delay = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  chartInstance.c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_delay = sf_mex_dup(sf_mex_getcell(c2_u, 0));
  sf_mex_import("delay", sf_mex_dup(c2_delay), &c2_u0, 1, 5, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_delay);
  *c2_f_delay = c2_y;
  c2_b_delay = sf_mex_dup(sf_mex_getcell(c2_u, 1));
  sf_mex_import("pck_success", sf_mex_dup(c2_b_delay), &c2_u1, 1, 5, 0U, 0, 0U,
                0);
  c2_b_y = c2_u1;
  sf_mex_destroy(&c2_b_delay);
  *c2_pck_success = c2_b_y;
  c2_c_delay = sf_mex_dup(sf_mex_getcell(c2_u, 2));
  sf_mex_import("pck_total", sf_mex_dup(c2_c_delay), &c2_u2, 1, 5, 0U, 0, 0U, 0);
  c2_c_y = c2_u2;
  sf_mex_destroy(&c2_c_delay);
  *c2_pck_total = c2_c_y;
  c2_d_delay = sf_mex_dup(sf_mex_getcell(c2_u, 3));
  sf_mex_import("sensor_v", sf_mex_dup(c2_d_delay), &c2_u3, 1, 5, 0U, 0, 0U, 0);
  c2_d_y = c2_u3;
  sf_mex_destroy(&c2_d_delay);
  *c2_sensor_v = c2_d_y;
  c2_e_delay = sf_mex_dup(sf_mex_getcell(c2_u, 4));
  sf_mex_import("src_addr", sf_mex_dup(c2_e_delay), &c2_u4, 1, 5, 0U, 0, 0U, 0);
  c2_e_y = c2_u4;
  sf_mex_destroy(&c2_e_delay);
  *c2_src_addr = c2_e_y;
  c2_timestamp = sf_mex_dup(sf_mex_getcell(c2_u, 5));
  sf_mex_import("timestamp", sf_mex_dup(c2_timestamp), &c2_u5, 1, 7, 0U, 0, 0U,
                0);
  c2_f_y = c2_u5;
  sf_mex_destroy(&c2_timestamp);
  *c2_b_timestamp = c2_f_y;
  c2_b_is_active_c2_sensor_block_complete = sf_mex_dup(sf_mex_getcell(c2_u, 6));
  sf_mex_import("is_active_c2_sensor_block_complete", sf_mex_dup
                (c2_b_is_active_c2_sensor_block_complete), &c2_u6, 1, 3, 0U, 0,
                0U, 0);
  c2_g_y = c2_u6;
  sf_mex_destroy(&c2_b_is_active_c2_sensor_block_complete);
  chartInstance.c2_is_active_c2_sensor_block_complete = c2_g_y;
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_sensor_block_complete();
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_sensor_block_complete(void)
{
}

static void sf_c2_sensor_block_complete(void)
{
  int32_T c2_i0;
  int32_T c2_previousEvent;
  uint32_T *c2_timestamp;
  uint16_T *c2_src_addr;
  uint16_T *c2_sensor_v;
  uint16_T *c2_pck_success;
  uint16_T *c2_pck_total;
  uint16_T *c2_delay;
  uint8_T (*c2_u)[26];
  c2_sensor_v = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_src_addr = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_timestamp = (uint32_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c2_delay = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c2_u = (uint8_T (*)[26])ssGetInputPortSignal(chartInstance.S, 0);
  c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,1);
  for (c2_i0 = 0; c2_i0 < 26; c2_i0 = c2_i0 + 1) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_u)[c2_i0], 0U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)*c2_timestamp, 1U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_src_addr, 2U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_sensor_v, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_pck_success, 4U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_pck_total, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_delay, 6U);
  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c2_c2_sensor_block_complete();
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_block_completeMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber
    );
}

static void c2_c2_sensor_block_complete(void)
{
  int32_T c2_i1;
  uint8_T c2_u[26];
  real_T c2_nargout = 6.0;
  real_T c2_nargin = 1.0;
  uint32_T c2_timestamp2;
  uint32_T c2_timestamp1;
  uint16_T c2_rssi;
  uint16_T c2_lqi;
  uint16_T c2_dst_addr;
  uint16_T c2_pan_id;
  uint8_T c2_seq_num;
  real_T c2_l;
  uint16_T c2_delay;
  uint16_T c2_pck_total;
  uint16_T c2_pck_success;
  uint16_T c2_sensor_v;
  uint16_T c2_src_addr;
  uint32_T c2_timestamp;
  uint16_T c2_a;
  uint16_T c2_a1;
  uint16_T c2_b_a;
  uint16_T c2_b;
  uint16_T c2_c_a;
  uint16_T c2_b_a1;
  uint16_T c2_d_a;
  uint16_T c2_b_b;
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
  uint16_T c2_k_a;
  uint16_T c2_f_a1;
  uint16_T c2_l_a;
  uint16_T c2_f_b;
  uint16_T c2_m_a;
  uint16_T c2_g_a1;
  uint16_T c2_n_a;
  uint16_T c2_g_b;
  uint32_T c2_o_a;
  uint32_T c2_h_a1;
  uint32_T c2_p_a;
  uint32_T c2_q_a;
  uint32_T c2_i_a1;
  uint32_T c2_h_b;
  uint32_T c2_r_a;
  uint32_T c2_j_a1;
  uint32_T c2_s_a;
  uint32_T c2_i_b;
  uint32_T c2_t_a;
  uint32_T c2_j_b;
  uint32_T c2_u_a;
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_d2;
  uint32_T c2_u7;
  uint16_T *c2_b_delay;
  uint16_T *c2_b_pck_total;
  uint16_T *c2_b_pck_success;
  uint16_T *c2_b_sensor_v;
  uint16_T *c2_b_src_addr;
  uint32_T *c2_b_timestamp;
  uint8_T (*c2_b_u)[26];
  c2_b_sensor_v = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c2_b_src_addr = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c2_b_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c2_b_timestamp = (uint32_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c2_b_delay = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c2_b_u = (uint8_T (*)[26])ssGetInputPortSignal(chartInstance.S, 0);
  c2_b_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,1);
  for (c2_i1 = 0; c2_i1 < 26; c2_i1 = c2_i1 + 1) {
    c2_u[c2_i1] = (*c2_b_u)[c2_i1];
  }

  sf_debug_symbol_scope_push(17U, 0U);
  sf_debug_symbol_scope_add("nargout", &c2_nargout, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c2_nargin, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("timestamp2", &c2_timestamp2, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("timestamp1", &c2_timestamp1, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("rssi", &c2_rssi, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("lqi", &c2_lqi, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("dst_addr", &c2_dst_addr, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("pan_id", &c2_pan_id, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("seq_num", &c2_seq_num, c2_e_sf_marshall);
  sf_debug_symbol_scope_add("l", &c2_l, c2_d_sf_marshall);
  sf_debug_symbol_scope_add("delay", &c2_delay, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("pck_total", &c2_pck_total, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("pck_success", &c2_pck_success, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("sensor_v", &c2_sensor_v, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("src_addr", &c2_src_addr, c2_c_sf_marshall);
  sf_debug_symbol_scope_add("timestamp", &c2_timestamp, c2_b_sf_marshall);
  sf_debug_symbol_scope_add("u", &c2_u, c2_sf_marshall);
  CV_EML_FCN(0, 0);

  /*  02    0f     61 88   e7     34 12     03 00      00 00    00 13    04 e3    0f ff    69    c9     00        c0  2d  40  58  */
  /*  02    0f     61 88   d2     34 12     03 00      00 00    00 00    00 00    07 ff    69    c2     84        32 8d de 02  */
  /*  02    0f     61 88   be     34 12     03 00      00 00    00 00    00 00    07 ff    67    ca     00        c4 f0 8d 03  */
  /*  id| length|        |seqnum|  PANID  | Src adr | Dst addr | xc    | theta  |  u_out  | lqi | rssi| mhrLen | timestamp */
  /*             */
  /*
     02   0f     61 88    9f     34 12     00 00      02 00     00 00    00 4b      00 4b   02 04    6b    c2     00      ba 15 28 00  */
  /*
     02   11     61 88    7b     34 12     00 00      02 00     ff ff    00 23      00 00   05 84    6c    e0     11      a0 b8 dc 3d  */
  /*  id| length|        |seqnum|  PANID  | Dest adr | Src addr | xc    | total |  success  | delay | lqi | rssi| mhrLen | timestamp */
  _SFD_EML_CALL(0,15);
  c2_l = 26.0;

  /*  */
  /*  packet information */
  /*  */
  _SFD_EML_CALL(0,20);
  c2_seq_num = c2_u[4];
  _SFD_EML_CALL(0,21);
  c2_a = (uint16_T)c2_u[6];
  c2_a1 = c2_a;
  c2_b_a = (uint16_T)(c2_a1 << 8);
  c2_b = (uint16_T)c2_u[5];
  c2_pan_id = (uint16_T)(c2_b_a | c2_b);
  _SFD_EML_CALL(0,22);
  c2_c_a = (uint16_T)c2_u[8];
  c2_b_a1 = c2_c_a;
  c2_d_a = (uint16_T)(c2_b_a1 << 8);
  c2_b_b = (uint16_T)c2_u[7];
  c2_dst_addr = (uint16_T)(c2_d_a | c2_b_b);
  _SFD_EML_CALL(0,23);
  c2_e_a = (uint16_T)c2_u[10];
  c2_c_a1 = c2_e_a;
  c2_f_a = (uint16_T)(c2_c_a1 << 8);
  c2_c_b = (uint16_T)c2_u[9];
  c2_src_addr = (uint16_T)(c2_f_a | c2_c_b);

  /*  */
  /*  Sensors and control voltage */
  /*  */
  /*  xc_sensor=bitor(bitshift(uint16(u(14)),8), uint16(u(15)))*9.2E-5; */
  /*  theta_sensor=bitor(bitshift(uint16(u(12)),8), uint16(u(13))) / 1024.0 * 2 * 3.14159265; */
  /*  u_control = bitor(bitshift(uint16(u(16)),8), uint16(u(17))); */
  /*  u_control = (u_control / 1638) * 4.8 -6; */
  /*  Sensor values */
  _SFD_EML_CALL(0,34);
  c2_g_a = (uint16_T)c2_u[11];
  c2_d_a1 = c2_g_a;
  c2_h_a = (uint16_T)(c2_d_a1 << 8);
  c2_d_b = (uint16_T)c2_u[12];
  c2_sensor_v = (uint16_T)(c2_h_a | c2_d_b);
  _SFD_EML_CALL(0,35);
  c2_i_a = (uint16_T)c2_u[13];
  c2_e_a1 = c2_i_a;
  c2_j_a = (uint16_T)(c2_e_a1 << 8);
  c2_e_b = (uint16_T)c2_u[14];
  c2_pck_total = (uint16_T)(c2_j_a | c2_e_b);
  _SFD_EML_CALL(0,36);
  c2_k_a = (uint16_T)c2_u[15];
  c2_f_a1 = c2_k_a;
  c2_l_a = (uint16_T)(c2_f_a1 << 8);
  c2_f_b = (uint16_T)c2_u[16];
  c2_pck_success = (uint16_T)(c2_l_a | c2_f_b);

  /* pck_total = uint8(u(14)); */
  /* pck_success = uint8(u(15)); */
  _SFD_EML_CALL(0,40);
  c2_m_a = (uint16_T)c2_u[17];
  c2_g_a1 = c2_m_a;
  c2_n_a = (uint16_T)(c2_g_a1 << 8);
  c2_g_b = (uint16_T)c2_u[18];
  c2_delay = (uint16_T)(c2_n_a | c2_g_b);

  /*  */
  /*  Timestamps and more packet information */
  /*  */
  _SFD_EML_CALL(0,45);
  c2_lqi = (uint16_T)c2_u[19];
  _SFD_EML_CALL(0,46);
  c2_rssi = (uint16_T)c2_u[20];
  _SFD_EML_CALL(0,48);
  c2_o_a = (uint32_T)c2_u[24];
  c2_h_a1 = c2_o_a;
  c2_p_a = c2_h_a1 << 24U;
  c2_q_a = (uint32_T)c2_u[23];
  c2_i_a1 = c2_q_a;
  c2_h_b = c2_i_a1 << 16U;
  c2_timestamp1 = c2_p_a | c2_h_b;
  _SFD_EML_CALL(0,49);
  c2_r_a = (uint32_T)c2_u[22];
  c2_j_a1 = c2_r_a;
  c2_s_a = c2_j_a1 << 8U;
  c2_i_b = (uint32_T)c2_u[21];
  c2_timestamp2 = c2_s_a | c2_i_b;
  _SFD_EML_CALL(0,50);
  c2_t_a = c2_timestamp1;
  c2_j_b = c2_timestamp2;
  c2_u_a = c2_t_a | c2_j_b;
  c2_d0 = (real_T)c2_u_a * 0.016;
  c2_d1 = c2_d0;
  c2_d1 = c2_d1 < 0.0 ? muDoubleScalarCeil(c2_d1 - 0.5) : muDoubleScalarFloor
    (c2_d1 + 0.5);
  c2_d2 = c2_d1;
  if (c2_d2 < 4.294967296E+09) {
    if (c2_d2 >= 0.0) {
      c2_u7 = (uint32_T)c2_d2;
    } else {
      c2_u7 = 0U;
    }
  } else if (c2_d2 >= 4.294967296E+09) {
    c2_u7 = MAX_uint32_T;
  } else {
    c2_u7 = 0U;
  }

  c2_timestamp = c2_u7;

  /* %in ms */
  _SFD_EML_CALL(0,-50);
  sf_debug_symbol_scope_pop();
  *c2_b_timestamp = c2_timestamp;
  *c2_b_src_addr = c2_src_addr;
  *c2_b_sensor_v = c2_sensor_v;
  *c2_b_pck_success = c2_pck_success;
  *c2_b_pck_total = c2_pck_total;
  *c2_b_delay = c2_delay;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i2;
  uint8_T c2_b_u[26];
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  for (c2_i2 = 0; c2_i2 < 26; c2_i2 = c2_i2 + 1) {
    c2_b_u[c2_i2] = (*((uint8_T (*)[26])c2_u))[c2_i2];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 3, 0U, 1U, 0U, 1, 26));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  uint32_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((uint32_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 7, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *c2_chartInstance, void *c2_u)
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

static const mxArray *c2_d_sf_marshall(void *c2_chartInstance, void *c2_u)
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

static const mxArray *c2_e_sf_marshall(void *c2_chartInstance, void *c2_u)
{
  const mxArray *c2_y = NULL;
  uint8_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  c2_y = NULL;
  c2_b_u = *((uint8_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_sensor_block_complete_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[35];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i3;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 35));
  for (c2_i3 = 0; c2_i3 < 35; c2_i3 = c2_i3 + 1) {
    c2_r0 = &c2_info[c2_i3];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c2_i3);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c2_i3);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[35])
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
  c2_info[32].name = "uint32";
  c2_info[32].dominantType = "uint8";
  c2_info[32].resolved = "[B]uint32";
  c2_info[32].fileLength = 0U;
  c2_info[32].fileTime1 = 0U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context = "";
  c2_info[33].name = "mtimes";
  c2_info[33].dominantType = "uint32";
  c2_info[33].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[33].fileLength = 3302U;
  c2_info[33].fileTime1 = 1242772494U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[34].name = "isinteger";
  c2_info[34].dominantType = "uint32";
  c2_info[34].resolved = "[B]isinteger";
  c2_info[34].fileLength = 0U;
  c2_info[34].fileTime1 = 0U;
  c2_info[34].fileTime2 = 0U;
}

static const mxArray *c2_f_sf_marshall(void *c2_chartInstance, void *c2_u)
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
void sf_c2_sensor_block_complete_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(298217280U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(434809150U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1388866338U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2396250989U);
}

mxArray *sf_c2_sensor_block_complete_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(126963094U);
    pr[1] = (double)(3763073213U);
    pr[2] = (double)(1027895601U);
    pr[3] = (double)(420626737U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(26);
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

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(7));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(5));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_sensor_block_complete(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[8],T\"delay\",},{M[1],M[12],T\"pck_success\",},{M[1],M[14],T\"pck_total\",},{M[1],M[11],T\"sensor_v\",},{M[1],M[13],T\"src_addr\",},{M[1],M[5],T\"timestamp\",},{M[8],M[0],T\"is_active_c2_sensor_block_complete\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_sensor_block_complete_get_check_sum(&mxChecksum);
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
           2,
           1,
           1,
           7,
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

          {
            unsigned int dimVector[1];
            dimVector[0]= 26;
            _SFD_SET_DATA_PROPS(0,1,1,0,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,1.0,
                                0,"u",0,(MexFcnForType)c2_sf_marshall);
          }

          _SFD_SET_DATA_PROPS(1,2,0,1,SF_UINT32,0,NULL,0,0,0,0.0,1.0,0,
                              "timestamp",0,(MexFcnForType)c2_b_sf_marshall);
          _SFD_SET_DATA_PROPS(2,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "src_addr",0,(MexFcnForType)c2_c_sf_marshall);
          _SFD_SET_DATA_PROPS(3,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "sensor_v",0,(MexFcnForType)c2_c_sf_marshall);
          _SFD_SET_DATA_PROPS(4,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_success",0,(MexFcnForType)c2_c_sf_marshall);
          _SFD_SET_DATA_PROPS(5,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "pck_total",0,(MexFcnForType)c2_c_sf_marshall);
          _SFD_SET_DATA_PROPS(6,2,0,1,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,"delay",0,
                              (MexFcnForType)c2_c_sf_marshall);
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,2118);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          uint8_T (*c2_u)[26];
          uint32_T *c2_timestamp;
          uint16_T *c2_src_addr;
          uint16_T *c2_sensor_v;
          uint16_T *c2_pck_success;
          uint16_T *c2_pck_total;
          uint16_T *c2_delay;
          c2_sensor_v = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 3);
          c2_src_addr = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          c2_pck_success = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 4);
          c2_timestamp = (uint32_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          c2_delay = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 6);
          c2_u = (uint8_T (*)[26])ssGetInputPortSignal(chartInstance.S, 0);
          c2_pck_total = (uint16_T *)ssGetOutputPortSignal(chartInstance.S, 5);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_u);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_timestamp);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_src_addr);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_sensor_v);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_pck_success);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_pck_total);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_delay);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_sensor_block_completeMachineNumber_,chartInstance.chartNumber,
         chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_sensor_block_complete(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c2_sensor_block_complete();
  initialize_c2_sensor_block_complete();
}

static void sf_opaque_enable_c2_sensor_block_complete(void *chartInstanceVar)
{
  enable_c2_sensor_block_complete();
}

static void sf_opaque_disable_c2_sensor_block_complete(void *chartInstanceVar)
{
  disable_c2_sensor_block_complete();
}

static void sf_opaque_gateway_c2_sensor_block_complete(void *chartInstanceVar)
{
  sf_c2_sensor_block_complete();
}

static mxArray* sf_opaque_get_sim_state_c2_sensor_block_complete(void
  *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c2_sensor_block_complete();
  return st;
}

static void sf_opaque_set_sim_state_c2_sensor_block_complete(void
  *chartInstanceVar, const mxArray *st)
{
  set_sim_state_c2_sensor_block_complete(sf_mex_dup(st));
}

static void sf_opaque_terminate_c2_sensor_block_complete(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c2_sensor_block_complete();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_sensor_block_complete(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_sensor_block_complete();
  }
}

static void mdlSetWorkWidths_c2_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block_complete",
      "sensor_block_complete",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block_complete",
                "sensor_block_complete",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block_complete",
      "sensor_block_complete",2,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block_complete",
        "sensor_block_complete",2,1);
      sf_mark_chart_reusable_outputs(S,"sensor_block_complete",
        "sensor_block_complete",2,6);
    }

    sf_set_rtw_dwork_info(S,"sensor_block_complete","sensor_block_complete",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(423789036U));
  ssSetChecksum1(S,(948775733U));
  ssSetChecksum2(S,(1928238389U));
  ssSetChecksum3(S,(2138315306U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_sensor_block_complete(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block_complete", "sensor_block_complete",
      2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_sensor_block_complete(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_sensor_block_complete;
  chartInstance.chartInfo.initializeChart =
    sf_opaque_initialize_c2_sensor_block_complete;
  chartInstance.chartInfo.terminateChart =
    sf_opaque_terminate_c2_sensor_block_complete;
  chartInstance.chartInfo.enableChart =
    sf_opaque_enable_c2_sensor_block_complete;
  chartInstance.chartInfo.disableChart =
    sf_opaque_disable_c2_sensor_block_complete;
  chartInstance.chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_sensor_block_complete;
  chartInstance.chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_sensor_block_complete;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_sensor_block_complete;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c2_sensor_block_complete;
  chartInstance.chartInfo.mdlStart = mdlStart_c2_sensor_block_complete;
  chartInstance.chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_sensor_block_complete;
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

void c2_sensor_block_complete_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_sensor_block_complete(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_sensor_block_complete(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_sensor_block_complete_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
