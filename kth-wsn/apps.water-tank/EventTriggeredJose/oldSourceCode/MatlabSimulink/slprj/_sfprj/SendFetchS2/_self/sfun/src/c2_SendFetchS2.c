/* Include files */

#include "blascompat32.h"
#include "SendFetchS2_sfun.h"
#include "c2_SendFetchS2.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SendFetchS2_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
static const char *c2_debug_family_names[97] = { "BI", "randState", "tBI",
  "symbol_time", "deltaCAP", "ttStart", "time_schedule1", "lateIN", "lateOUT",
  "time_schedule", "i", "scheduled_next", "slot", "y1L1_new", "y1L2_new",
  "y1INT_new", "y2L1_new", "y2L2_new", "y2INT_new", "ttStart_next", "data",
  "x10", "x20", "x30", "lvm", "temp17", "next_time", "ii", "j", "B", "pos",
  "slot_num", "l", "node", "randState2", "nargin", "nargout", "Y11", "Y12",
  "Y21", "Y22", "S1", "S2", "S3", "S4", "S5", "TIME", "I1", "I2", "current_BO",
  "time_schedule1_1", "time_schedule1_2", "time_schedule1_3", "time_schedule1_4",
  "time_schedule1_5", "time_schedule1_6", "time_schedule1_7", "time_schedule1_8",
  "time_schedule1_9", "time_schedule1_10", "time_schedule1_11", "INIT_in",
  "lateIN_1", "lateIN_2", "lateIN_3", "lateIN_4", "randStateIN", "BO", "TM1",
  "TA1", "TM2", "TA2", "TS1", "TS2", "TS3", "TS4", "TS5", "TS6", "TS7",
  "time_schedule_1", "time_schedule_2", "time_schedule_3", "time_schedule_4",
  "time_schedule_5", "time_schedule_6", "time_schedule_7", "time_schedule_8",
  "time_schedule_9", "time_schedule_10", "time_schedule_11", "TAU_K", "INIT",
  "lateOUT_1", "lateOUT_2", "lateOUT_3", "lateOUT_4", "randStateOUT" };

/* Function Declarations */
static void initialize_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance);
static void initialize_params_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance);
static void enable_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance);
static void disable_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_SendFetchS2
  (SFc2_SendFetchS2InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_SendFetchS2
  (SFc2_SendFetchS2InstanceStruct *chartInstance);
static void set_sim_state_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance);
static void sf_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance);
static void c2_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static void c2_rand(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T
                    c2_varargin_2);
static void c2_twister_state_vector(SFc2_SendFetchS2InstanceStruct
  *chartInstance, uint32_T c2_mt[625], real_T c2_seed, uint32_T c2_b_mt[625]);
static real_T c2_sum(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T c2_x
                     [11]);
static real_T c2_b_sum(SFc2_SendFetchS2InstanceStruct *chartInstance, boolean_T
  c2_x[4]);
static creal_T c2_mpower(SFc2_SendFetchS2InstanceStruct *chartInstance, creal_T
  c2_a);
static creal_T c2_power(SFc2_SendFetchS2InstanceStruct *chartInstance, creal_T
  c2_a, real_T c2_b);
static real_T c2_b_rand(SFc2_SendFetchS2InstanceStruct *chartInstance);
static void c2_eml_rand_mt19937ar(SFc2_SendFetchS2InstanceStruct *chartInstance,
  uint32_T c2_state[625]);
static void c2_genrandu(SFc2_SendFetchS2InstanceStruct *chartInstance, uint32_T
  c2_mt[625], uint32_T c2_b_mt[625], real_T *c2_r);
static void c2_sort(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T c2_x
                    [11], real_T c2_b_x[11], real_T c2_idx[11]);
static void c2_eml_sort_idx(SFc2_SendFetchS2InstanceStruct *chartInstance,
  real_T c2_x[11], int32_T c2_idx[11]);
static void c2_c_rand(SFc2_SendFetchS2InstanceStruct *chartInstance, uint32_T
                      c2_r[625]);
static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_f_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_g_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_h_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_i_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_j_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_k_sf_marshall(void *chartInstanceVoid, void *c2_u);
static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[83]);
static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[83]);
static const mxArray *c2_l_sf_marshall(void *chartInstanceVoid, void *c2_u);
static const mxArray *c2_m_sf_marshall(void *chartInstanceVoid, void *c2_u);
static real_T c2_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_BO, const char_T *c2_name);
static void c2_b_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_TAU_K, const char_T *c2_name, real_T c2_y[9]);
static uint8_T c2_c_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_method, const char_T *c2_name);
static void c2_d_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_b_twister_state, const char_T *c2_name, uint32_T c2_y[625]);
static uint32_T c2_e_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_v4_state, const char_T *c2_name);
static uint8_T c2_f_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_SendFetchS2, const char_T
  *c2_name);
static void init_dsm_address_info(SFc2_SendFetchS2InstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_twister_state_not_empty = FALSE;
  chartInstance->c2_method_not_empty = FALSE;
  chartInstance->c2_v4_state_not_empty = FALSE;
  chartInstance->c2_is_active_c2_SendFetchS2 = 0U;
}

static void initialize_params_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance)
{
}

static void enable_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_SendFetchS2
  (SFc2_SendFetchS2InstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_SendFetchS2
  (SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_i0;
  real_T c2_e_hoistedGlobal[9];
  int32_T c2_i1;
  real_T c2_e_u[9];
  const mxArray *c2_f_y = NULL;
  real_T c2_f_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_g_hoistedGlobal;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_h_hoistedGlobal;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_i_hoistedGlobal;
  real_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  real_T c2_j_hoistedGlobal;
  real_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real_T c2_k_hoistedGlobal;
  real_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  real_T c2_l_hoistedGlobal;
  real_T c2_l_u;
  const mxArray *c2_m_y = NULL;
  real_T c2_m_hoistedGlobal;
  real_T c2_m_u;
  const mxArray *c2_n_y = NULL;
  real_T c2_n_hoistedGlobal;
  real_T c2_n_u;
  const mxArray *c2_o_y = NULL;
  real_T c2_o_hoistedGlobal;
  real_T c2_o_u;
  const mxArray *c2_p_y = NULL;
  real_T c2_p_hoistedGlobal;
  real_T c2_p_u;
  const mxArray *c2_q_y = NULL;
  real_T c2_q_hoistedGlobal;
  real_T c2_q_u;
  const mxArray *c2_r_y = NULL;
  real_T c2_r_hoistedGlobal;
  real_T c2_r_u;
  const mxArray *c2_s_y = NULL;
  real_T c2_s_hoistedGlobal;
  real_T c2_s_u;
  const mxArray *c2_t_y = NULL;
  real_T c2_t_hoistedGlobal;
  real_T c2_t_u;
  const mxArray *c2_u_y = NULL;
  real_T c2_u_hoistedGlobal;
  real_T c2_u_u;
  const mxArray *c2_v_y = NULL;
  real_T c2_v_hoistedGlobal;
  real_T c2_v_u;
  const mxArray *c2_w_y = NULL;
  real_T c2_w_hoistedGlobal;
  real_T c2_w_u;
  const mxArray *c2_x_y = NULL;
  real_T c2_x_hoistedGlobal;
  real_T c2_x_u;
  const mxArray *c2_y_y = NULL;
  real_T c2_y_hoistedGlobal;
  real_T c2_y_u;
  const mxArray *c2_ab_y = NULL;
  real_T c2_ab_hoistedGlobal;
  real_T c2_ab_u;
  const mxArray *c2_bb_y = NULL;
  real_T c2_bb_hoistedGlobal;
  real_T c2_bb_u;
  const mxArray *c2_cb_y = NULL;
  real_T c2_cb_hoistedGlobal;
  real_T c2_cb_u;
  const mxArray *c2_db_y = NULL;
  real_T c2_db_hoistedGlobal;
  real_T c2_db_u;
  const mxArray *c2_eb_y = NULL;
  real_T c2_eb_hoistedGlobal;
  real_T c2_eb_u;
  const mxArray *c2_fb_y = NULL;
  uint8_T c2_fb_hoistedGlobal;
  uint8_T c2_fb_u;
  const mxArray *c2_gb_y = NULL;
  int32_T c2_i2;
  uint32_T c2_gb_hoistedGlobal[625];
  int32_T c2_i3;
  uint32_T c2_gb_u[625];
  const mxArray *c2_hb_y = NULL;
  uint32_T c2_hb_hoistedGlobal;
  uint32_T c2_hb_u;
  const mxArray *c2_ib_y = NULL;
  uint8_T c2_ib_hoistedGlobal;
  uint8_T c2_ib_u;
  const mxArray *c2_jb_y = NULL;
  real_T *c2_BO;
  real_T *c2_INIT;
  real_T *c2_TA1;
  real_T *c2_TA2;
  real_T *c2_TM1;
  real_T *c2_TM2;
  real_T *c2_TS1;
  real_T *c2_TS2;
  real_T *c2_TS3;
  real_T *c2_TS4;
  real_T *c2_TS5;
  real_T *c2_TS6;
  real_T *c2_TS7;
  real_T *c2_lateOUT_1;
  real_T *c2_lateOUT_2;
  real_T *c2_lateOUT_3;
  real_T *c2_lateOUT_4;
  real_T *c2_randStateOUT;
  real_T *c2_time_schedule_1;
  real_T *c2_time_schedule_10;
  real_T *c2_time_schedule_11;
  real_T *c2_time_schedule_2;
  real_T *c2_time_schedule_3;
  real_T *c2_time_schedule_4;
  real_T *c2_time_schedule_5;
  real_T *c2_time_schedule_6;
  real_T *c2_time_schedule_7;
  real_T *c2_time_schedule_8;
  real_T *c2_time_schedule_9;
  real_T (*c2_TAU_K)[9];
  c2_randStateOUT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 30);
  c2_lateOUT_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 29);
  c2_lateOUT_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 28);
  c2_lateOUT_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 27);
  c2_lateOUT_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 26);
  c2_INIT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 25);
  c2_TAU_K = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 24);
  c2_time_schedule_11 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 23);
  c2_time_schedule_10 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 22);
  c2_time_schedule_9 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 21);
  c2_time_schedule_8 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 20);
  c2_time_schedule_7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 19);
  c2_time_schedule_6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 18);
  c2_time_schedule_5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 17);
  c2_time_schedule_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 16);
  c2_time_schedule_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 15);
  c2_time_schedule_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 14);
  c2_time_schedule_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 13);
  c2_TS7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 12);
  c2_TS6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 11);
  c2_TS5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 10);
  c2_TS4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c2_TS3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TS2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_TS1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_TA2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_TM2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_TA1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_TM1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_BO = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(34));
  c2_hoistedGlobal = *c2_BO;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_INIT;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_TA1;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *c2_TA2;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 3, c2_e_y);
  for (c2_i0 = 0; c2_i0 < 9; c2_i0 = c2_i0 + 1) {
    c2_e_hoistedGlobal[c2_i0] = (*c2_TAU_K)[c2_i0];
  }

  for (c2_i1 = 0; c2_i1 < 9; c2_i1 = c2_i1 + 1) {
    c2_e_u[c2_i1] = c2_e_hoistedGlobal[c2_i1];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 1U, 0U, 2, 1, 9));
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = *c2_TM1;
  c2_f_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_hoistedGlobal = *c2_TM2;
  c2_g_u = c2_g_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_h_hoistedGlobal = *c2_TS1;
  c2_h_u = c2_h_hoistedGlobal;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_i_hoistedGlobal = *c2_TS2;
  c2_i_u = c2_i_hoistedGlobal;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 8, c2_j_y);
  c2_j_hoistedGlobal = *c2_TS3;
  c2_j_u = c2_j_hoistedGlobal;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 9, c2_k_y);
  c2_k_hoistedGlobal = *c2_TS4;
  c2_k_u = c2_k_hoistedGlobal;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 10, c2_l_y);
  c2_l_hoistedGlobal = *c2_TS5;
  c2_l_u = c2_l_hoistedGlobal;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_l_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 11, c2_m_y);
  c2_m_hoistedGlobal = *c2_TS6;
  c2_m_u = c2_m_hoistedGlobal;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_m_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 12, c2_n_y);
  c2_n_hoistedGlobal = *c2_TS7;
  c2_n_u = c2_n_hoistedGlobal;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_n_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 13, c2_o_y);
  c2_o_hoistedGlobal = *c2_lateOUT_1;
  c2_o_u = c2_o_hoistedGlobal;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_o_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 14, c2_p_y);
  c2_p_hoistedGlobal = *c2_lateOUT_2;
  c2_p_u = c2_p_hoistedGlobal;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_p_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 15, c2_q_y);
  c2_q_hoistedGlobal = *c2_lateOUT_3;
  c2_q_u = c2_q_hoistedGlobal;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_q_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 16, c2_r_y);
  c2_r_hoistedGlobal = *c2_lateOUT_4;
  c2_r_u = c2_r_hoistedGlobal;
  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", &c2_r_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 17, c2_s_y);
  c2_s_hoistedGlobal = *c2_randStateOUT;
  c2_s_u = c2_s_hoistedGlobal;
  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_create("y", &c2_s_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 18, c2_t_y);
  c2_t_hoistedGlobal = *c2_time_schedule_1;
  c2_t_u = c2_t_hoistedGlobal;
  c2_u_y = NULL;
  sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_t_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 19, c2_u_y);
  c2_u_hoistedGlobal = *c2_time_schedule_10;
  c2_u_u = c2_u_hoistedGlobal;
  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_create("y", &c2_u_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 20, c2_v_y);
  c2_v_hoistedGlobal = *c2_time_schedule_11;
  c2_v_u = c2_v_hoistedGlobal;
  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_create("y", &c2_v_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 21, c2_w_y);
  c2_w_hoistedGlobal = *c2_time_schedule_2;
  c2_w_u = c2_w_hoistedGlobal;
  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_w_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 22, c2_x_y);
  c2_x_hoistedGlobal = *c2_time_schedule_3;
  c2_x_u = c2_x_hoistedGlobal;
  c2_y_y = NULL;
  sf_mex_assign(&c2_y_y, sf_mex_create("y", &c2_x_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 23, c2_y_y);
  c2_y_hoistedGlobal = *c2_time_schedule_4;
  c2_y_u = c2_y_hoistedGlobal;
  c2_ab_y = NULL;
  sf_mex_assign(&c2_ab_y, sf_mex_create("y", &c2_y_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 24, c2_ab_y);
  c2_ab_hoistedGlobal = *c2_time_schedule_5;
  c2_ab_u = c2_ab_hoistedGlobal;
  c2_bb_y = NULL;
  sf_mex_assign(&c2_bb_y, sf_mex_create("y", &c2_ab_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 25, c2_bb_y);
  c2_bb_hoistedGlobal = *c2_time_schedule_6;
  c2_bb_u = c2_bb_hoistedGlobal;
  c2_cb_y = NULL;
  sf_mex_assign(&c2_cb_y, sf_mex_create("y", &c2_bb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 26, c2_cb_y);
  c2_cb_hoistedGlobal = *c2_time_schedule_7;
  c2_cb_u = c2_cb_hoistedGlobal;
  c2_db_y = NULL;
  sf_mex_assign(&c2_db_y, sf_mex_create("y", &c2_cb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 27, c2_db_y);
  c2_db_hoistedGlobal = *c2_time_schedule_8;
  c2_db_u = c2_db_hoistedGlobal;
  c2_eb_y = NULL;
  sf_mex_assign(&c2_eb_y, sf_mex_create("y", &c2_db_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 28, c2_eb_y);
  c2_eb_hoistedGlobal = *c2_time_schedule_9;
  c2_eb_u = c2_eb_hoistedGlobal;
  c2_fb_y = NULL;
  sf_mex_assign(&c2_fb_y, sf_mex_create("y", &c2_eb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 29, c2_fb_y);
  c2_fb_hoistedGlobal = chartInstance->c2_method;
  c2_fb_u = c2_fb_hoistedGlobal;
  c2_gb_y = NULL;
  if (!chartInstance->c2_method_not_empty) {
    sf_mex_assign(&c2_gb_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_gb_y, sf_mex_create("y", &c2_fb_u, 3, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c2_y, 30, c2_gb_y);
  for (c2_i2 = 0; c2_i2 < 625; c2_i2 = c2_i2 + 1) {
    c2_gb_hoistedGlobal[c2_i2] = chartInstance->c2_twister_state[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 625; c2_i3 = c2_i3 + 1) {
    c2_gb_u[c2_i3] = c2_gb_hoistedGlobal[c2_i3];
  }

  c2_hb_y = NULL;
  if (!chartInstance->c2_twister_state_not_empty) {
    sf_mex_assign(&c2_hb_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_hb_y, sf_mex_create("y", &c2_gb_u, 7, 0U, 1U, 0U, 1, 625));
  }

  sf_mex_setcell(c2_y, 31, c2_hb_y);
  c2_hb_hoistedGlobal = chartInstance->c2_v4_state;
  c2_hb_u = c2_hb_hoistedGlobal;
  c2_ib_y = NULL;
  if (!chartInstance->c2_v4_state_not_empty) {
    sf_mex_assign(&c2_ib_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c2_ib_y, sf_mex_create("y", &c2_hb_u, 7, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c2_y, 32, c2_ib_y);
  c2_ib_hoistedGlobal = chartInstance->c2_is_active_c2_SendFetchS2;
  c2_ib_u = c2_ib_hoistedGlobal;
  c2_jb_y = NULL;
  sf_mex_assign(&c2_jb_y, sf_mex_create("y", &c2_ib_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c2_y, 33, c2_jb_y);
  sf_mex_assign(&c2_st, c2_y);
  return c2_st;
}

static void set_sim_state_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[9];
  int32_T c2_i4;
  uint32_T c2_uv0[625];
  int32_T c2_i5;
  real_T *c2_BO;
  real_T *c2_INIT;
  real_T *c2_TA1;
  real_T *c2_TA2;
  real_T *c2_TM1;
  real_T *c2_TM2;
  real_T *c2_TS1;
  real_T *c2_TS2;
  real_T *c2_TS3;
  real_T *c2_TS4;
  real_T *c2_TS5;
  real_T *c2_TS6;
  real_T *c2_TS7;
  real_T *c2_lateOUT_1;
  real_T *c2_lateOUT_2;
  real_T *c2_lateOUT_3;
  real_T *c2_lateOUT_4;
  real_T *c2_randStateOUT;
  real_T *c2_time_schedule_1;
  real_T *c2_time_schedule_10;
  real_T *c2_time_schedule_11;
  real_T *c2_time_schedule_2;
  real_T *c2_time_schedule_3;
  real_T *c2_time_schedule_4;
  real_T *c2_time_schedule_5;
  real_T *c2_time_schedule_6;
  real_T *c2_time_schedule_7;
  real_T *c2_time_schedule_8;
  real_T *c2_time_schedule_9;
  real_T (*c2_TAU_K)[9];
  c2_randStateOUT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 30);
  c2_lateOUT_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 29);
  c2_lateOUT_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 28);
  c2_lateOUT_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 27);
  c2_lateOUT_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 26);
  c2_INIT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 25);
  c2_TAU_K = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 24);
  c2_time_schedule_11 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 23);
  c2_time_schedule_10 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 22);
  c2_time_schedule_9 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 21);
  c2_time_schedule_8 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 20);
  c2_time_schedule_7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 19);
  c2_time_schedule_6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 18);
  c2_time_schedule_5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 17);
  c2_time_schedule_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 16);
  c2_time_schedule_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 15);
  c2_time_schedule_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 14);
  c2_time_schedule_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 13);
  c2_TS7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 12);
  c2_TS6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 11);
  c2_TS5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 10);
  c2_TS4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c2_TS3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TS2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_TS1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_TA2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_TM2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_TA1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_TM1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_BO = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  *c2_BO = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
    "BO");
  *c2_INIT = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    1)), "INIT");
  *c2_TA1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
    "TA1");
  *c2_TA2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 3)),
    "TA2");
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
                        "TAU_K", c2_dv0);
  for (c2_i4 = 0; c2_i4 < 9; c2_i4 = c2_i4 + 1) {
    (*c2_TAU_K)[c2_i4] = c2_dv0[c2_i4];
  }

  *c2_TM1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 5)),
    "TM1");
  *c2_TM2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 6)),
    "TM2");
  *c2_TS1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 7)),
    "TS1");
  *c2_TS2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 8)),
    "TS2");
  *c2_TS3 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 9)),
    "TS3");
  *c2_TS4 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    10)), "TS4");
  *c2_TS5 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    11)), "TS5");
  *c2_TS6 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    12)), "TS6");
  *c2_TS7 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    13)), "TS7");
  *c2_lateOUT_1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 14)), "lateOUT_1");
  *c2_lateOUT_2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 15)), "lateOUT_2");
  *c2_lateOUT_3 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 16)), "lateOUT_3");
  *c2_lateOUT_4 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 17)), "lateOUT_4");
  *c2_randStateOUT = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 18)), "randStateOUT");
  *c2_time_schedule_1 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 19)), "time_schedule_1");
  *c2_time_schedule_10 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 20)), "time_schedule_10");
  *c2_time_schedule_11 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 21)), "time_schedule_11");
  *c2_time_schedule_2 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 22)), "time_schedule_2");
  *c2_time_schedule_3 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 23)), "time_schedule_3");
  *c2_time_schedule_4 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 24)), "time_schedule_4");
  *c2_time_schedule_5 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 25)), "time_schedule_5");
  *c2_time_schedule_6 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 26)), "time_schedule_6");
  *c2_time_schedule_7 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 27)), "time_schedule_7");
  *c2_time_schedule_8 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 28)), "time_schedule_8");
  *c2_time_schedule_9 = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 29)), "time_schedule_9");
  chartInstance->c2_method = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 30)), "method");
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 31)),
                        "twister_state", c2_uv0);
  for (c2_i5 = 0; c2_i5 < 625; c2_i5 = c2_i5 + 1) {
    chartInstance->c2_twister_state[c2_i5] = c2_uv0[c2_i5];
  }

  chartInstance->c2_v4_state = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 32)), "v4_state");
  chartInstance->c2_is_active_c2_SendFetchS2 = c2_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 33)),
     "is_active_c2_SendFetchS2");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_SendFetchS2(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct
  *chartInstance)
{
}

static void sf_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  int32_T c2_i6;
  int32_T c2_previousEvent;
  real_T *c2_Y11;
  real_T *c2_BO;
  real_T *c2_Y12;
  real_T *c2_Y21;
  real_T *c2_Y22;
  real_T *c2_S1;
  real_T *c2_S2;
  real_T *c2_S3;
  real_T *c2_S4;
  real_T *c2_S5;
  real_T *c2_TM1;
  real_T *c2_TA1;
  real_T *c2_TM2;
  real_T *c2_TA2;
  real_T *c2_TS1;
  real_T *c2_TS2;
  real_T *c2_TS3;
  real_T *c2_TS4;
  real_T *c2_TS5;
  real_T *c2_TS6;
  real_T *c2_TS7;
  real_T *c2_TIME;
  real_T *c2_I1;
  real_T *c2_I2;
  real_T *c2_current_BO;
  real_T *c2_time_schedule_1;
  real_T *c2_time_schedule1_1;
  real_T *c2_time_schedule1_2;
  real_T *c2_time_schedule1_3;
  real_T *c2_time_schedule1_4;
  real_T *c2_time_schedule1_5;
  real_T *c2_time_schedule1_6;
  real_T *c2_time_schedule1_7;
  real_T *c2_time_schedule1_8;
  real_T *c2_time_schedule1_9;
  real_T *c2_time_schedule1_10;
  real_T *c2_time_schedule1_11;
  real_T *c2_time_schedule_2;
  real_T *c2_time_schedule_3;
  real_T *c2_time_schedule_4;
  real_T *c2_time_schedule_5;
  real_T *c2_time_schedule_6;
  real_T *c2_time_schedule_7;
  real_T *c2_time_schedule_8;
  real_T *c2_time_schedule_9;
  real_T *c2_time_schedule_10;
  real_T *c2_time_schedule_11;
  real_T *c2_INIT;
  real_T *c2_INIT_in;
  real_T *c2_lateIN_1;
  real_T *c2_lateIN_2;
  real_T *c2_lateOUT_1;
  real_T *c2_lateOUT_2;
  real_T *c2_lateIN_3;
  real_T *c2_lateIN_4;
  real_T *c2_lateOUT_3;
  real_T *c2_lateOUT_4;
  real_T *c2_randStateIN;
  real_T *c2_randStateOUT;
  real_T (*c2_TAU_K)[9];
  c2_randStateOUT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 30);
  c2_randStateIN = (real_T *)ssGetInputPortSignal(chartInstance->S, 29);
  c2_lateOUT_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 29);
  c2_lateOUT_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 28);
  c2_lateIN_4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 28);
  c2_lateIN_3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 27);
  c2_lateOUT_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 27);
  c2_lateOUT_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 26);
  c2_lateIN_2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 26);
  c2_lateIN_1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 25);
  c2_INIT_in = (real_T *)ssGetInputPortSignal(chartInstance->S, 24);
  c2_INIT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 25);
  c2_TAU_K = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 24);
  c2_time_schedule_11 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 23);
  c2_time_schedule_10 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 22);
  c2_time_schedule_9 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 21);
  c2_time_schedule_8 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 20);
  c2_time_schedule_7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 19);
  c2_time_schedule_6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 18);
  c2_time_schedule_5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 17);
  c2_time_schedule_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 16);
  c2_time_schedule_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 15);
  c2_time_schedule_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 14);
  c2_time_schedule1_11 = (real_T *)ssGetInputPortSignal(chartInstance->S, 23);
  c2_time_schedule1_10 = (real_T *)ssGetInputPortSignal(chartInstance->S, 22);
  c2_time_schedule1_9 = (real_T *)ssGetInputPortSignal(chartInstance->S, 21);
  c2_time_schedule1_8 = (real_T *)ssGetInputPortSignal(chartInstance->S, 20);
  c2_time_schedule1_7 = (real_T *)ssGetInputPortSignal(chartInstance->S, 19);
  c2_time_schedule1_6 = (real_T *)ssGetInputPortSignal(chartInstance->S, 18);
  c2_time_schedule1_5 = (real_T *)ssGetInputPortSignal(chartInstance->S, 17);
  c2_time_schedule1_4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 16);
  c2_time_schedule1_3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 15);
  c2_time_schedule1_2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 14);
  c2_time_schedule1_1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 13);
  c2_time_schedule_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 13);
  c2_current_BO = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c2_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c2_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c2_TIME = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c2_TS7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 12);
  c2_TS6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 11);
  c2_TS5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 10);
  c2_TS4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c2_TS3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TS2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_TS1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_TA2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_TM2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_TA1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_TM1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_S5 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_S4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_S3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_S2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_S1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_Y22 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_Y21 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_Y12 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_BO = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Y11 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,0);
  _SFD_DATA_RANGE_CHECK(*c2_Y11, 0U);
  _SFD_DATA_RANGE_CHECK(*c2_BO, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_Y12, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_Y21, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_Y22, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_S1, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_S2, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_S3, 7U);
  _SFD_DATA_RANGE_CHECK(*c2_S4, 8U);
  _SFD_DATA_RANGE_CHECK(*c2_S5, 9U);
  _SFD_DATA_RANGE_CHECK(*c2_TM1, 10U);
  _SFD_DATA_RANGE_CHECK(*c2_TA1, 11U);
  _SFD_DATA_RANGE_CHECK(*c2_TM2, 12U);
  _SFD_DATA_RANGE_CHECK(*c2_TA2, 13U);
  _SFD_DATA_RANGE_CHECK(*c2_TS1, 14U);
  _SFD_DATA_RANGE_CHECK(*c2_TS2, 15U);
  _SFD_DATA_RANGE_CHECK(*c2_TS3, 16U);
  _SFD_DATA_RANGE_CHECK(*c2_TS4, 17U);
  _SFD_DATA_RANGE_CHECK(*c2_TS5, 18U);
  _SFD_DATA_RANGE_CHECK(*c2_TS6, 19U);
  _SFD_DATA_RANGE_CHECK(*c2_TS7, 20U);
  _SFD_DATA_RANGE_CHECK(*c2_TIME, 21U);
  _SFD_DATA_RANGE_CHECK(*c2_I1, 22U);
  _SFD_DATA_RANGE_CHECK(*c2_I2, 23U);
  _SFD_DATA_RANGE_CHECK(*c2_current_BO, 24U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_1, 25U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_1, 26U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_2, 27U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_3, 28U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_4, 29U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_5, 30U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_6, 31U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_7, 32U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_8, 33U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_9, 34U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_10, 35U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule1_11, 36U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_2, 37U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_3, 38U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_4, 39U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_5, 40U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_6, 41U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_7, 42U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_8, 43U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_9, 44U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_10, 45U);
  _SFD_DATA_RANGE_CHECK(*c2_time_schedule_11, 46U);
  for (c2_i6 = 0; c2_i6 < 9; c2_i6 = c2_i6 + 1) {
    _SFD_DATA_RANGE_CHECK((*c2_TAU_K)[c2_i6], 47U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_INIT, 48U);
  _SFD_DATA_RANGE_CHECK(*c2_INIT_in, 49U);
  _SFD_DATA_RANGE_CHECK(*c2_lateIN_1, 50U);
  _SFD_DATA_RANGE_CHECK(*c2_lateIN_2, 51U);
  _SFD_DATA_RANGE_CHECK(*c2_lateOUT_1, 52U);
  _SFD_DATA_RANGE_CHECK(*c2_lateOUT_2, 53U);
  _SFD_DATA_RANGE_CHECK(*c2_lateIN_3, 54U);
  _SFD_DATA_RANGE_CHECK(*c2_lateIN_4, 55U);
  _SFD_DATA_RANGE_CHECK(*c2_lateOUT_3, 56U);
  _SFD_DATA_RANGE_CHECK(*c2_lateOUT_4, 57U);
  _SFD_DATA_RANGE_CHECK(*c2_randStateIN, 58U);
  _SFD_DATA_RANGE_CHECK(*c2_randStateOUT, 59U);
  c2_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c2_c2_SendFetchS2(chartInstance);
  _sfEvent_ = c2_previousEvent;
  sf_debug_check_for_state_inconsistency(_SendFetchS2MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_c2_SendFetchS2(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_f_hoistedGlobal;
  real_T c2_g_hoistedGlobal;
  real_T c2_h_hoistedGlobal;
  real_T c2_i_hoistedGlobal;
  real_T c2_j_hoistedGlobal;
  real_T c2_k_hoistedGlobal;
  real_T c2_l_hoistedGlobal;
  real_T c2_m_hoistedGlobal;
  real_T c2_n_hoistedGlobal;
  real_T c2_o_hoistedGlobal;
  real_T c2_p_hoistedGlobal;
  real_T c2_q_hoistedGlobal;
  real_T c2_r_hoistedGlobal;
  real_T c2_s_hoistedGlobal;
  real_T c2_t_hoistedGlobal;
  real_T c2_u_hoistedGlobal;
  real_T c2_v_hoistedGlobal;
  real_T c2_w_hoistedGlobal;
  real_T c2_x_hoistedGlobal;
  real_T c2_y_hoistedGlobal;
  real_T c2_ab_hoistedGlobal;
  real_T c2_bb_hoistedGlobal;
  real_T c2_cb_hoistedGlobal;
  real_T c2_db_hoistedGlobal;
  real_T c2_eb_hoistedGlobal;
  real_T c2_Y11;
  real_T c2_Y12;
  real_T c2_Y21;
  real_T c2_Y22;
  real_T c2_S1;
  real_T c2_S2;
  real_T c2_S3;
  real_T c2_S4;
  real_T c2_S5;
  real_T c2_TIME;
  real_T c2_I1;
  real_T c2_I2;
  real_T c2_current_BO;
  real_T c2_time_schedule1_1;
  real_T c2_time_schedule1_2;
  real_T c2_time_schedule1_3;
  real_T c2_time_schedule1_4;
  real_T c2_time_schedule1_5;
  real_T c2_time_schedule1_6;
  real_T c2_time_schedule1_7;
  real_T c2_time_schedule1_8;
  real_T c2_time_schedule1_9;
  real_T c2_time_schedule1_10;
  real_T c2_time_schedule1_11;
  real_T c2_INIT_in;
  real_T c2_lateIN_1;
  real_T c2_lateIN_2;
  real_T c2_lateIN_3;
  real_T c2_lateIN_4;
  real_T c2_randStateIN;
  uint32_T c2_debug_family_var_map[97];
  static const char *c2_sv0[97] = { "BI", "randState", "tBI", "symbol_time",
    "deltaCAP", "ttStart", "time_schedule1", "lateIN",
    "lateOUT", "time_schedule", "i", "scheduled_next", "slot", "y1L1_new",
    "y1L2_new", "y1INT_new",
    "y2L1_new", "y2L2_new", "y2INT_new", "ttStart_next", "data", "x10", "x20",
    "x30", "lvm", "temp17"
    , "next_time", "ii", "j", "B", "pos", "slot_num", "l", "node", "randState2",
    "nargin", "nargout",
    "Y11", "Y12", "Y21", "Y22", "S1", "S2", "S3", "S4", "S5", "TIME", "I1", "I2",
    "current_BO",
    "time_schedule1_1", "time_schedule1_2", "time_schedule1_3",
    "time_schedule1_4", "time_schedule1_5"
    , "time_schedule1_6", "time_schedule1_7", "time_schedule1_8",
    "time_schedule1_9",
    "time_schedule1_10", "time_schedule1_11", "INIT_in", "lateIN_1", "lateIN_2",
    "lateIN_3",
    "lateIN_4", "randStateIN", "BO", "TM1", "TA1", "TM2", "TA2", "TS1", "TS2",
    "TS3", "TS4", "TS5",
    "TS6", "TS7", "time_schedule_1", "time_schedule_2", "time_schedule_3",
    "time_schedule_4",
    "time_schedule_5", "time_schedule_6", "time_schedule_7", "time_schedule_8",
    "time_schedule_9",
    "time_schedule_10", "time_schedule_11", "TAU_K", "INIT", "lateOUT_1",
    "lateOUT_2", "lateOUT_3",
    "lateOUT_4", "randStateOUT" };

  real_T c2_BI[11];
  real_T c2_randState;
  real_T c2_tBI;
  real_T c2_symbol_time;
  real_T c2_deltaCAP;
  real_T c2_ttStart;
  real_T c2_time_schedule1[11];
  real_T c2_lateIN[4];
  real_T c2_lateOUT[4];
  real_T c2_time_schedule[11];
  real_T c2_i;
  real_T c2_scheduled_next[22];
  real_T c2_slot[11];
  real_T c2_y1L1_new;
  real_T c2_y1L2_new;
  real_T c2_y1INT_new;
  real_T c2_y2L1_new;
  real_T c2_y2L2_new;
  real_T c2_y2INT_new;
  real_T c2_ttStart_next;
  struct smQvuE1V6fJPmdrAbGjhegG c2_data;
  creal_T c2_x10;
  creal_T c2_x20;
  creal_T c2_x30;
  real_T c2_lvm[600];
  creal_T c2_temp17[100];
  real_T c2_next_time;
  real_T c2_ii;
  real_T c2_j;
  real_T c2_B[11];
  real_T c2_pos[11];
  real_T c2_slot_num;
  real_T c2_l;
  real_T c2_node;
  uint32_T c2_randState2[625];
  real_T c2_nargin = 30.0;
  real_T c2_nargout = 30.0;
  real_T c2_BO;
  real_T c2_TM1;
  real_T c2_TA1;
  real_T c2_TM2;
  real_T c2_TA2;
  real_T c2_TS1;
  real_T c2_TS2;
  real_T c2_TS3;
  real_T c2_TS4;
  real_T c2_TS5;
  real_T c2_TS6;
  real_T c2_TS7;
  real_T c2_time_schedule_1;
  real_T c2_time_schedule_2;
  real_T c2_time_schedule_3;
  real_T c2_time_schedule_4;
  real_T c2_time_schedule_5;
  real_T c2_time_schedule_6;
  real_T c2_time_schedule_7;
  real_T c2_time_schedule_8;
  real_T c2_time_schedule_9;
  real_T c2_time_schedule_10;
  real_T c2_time_schedule_11;
  real_T c2_TAU_K[9];
  real_T c2_INIT;
  real_T c2_lateOUT_1;
  real_T c2_lateOUT_2;
  real_T c2_lateOUT_3;
  real_T c2_lateOUT_4;
  real_T c2_randStateOUT;
  int32_T c2_i7;
  static char_T c2_cv0[60] = { '+', '-', '-', '-', '-', '-', '-', '-', '-', '-',
    '-', '-', '-', '-', '-', '-', '-', '-', '-', '+',
    '\\', 'n', 'S', 't', 'a', 'r', 't', 'i', 'n', 'g', ' ', 'E', 'm', 'b', 'e',
    'd', 'd', 'e', 'd', ' ',
    'M', 'A', 'T', 'L', 'A', 'B', ' ', 's', 'c', 'r', 'i', 'p', 't', '.', '.',
    '.', '\\', 'n', '\\', 'n' };

  char_T c2_u[60];
  const mxArray *c2_y = NULL;
  int32_T c2_i8;
  static real_T c2_dv1[11] = { 0.2344, 0.4688, 0.9375, 1.875, 3.75, 7.5, 15.0,
    30.0, 60.0, 120.0, 240.0 };

  int32_T c2_i9;
  real_T c2_a;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  real_T c2_b_time_schedule1[11];
  int32_T c2_i14;
  int32_T c2_i15;
  static char_T c2_cv1[51] = { 'F', 'i', 'r', 's', 't', ' ', 'r', 'u', 'n', ' ',
    'o', 'f', ' ', 't', 'h', 'e', ' ', 's', 'c', 'r', 'i'
    , 'p', 't', '!', '\\', 'n', '\\', 'n', ' ', 'I', 'N', 'I', 'T', 'I', 'A',
    'L', 'I', 'Z', 'I', 'N', 'G'
    , '.', '.', '.', '.', '.', ' ', '\\', 'n', '\\', 'n' };

  char_T c2_b_u[51];
  const mxArray *c2_b_y = NULL;
  real_T c2_b_a;
  int32_T c2_i16;
  real_T c2_c_y[11];
  int32_T c2_i17;
  real_T c2_b_i;
  real_T c2_c_a;
  real_T c2_d_y;
  real_T c2_d_a;
  real_T c2_e_y;
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  static char_T c2_cv2[137] = { 'L', 'a', 's', 't', ' ', 'b', 'e', 'a', 'c', 'o',
    'n', '+', 'd', 'e', 'l', 't', 'a', 'C', 'A', 'P',
    ':', ' ', '%', 'f', ' ', '+', ' ', '%', 'f', ' ', '=', ' ', '%', 'f', ' ',
    ' ', ' ', ' ', '(', 'A',
    'n', 'y', 't', 'h', 'i', 'n', 'g', ' ', 'i', 'n', ' ', 'b', 'e', 't', 'w',
    'e', 'e', 'n', ' ', 's',
    'h', 'o', 'u', 'l', 'd', ' ', 'h', 'a', 'v', 'e', ' ', 'b', 'e', 'e', 'n',
    ' ', 's', 'c', 'h', 'e',
    'd', 'u', 'l', 'e', 'd', ' ', 't', 'o', ' ', 't', 'r', 'a', 'n', 's', 'm',
    'i', 't', '!', ')', '\\',
    'n', 'N', 'e', 'x', 't', ' ', 'b', 'e', 'a', 'c', 'o', 'n', '+', 'd', 'e',
    'l', 't', 'a', 'C', 'A',
    'P', ':', ' ', '%', 'f', ' ', '+', ' ', '%', 'f', ' ', '=', ' ', '%', 'f',
    '\\', 'n' };

  char_T c2_c_u[137];
  const mxArray *c2_f_y = NULL;
  real_T c2_d_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_f_u;
  const mxArray *c2_i_y = NULL;
  real_T c2_g_u;
  const mxArray *c2_j_y = NULL;
  real_T c2_h_u;
  const mxArray *c2_k_y = NULL;
  real_T c2_i_u;
  const mxArray *c2_l_y = NULL;
  int32_T c2_i22;
  static char_T c2_cv3[36] = { '\\', 'n', 'S', 'c', 'h', 'e', 'd', 'u', 'l', 'e',
    'd', ' ', 't', 'i', 'm', 'e', ' ', 'f', 'o', 'r',
    ' ', 't', 'r', 'a', 'n', 's', 'm', 'i', 's', 's', 'i', 'o', 'n', ':', '\\',
    'n' };

  char_T c2_j_u[36];
  const mxArray *c2_m_y = NULL;
  int32_T c2_i23;
  static char_T c2_cv4[10] = { 'N', 'o', 'd', 'e', ' ', '1', ':', ' ', '%', 'f'
  };

  char_T c2_k_u[10];
  const mxArray *c2_n_y = NULL;
  real_T c2_l_u;
  const mxArray *c2_o_y = NULL;
  int32_T c2_i24;
  boolean_T c2_b_lateIN[4];
  int32_T c2_i25;
  static char_T c2_cv5[29] = { ' ', '*', 'I', 'S', ' ', 'G', 'O', 'N', 'N', 'A',
    ' ', 'M', 'I', 'S', 'S', ' ', 'T', 'H', 'E', ' ', 'D'
    , 'E', 'A', 'D', 'L', 'I', 'N', 'E', '!' };

  char_T c2_m_u[29];
  const mxArray *c2_p_y = NULL;
  int32_T c2_i26;
  static char_T c2_cv6[4] = { ' ', '(', '*', ')' };

  char_T c2_n_u[4];
  const mxArray *c2_q_y = NULL;
  int32_T c2_i27;
  static char_T c2_cv7[2] = { '\\', 'n' };

  char_T c2_o_u[2];
  const mxArray *c2_r_y = NULL;
  int32_T c2_i28;
  static char_T c2_cv8[10] = { 'N', 'o', 'd', 'e', ' ', '2', ':', ' ', '%', 'f'
  };

  char_T c2_p_u[10];
  const mxArray *c2_s_y = NULL;
  real_T c2_q_u;
  const mxArray *c2_t_y = NULL;
  int32_T c2_i29;
  boolean_T c2_c_lateIN[4];
  int32_T c2_i30;
  char_T c2_r_u[29];
  const mxArray *c2_u_y = NULL;
  int32_T c2_i31;
  char_T c2_s_u[4];
  const mxArray *c2_v_y = NULL;
  int32_T c2_i32;
  char_T c2_t_u[2];
  const mxArray *c2_w_y = NULL;
  int32_T c2_i33;
  static char_T c2_cv9[10] = { 'N', 'o', 'd', 'e', ' ', '3', ':', ' ', '%', 'f'
  };

  char_T c2_u_u[10];
  const mxArray *c2_x_y = NULL;
  real_T c2_v_u;
  const mxArray *c2_y_y = NULL;
  int32_T c2_i34;
  boolean_T c2_d_lateIN[4];
  int32_T c2_i35;
  char_T c2_w_u[29];
  const mxArray *c2_ab_y = NULL;
  int32_T c2_i36;
  char_T c2_x_u[4];
  const mxArray *c2_bb_y = NULL;
  int32_T c2_i37;
  char_T c2_y_u[2];
  const mxArray *c2_cb_y = NULL;
  int32_T c2_i38;
  static char_T c2_cv10[10] = { 'N', 'o', 'd', 'e', ' ', '4', ':', ' ', '%', 'f'
  };

  char_T c2_ab_u[10];
  const mxArray *c2_db_y = NULL;
  real_T c2_bb_u;
  const mxArray *c2_eb_y = NULL;
  int32_T c2_i39;
  boolean_T c2_e_lateIN[4];
  int32_T c2_i40;
  char_T c2_cb_u[29];
  const mxArray *c2_fb_y = NULL;
  int32_T c2_i41;
  char_T c2_db_u[4];
  const mxArray *c2_gb_y = NULL;
  int32_T c2_i42;
  char_T c2_eb_u[2];
  const mxArray *c2_hb_y = NULL;
  int32_T c2_i43;
  static char_T c2_cv11[10] = { 'N', 'o', 'd', 'e', ' ', '5', ':', ' ', '%', 'f'
  };

  char_T c2_fb_u[10];
  const mxArray *c2_ib_y = NULL;
  real_T c2_gb_u;
  const mxArray *c2_jb_y = NULL;
  int32_T c2_i44;
  boolean_T c2_f_lateIN[4];
  int32_T c2_i45;
  char_T c2_hb_u[29];
  const mxArray *c2_kb_y = NULL;
  int32_T c2_i46;
  char_T c2_ib_u[4];
  const mxArray *c2_lb_y = NULL;
  int32_T c2_i47;
  char_T c2_jb_u[2];
  const mxArray *c2_mb_y = NULL;
  int32_T c2_i48;
  static char_T c2_cv12[10] = { 'N', 'o', 'd', 'e', ' ', '6', ':', ' ', '%', 'f'
  };

  char_T c2_kb_u[10];
  const mxArray *c2_nb_y = NULL;
  real_T c2_lb_u;
  const mxArray *c2_ob_y = NULL;
  int32_T c2_i49;
  boolean_T c2_g_lateIN[4];
  int32_T c2_i50;
  char_T c2_mb_u[29];
  const mxArray *c2_pb_y = NULL;
  int32_T c2_i51;
  char_T c2_nb_u[4];
  const mxArray *c2_qb_y = NULL;
  int32_T c2_i52;
  char_T c2_ob_u[2];
  const mxArray *c2_rb_y = NULL;
  int32_T c2_i53;
  static char_T c2_cv13[10] = { 'N', 'o', 'd', 'e', ' ', '7', ':', ' ', '%', 'f'
  };

  char_T c2_pb_u[10];
  const mxArray *c2_sb_y = NULL;
  real_T c2_qb_u;
  const mxArray *c2_tb_y = NULL;
  int32_T c2_i54;
  boolean_T c2_h_lateIN[4];
  int32_T c2_i55;
  char_T c2_rb_u[29];
  const mxArray *c2_ub_y = NULL;
  int32_T c2_i56;
  char_T c2_sb_u[4];
  const mxArray *c2_vb_y = NULL;
  int32_T c2_i57;
  char_T c2_tb_u[2];
  const mxArray *c2_wb_y = NULL;
  int32_T c2_i58;
  static char_T c2_cv14[10] = { 'N', 'o', 'd', 'e', ' ', '8', ':', ' ', '%', 'f'
  };

  char_T c2_ub_u[10];
  const mxArray *c2_xb_y = NULL;
  real_T c2_vb_u;
  const mxArray *c2_yb_y = NULL;
  int32_T c2_i59;
  boolean_T c2_i_lateIN[4];
  int32_T c2_i60;
  char_T c2_wb_u[29];
  const mxArray *c2_ac_y = NULL;
  int32_T c2_i61;
  char_T c2_xb_u[4];
  const mxArray *c2_bc_y = NULL;
  int32_T c2_i62;
  char_T c2_yb_u[2];
  const mxArray *c2_cc_y = NULL;
  int32_T c2_i63;
  static char_T c2_cv15[10] = { 'N', 'o', 'd', 'e', ' ', '9', ':', ' ', '%', 'f'
  };

  char_T c2_ac_u[10];
  const mxArray *c2_dc_y = NULL;
  real_T c2_bc_u;
  const mxArray *c2_ec_y = NULL;
  int32_T c2_i64;
  boolean_T c2_j_lateIN[4];
  int32_T c2_i65;
  char_T c2_cc_u[29];
  const mxArray *c2_fc_y = NULL;
  int32_T c2_i66;
  char_T c2_dc_u[4];
  const mxArray *c2_gc_y = NULL;
  int32_T c2_i67;
  char_T c2_ec_u[2];
  const mxArray *c2_hc_y = NULL;
  int32_T c2_i68;
  static char_T c2_cv16[11] = { 'N', 'o', 'd', 'e', ' ', '1', '0', ':', ' ', '%',
    'f' };

  char_T c2_fc_u[11];
  const mxArray *c2_ic_y = NULL;
  real_T c2_gc_u;
  const mxArray *c2_jc_y = NULL;
  int32_T c2_i69;
  boolean_T c2_k_lateIN[4];
  int32_T c2_i70;
  char_T c2_hc_u[29];
  const mxArray *c2_kc_y = NULL;
  int32_T c2_i71;
  char_T c2_ic_u[4];
  const mxArray *c2_lc_y = NULL;
  int32_T c2_i72;
  char_T c2_jc_u[2];
  const mxArray *c2_mc_y = NULL;
  int32_T c2_i73;
  static char_T c2_cv17[11] = { 'N', 'o', 'd', 'e', ' ', '1', '1', ':', ' ', '%',
    'f' };

  char_T c2_kc_u[11];
  const mxArray *c2_nc_y = NULL;
  real_T c2_lc_u;
  const mxArray *c2_oc_y = NULL;
  int32_T c2_i74;
  boolean_T c2_l_lateIN[4];
  int32_T c2_i75;
  char_T c2_mc_u[29];
  const mxArray *c2_pc_y = NULL;
  int32_T c2_i76;
  char_T c2_nc_u[4];
  const mxArray *c2_qc_y = NULL;
  int32_T c2_i77;
  char_T c2_oc_u[2];
  const mxArray *c2_rc_y = NULL;
  int32_T c2_i78;
  boolean_T c2_m_lateIN[4];
  real_T c2_b_y1L1_new[3];
  int32_T c2_i79;
  int32_T c2_i80;
  static real_T c2_dv2[600] = { 1.3171975181322750E+00, 4.2028561176952621E+00,
    8.5687909203510912E+00, 1.4346716667923261E+01,
    2.1488568694427499E+01, 2.9961280988022395E+01, 3.9738285001667805E+01,
    5.0789397155606252E+01,
    6.3070294724319410E+01, 7.6512414865634298E+01, 9.1013830005079626E+01,
    1.0643143823529329E+02,
    1.2257464636703693E+02, 1.3920060475797564E+02, 1.5601096805982573E+02,
    1.7265009725746239E+02,
    1.8870457990937155E+02, 2.0370392256901715E+02, 2.1712225814701745E+02,
    2.2838090841508608E+02,
    2.3685164553916303E+02, 2.4186050456269436E+02, 2.4269200966339031E+02,
    2.3859368963964289E+02,
    2.2878077158148028E+02, 2.1244095540010909E+02, 1.8873918535893833E+02,
    1.5682234763468836E+02,
    1.1582383501510981E+02, 6.4867930969351391E+01, 3.0739754317481243E+00,
    -7.0439716307895083E+01,
    -1.5655220220636210E+02, -2.5613654316214894E+02, -3.7005672136407674E+02,
    -4.9916484970730465E+02,
    -6.4429867036690746E+02, -8.0627933704395116E+02, -9.8590947249726514E+02,
    -1.1839714906725326E+03,
    -1.4012261709683810E+03, -1.6384114708700913E+03, -1.8962415622735002E+03,
    -2.1754060762571844E+03,
    -2.4765695407882504E+03, -2.8003709958185236E+03, -3.1474237704017687E+03,
    -3.5183154068009840E+03,
    -3.9136077170248777E+03, -4.3338369578051534E+03, -4.7795141106763695E+03,
    -5.2511252545257848E+03,
    -5.7491320187234278E+03, -6.2739721057072156E+03, -6.8260598726702365E+03,
    -7.4057869627676428E+03,
    -8.0135229770187161E+03, -8.6496161788196277E+03, -9.3143942236974872E+03,
    -1.0008164907622835E+04,
    -1.0731216927852225E+04, -1.1483820650892883E+04, -1.2266228882765838E+04,
    -1.3078677637292247E+04,
    -1.3921386898639124E+04, -1.4794561374835715E+04, -1.5698391239411663E+04,
    -1.6633052858712566E+04,
    -1.7598709502820162E+04, -1.8595512038343561E+04, -1.9623599601656544E+04,
    -2.0683100251435575E+04,
    -2.1774131599605891E+04, -2.2896801420028907E+04, -2.4051208234467726E+04,
    -2.5237441875547156E+04,
    -2.6455584026584558E+04, -2.7705708738308502E+04, -2.8987882922604411E+04,
    -3.0302166823533655E+04,
    -3.1648614465964383E+04, -3.3027274082229982E+04, -3.4438188517298411E+04,
    -3.5881395612989771E+04,
    -3.7356928571824435E+04, -3.8864816301121296E+04, -4.0405083737992019E+04,
    -4.1977752155899369E+04,
    -4.3582839453461333E+04, -4.5220360426191946E+04, -4.6890327021873411E+04,
    -4.8592748580253661E+04,
    -5.0327632057759140E+04, -5.2094982237905409E+04, -5.3894801928077184E+04,
    -5.5727092143337904E+04,
    -5.7591852277912702E+04, -5.9489080264973803E+04, -6.1418772725339040E+04,
    -6.3380925105675939E+04,
    1.5722808351799813E+00, 6.3326780229806445E+00, 1.4546954907368445E+01,
    2.6651694364379637E+01,
    4.3152486824096286E+01, 6.4542255268332383E+01, 9.1237160630552353E+01,
    1.2352772104411133E+02,
    1.6154297775575174E+02, 2.0522574196129131E+02, 2.5431715494560808E+02,
    3.0834898484773362E+02,
    3.6664226444849032E+02, 4.2831104385500248E+02, 4.9227018882337666E+02,
    5.5724629926500847E+02,
    6.2179095319051453E+02, 6.8429559923465149E+02, 7.4300752647447553E+02,
    7.9604643415225132E+02,
    8.4142120689285446E+02, 8.7704657386694339E+02, 9.0075939392338876E+02,
    9.1033436382332332E+02,
    9.0349899416081189E+02, 8.7794773812609151E+02, 8.3135519269047938E+02,
    7.6138832075466701E+02,
    6.6571766694166308E+02, 5.4202755961859827E+02, 3.8802530793358017E+02,
    2.0144941564067267E+02,
    -1.9923156303569613E+01, -2.7827063856228074E+02, -5.7572096805581032E+02,
    -9.1434711873960123E+02,
    -1.2961631962224760E+03, -1.7231212425055539E+03, -2.1971086966958019E+03,
    -2.7199464579315713E+03,
    -3.2933874977907253E+03, -3.9191159709979884E+03, -4.5987467751889089E+03,
    -5.3338255127231751E+03,
    -6.1258288099850006E+03, -6.9761649521915624E+03, -7.8861747943924538E+03,
    -8.8571329120355058E+03,
    -9.8902489571551559E+03, -1.0986669188879627E+04, -1.2147478149523053E+04,
    -1.3373700460012542E+04,
    -1.4666302710779997E+04, -1.6026195426514145E+04, -1.7454235085314423E+04,
    -1.8951226174807842E+04,
    -2.0517923269681134E+04, -2.2155033116846738E+04, -2.3863216716097595E+04,
    -2.5643091385622327E+04,
    -2.7495232803146810E+04, -2.9420177014748908E+04, -3.1418422404562723E+04,
    -3.3490431619656301E+04,
    -3.5636633445330728E+04, -3.7857424626965469E+04, -4.0153171635319464E+04,
    -4.2524212372903981E+04,
    -4.4970857819672667E+04, -4.7493393616834634E+04, -5.0092081588090739E+04,
    -5.2767161198030553E+04,
    -5.5518850947806968E+04, -5.8347349708538604E+04, -6.1252837993174777E+04,
    -6.4235479167803380E+04,
    -6.7295420603589460E+04, -7.0432794770704932E+04, -7.3647720275753454E+04,
    -7.6940302844312275E+04,
    -8.0310636250299722E+04, -8.3758803193954023E+04, -8.7284876130250937E+04,
    -9.0888918049629981E+04,
    -9.4570983212913197E+04, -9.8331117842303836E+04, -1.0216936077035205E+05,
    -1.0608574404875771E+05,
    -1.1008029351885084E+05, -1.1415302934556849E+05, -1.1830396651670118E+05,
    -1.2253311530914571E+05,
    -1.2684048172385166E+05, -1.3122606789110150E+05, -1.3568987244770775E+05,
    -1.4023189088766801E+05,
    -1.4485211588774473E+05, -1.4955053760940096E+05, -1.5432714397845013E+05,
    -1.5918192094373191E+05,
    3.5855233433181866E-01, 1.4696370601512854E+00, 3.3700528705610679E+00,
    6.0748344907731280E+00,
    9.5770423327920611E+00, 1.3847447791256243E+01, 1.8834271107342317E+01,
    2.4463072615509830E+01,
    3.0636855937576755E+01, 3.7236410447321347E+01, 4.4120897769942815E+01,
    5.1128671313718101E+01,
    5.8078307325923895E+01, 6.4769819488655543E+01, 7.0986025634195897E+01,
    7.6494033976299136E+01,
    8.1046816699442317E+01, 8.4384840332766771E+01, 8.6237724677693663E+01,
    8.6325904864442464E+01,
    8.4362274160010060E+01, 8.0053788272207228E+01, 7.3103014969782549E+01,
    6.3209615781669243E+01,
    5.0071749291040874E+01, 3.3387388065768192E+01, 1.2855543546206022E+01,
    -1.1822604763379388E+01,
    -4.0942677680175862E+01, -7.4796162655825441E+01, -1.1366957706894105E+02,
    -1.5784372646465587E+02,
    -2.0759305482115326E+02, -2.6318508321373326E+02, -3.2487993269702770E+02,
    -3.9292992681712502E+02,
    -4.6757926888004295E+02, -5.4906378892463090E+02, -6.3761075526119805E+02,
    -7.3343874542782737E+02,
    -8.3675757147162335E+02, -9.4776825457046800E+02, -1.0666630441619996E+03,
    -1.1936254769312829E+03,
    -1.3288304712191295E+03, -1.4724444526422333E+03, -1.6246255069582969E+03,
    -1.7855235564589691E+03,
    -1.9552805564265043E+03, -2.1340307084427791E+03, -2.3219006875889968E+03,
    -2.5190098808183029E+03,
    -2.7254706340200810E+03, -2.9413885055220130E+03, -3.1668625239933758E+03,
    -3.4019854489192494E+03,
    -3.6468440320102609E+03, -3.9015192780953644E+03, -4.1660867042164382E+03,
    -4.4406165958026477E+03,
    -4.7251742589501182E+03, -5.0198202679686310E+03, -5.3246107074822949E+03,
    -5.6395974084854306E+03,
    -5.9648281778596047E+03, -6.3003470209521902E+03, -6.6461943569027408E+03,
    -7.0024072264803235E+03,
    -7.3690194922642149E+03, -7.7460620310617469E+03, -8.1335629185118778E+03,
    -8.5315476058710738E+03,
    -8.9400390890205199E+03, -9.3590580697701971E+03, -9.7886231095674448E+03,
    -1.0228750775744471E+04,
    -1.0679455780462684E+04, -1.1140751112530597E+04, -1.1612648162288360E+04,
    -1.2095156839764237E+04,
    -1.2588285686318975E+04, -1.3092041980000909E+04, -1.3606431834840723E+04,
    -1.4131460294317601E+04,
    -1.4667131419230780E+04, -1.5213448370210199E+04, -1.5770413485099318E+04,
    -1.6338028351440562E+04,
    -1.6916293874291314E+04, -1.7505210339593144E+04, -1.8104777473313781E+04,
    -1.8714994496574702E+04,
    -1.9335860176972263E+04, -1.9967372876293593E+04, -2.0609530594822678E+04,
    -2.1262331012424453E+04,
    -2.1925771526588906E+04, -2.2599849287609275E+04, -2.3284561231062442E+04,
    -2.3979904107751699E+04,
    4.0442924234707789E-01, 2.1201344142091330E+00, 7.0857042841009843E+00,
    1.6837700324913840E+01,
    3.2534920417671628E+01, 5.4984137906989417E+01, 8.4666701600885290E+01,
    1.2176548040290800E+02,
    1.6619172409131875E+02, 2.1761148977697212E+02, 2.7547135235021688E+02,
    3.3902317740892454E+02,
    4.0734778742278741E+02, 4.7937739692288574E+02, 5.5391673099242485E+02,
    6.2966277393587268E+02,
    7.0522312235006439E+02, 7.7913293950600166E+02, 8.4987052652679745E+02,
    9.1587154082544816E+02,
    9.7554190411239881E+02, 1.0272694514208922E+03, 1.0694343794101671E+03,
    1.1004185570354589E+03,
    1.1186137648249153E+03, 1.1224289307480137E+03, 1.1102964312364500E+03,
    1.0806775255363450E+03,
    1.0320669904125798E+03, 9.6299702045050026E+02, 8.7204045694391925E+02,
    7.5781340573827129E+02,
    6.1897730151373912E+02, 4.5424047292956925E+02, 2.6235925987838891E+02,
    4.2138730876848967E+01,
    -2.0756695460503397E+02, -4.8785449725970898E+02, -7.9977168400393884E+02,
    -1.1443178535102791E+03,
    -1.5224445686419247E+03, -1.9350564660073082E+03, -2.3830122554969339E+03,
    -2.8671258451674121E+03,
    -3.3881675691937189E+03, -3.9468654988185926E+03, -4.5439068182891815E+03,
    -5.1799392496875189E+03,
    -5.8555725123375296E+03, -6.5713798041110294E+03, -7.3278992934641819E+03,
    -8.1256356124208414E+03,
    -8.9650613419849851E+03, -9.8466184826189583E+03, -1.0770719903473593E+04,
    -1.1737750765005912E+04,
    -1.2748069910479848E+04, -1.3802011222616829E+04, -1.4899884942357530E+04,
    -1.6041978947314870E+04,
    -1.7228559988051329E+04, -1.8459874880803145E+04, -1.9736151655706926E+04,
    -2.1057600659964028E+04,
    -2.2424415615712398E+04, -2.3836774632664652E+04, -2.5294841175820911E+04,
    -2.6798764988781440E+04,
    -2.8348682973365794E+04, -2.9944720026400053E+04, -3.1586989834661559E+04,
    -3.3275595629075913E+04,
    -3.5010630899345568E+04, -3.6792180070255679E+04, -3.8620319140952386E+04,
    -4.0495116288524827E+04,
    -4.2416632437245309E+04, -4.4384921794834074E+04, -4.6400032357118151E+04,
    -4.8462006382446736E+04,
    -5.0570880837214834E+04, -5.2726687813827273E+04, -5.4929454922409983E+04,
    -5.7179205657550192E+04,
    -5.9475959741312297E+04, -6.1819733443744924E+04, -6.4210539882055738E+04,
    -6.6648389299592018E+04,
    -6.9133289325728489E+04, -7.1665245217717980E+04, -7.4244260085526854E+04,
    -7.6870335100626355E+04,
    -7.9543469689680001E+04, -8.2263661714018119E+04, -8.5030907635757758E+04,
    -8.7845202671376726E+04,
    -9.0706540933522265E+04, -9.3614915561791364E+04, -9.6570318843180357E+04,
    -9.9572742322872218E+04,
    1.3159690650155653E+00, 5.1827424723368267E+00, 1.1465216826809581E+01,
    2.0007805056288021E+01,
    3.0632158318465883E+01, 4.3135914942316276E+01, 5.7292305277006705E+01,
    7.2850458353680096E+01,
    8.9536273030952316E+01, 1.0705373243496234E+02, 1.2508655577305132E+02,
    1.4330009583786776E+02,
    1.6134340363490870E+02, 1.7885139352176861E+02, 1.9544705304100802E+02,
    2.1074365128818800E+02,
    2.2434690823007099E+02, 2.3585709493666664E+02, 2.4487104128481764E+02,
    2.5098403340505823E+02,
    2.5379158805406044E+02, 2.5289109527804180E+02, 2.4788332426188720E+02,
    2.3837379020499225E+02,
    2.2397398249446709E+02, 2.0430245642130635E+02, 1.7898579226265687E+02,
    1.4765942678568368E+02,
    1.0996836316330220E+02, 6.5567765972031737E+01, 1.4123448405854163E+01,
    -4.4687740878614193E+01,
    -1.1117762374079842E+02, -1.8564648380959386E+02, -2.6838293556011723E+02,
    -3.5966386394166750E+02,
    -4.5975442752813660E+02, -5.6890811845446513E+02, -6.8736687273109692E+02,
    -8.1536122488191563E+02,
    -9.5311050122054087E+02, -1.1008230464574740E+03, -1.2586964787101258E+03,
    -1.4269179683639161E+03,
    -1.6056645366014400E+03, -1.7951033697746543E+03, -1.9953921461398070E+03,
    -2.2066793718043600E+03,
    -2.4291047230482341E+03, -2.6627993924772736E+03, -2.9078864367445922E+03,
    -3.1644811238350326E+03,
    -3.4326912781494525E+03, -3.7126176218494420E+03, -4.0043541111293484E+03,
    -4.3079882662722584E+03,
    -4.6236014945201259E+03, -4.9512694049463407E+03, -5.2910621146628937E+03,
    -5.6430445458238364E+03,
    -6.0072767130040456E+03, -6.3838140006369267E+03, -6.7727074302884048E+03,
    -7.1740039176277060E+03,
    -7.5877465190290159E+03, -8.0139746678025258E+03, -8.4527244001100134E+03,
    -8.9040285706689410E+03,
    -9.3679170583911618E+03, -9.8444169621383298E+03, -1.0333552786806300E+04,
    -1.0835346619976004E+04,
    -1.1349818299389060E+04, -1.1876985571522608E+04, -1.2416864241551024E+04,
    -1.2969468314991464E+04,
    -1.3534810131337068E+04, -1.4112900489985703E+04, -1.4703748768773870E+04,
    -1.5307363035425577E+04,
    -1.5923750152223942E+04, -1.6552915874210157E+04, -1.7194864941209838E+04,
    -1.7849601163981439E+04,
    -1.8517127504774395E+04, -1.9197446152578221E+04, -1.9890558593335409E+04,
    -2.0596465675382446E+04,
    -2.1315167670375980E+04, -2.2046664329950050E+04, -2.2790954938343813E+04,
    -2.3548038361227322E+04,
    -2.4317913090945836E+04, -2.5100577288393153E+04, -2.5896028821714342E+04,
    -2.6704265302031039E+04,
    -2.7525284116372175E+04, -2.8359082457984750E+04, -2.9205657354190695E+04,
    -3.0065005691947837E+04,
    5.6798104330594157E-01, 1.2576991751710409E+00, 2.0481759946548408E+00,
    2.9186787469718247E+00,
    3.8484815630857359E+00, 4.8166839092474589E+00, 5.8020793000798960E+00,
    6.7830675515403129E+00,
    7.7376042034952377E+00, 8.6431811810331194E+00, 9.4768332543066691E+00,
    1.0215165372038847E+01,
    1.0834396463798875E+01, 1.1310415815978915E+01, 1.1618848615501992E+01,
    1.1735127716421459E+01,
    1.1634569113094354E+01, 1.1292448996912935E+01, 1.0684080630599823E+01,
    9.7848895949085488E+00,
    8.5704862481814992E+00, 7.0167344911593315E+00, 5.0998161496844290E+00,
    2.7962904787264158E+00,
    8.3148454849560238E-02, -3.0621383367413912E+00, -6.6615742977010477E+00,
    -1.0736601874923885E+01,
    -1.5308068983078410E+01, -2.0396201629143086E+01, -2.6020581527052983E+01,
    -3.2200128452721145E+01,
    -3.8953087062103791E+01, -4.6297017876149063E+01, -5.4248792125061712E+01,
    -6.2824590139139403E+01,
    -7.2039902973421789E+01, -8.1909536957605184E+01, -9.2447620870289470E+01,
    -1.0366761544692287E+02,
    -1.1558232494317197E+02, -1.2820391048933712E+02, -1.4154390498639734E+02,
    -1.5561322930992094E+02,
    -1.7042220960409574E+02, -1.8598059546423781E+02, -2.0229757882212689E+02,
    -2.1938181336418472E+02,
    -2.3724143432774790E+02, -2.5588407853535355E+02, -2.7531690454098253E+02,
    -2.9554661277553157E+02,
    -3.1657946559136701E+02, -3.3842130711762815E+02, -3.6107758284898415E+02,
    -3.8455335890081062E+02,
    -4.0885334087324833E+02, -4.3398189227535147E+02, -4.5994305246855299E+02,
    -4.8674055409600356E+02,
    -5.1437783997099496E+02, -5.4285807940370660E+02, -5.7218418395094830E+02,
    -6.0235882257842707E+02,
    -6.3338443622942395E+02, -6.6526325179760795E+02, -6.9799729550512075E+02,
    -7.3158840569003553E+02,
    -7.6603824500989538E+02, -8.0134831207026434E+02, -8.3751995248914454E+02,
    -8.7455436940971390E+02,
    -9.1245263347520870E+02, -9.5121569228085752E+02, -9.9084437931867433E+02,
    -1.0313394224315916E+03,
    -1.0727014517939447E+03, -1.1149310074356558E+03, -1.1580285463277039E+03,
    -1.2019944490465350E+03,
    -1.2468290260350800E+03, -1.2925325234779291E+03, -1.3391051288080228E+03,
    -1.3865469758619590E+03,
    -1.4348581497007231E+03, -1.4840386911122494E+03, -1.5340886008118391E+03,
    -1.5850078433560020E+03,
    -1.6367963507848394E+03, -1.6894540260075898E+03, -1.7429807459454357E+03,
    -1.7973763644451933E+03,
    -1.8526407149769502E+03, -1.9087736131282052E+03, -1.9657748589065693E+03,
    -2.0236442388624985E+03,
    -2.0823815280430854E+03, -2.1419864917873979E+03, -2.2024588873733496E+03,
    -2.2637984655256205E+03 };

  creal_T c2_e_a;
  creal_T c2_b;
  creal_T c2_sc_y;
  creal_T c2_f_a;
  creal_T c2_b_b;
  creal_T c2_tc_y;
  creal_T c2_g_a;
  creal_T c2_c_b;
  creal_T c2_uc_y;
  creal_T c2_dc0;
  creal_T c2_dc1;
  creal_T c2_dc2;
  creal_T c2_dcv0[6];
  int32_T c2_i81;
  creal_T c2_d_b[6];
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_hoistedExpr;
  creal_T c2_dc3;
  creal_T c2_dc4;
  real_T c2_b_ii;
  int32_T c2_i84;
  static char_T c2_cv18[27] = { 'T', 'a', 'n', 'k', ' ', '1', ' ', '=', '>', ' ',
    'n', 'e', 'x', 't', '_', 't', 'i', 'm', 'e', ' ',
    '=', ' ', '%', 'u', ' ', '\\', 'n' };

  char_T c2_pc_u[27];
  const mxArray *c2_vc_y = NULL;
  real_T c2_qc_u;
  const mxArray *c2_wc_y = NULL;
  int32_T c2_i85;
  static char_T c2_cv19[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '1',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_rc_u[71];
  const mxArray *c2_xc_y = NULL;
  real_T c2_sc_u;
  const mxArray *c2_yc_y = NULL;
  real_T c2_tc_u;
  const mxArray *c2_ad_y = NULL;
  int32_T c2_i86;
  static char_T c2_cv20[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '2',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_uc_u[71];
  const mxArray *c2_bd_y = NULL;
  real_T c2_vc_u;
  const mxArray *c2_cd_y = NULL;
  real_T c2_wc_u;
  const mxArray *c2_dd_y = NULL;
  int32_T c2_i87;
  boolean_T c2_n_lateIN[4];
  real_T c2_b_y2L1_new[3];
  int32_T c2_i88;
  int32_T c2_i89;
  creal_T c2_h_a;
  creal_T c2_e_b;
  creal_T c2_ed_y;
  creal_T c2_i_a;
  creal_T c2_f_b;
  creal_T c2_fd_y;
  creal_T c2_j_a;
  creal_T c2_g_b;
  creal_T c2_gd_y;
  creal_T c2_dc5;
  creal_T c2_dc6;
  creal_T c2_dc7;
  creal_T c2_dcv1[6];
  int32_T c2_i90;
  creal_T c2_h_b[6];
  int32_T c2_i91;
  int32_T c2_i92;
  int32_T c2_b_hoistedExpr;
  creal_T c2_dc8;
  creal_T c2_dc9;
  real_T c2_c_ii;
  int32_T c2_i93;
  static char_T c2_cv21[27] = { 'T', 'a', 'n', 'k', ' ', '2', ' ', '=', '>', ' ',
    'n', 'e', 'x', 't', '_', 't', 'i', 'm', 'e', ' ',
    '=', ' ', '%', 'u', ' ', '\\', 'n' };

  char_T c2_xc_u[27];
  const mxArray *c2_hd_y = NULL;
  real_T c2_yc_u;
  const mxArray *c2_id_y = NULL;
  int32_T c2_i94;
  static char_T c2_cv22[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '3',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_ad_u[71];
  const mxArray *c2_jd_y = NULL;
  real_T c2_bd_u;
  const mxArray *c2_kd_y = NULL;
  real_T c2_cd_u;
  const mxArray *c2_ld_y = NULL;
  int32_T c2_i95;
  static char_T c2_cv23[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '4',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_dd_u[71];
  const mxArray *c2_md_y = NULL;
  real_T c2_ed_u;
  const mxArray *c2_nd_y = NULL;
  real_T c2_fd_u;
  const mxArray *c2_od_y = NULL;
  int32_T c2_i96;
  boolean_T c2_o_lateIN[4];
  real_T c2_i_b;
  real_T c2_pd_y;
  int32_T c2_i97;
  static char_T c2_cv24[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '5',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_gd_u[71];
  const mxArray *c2_qd_y = NULL;
  real_T c2_hd_u;
  const mxArray *c2_rd_y = NULL;
  real_T c2_id_u;
  const mxArray *c2_sd_y = NULL;
  int32_T c2_i98;
  boolean_T c2_p_lateIN[4];
  real_T c2_j_b;
  real_T c2_td_y;
  int32_T c2_i99;
  static char_T c2_cv25[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '6',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_jd_u[71];
  const mxArray *c2_ud_y = NULL;
  real_T c2_kd_u;
  const mxArray *c2_vd_y = NULL;
  real_T c2_ld_u;
  const mxArray *c2_wd_y = NULL;
  int32_T c2_i100;
  boolean_T c2_q_lateIN[4];
  real_T c2_k_b;
  real_T c2_xd_y;
  int32_T c2_i101;
  static char_T c2_cv26[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '7',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_md_u[71];
  const mxArray *c2_yd_y = NULL;
  real_T c2_nd_u;
  const mxArray *c2_ae_y = NULL;
  real_T c2_od_u;
  const mxArray *c2_be_y = NULL;
  int32_T c2_i102;
  boolean_T c2_r_lateIN[4];
  real_T c2_l_b;
  real_T c2_ce_y;
  int32_T c2_i103;
  static char_T c2_cv27[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '8',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_pd_u[71];
  const mxArray *c2_de_y = NULL;
  real_T c2_qd_u;
  const mxArray *c2_ee_y = NULL;
  real_T c2_rd_u;
  const mxArray *c2_fe_y = NULL;
  int32_T c2_i104;
  boolean_T c2_s_lateIN[4];
  real_T c2_m_b;
  real_T c2_ge_y;
  int32_T c2_i105;
  static char_T c2_cv28[71] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '9',
    ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ', 't',
    'i', 'm', 'e', ':', ' ',
    '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e', 'd',
    'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_sd_u[71];
  const mxArray *c2_he_y = NULL;
  real_T c2_td_u;
  const mxArray *c2_ie_y = NULL;
  real_T c2_ud_u;
  const mxArray *c2_je_y = NULL;
  int32_T c2_i106;
  boolean_T c2_t_lateIN[4];
  real_T c2_n_b;
  real_T c2_ke_y;
  int32_T c2_i107;
  static char_T c2_cv29[72] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '1',
    '0', ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e',
    'd', 'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ',
    't', 'i', 'm', 'e', ':',
    ' ', '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e',
    'd', 'u', 'l', 'e', 'd',
    ' ', 't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_vd_u[72];
  const mxArray *c2_le_y = NULL;
  real_T c2_wd_u;
  const mxArray *c2_me_y = NULL;
  real_T c2_xd_u;
  const mxArray *c2_ne_y = NULL;
  int32_T c2_i108;
  boolean_T c2_u_lateIN[4];
  real_T c2_o_b;
  real_T c2_oe_y;
  int32_T c2_i109;
  static char_T c2_cv30[72] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '1',
    '1', ' ', 'w', 'a', 's', ' ', 's', 'c', 'h', 'e',
    'd', 'u', 'l', 'e', 'd', '!', ' ', '=', '>', ' ', 'n', 'e', 'x', 't', ' ',
    't', 'i', 'm', 'e', ':',
    ' ', '%', 'f', ' ', '=', '>', ' ', 'n', 'e', 'w', ' ', 's', 'c', 'h', 'e',
    'd', 'u', 'l', 'e', 'd',
    ' ', 't', 'i', 'm', 'e', ':', ' ', '%', 'f', ' ', '\\', 'n' };

  char_T c2_yd_u[72];
  const mxArray *c2_pe_y = NULL;
  real_T c2_ae_u;
  const mxArray *c2_qe_y = NULL;
  real_T c2_be_u;
  const mxArray *c2_re_y = NULL;
  int32_T c2_i110;
  static char_T c2_cv31[73] = { '\\', 'n', 'N', 'o', 'w', ' ', 'c', 'h', 'e',
    'c', 'k', ' ', 'i', 'f', ' ', 'w', 'e', ' ', 'n', 'e',
    'e', 'd', ' ', 't', 'o', ' ', 'a', 'l', 'l', 'o', 'c', 'a', 't', 'e', ' ',
    's', 'l', 'o', 't', 's',
    ' ', 'i', 'n', ' ', 't', 'h', 'e', ' ', 'n', 'e', 'x', 't', ' ', 'b', 'e',
    'a', 'c', 'o', 'n', ' ',
    'i', 'n', 't', 'e', 'r', 'v', 'a', 'l', '.', '.', '.', '\\', 'n' };

  char_T c2_ce_u[73];
  const mxArray *c2_se_y = NULL;
  int32_T c2_i111;
  static char_T c2_cv32[57] = { 'N', 'e', 'x', 't', ' ', 'b', 'e', 'a', 'c', 'o',
    'n', '+', 'b', 'e', 'a', 'c', 'o', 'n', ' ', 'i',
    'n', 't', 'e', 'r', 'v', 'a', 'l', '+', 'd', 'e', 'l', 't', 'a', 'C', 'A',
    'P', ':', ' ', '%', 'f',
    ' ', '+', ' ', '%', 'f', ' ', '+', ' ', '%', 'f', ' ', '=', ' ', '%', 'f',
    '\\', 'n' };

  char_T c2_de_u[57];
  const mxArray *c2_te_y = NULL;
  real_T c2_ee_u;
  const mxArray *c2_ue_y = NULL;
  real_T c2_fe_u;
  const mxArray *c2_ve_y = NULL;
  real_T c2_ge_u;
  const mxArray *c2_we_y = NULL;
  real_T c2_he_u;
  const mxArray *c2_xe_y = NULL;
  int32_T c2_i112;
  static char_T c2_cv33[41] = { 'A', 'n', 'y', 't', 'h', 'i', 'n', 'g', ' ', 'b',
    'e', 'f', 'o', 'r', 'e', ' ', 't', 'h', 'a', 't',
    ' ', 's', 'h', 'o', 'u', 'l', 'd', ' ', 'g', 'e', 't', ' ', 'a', ' ', 's',
    'l', 'o', 't', '!', '\\',
    'n' };

  char_T c2_ie_u[41];
  const mxArray *c2_ye_y = NULL;
  int32_T c2_i113;
  static char_T c2_cv34[34] = { 'S', 'c', 'h', 'e', 'd', 'u', 'l', 'e', 'd', ' ',
    't', 'i', 'm', 'e', ' ', 'f', 'o', 'r', ' ', 't',
    'r', 'a', 'n', 's', 'm', 'i', 's', 's', 'i', 'o', 'n', ':', '\\', 'n' };

  char_T c2_je_u[34];
  const mxArray *c2_af_y = NULL;
  int32_T c2_i114;
  char_T c2_ke_u[10];
  const mxArray *c2_bf_y = NULL;
  real_T c2_le_u;
  const mxArray *c2_cf_y = NULL;
  int32_T c2_i115;
  char_T c2_me_u[4];
  const mxArray *c2_df_y = NULL;
  int32_T c2_i116;
  char_T c2_ne_u[2];
  const mxArray *c2_ef_y = NULL;
  int32_T c2_i117;
  char_T c2_oe_u[10];
  const mxArray *c2_ff_y = NULL;
  real_T c2_pe_u;
  const mxArray *c2_gf_y = NULL;
  int32_T c2_i118;
  char_T c2_qe_u[4];
  const mxArray *c2_hf_y = NULL;
  int32_T c2_i119;
  char_T c2_re_u[2];
  const mxArray *c2_if_y = NULL;
  int32_T c2_i120;
  char_T c2_se_u[10];
  const mxArray *c2_jf_y = NULL;
  real_T c2_te_u;
  const mxArray *c2_kf_y = NULL;
  int32_T c2_i121;
  char_T c2_ue_u[4];
  const mxArray *c2_lf_y = NULL;
  int32_T c2_i122;
  char_T c2_ve_u[2];
  const mxArray *c2_mf_y = NULL;
  int32_T c2_i123;
  char_T c2_we_u[10];
  const mxArray *c2_nf_y = NULL;
  real_T c2_xe_u;
  const mxArray *c2_of_y = NULL;
  int32_T c2_i124;
  char_T c2_ye_u[4];
  const mxArray *c2_pf_y = NULL;
  int32_T c2_i125;
  char_T c2_af_u[2];
  const mxArray *c2_qf_y = NULL;
  int32_T c2_i126;
  char_T c2_bf_u[10];
  const mxArray *c2_rf_y = NULL;
  real_T c2_cf_u;
  const mxArray *c2_sf_y = NULL;
  int32_T c2_i127;
  char_T c2_df_u[4];
  const mxArray *c2_tf_y = NULL;
  int32_T c2_i128;
  char_T c2_ef_u[2];
  const mxArray *c2_uf_y = NULL;
  int32_T c2_i129;
  char_T c2_ff_u[10];
  const mxArray *c2_vf_y = NULL;
  real_T c2_gf_u;
  const mxArray *c2_wf_y = NULL;
  int32_T c2_i130;
  char_T c2_hf_u[4];
  const mxArray *c2_xf_y = NULL;
  int32_T c2_i131;
  char_T c2_if_u[2];
  const mxArray *c2_yf_y = NULL;
  int32_T c2_i132;
  char_T c2_jf_u[10];
  const mxArray *c2_ag_y = NULL;
  real_T c2_kf_u;
  const mxArray *c2_bg_y = NULL;
  int32_T c2_i133;
  char_T c2_lf_u[4];
  const mxArray *c2_cg_y = NULL;
  int32_T c2_i134;
  char_T c2_mf_u[2];
  const mxArray *c2_dg_y = NULL;
  int32_T c2_i135;
  char_T c2_nf_u[10];
  const mxArray *c2_eg_y = NULL;
  real_T c2_of_u;
  const mxArray *c2_fg_y = NULL;
  int32_T c2_i136;
  char_T c2_pf_u[4];
  const mxArray *c2_gg_y = NULL;
  int32_T c2_i137;
  char_T c2_qf_u[2];
  const mxArray *c2_hg_y = NULL;
  int32_T c2_i138;
  char_T c2_rf_u[10];
  const mxArray *c2_ig_y = NULL;
  real_T c2_sf_u;
  const mxArray *c2_jg_y = NULL;
  int32_T c2_i139;
  char_T c2_tf_u[4];
  const mxArray *c2_kg_y = NULL;
  int32_T c2_i140;
  char_T c2_uf_u[2];
  const mxArray *c2_lg_y = NULL;
  int32_T c2_i141;
  char_T c2_vf_u[11];
  const mxArray *c2_mg_y = NULL;
  real_T c2_wf_u;
  const mxArray *c2_ng_y = NULL;
  int32_T c2_i142;
  char_T c2_xf_u[4];
  const mxArray *c2_og_y = NULL;
  int32_T c2_i143;
  char_T c2_yf_u[2];
  const mxArray *c2_pg_y = NULL;
  int32_T c2_i144;
  char_T c2_ag_u[11];
  const mxArray *c2_qg_y = NULL;
  real_T c2_bg_u;
  const mxArray *c2_rg_y = NULL;
  int32_T c2_i145;
  char_T c2_cg_u[4];
  const mxArray *c2_sg_y = NULL;
  int32_T c2_i146;
  char_T c2_dg_u[2];
  const mxArray *c2_tg_y = NULL;
  real_T c2_c_i;
  int32_T c2_i147;
  static char_T c2_cv35[60] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '%',
    'u', ' ', 's', 'h', 'o', 'u', 'l', 'd', ' ', 'g',
    'e', 't', ' ', 'a', ' ', 's', 'l', 'o', 't', ' ', 'i', 'n', ' ', 't', 'h',
    'e', ' ', 'n', 'e', 'x',
    't', ' ', 'b', 'e', 'a', 'c', 'o', 'n', ' ', 'i', 'n', 't', 'e', 'r', 'v',
    'a', 'l', '!', '\\', 'n' };

  char_T c2_eg_u[60];
  const mxArray *c2_ug_y = NULL;
  real_T c2_fg_u;
  const mxArray *c2_vg_y = NULL;
  int32_T c2_i148;
  int32_T c2_i149;
  real_T c2_b_scheduled_next[11];
  real_T c2_b_pos[11];
  real_T c2_b_B[11];
  int32_T c2_i150;
  int32_T c2_i151;
  int32_T c2_i152;
  static char_T c2_cv36[24] = { '\\', 'n', 'H', 'a', 'n', 'd', 'i', 'n', 'g',
    ' ', 'o', 'u', 't', ' ', 's', 'l', 'o', 't', 's', '.',
    '.', '.', '\\', 'n' };

  char_T c2_gg_u[24];
  const mxArray *c2_wg_y = NULL;
  real_T c2_loop_ub;
  real_T c2_d_i;
  real_T c2_k_a;
  real_T c2_xg_y;
  real_T c2_l_a;
  real_T c2_yg_y;
  real_T c2_m_a;
  real_T c2_ah_y;
  real_T c2_n_a;
  real_T c2_bh_y;
  int32_T c2_i153;
  static char_T c2_cv37[26] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '%',
    'u', ' ', 'g', 'o', 't', ' ', 's', 'l', 'o', 't',
    ' ', '%', 'u', '!', '\\', 'n' };

  char_T c2_hg_u[26];
  const mxArray *c2_ch_y = NULL;
  real_T c2_ig_u;
  const mxArray *c2_dh_y = NULL;
  real_T c2_jg_u;
  const mxArray *c2_eh_y = NULL;
  int32_T c2_i154;
  char_T c2_kg_u[26];
  const mxArray *c2_fh_y = NULL;
  real_T c2_lg_u;
  const mxArray *c2_gh_y = NULL;
  real_T c2_mg_u;
  const mxArray *c2_hh_y = NULL;
  real_T c2_o_a;
  real_T c2_ih_y;
  real_T c2_p_a;
  real_T c2_jh_y;
  real_T c2_q_a;
  real_T c2_kh_y;
  real_T c2_r_a;
  real_T c2_lh_y;
  int32_T c2_i155;
  char_T c2_ng_u[26];
  const mxArray *c2_mh_y = NULL;
  real_T c2_og_u;
  const mxArray *c2_nh_y = NULL;
  real_T c2_pg_u;
  const mxArray *c2_oh_y = NULL;
  int32_T c2_i156;
  char_T c2_qg_u[26];
  const mxArray *c2_ph_y = NULL;
  real_T c2_rg_u;
  const mxArray *c2_qh_y = NULL;
  real_T c2_sg_u;
  const mxArray *c2_rh_y = NULL;
  real_T c2_b_loop_ub;
  real_T c2_e_i;
  int32_T c2_i157;
  char_T c2_tg_u[26];
  const mxArray *c2_sh_y = NULL;
  real_T c2_ug_u;
  const mxArray *c2_th_y = NULL;
  real_T c2_vg_u;
  const mxArray *c2_uh_y = NULL;
  real_T c2_s_a;
  real_T c2_vh_y;
  real_T c2_t_a;
  real_T c2_wh_y;
  int32_T c2_i158;
  static char_T c2_cv38[54] = { ' ', '=', '>', ' ', 'N', 'o', 'd', 'e', ' ', '%',
    'u', ' ', 'd', 'i', 'd', ' ', 'n', 'o', 't', ' ',
    'g', 'e', 't', ' ', 'a', ' ', 's', 'l', 'o', 't', ' ', 'a', 'n', 'd', ' ',
    'i', 's', ' ', 'g', 'o',
    'n', 'n', 'a', ' ', 'b', 'e', ' ', 'l', 'a', 't', 'e', '!', '\\', 'n' };

  char_T c2_wg_u[54];
  const mxArray *c2_xh_y = NULL;
  real_T c2_xg_u;
  const mxArray *c2_yh_y = NULL;
  real_T c2_f_i;
  uint32_T c2_uv1[625];
  int32_T c2_i159;
  int32_T c2_i160;
  real_T *c2_b_Y11;
  real_T *c2_b_Y12;
  real_T *c2_b_Y21;
  real_T *c2_b_Y22;
  real_T *c2_b_S1;
  real_T *c2_b_S2;
  real_T *c2_b_S3;
  real_T *c2_b_S4;
  real_T *c2_b_S5;
  real_T *c2_b_TIME;
  real_T *c2_b_I1;
  real_T *c2_b_I2;
  real_T *c2_b_current_BO;
  real_T *c2_b_time_schedule1_1;
  real_T *c2_b_time_schedule1_2;
  real_T *c2_b_time_schedule1_3;
  real_T *c2_b_time_schedule1_4;
  real_T *c2_b_time_schedule1_5;
  real_T *c2_b_time_schedule1_6;
  real_T *c2_b_time_schedule1_7;
  real_T *c2_b_time_schedule1_8;
  real_T *c2_b_time_schedule1_9;
  real_T *c2_b_time_schedule1_10;
  real_T *c2_b_time_schedule1_11;
  real_T *c2_b_INIT_in;
  real_T *c2_b_lateIN_1;
  real_T *c2_b_lateIN_2;
  real_T *c2_b_lateIN_3;
  real_T *c2_b_lateIN_4;
  real_T *c2_b_randStateIN;
  real_T *c2_b_BO;
  real_T *c2_b_TM1;
  real_T *c2_b_TA1;
  real_T *c2_b_TM2;
  real_T *c2_b_TA2;
  real_T *c2_b_TS1;
  real_T *c2_b_TS2;
  real_T *c2_b_TS3;
  real_T *c2_b_TS4;
  real_T *c2_b_TS5;
  real_T *c2_b_TS6;
  real_T *c2_b_TS7;
  real_T *c2_b_time_schedule_1;
  real_T *c2_b_time_schedule_2;
  real_T *c2_b_time_schedule_3;
  real_T *c2_b_time_schedule_4;
  real_T *c2_b_time_schedule_5;
  real_T *c2_b_time_schedule_6;
  real_T *c2_b_time_schedule_7;
  real_T *c2_b_time_schedule_8;
  real_T *c2_b_time_schedule_9;
  real_T *c2_b_time_schedule_10;
  real_T *c2_b_time_schedule_11;
  real_T *c2_b_INIT;
  real_T *c2_b_lateOUT_1;
  real_T *c2_b_lateOUT_2;
  real_T *c2_b_lateOUT_3;
  real_T *c2_b_lateOUT_4;
  real_T *c2_b_randStateOUT;
  real_T (*c2_b_TAU_K)[9];
  c2_b_randStateOUT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 30);
  c2_b_randStateIN = (real_T *)ssGetInputPortSignal(chartInstance->S, 29);
  c2_b_lateOUT_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 29);
  c2_b_lateOUT_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 28);
  c2_b_lateIN_4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 28);
  c2_b_lateIN_3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 27);
  c2_b_lateOUT_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 27);
  c2_b_lateOUT_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 26);
  c2_b_lateIN_2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 26);
  c2_b_lateIN_1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 25);
  c2_b_INIT_in = (real_T *)ssGetInputPortSignal(chartInstance->S, 24);
  c2_b_INIT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 25);
  c2_b_TAU_K = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 24);
  c2_b_time_schedule_11 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 23);
  c2_b_time_schedule_10 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 22);
  c2_b_time_schedule_9 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 21);
  c2_b_time_schedule_8 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 20);
  c2_b_time_schedule_7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 19);
  c2_b_time_schedule_6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 18);
  c2_b_time_schedule_5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 17);
  c2_b_time_schedule_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 16);
  c2_b_time_schedule_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 15);
  c2_b_time_schedule_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 14);
  c2_b_time_schedule1_11 = (real_T *)ssGetInputPortSignal(chartInstance->S, 23);
  c2_b_time_schedule1_10 = (real_T *)ssGetInputPortSignal(chartInstance->S, 22);
  c2_b_time_schedule1_9 = (real_T *)ssGetInputPortSignal(chartInstance->S, 21);
  c2_b_time_schedule1_8 = (real_T *)ssGetInputPortSignal(chartInstance->S, 20);
  c2_b_time_schedule1_7 = (real_T *)ssGetInputPortSignal(chartInstance->S, 19);
  c2_b_time_schedule1_6 = (real_T *)ssGetInputPortSignal(chartInstance->S, 18);
  c2_b_time_schedule1_5 = (real_T *)ssGetInputPortSignal(chartInstance->S, 17);
  c2_b_time_schedule1_4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 16);
  c2_b_time_schedule1_3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 15);
  c2_b_time_schedule1_2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 14);
  c2_b_time_schedule1_1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 13);
  c2_b_time_schedule_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 13);
  c2_b_current_BO = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
  c2_b_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
  c2_b_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
  c2_b_TIME = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
  c2_b_TS7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 12);
  c2_b_TS6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 11);
  c2_b_TS5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 10);
  c2_b_TS4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
  c2_b_TS3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_b_TS2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_b_TS1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_b_TA2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_b_TM2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_b_TA1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_b_TM1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_S5 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
  c2_b_S4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
  c2_b_S3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c2_b_S2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c2_b_S1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_b_Y22 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_Y21 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_Y12 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_BO = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_Y11 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  c2_hoistedGlobal = *c2_b_Y11;
  c2_b_hoistedGlobal = *c2_b_Y12;
  c2_c_hoistedGlobal = *c2_b_Y21;
  c2_d_hoistedGlobal = *c2_b_Y22;
  c2_e_hoistedGlobal = *c2_b_S1;
  c2_f_hoistedGlobal = *c2_b_S2;
  c2_g_hoistedGlobal = *c2_b_S3;
  c2_h_hoistedGlobal = *c2_b_S4;
  c2_i_hoistedGlobal = *c2_b_S5;
  c2_j_hoistedGlobal = *c2_b_TIME;
  c2_k_hoistedGlobal = *c2_b_I1;
  c2_l_hoistedGlobal = *c2_b_I2;
  c2_m_hoistedGlobal = *c2_b_current_BO;
  c2_n_hoistedGlobal = *c2_b_time_schedule1_1;
  c2_o_hoistedGlobal = *c2_b_time_schedule1_2;
  c2_p_hoistedGlobal = *c2_b_time_schedule1_3;
  c2_q_hoistedGlobal = *c2_b_time_schedule1_4;
  c2_r_hoistedGlobal = *c2_b_time_schedule1_5;
  c2_s_hoistedGlobal = *c2_b_time_schedule1_6;
  c2_t_hoistedGlobal = *c2_b_time_schedule1_7;
  c2_u_hoistedGlobal = *c2_b_time_schedule1_8;
  c2_v_hoistedGlobal = *c2_b_time_schedule1_9;
  c2_w_hoistedGlobal = *c2_b_time_schedule1_10;
  c2_x_hoistedGlobal = *c2_b_time_schedule1_11;
  c2_y_hoistedGlobal = *c2_b_INIT_in;
  c2_ab_hoistedGlobal = *c2_b_lateIN_1;
  c2_bb_hoistedGlobal = *c2_b_lateIN_2;
  c2_cb_hoistedGlobal = *c2_b_lateIN_3;
  c2_db_hoistedGlobal = *c2_b_lateIN_4;
  c2_eb_hoistedGlobal = *c2_b_randStateIN;
  c2_Y11 = c2_hoistedGlobal;
  c2_Y12 = c2_b_hoistedGlobal;
  c2_Y21 = c2_c_hoistedGlobal;
  c2_Y22 = c2_d_hoistedGlobal;
  c2_S1 = c2_e_hoistedGlobal;
  c2_S2 = c2_f_hoistedGlobal;
  c2_S3 = c2_g_hoistedGlobal;
  c2_S4 = c2_h_hoistedGlobal;
  c2_S5 = c2_i_hoistedGlobal;
  c2_TIME = c2_j_hoistedGlobal;
  c2_I1 = c2_k_hoistedGlobal;
  c2_I2 = c2_l_hoistedGlobal;
  c2_current_BO = c2_m_hoistedGlobal;
  c2_time_schedule1_1 = c2_n_hoistedGlobal;
  c2_time_schedule1_2 = c2_o_hoistedGlobal;
  c2_time_schedule1_3 = c2_p_hoistedGlobal;
  c2_time_schedule1_4 = c2_q_hoistedGlobal;
  c2_time_schedule1_5 = c2_r_hoistedGlobal;
  c2_time_schedule1_6 = c2_s_hoistedGlobal;
  c2_time_schedule1_7 = c2_t_hoistedGlobal;
  c2_time_schedule1_8 = c2_u_hoistedGlobal;
  c2_time_schedule1_9 = c2_v_hoistedGlobal;
  c2_time_schedule1_10 = c2_w_hoistedGlobal;
  c2_time_schedule1_11 = c2_x_hoistedGlobal;
  c2_INIT_in = c2_y_hoistedGlobal;
  c2_lateIN_1 = c2_ab_hoistedGlobal;
  c2_lateIN_2 = c2_bb_hoistedGlobal;
  c2_lateIN_3 = c2_cb_hoistedGlobal;
  c2_lateIN_4 = c2_db_hoistedGlobal;
  c2_randStateIN = c2_eb_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 97U, 97U, c2_sv0, c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c2_BI, c2_k_sf_marshall, 0U);
  sf_debug_symbol_scope_add_eml(&c2_randState, c2_sf_marshall, 1U);
  sf_debug_symbol_scope_add_eml(&c2_tBI, c2_sf_marshall, 2U);
  sf_debug_symbol_scope_add_eml(&c2_symbol_time, c2_sf_marshall, 3U);
  sf_debug_symbol_scope_add_eml(&c2_deltaCAP, c2_sf_marshall, 4U);
  sf_debug_symbol_scope_add_eml(&c2_ttStart, c2_sf_marshall, 5U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1, c2_d_sf_marshall, 6U);
  sf_debug_symbol_scope_add_eml(&c2_lateIN, c2_j_sf_marshall, 7U);
  sf_debug_symbol_scope_add_eml(&c2_lateOUT, c2_j_sf_marshall, 8U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule, c2_d_sf_marshall, 9U);
  sf_debug_symbol_scope_add_eml(&c2_i, c2_sf_marshall, 10U);
  sf_debug_symbol_scope_add_eml(&c2_scheduled_next, c2_i_sf_marshall, 11U);
  sf_debug_symbol_scope_add_eml(&c2_slot, c2_d_sf_marshall, 12U);
  sf_debug_symbol_scope_add_eml(&c2_y1L1_new, c2_sf_marshall, 13U);
  sf_debug_symbol_scope_add_eml(&c2_y1L2_new, c2_sf_marshall, 14U);
  sf_debug_symbol_scope_add_eml(&c2_y1INT_new, c2_sf_marshall, 15U);
  sf_debug_symbol_scope_add_eml(&c2_y2L1_new, c2_sf_marshall, 16U);
  sf_debug_symbol_scope_add_eml(&c2_y2L2_new, c2_sf_marshall, 17U);
  sf_debug_symbol_scope_add_eml(&c2_y2INT_new, c2_sf_marshall, 18U);
  sf_debug_symbol_scope_add_eml(&c2_ttStart_next, c2_sf_marshall, 19U);
  sf_debug_symbol_scope_add_eml(&c2_data, c2_h_sf_marshall, 20U);
  sf_debug_symbol_scope_add_eml(&c2_x10, c2_g_sf_marshall, 21U);
  sf_debug_symbol_scope_add_eml(&c2_x20, c2_g_sf_marshall, 22U);
  sf_debug_symbol_scope_add_eml(&c2_x30, c2_g_sf_marshall, 23U);
  sf_debug_symbol_scope_add_eml(&c2_lvm, c2_f_sf_marshall, 24U);
  sf_debug_symbol_scope_add_eml(&c2_temp17, c2_e_sf_marshall, 25U);
  sf_debug_symbol_scope_add_eml(&c2_next_time, c2_sf_marshall, 26U);
  sf_debug_symbol_scope_add_eml(&c2_ii, c2_sf_marshall, 27U);
  sf_debug_symbol_scope_add_eml(&c2_j, c2_sf_marshall, 28U);
  sf_debug_symbol_scope_add_eml(&c2_B, c2_d_sf_marshall, 29U);
  sf_debug_symbol_scope_add_eml(&c2_pos, c2_d_sf_marshall, 30U);
  sf_debug_symbol_scope_add_eml(&c2_slot_num, c2_sf_marshall, 31U);
  sf_debug_symbol_scope_add_eml(&c2_l, c2_sf_marshall, 32U);
  sf_debug_symbol_scope_add_eml(&c2_node, c2_sf_marshall, 33U);
  sf_debug_symbol_scope_add_eml(&c2_randState2, c2_c_sf_marshall, 34U);
  sf_debug_symbol_scope_add_eml(&c2_nargin, c2_sf_marshall, 35U);
  sf_debug_symbol_scope_add_eml(&c2_nargout, c2_sf_marshall, 36U);
  sf_debug_symbol_scope_add_eml(&c2_Y11, c2_sf_marshall, 37U);
  sf_debug_symbol_scope_add_eml(&c2_Y12, c2_sf_marshall, 38U);
  sf_debug_symbol_scope_add_eml(&c2_Y21, c2_sf_marshall, 39U);
  sf_debug_symbol_scope_add_eml(&c2_Y22, c2_sf_marshall, 40U);
  sf_debug_symbol_scope_add_eml(&c2_S1, c2_sf_marshall, 41U);
  sf_debug_symbol_scope_add_eml(&c2_S2, c2_sf_marshall, 42U);
  sf_debug_symbol_scope_add_eml(&c2_S3, c2_sf_marshall, 43U);
  sf_debug_symbol_scope_add_eml(&c2_S4, c2_sf_marshall, 44U);
  sf_debug_symbol_scope_add_eml(&c2_S5, c2_sf_marshall, 45U);
  sf_debug_symbol_scope_add_eml(&c2_TIME, c2_sf_marshall, 46U);
  sf_debug_symbol_scope_add_eml(&c2_I1, c2_sf_marshall, 47U);
  sf_debug_symbol_scope_add_eml(&c2_I2, c2_sf_marshall, 48U);
  sf_debug_symbol_scope_add_eml(&c2_current_BO, c2_sf_marshall, 49U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_1, c2_sf_marshall, 50U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_2, c2_sf_marshall, 51U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_3, c2_sf_marshall, 52U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_4, c2_sf_marshall, 53U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_5, c2_sf_marshall, 54U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_6, c2_sf_marshall, 55U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_7, c2_sf_marshall, 56U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_8, c2_sf_marshall, 57U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_9, c2_sf_marshall, 58U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_10, c2_sf_marshall, 59U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule1_11, c2_sf_marshall, 60U);
  sf_debug_symbol_scope_add_eml(&c2_INIT_in, c2_sf_marshall, 61U);
  sf_debug_symbol_scope_add_eml(&c2_lateIN_1, c2_sf_marshall, 62U);
  sf_debug_symbol_scope_add_eml(&c2_lateIN_2, c2_sf_marshall, 63U);
  sf_debug_symbol_scope_add_eml(&c2_lateIN_3, c2_sf_marshall, 64U);
  sf_debug_symbol_scope_add_eml(&c2_lateIN_4, c2_sf_marshall, 65U);
  sf_debug_symbol_scope_add_eml(&c2_randStateIN, c2_sf_marshall, 66U);
  sf_debug_symbol_scope_add_eml(&c2_BO, c2_sf_marshall, 67U);
  sf_debug_symbol_scope_add_eml(&c2_TM1, c2_sf_marshall, 68U);
  sf_debug_symbol_scope_add_eml(&c2_TA1, c2_sf_marshall, 69U);
  sf_debug_symbol_scope_add_eml(&c2_TM2, c2_sf_marshall, 70U);
  sf_debug_symbol_scope_add_eml(&c2_TA2, c2_sf_marshall, 71U);
  sf_debug_symbol_scope_add_eml(&c2_TS1, c2_sf_marshall, 72U);
  sf_debug_symbol_scope_add_eml(&c2_TS2, c2_sf_marshall, 73U);
  sf_debug_symbol_scope_add_eml(&c2_TS3, c2_sf_marshall, 74U);
  sf_debug_symbol_scope_add_eml(&c2_TS4, c2_sf_marshall, 75U);
  sf_debug_symbol_scope_add_eml(&c2_TS5, c2_sf_marshall, 76U);
  sf_debug_symbol_scope_add_eml(&c2_TS6, c2_sf_marshall, 77U);
  sf_debug_symbol_scope_add_eml(&c2_TS7, c2_sf_marshall, 78U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_1, c2_sf_marshall, 79U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_2, c2_sf_marshall, 80U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_3, c2_sf_marshall, 81U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_4, c2_sf_marshall, 82U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_5, c2_sf_marshall, 83U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_6, c2_sf_marshall, 84U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_7, c2_sf_marshall, 85U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_8, c2_sf_marshall, 86U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_9, c2_sf_marshall, 87U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_10, c2_sf_marshall, 88U);
  sf_debug_symbol_scope_add_eml(&c2_time_schedule_11, c2_sf_marshall, 89U);
  sf_debug_symbol_scope_add_eml(&c2_TAU_K, c2_b_sf_marshall, 90U);
  sf_debug_symbol_scope_add_eml(&c2_INIT, c2_sf_marshall, 91U);
  sf_debug_symbol_scope_add_eml(&c2_lateOUT_1, c2_sf_marshall, 92U);
  sf_debug_symbol_scope_add_eml(&c2_lateOUT_2, c2_sf_marshall, 93U);
  sf_debug_symbol_scope_add_eml(&c2_lateOUT_3, c2_sf_marshall, 94U);
  sf_debug_symbol_scope_add_eml(&c2_lateOUT_4, c2_sf_marshall, 95U);
  sf_debug_symbol_scope_add_eml(&c2_randStateOUT, c2_sf_marshall, 96U);
  CV_EML_FCN(0, 0);

  /*  This block supports the Embedded MATLAB subset. */
  /*  See the help menu for details. */
  _SFD_EML_CALL(0,5);

  /*  for debugging */
  /* eml.extrinsic('rand'); */
  _SFD_EML_CALL(0,8);
  for (c2_i7 = 0; c2_i7 < 60; c2_i7 = c2_i7 + 1) {
    c2_u[c2_i7] = c2_cv0[c2_i7];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 10, 0U, 1U, 0U, 2, 1, 60));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_y);

  /*  Beacon intervals for beacon orders 4...14 */
  _SFD_EML_CALL(0,11);
  for (c2_i8 = 0; c2_i8 < 11; c2_i8 = c2_i8 + 1) {
    c2_BI[c2_i8] = c2_dv1[c2_i8];
  }

  _SFD_EML_CALL(0,12);
  c2_randState = c2_randStateIN;
  _SFD_EML_CALL(0,13);
  c2_rand(chartInstance, c2_randState);

  /* 	NODES: */
  /* 	  1 -> Sensor Tank 1 (TM1) */
  /* 	  2 -> Actuator Tank 1 (TA1) */
  /* 	  3 -> Sensor Tank 2 (TM2) */
  /* 	  4 -> Actuator Tank 2 (TA2) */
  /* 	  5 -> Sensor 1 (TS1)  */
  /* 	  6 -> Sensor 2 (TS2) */
  /* 	  7 -> Sensor 3 (TS3) */
  /* 	  8 -> Sensor 4 (TS4) */
  /* 	  9 -> Sensor 5 (TS5) */
  /* % Parameters */
  _SFD_EML_CALL(0,29);
  c2_tBI = c2_BI[_SFD_EML_ARRAY_BOUNDS_CHECK("BI", (int32_T)_SFD_INTEGER_CHECK(
    "", c2_current_BO - 3.0), 1, 11, 1, 0) - 1];
  _SFD_EML_CALL(0,30);
  c2_symbol_time = 1.5259E-05;

  /*  time in seconds */
  _SFD_EML_CALL(0,31);
  c2_deltaCAP = 0.2637;

  /*  time in seconds */
  _SFD_EML_CALL(0,33);
  for (c2_i9 = 0; c2_i9 < 9; c2_i9 = c2_i9 + 1) {
    c2_TAU_K[c2_i9] = 1.0E+04;
  }

  /* set tau_k's to huge value so that we don't mess with calculating the min later on */
  _SFD_EML_CALL(0,35);
  c2_ttStart = c2_TIME;

  /*  time in symbols for the beacon broadcast */
  _SFD_EML_CALL(0,36);
  c2_a = c2_ttStart;
  c2_ttStart = c2_a * 1.5259E-05;

  /*  time in seconds */
  _SFD_EML_CALL(0,38);
  for (c2_i10 = 0; c2_i10 < 11; c2_i10 = c2_i10 + 1) {
    c2_time_schedule1[c2_i10] = 0.0;
  }

  _SFD_EML_CALL(0,39);
  c2_time_schedule1[0] = c2_time_schedule1_1;
  _SFD_EML_CALL(0,40);
  c2_time_schedule1[1] = c2_time_schedule1_2;
  _SFD_EML_CALL(0,41);
  c2_time_schedule1[2] = c2_time_schedule1_3;
  _SFD_EML_CALL(0,42);
  c2_time_schedule1[3] = c2_time_schedule1_4;
  _SFD_EML_CALL(0,43);
  c2_time_schedule1[4] = c2_time_schedule1_5;
  _SFD_EML_CALL(0,44);
  c2_time_schedule1[5] = c2_time_schedule1_6;
  _SFD_EML_CALL(0,45);
  c2_time_schedule1[6] = c2_time_schedule1_7;
  _SFD_EML_CALL(0,46);
  c2_time_schedule1[7] = c2_time_schedule1_8;
  _SFD_EML_CALL(0,47);
  c2_time_schedule1[8] = c2_time_schedule1_9;
  _SFD_EML_CALL(0,48);
  c2_time_schedule1[9] = c2_time_schedule1_10;
  _SFD_EML_CALL(0,49);
  c2_time_schedule1[10] = c2_time_schedule1_11;
  _SFD_EML_CALL(0,51);
  for (c2_i11 = 0; c2_i11 < 4; c2_i11 = c2_i11 + 1) {
    c2_lateIN[c2_i11] = 0.0;
  }

  _SFD_EML_CALL(0,52);
  for (c2_i12 = 0; c2_i12 < 4; c2_i12 = c2_i12 + 1) {
    c2_lateOUT[c2_i12] = c2_lateIN[c2_i12];
  }

  _SFD_EML_CALL(0,53);
  c2_lateIN[0] = c2_lateIN_1;
  _SFD_EML_CALL(0,54);
  c2_lateIN[1] = c2_lateIN_2;
  _SFD_EML_CALL(0,55);
  c2_lateIN[2] = c2_lateIN_3;
  _SFD_EML_CALL(0,56);
  c2_lateIN[3] = c2_lateIN_4;
  _SFD_EML_CALL(0,58);
  c2_INIT = 0.0;
  _SFD_EML_CALL(0,59);
  for (c2_i13 = 0; c2_i13 < 11; c2_i13 = c2_i13 + 1) {
    c2_b_time_schedule1[c2_i13] = c2_time_schedule1[c2_i13];
  }

  if (CV_EML_IF(0, 0, c2_sum(chartInstance, c2_b_time_schedule1) == 0.0)) {
    _SFD_EML_CALL(0,60);
    if (CV_EML_IF(0, 1, c2_INIT_in == 0.0)) {
      _SFD_EML_CALL(0,61);
      c2_BO = 6.0;
      _SFD_EML_CALL(0,62);
      c2_TM1 = 0.0;
      _SFD_EML_CALL(0,63);
      c2_TA1 = 0.0;
      _SFD_EML_CALL(0,64);
      c2_TM2 = 0.0;
      _SFD_EML_CALL(0,65);
      c2_TA2 = 0.0;
      _SFD_EML_CALL(0,66);
      c2_TS1 = 0.0;
      _SFD_EML_CALL(0,67);
      c2_TS2 = 0.0;
      _SFD_EML_CALL(0,68);
      c2_TS3 = 0.0;
      _SFD_EML_CALL(0,69);
      c2_TS4 = 0.0;
      _SFD_EML_CALL(0,70);
      c2_TS5 = 0.0;
      _SFD_EML_CALL(0,71);
      c2_TS6 = 0.0;
      _SFD_EML_CALL(0,72);
      c2_TS7 = 0.0;
      _SFD_EML_CALL(0,73);
      c2_time_schedule_1 = 0.0;
      _SFD_EML_CALL(0,74);
      c2_time_schedule_2 = 0.0;
      _SFD_EML_CALL(0,75);
      c2_time_schedule_3 = 0.0;
      _SFD_EML_CALL(0,76);
      c2_time_schedule_4 = 0.0;
      _SFD_EML_CALL(0,77);
      c2_time_schedule_5 = 0.0;
      _SFD_EML_CALL(0,78);
      c2_time_schedule_6 = 0.0;
      _SFD_EML_CALL(0,79);
      c2_time_schedule_7 = 0.0;
      _SFD_EML_CALL(0,80);
      c2_time_schedule_8 = 0.0;
      _SFD_EML_CALL(0,81);
      c2_time_schedule_9 = 0.0;
      _SFD_EML_CALL(0,82);
      c2_time_schedule_10 = 0.0;
      _SFD_EML_CALL(0,83);
      c2_time_schedule_11 = 0.0;
      _SFD_EML_CALL(0,84);
      for (c2_i14 = 0; c2_i14 < 9; c2_i14 = c2_i14 + 1) {
        c2_TAU_K[c2_i14] = 0.0;
      }

      _SFD_EML_CALL(0,85);
      c2_INIT = 1.0;
      _SFD_EML_CALL(0,87);
      c2_lateOUT_1 = 8.0;

      /*  in the INIT phase only nodes 1-7 receive slots. */
      _SFD_EML_CALL(0,88);
      c2_lateOUT_2 = 9.0;
      _SFD_EML_CALL(0,89);
      c2_lateOUT_3 = 10.0;
      _SFD_EML_CALL(0,90);
      c2_lateOUT_4 = 11.0;
      _SFD_EML_CALL(0,92);
      c2_randStateOUT = 10.4;
      _SFD_EML_CALL(0,94);
      for (c2_i15 = 0; c2_i15 < 51; c2_i15 = c2_i15 + 1) {
        c2_b_u[c2_i15] = c2_cv1[c2_i15];
      }

      c2_b_y = NULL;
      sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 10, 0U, 1U, 0U, 2, 1,
        51));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_b_y);
      goto label_1;
    } else {
      _SFD_EML_CALL(0,98);
      c2_b_a = c2_ttStart;
      for (c2_i16 = 0; c2_i16 < 11; c2_i16 = c2_i16 + 1) {
        c2_c_y[c2_i16] = c2_b_a;
      }

      for (c2_i17 = 0; c2_i17 < 11; c2_i17 = c2_i17 + 1) {
        c2_time_schedule[c2_i17] = c2_c_y[c2_i17] + c2_deltaCAP;
      }

      /* SO=5 -> each slot has a duration of 1920 symbols */
      /* Initial scheduling set up at the BS assigns slots 1->1, 2->2 ... 7->7 */
      c2_i = 1.0;
      c2_b_i = 1.0;
      while (c2_b_i <= 7.0) {
        c2_i = c2_b_i;
        CV_EML_FOR(0, 0, 1);

        /*  there are only 7 available slots in the CFP */
        _SFD_EML_CALL(0,102);
        c2_c_a = c2_i - 1.0;
        c2_d_y = c2_c_a * 1920.0;
        c2_d_a = c2_d_y;
        c2_e_y = c2_d_a * 1.5259E-05;
        c2_time_schedule[_SFD_EML_ARRAY_BOUNDS_CHECK("time_schedule", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 11, 1, 0) - 1] =
          c2_time_schedule[_SFD_EML_ARRAY_BOUNDS_CHECK("time_schedule", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 11, 1, 0) - 1] + c2_e_y;
        c2_b_i = c2_b_i + 1.0;
        sf_mex_listen_for_ctrl_c(chartInstance->S);
      }

      CV_EML_FOR(0, 0, 0);

      /*          temp = time_schedule(1); */
      /*          time_schedule(1) = time_schedule(3); */
      /*          time_schedule(3) = temp; */
      /*          temp = time_schedule(2); */
      /*          time_schedule(2) = time_schedule(4); */
      /*          time_schedule(4) = temp; */
    }
  } else {
    _SFD_EML_CALL(0,114);
    for (c2_i18 = 0; c2_i18 < 11; c2_i18 = c2_i18 + 1) {
      c2_time_schedule[c2_i18] = c2_time_schedule1[c2_i18];
    }
  }

  _SFD_EML_CALL(0,116);
  for (c2_i19 = 0; c2_i19 < 22; c2_i19 = c2_i19 + 1) {
    c2_scheduled_next[c2_i19] = 1.0E+04;
  }

  /*  clean slots */
  _SFD_EML_CALL(0,119);
  for (c2_i20 = 0; c2_i20 < 11; c2_i20 = c2_i20 + 1) {
    c2_slot[c2_i20] = 16.0;
  }

  /* slot 16 means do not allocate any slot. */
  /* % ST triggering */
  _SFD_EML_CALL(0,124);
  c2_y1L1_new = c2_Y11;
  _SFD_EML_CALL(0,125);
  c2_y1L2_new = c2_Y12;
  _SFD_EML_CALL(0,126);
  c2_y1INT_new = c2_I1;
  _SFD_EML_CALL(0,127);
  c2_y2L1_new = c2_Y21;
  _SFD_EML_CALL(0,128);
  c2_y2L2_new = c2_Y22;
  _SFD_EML_CALL(0,129);
  c2_y2INT_new = c2_I2;
  _SFD_EML_CALL(0,131);
  c2_ttStart_next = c2_ttStart + c2_tBI;

  /*  this is the time of the start of the next superframe */
  /*  1) Compute new sampling times */
  /*  control loops */
  /* execute ST to get tk+1 of i */
  /*  output should be tau_k */
  _SFD_EML_CALL(0,138);
  for (c2_i21 = 0; c2_i21 < 137; c2_i21 = c2_i21 + 1) {
    c2_c_u[c2_i21] = c2_cv2[c2_i21];
  }

  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_c_u, 10, 0U, 1U, 0U, 2, 1, 137));
  c2_d_u = c2_ttStart;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0));
  c2_e_u = c2_deltaCAP;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0));
  c2_f_u = c2_ttStart + c2_deltaCAP;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0));
  c2_g_u = c2_ttStart_next;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0));
  c2_h_u = c2_deltaCAP;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0));
  c2_i_u = c2_ttStart_next + c2_deltaCAP;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_i_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 7U, 14, c2_f_y, 14, c2_g_y, 14, c2_h_y, 14,
                    c2_i_y, 14, c2_j_y, 14, c2_k_y, 14, c2_l_y);
  _SFD_EML_CALL(0,139);
  for (c2_i22 = 0; c2_i22 < 36; c2_i22 = c2_i22 + 1) {
    c2_j_u[c2_i22] = c2_cv3[c2_i22];
  }

  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_j_u, 10, 0U, 1U, 0U, 2, 1, 36));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_m_y);
  _SFD_EML_CALL(0,140);
  for (c2_i23 = 0; c2_i23 < 10; c2_i23 = c2_i23 + 1) {
    c2_k_u[c2_i23] = c2_cv4[c2_i23];
  }

  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_k_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_l_u = c2_time_schedule[0];
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_l_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_n_y, 14, c2_o_y);
  _SFD_EML_CALL(0,141);
  if (CV_EML_IF(0, 2, c2_time_schedule[0] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,142);
    for (c2_i24 = 0; c2_i24 < 4; c2_i24 = c2_i24 + 1) {
      c2_b_lateIN[c2_i24] = (c2_lateIN[c2_i24] == 1.0);
    }

    if (CV_EML_IF(0, 3, c2_b_sum(chartInstance, c2_b_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,143);
      for (c2_i25 = 0; c2_i25 < 29; c2_i25 = c2_i25 + 1) {
        c2_m_u[c2_i25] = c2_cv5[c2_i25];
      }

      c2_p_y = NULL;
      sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_m_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_p_y);
    } else {
      _SFD_EML_CALL(0,145);
      for (c2_i26 = 0; c2_i26 < 4; c2_i26 = c2_i26 + 1) {
        c2_n_u[c2_i26] = c2_cv6[c2_i26];
      }

      c2_q_y = NULL;
      sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_n_u, 10, 0U, 1U, 0U, 2, 1, 4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_q_y);
    }
  }

  _SFD_EML_CALL(0,148);
  for (c2_i27 = 0; c2_i27 < 2; c2_i27 = c2_i27 + 1) {
    c2_o_u[c2_i27] = c2_cv7[c2_i27];
  }

  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_o_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_r_y);
  _SFD_EML_CALL(0,149);
  for (c2_i28 = 0; c2_i28 < 10; c2_i28 = c2_i28 + 1) {
    c2_p_u[c2_i28] = c2_cv8[c2_i28];
  }

  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", &c2_p_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_q_u = c2_time_schedule[1];
  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_create("y", &c2_q_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_s_y, 14, c2_t_y);
  _SFD_EML_CALL(0,150);
  if (CV_EML_IF(0, 4, c2_time_schedule[1] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,151);
    for (c2_i29 = 0; c2_i29 < 4; c2_i29 = c2_i29 + 1) {
      c2_c_lateIN[c2_i29] = (c2_lateIN[c2_i29] == 2.0);
    }

    if (CV_EML_IF(0, 5, c2_b_sum(chartInstance, c2_c_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,152);
      for (c2_i30 = 0; c2_i30 < 29; c2_i30 = c2_i30 + 1) {
        c2_r_u[c2_i30] = c2_cv5[c2_i30];
      }

      c2_u_y = NULL;
      sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_r_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_u_y);
    } else {
      _SFD_EML_CALL(0,154);
      for (c2_i31 = 0; c2_i31 < 4; c2_i31 = c2_i31 + 1) {
        c2_s_u[c2_i31] = c2_cv6[c2_i31];
      }

      c2_v_y = NULL;
      sf_mex_assign(&c2_v_y, sf_mex_create("y", &c2_s_u, 10, 0U, 1U, 0U, 2, 1, 4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_v_y);
    }
  }

  _SFD_EML_CALL(0,157);
  for (c2_i32 = 0; c2_i32 < 2; c2_i32 = c2_i32 + 1) {
    c2_t_u[c2_i32] = c2_cv7[c2_i32];
  }

  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_create("y", &c2_t_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_w_y);
  _SFD_EML_CALL(0,158);
  for (c2_i33 = 0; c2_i33 < 10; c2_i33 = c2_i33 + 1) {
    c2_u_u[c2_i33] = c2_cv9[c2_i33];
  }

  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_u_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_v_u = c2_time_schedule[2];
  c2_y_y = NULL;
  sf_mex_assign(&c2_y_y, sf_mex_create("y", &c2_v_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_x_y, 14, c2_y_y);
  _SFD_EML_CALL(0,159);
  if (CV_EML_IF(0, 6, c2_time_schedule[2] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,160);
    for (c2_i34 = 0; c2_i34 < 4; c2_i34 = c2_i34 + 1) {
      c2_d_lateIN[c2_i34] = (c2_lateIN[c2_i34] == 3.0);
    }

    if (CV_EML_IF(0, 7, c2_b_sum(chartInstance, c2_d_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,161);
      for (c2_i35 = 0; c2_i35 < 29; c2_i35 = c2_i35 + 1) {
        c2_w_u[c2_i35] = c2_cv5[c2_i35];
      }

      c2_ab_y = NULL;
      sf_mex_assign(&c2_ab_y, sf_mex_create("y", &c2_w_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_ab_y);
    } else {
      _SFD_EML_CALL(0,163);
      for (c2_i36 = 0; c2_i36 < 4; c2_i36 = c2_i36 + 1) {
        c2_x_u[c2_i36] = c2_cv6[c2_i36];
      }

      c2_bb_y = NULL;
      sf_mex_assign(&c2_bb_y, sf_mex_create("y", &c2_x_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_bb_y);
    }
  }

  _SFD_EML_CALL(0,166);
  for (c2_i37 = 0; c2_i37 < 2; c2_i37 = c2_i37 + 1) {
    c2_y_u[c2_i37] = c2_cv7[c2_i37];
  }

  c2_cb_y = NULL;
  sf_mex_assign(&c2_cb_y, sf_mex_create("y", &c2_y_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_cb_y);
  _SFD_EML_CALL(0,167);
  for (c2_i38 = 0; c2_i38 < 10; c2_i38 = c2_i38 + 1) {
    c2_ab_u[c2_i38] = c2_cv10[c2_i38];
  }

  c2_db_y = NULL;
  sf_mex_assign(&c2_db_y, sf_mex_create("y", &c2_ab_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_bb_u = c2_time_schedule[3];
  c2_eb_y = NULL;
  sf_mex_assign(&c2_eb_y, sf_mex_create("y", &c2_bb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_db_y, 14, c2_eb_y);
  _SFD_EML_CALL(0,168);
  if (CV_EML_IF(0, 8, c2_time_schedule[3] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,169);
    for (c2_i39 = 0; c2_i39 < 4; c2_i39 = c2_i39 + 1) {
      c2_e_lateIN[c2_i39] = (c2_lateIN[c2_i39] == 4.0);
    }

    if (CV_EML_IF(0, 9, c2_b_sum(chartInstance, c2_e_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,170);
      for (c2_i40 = 0; c2_i40 < 29; c2_i40 = c2_i40 + 1) {
        c2_cb_u[c2_i40] = c2_cv5[c2_i40];
      }

      c2_fb_y = NULL;
      sf_mex_assign(&c2_fb_y, sf_mex_create("y", &c2_cb_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_fb_y);
    } else {
      _SFD_EML_CALL(0,172);
      for (c2_i41 = 0; c2_i41 < 4; c2_i41 = c2_i41 + 1) {
        c2_db_u[c2_i41] = c2_cv6[c2_i41];
      }

      c2_gb_y = NULL;
      sf_mex_assign(&c2_gb_y, sf_mex_create("y", &c2_db_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_gb_y);
    }
  }

  _SFD_EML_CALL(0,175);
  for (c2_i42 = 0; c2_i42 < 2; c2_i42 = c2_i42 + 1) {
    c2_eb_u[c2_i42] = c2_cv7[c2_i42];
  }

  c2_hb_y = NULL;
  sf_mex_assign(&c2_hb_y, sf_mex_create("y", &c2_eb_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_hb_y);
  _SFD_EML_CALL(0,176);
  for (c2_i43 = 0; c2_i43 < 10; c2_i43 = c2_i43 + 1) {
    c2_fb_u[c2_i43] = c2_cv11[c2_i43];
  }

  c2_ib_y = NULL;
  sf_mex_assign(&c2_ib_y, sf_mex_create("y", &c2_fb_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_gb_u = c2_time_schedule[4];
  c2_jb_y = NULL;
  sf_mex_assign(&c2_jb_y, sf_mex_create("y", &c2_gb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ib_y, 14, c2_jb_y);
  _SFD_EML_CALL(0,177);
  if (CV_EML_IF(0, 10, c2_time_schedule[4] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,178);
    for (c2_i44 = 0; c2_i44 < 4; c2_i44 = c2_i44 + 1) {
      c2_f_lateIN[c2_i44] = (c2_lateIN[c2_i44] == 5.0);
    }

    if (CV_EML_IF(0, 11, c2_b_sum(chartInstance, c2_f_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,179);
      for (c2_i45 = 0; c2_i45 < 29; c2_i45 = c2_i45 + 1) {
        c2_hb_u[c2_i45] = c2_cv5[c2_i45];
      }

      c2_kb_y = NULL;
      sf_mex_assign(&c2_kb_y, sf_mex_create("y", &c2_hb_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_kb_y);
    } else {
      _SFD_EML_CALL(0,181);
      for (c2_i46 = 0; c2_i46 < 4; c2_i46 = c2_i46 + 1) {
        c2_ib_u[c2_i46] = c2_cv6[c2_i46];
      }

      c2_lb_y = NULL;
      sf_mex_assign(&c2_lb_y, sf_mex_create("y", &c2_ib_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_lb_y);
    }
  }

  _SFD_EML_CALL(0,184);
  for (c2_i47 = 0; c2_i47 < 2; c2_i47 = c2_i47 + 1) {
    c2_jb_u[c2_i47] = c2_cv7[c2_i47];
  }

  c2_mb_y = NULL;
  sf_mex_assign(&c2_mb_y, sf_mex_create("y", &c2_jb_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_mb_y);
  _SFD_EML_CALL(0,185);
  for (c2_i48 = 0; c2_i48 < 10; c2_i48 = c2_i48 + 1) {
    c2_kb_u[c2_i48] = c2_cv12[c2_i48];
  }

  c2_nb_y = NULL;
  sf_mex_assign(&c2_nb_y, sf_mex_create("y", &c2_kb_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_lb_u = c2_time_schedule[5];
  c2_ob_y = NULL;
  sf_mex_assign(&c2_ob_y, sf_mex_create("y", &c2_lb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_nb_y, 14, c2_ob_y);
  _SFD_EML_CALL(0,186);
  if (CV_EML_IF(0, 12, c2_time_schedule[5] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,187);
    for (c2_i49 = 0; c2_i49 < 4; c2_i49 = c2_i49 + 1) {
      c2_g_lateIN[c2_i49] = (c2_lateIN[c2_i49] == 6.0);
    }

    if (CV_EML_IF(0, 13, c2_b_sum(chartInstance, c2_g_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,188);
      for (c2_i50 = 0; c2_i50 < 29; c2_i50 = c2_i50 + 1) {
        c2_mb_u[c2_i50] = c2_cv5[c2_i50];
      }

      c2_pb_y = NULL;
      sf_mex_assign(&c2_pb_y, sf_mex_create("y", &c2_mb_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_pb_y);
    } else {
      _SFD_EML_CALL(0,190);
      for (c2_i51 = 0; c2_i51 < 4; c2_i51 = c2_i51 + 1) {
        c2_nb_u[c2_i51] = c2_cv6[c2_i51];
      }

      c2_qb_y = NULL;
      sf_mex_assign(&c2_qb_y, sf_mex_create("y", &c2_nb_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_qb_y);
    }
  }

  _SFD_EML_CALL(0,193);
  for (c2_i52 = 0; c2_i52 < 2; c2_i52 = c2_i52 + 1) {
    c2_ob_u[c2_i52] = c2_cv7[c2_i52];
  }

  c2_rb_y = NULL;
  sf_mex_assign(&c2_rb_y, sf_mex_create("y", &c2_ob_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_rb_y);
  _SFD_EML_CALL(0,194);
  for (c2_i53 = 0; c2_i53 < 10; c2_i53 = c2_i53 + 1) {
    c2_pb_u[c2_i53] = c2_cv13[c2_i53];
  }

  c2_sb_y = NULL;
  sf_mex_assign(&c2_sb_y, sf_mex_create("y", &c2_pb_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_qb_u = c2_time_schedule[6];
  c2_tb_y = NULL;
  sf_mex_assign(&c2_tb_y, sf_mex_create("y", &c2_qb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_sb_y, 14, c2_tb_y);
  _SFD_EML_CALL(0,195);
  if (CV_EML_IF(0, 14, c2_time_schedule[6] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,196);
    for (c2_i54 = 0; c2_i54 < 4; c2_i54 = c2_i54 + 1) {
      c2_h_lateIN[c2_i54] = (c2_lateIN[c2_i54] == 7.0);
    }

    if (CV_EML_IF(0, 15, c2_b_sum(chartInstance, c2_h_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,197);
      for (c2_i55 = 0; c2_i55 < 29; c2_i55 = c2_i55 + 1) {
        c2_rb_u[c2_i55] = c2_cv5[c2_i55];
      }

      c2_ub_y = NULL;
      sf_mex_assign(&c2_ub_y, sf_mex_create("y", &c2_rb_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_ub_y);
    } else {
      _SFD_EML_CALL(0,199);
      for (c2_i56 = 0; c2_i56 < 4; c2_i56 = c2_i56 + 1) {
        c2_sb_u[c2_i56] = c2_cv6[c2_i56];
      }

      c2_vb_y = NULL;
      sf_mex_assign(&c2_vb_y, sf_mex_create("y", &c2_sb_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_vb_y);
    }
  }

  _SFD_EML_CALL(0,202);
  for (c2_i57 = 0; c2_i57 < 2; c2_i57 = c2_i57 + 1) {
    c2_tb_u[c2_i57] = c2_cv7[c2_i57];
  }

  c2_wb_y = NULL;
  sf_mex_assign(&c2_wb_y, sf_mex_create("y", &c2_tb_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_wb_y);
  _SFD_EML_CALL(0,203);
  for (c2_i58 = 0; c2_i58 < 10; c2_i58 = c2_i58 + 1) {
    c2_ub_u[c2_i58] = c2_cv14[c2_i58];
  }

  c2_xb_y = NULL;
  sf_mex_assign(&c2_xb_y, sf_mex_create("y", &c2_ub_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_vb_u = c2_time_schedule[7];
  c2_yb_y = NULL;
  sf_mex_assign(&c2_yb_y, sf_mex_create("y", &c2_vb_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_xb_y, 14, c2_yb_y);
  _SFD_EML_CALL(0,204);
  if (CV_EML_IF(0, 16, c2_time_schedule[7] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,205);
    for (c2_i59 = 0; c2_i59 < 4; c2_i59 = c2_i59 + 1) {
      c2_i_lateIN[c2_i59] = (c2_lateIN[c2_i59] == 8.0);
    }

    if (CV_EML_IF(0, 17, c2_b_sum(chartInstance, c2_i_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,206);
      for (c2_i60 = 0; c2_i60 < 29; c2_i60 = c2_i60 + 1) {
        c2_wb_u[c2_i60] = c2_cv5[c2_i60];
      }

      c2_ac_y = NULL;
      sf_mex_assign(&c2_ac_y, sf_mex_create("y", &c2_wb_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_ac_y);
    } else {
      _SFD_EML_CALL(0,208);
      for (c2_i61 = 0; c2_i61 < 4; c2_i61 = c2_i61 + 1) {
        c2_xb_u[c2_i61] = c2_cv6[c2_i61];
      }

      c2_bc_y = NULL;
      sf_mex_assign(&c2_bc_y, sf_mex_create("y", &c2_xb_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_bc_y);
    }
  }

  _SFD_EML_CALL(0,211);
  for (c2_i62 = 0; c2_i62 < 2; c2_i62 = c2_i62 + 1) {
    c2_yb_u[c2_i62] = c2_cv7[c2_i62];
  }

  c2_cc_y = NULL;
  sf_mex_assign(&c2_cc_y, sf_mex_create("y", &c2_yb_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_cc_y);
  _SFD_EML_CALL(0,212);
  for (c2_i63 = 0; c2_i63 < 10; c2_i63 = c2_i63 + 1) {
    c2_ac_u[c2_i63] = c2_cv15[c2_i63];
  }

  c2_dc_y = NULL;
  sf_mex_assign(&c2_dc_y, sf_mex_create("y", &c2_ac_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_bc_u = c2_time_schedule[8];
  c2_ec_y = NULL;
  sf_mex_assign(&c2_ec_y, sf_mex_create("y", &c2_bc_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_dc_y, 14, c2_ec_y);
  _SFD_EML_CALL(0,213);
  if (CV_EML_IF(0, 18, c2_time_schedule[8] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,214);
    for (c2_i64 = 0; c2_i64 < 4; c2_i64 = c2_i64 + 1) {
      c2_j_lateIN[c2_i64] = (c2_lateIN[c2_i64] == 9.0);
    }

    if (CV_EML_IF(0, 19, c2_b_sum(chartInstance, c2_j_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,215);
      for (c2_i65 = 0; c2_i65 < 29; c2_i65 = c2_i65 + 1) {
        c2_cc_u[c2_i65] = c2_cv5[c2_i65];
      }

      c2_fc_y = NULL;
      sf_mex_assign(&c2_fc_y, sf_mex_create("y", &c2_cc_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_fc_y);
    } else {
      _SFD_EML_CALL(0,217);
      for (c2_i66 = 0; c2_i66 < 4; c2_i66 = c2_i66 + 1) {
        c2_dc_u[c2_i66] = c2_cv6[c2_i66];
      }

      c2_gc_y = NULL;
      sf_mex_assign(&c2_gc_y, sf_mex_create("y", &c2_dc_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_gc_y);
    }
  }

  _SFD_EML_CALL(0,220);
  for (c2_i67 = 0; c2_i67 < 2; c2_i67 = c2_i67 + 1) {
    c2_ec_u[c2_i67] = c2_cv7[c2_i67];
  }

  c2_hc_y = NULL;
  sf_mex_assign(&c2_hc_y, sf_mex_create("y", &c2_ec_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_hc_y);
  _SFD_EML_CALL(0,221);
  for (c2_i68 = 0; c2_i68 < 11; c2_i68 = c2_i68 + 1) {
    c2_fc_u[c2_i68] = c2_cv16[c2_i68];
  }

  c2_ic_y = NULL;
  sf_mex_assign(&c2_ic_y, sf_mex_create("y", &c2_fc_u, 10, 0U, 1U, 0U, 2, 1, 11));
  c2_gc_u = c2_time_schedule[9];
  c2_jc_y = NULL;
  sf_mex_assign(&c2_jc_y, sf_mex_create("y", &c2_gc_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ic_y, 14, c2_jc_y);
  _SFD_EML_CALL(0,222);
  if (CV_EML_IF(0, 20, c2_time_schedule[9] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,223);
    for (c2_i69 = 0; c2_i69 < 4; c2_i69 = c2_i69 + 1) {
      c2_k_lateIN[c2_i69] = (c2_lateIN[c2_i69] == 10.0);
    }

    if (CV_EML_IF(0, 21, c2_b_sum(chartInstance, c2_k_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,224);
      for (c2_i70 = 0; c2_i70 < 29; c2_i70 = c2_i70 + 1) {
        c2_hc_u[c2_i70] = c2_cv5[c2_i70];
      }

      c2_kc_y = NULL;
      sf_mex_assign(&c2_kc_y, sf_mex_create("y", &c2_hc_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_kc_y);
    } else {
      _SFD_EML_CALL(0,226);
      for (c2_i71 = 0; c2_i71 < 4; c2_i71 = c2_i71 + 1) {
        c2_ic_u[c2_i71] = c2_cv6[c2_i71];
      }

      c2_lc_y = NULL;
      sf_mex_assign(&c2_lc_y, sf_mex_create("y", &c2_ic_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_lc_y);
    }
  }

  _SFD_EML_CALL(0,229);
  for (c2_i72 = 0; c2_i72 < 2; c2_i72 = c2_i72 + 1) {
    c2_jc_u[c2_i72] = c2_cv7[c2_i72];
  }

  c2_mc_y = NULL;
  sf_mex_assign(&c2_mc_y, sf_mex_create("y", &c2_jc_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_mc_y);
  _SFD_EML_CALL(0,230);
  for (c2_i73 = 0; c2_i73 < 11; c2_i73 = c2_i73 + 1) {
    c2_kc_u[c2_i73] = c2_cv17[c2_i73];
  }

  c2_nc_y = NULL;
  sf_mex_assign(&c2_nc_y, sf_mex_create("y", &c2_kc_u, 10, 0U, 1U, 0U, 2, 1, 11));
  c2_lc_u = c2_time_schedule[10];
  c2_oc_y = NULL;
  sf_mex_assign(&c2_oc_y, sf_mex_create("y", &c2_lc_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_nc_y, 14, c2_oc_y);
  _SFD_EML_CALL(0,231);
  if (CV_EML_IF(0, 22, c2_time_schedule[10] <= c2_ttStart_next + c2_deltaCAP)) {
    _SFD_EML_CALL(0,232);
    for (c2_i74 = 0; c2_i74 < 4; c2_i74 = c2_i74 + 1) {
      c2_l_lateIN[c2_i74] = (c2_lateIN[c2_i74] == 11.0);
    }

    if (CV_EML_IF(0, 23, c2_b_sum(chartInstance, c2_l_lateIN) != 0.0) != 0.0) {
      _SFD_EML_CALL(0,233);
      for (c2_i75 = 0; c2_i75 < 29; c2_i75 = c2_i75 + 1) {
        c2_mc_u[c2_i75] = c2_cv5[c2_i75];
      }

      c2_pc_y = NULL;
      sf_mex_assign(&c2_pc_y, sf_mex_create("y", &c2_mc_u, 10, 0U, 1U, 0U, 2, 1,
        29));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_pc_y);
    } else {
      _SFD_EML_CALL(0,235);
      for (c2_i76 = 0; c2_i76 < 4; c2_i76 = c2_i76 + 1) {
        c2_nc_u[c2_i76] = c2_cv6[c2_i76];
      }

      c2_qc_y = NULL;
      sf_mex_assign(&c2_qc_y, sf_mex_create("y", &c2_nc_u, 10, 0U, 1U, 0U, 2, 1,
        4));
      sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_qc_y);
    }
  }

  _SFD_EML_CALL(0,238);
  for (c2_i77 = 0; c2_i77 < 2; c2_i77 = c2_i77 + 1) {
    c2_oc_u[c2_i77] = c2_cv7[c2_i77];
  }

  c2_rc_y = NULL;
  sf_mex_assign(&c2_rc_y, sf_mex_create("y", &c2_oc_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_rc_y);
  _SFD_EML_CALL(0,240);
  if (CV_EML_COND(0, 0, c2_time_schedule[0] <= c2_ttStart_next + c2_deltaCAP)) {
    for (c2_i78 = 0; c2_i78 < 4; c2_i78 = c2_i78 + 1) {
      c2_m_lateIN[c2_i78] = (c2_lateIN[c2_i78] == 1.0);
    }

    if (!(CV_EML_COND(0, 1, c2_b_sum(chartInstance, c2_m_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 0, TRUE);
      CV_EML_IF(0, 24, TRUE);
      _SFD_EML_CALL(0,241);
      c2_b_y1L1_new[0] = c2_y1L1_new - 10.0;
      c2_b_y1L1_new[1] = c2_y1L2_new - 10.0;
      c2_b_y1L1_new[2] = c2_y1INT_new;
      for (c2_i79 = 0; c2_i79 < 3; c2_i79 = c2_i79 + 1) {
        c2_data.x[c2_i79] = c2_b_y1L1_new[c2_i79];
      }

      _SFD_EML_CALL(0,242);
      c2_x10.re = c2_data.x[0] - 5.0;
      c2_x10.im = 0.0;
      _SFD_EML_CALL(0,243);
      c2_x20.re = c2_data.x[1] - 5.0;
      c2_x20.im = 0.0;
      _SFD_EML_CALL(0,244);
      c2_x30.re = c2_data.x[2] + 172.0;
      c2_x30.im = 0.0;
      _SFD_EML_CALL(0,247);
      for (c2_i80 = 0; c2_i80 < 600; c2_i80 = c2_i80 + 1) {
        c2_lvm[c2_i80] = c2_dv2[c2_i80];
      }

      _SFD_EML_CALL(0,348);
      c2_e_a = c2_x10;
      c2_b = c2_x20;
      c2_sc_y.re = c2_e_a.re * c2_b.re - c2_e_a.im * c2_b.im;
      c2_sc_y.im = c2_e_a.re * c2_b.im + c2_e_a.im * c2_b.re;
      c2_f_a = c2_x10;
      c2_b_b = c2_x30;
      c2_tc_y.re = c2_f_a.re * c2_b_b.re - c2_f_a.im * c2_b_b.im;
      c2_tc_y.im = c2_f_a.re * c2_b_b.im + c2_f_a.im * c2_b_b.re;
      c2_g_a = c2_x20;
      c2_c_b = c2_x30;
      c2_uc_y.re = c2_g_a.re * c2_c_b.re - c2_g_a.im * c2_c_b.im;
      c2_uc_y.im = c2_g_a.re * c2_c_b.im + c2_g_a.im * c2_c_b.re;
      c2_dc0 = c2_mpower(chartInstance, c2_x10);
      c2_dc1 = c2_mpower(chartInstance, c2_x20);
      c2_dc2 = c2_mpower(chartInstance, c2_x30);
      c2_dcv0[0].re = c2_dc0.re;
      c2_dcv0[0].im = -c2_dc0.im;
      c2_dcv0[1].re = c2_sc_y.re;
      c2_dcv0[1].im = -c2_sc_y.im;
      c2_dcv0[2].re = c2_tc_y.re;
      c2_dcv0[2].im = -c2_tc_y.im;
      c2_dcv0[3].re = c2_dc1.re;
      c2_dcv0[3].im = -c2_dc1.im;
      c2_dcv0[4].re = c2_uc_y.re;
      c2_dcv0[4].im = -c2_uc_y.im;
      c2_dcv0[5].re = c2_dc2.re;
      c2_dcv0[5].im = -c2_dc2.im;
      for (c2_i81 = 0; c2_i81 < 6; c2_i81 = c2_i81 + 1) {
        c2_d_b[c2_i81] = c2_dcv0[c2_i81];
      }

      for (c2_i82 = 0; c2_i82 < 100; c2_i82 = c2_i82 + 1) {
        c2_temp17[c2_i82].re = 0.0;
        c2_temp17[c2_i82].im = 0.0;
        for (c2_i83 = 0; c2_i83 < 6; c2_i83 = c2_i83 + 1) {
          c2_hoistedExpr = c2_i82 + 100 * c2_i83;
          c2_dc3.re = c2_dv2[c2_hoistedExpr];
          c2_dc3.im = 0.0;
          c2_dc4.re = c2_dc3.re * c2_d_b[c2_i83].re - c2_dc3.im * c2_d_b[c2_i83]
            .im;
          c2_dc4.im = c2_dc3.re * c2_d_b[c2_i83].im + c2_dc3.im * c2_d_b[c2_i83]
            .re;
          c2_temp17[c2_i82].re = c2_temp17[c2_i82].re + c2_dc4.re;
          c2_temp17[c2_i82].im = c2_temp17[c2_i82].im + c2_dc4.im;
        }
      }

      _SFD_EML_CALL(0,349);
      c2_next_time = 0.0;
      c2_ii = 1.0;
      c2_b_ii = 1.0;
     label_2:
      ;
      if (c2_b_ii <= 100.0) {
        c2_ii = c2_b_ii;
        CV_EML_FOR(0, 1, 1);
        _SFD_EML_CALL(0,352);
        if (CV_EML_IF(0, 25, c2_temp17[_SFD_EML_ARRAY_BOUNDS_CHECK("temp17",
              (int32_T)_SFD_INTEGER_CHECK("ii", c2_ii), 1, 100, 1, 0) - 1].re
                      < 0.0)) {
          _SFD_EML_CALL(0,353);
          c2_next_time = c2_ii;
        } else {
          c2_b_ii = c2_b_ii + 1.0;
          sf_mex_listen_for_ctrl_c(chartInstance->S);
          goto label_2;
        }
      } else {
        CV_EML_FOR(0, 1, 0);
      }

      _SFD_EML_CALL(0,357);
      for (c2_i84 = 0; c2_i84 < 27; c2_i84 = c2_i84 + 1) {
        c2_pc_u[c2_i84] = c2_cv18[c2_i84];
      }

      c2_vc_y = NULL;
      sf_mex_assign(&c2_vc_y, sf_mex_create("y", &c2_pc_u, 10, 0U, 1U, 0U, 2, 1,
        27));
      c2_qc_u = c2_next_time;
      c2_wc_y = NULL;
      sf_mex_assign(&c2_wc_y, sf_mex_create("y", &c2_qc_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_vc_y, 14, c2_wc_y);
      _SFD_EML_CALL(0,358);
      if (CV_EML_IF(0, 26, c2_next_time > 10.0)) {
        _SFD_EML_CALL(0,359);
        c2_next_time = 10.0;
      }

      _SFD_EML_CALL(0,361);
      if (CV_EML_IF(0, 27, c2_next_time == 0.0)) {
        _SFD_EML_CALL(0,362);
        c2_next_time = 10.0;
      }

      /* next_time = 1; */
      /*
         next_time = 1/1000*(x30^2+x10^2+x20^2)/(.1790e-3*x10*x20+.8050e-4*x20^2+.2220e-3*x10^2-.1000e-2*x20*x30+.2970e-4*x10*x30+(.8450e-7*x10^2*x30^2+.2100e-5*x10^2*x20^2-.4130e-6*x20^2*x10*x30+.1280e-6*x10*x30^2*x20+.2660e-6*x10^3*x20-.4940e-7*x20^3*x30+.2160e-6*x20^3*x10+.2950e-5*x20^2*x30^2+.1310e-6*x10^4+.1960e-5*x20^4+.1590e-8*x30^4-.4720e-7*x10^3*x30+.1120e-6*x30^3*x20-.6030e-7*x30^3*x10-.3210e-6*x10^2*x20*x30)^(1/2)); */
      /*
         next_time = .70000e-2/(-.11865e-1*x20*x30+.95396e-3*x20^2+.21198e-2*x10*x20+.35169e-3*x10*x30+.26262e-2*x10^2+.20604e-37*(-.13900e71*x30^3*x10+.25736e71*x30^3*x20-.13082e72*x10*x20^2*x30+.36513e69*x30^4+.69318e71*x10^3*x20-.95486e70*x10^3*x30+.35134e71*x10^4+.45169e72*x20^4+.78155e72*x20^2*x30^2+.49082e72*x10^2*x20^2-.11756e72*x10^2*x20*x30+.52617e71*x20^3*x10-.27592e71*x20^3*x30+.19544e71*x10^2*x30^2+.23431e71*x20*x30^2*x10)^(1/2))*(x10^2+x20^2+x30^2); */
      /* force low TAU for comparison purposes. */
      /*    next_time = 1; % this is the lowest tau obtained running the regular program */
      /*      if imag(next_time)==0 */
      /*          if next_time < 0.9375 */
      /*
         next_time = next_time + 0.1809; %0.1809 instead of 0.1808 because lt(min(TAU_K), 0.9375) == TRUE even if min(TAU_K)==0.9375  */
      /*          end */
      /*      end */
      /*      if imag(next_time)~=0 || next_time>=10 || next_time<0 */
      /*          next_time=10; */
      /*      end */
      _SFD_EML_CALL(0,383);
      c2_time_schedule[0] = c2_time_schedule[0] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,384);
      c2_time_schedule[1] = c2_time_schedule[1] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,385);
      c2_TAU_K[0] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,387);
      for (c2_i85 = 0; c2_i85 < 71; c2_i85 = c2_i85 + 1) {
        c2_rc_u[c2_i85] = c2_cv19[c2_i85];
      }

      c2_xc_y = NULL;
      sf_mex_assign(&c2_xc_y, sf_mex_create("y", &c2_rc_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_sc_u = c2_TAU_K[0];
      c2_yc_y = NULL;
      sf_mex_assign(&c2_yc_y, sf_mex_create("y", &c2_sc_u, 0, 0U, 0U, 0U, 0));
      c2_tc_u = c2_time_schedule[0];
      c2_ad_y = NULL;
      sf_mex_assign(&c2_ad_y, sf_mex_create("y", &c2_tc_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_xc_y, 14, c2_yc_y, 14, c2_ad_y);
      _SFD_EML_CALL(0,388);
      for (c2_i86 = 0; c2_i86 < 71; c2_i86 = c2_i86 + 1) {
        c2_uc_u[c2_i86] = c2_cv20[c2_i86];
      }

      c2_bd_y = NULL;
      sf_mex_assign(&c2_bd_y, sf_mex_create("y", &c2_uc_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_vc_u = c2_TAU_K[0];
      c2_cd_y = NULL;
      sf_mex_assign(&c2_cd_y, sf_mex_create("y", &c2_vc_u, 0, 0U, 0U, 0U, 0));
      c2_wc_u = c2_time_schedule[1];
      c2_dd_y = NULL;
      sf_mex_assign(&c2_dd_y, sf_mex_create("y", &c2_wc_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_bd_y, 14, c2_cd_y, 14, c2_dd_y);
      goto label_3;
    }
  }

  CV_EML_MCDC(0, 0, FALSE);
  CV_EML_IF(0, 24, FALSE);
 label_3:
  ;
  _SFD_EML_CALL(0,392);
  if (CV_EML_COND(0, 2, c2_time_schedule[2] <= c2_ttStart_next + c2_deltaCAP)) {
    for (c2_i87 = 0; c2_i87 < 4; c2_i87 = c2_i87 + 1) {
      c2_n_lateIN[c2_i87] = (c2_lateIN[c2_i87] == 3.0);
    }

    if (!(CV_EML_COND(0, 3, c2_b_sum(chartInstance, c2_n_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 1, TRUE);
      CV_EML_IF(0, 28, TRUE);
      _SFD_EML_CALL(0,393);
      c2_b_y2L1_new[0] = c2_y2L1_new - 10.0;
      c2_b_y2L1_new[1] = c2_y2L2_new - 10.0;
      c2_b_y2L1_new[2] = c2_y2INT_new;
      for (c2_i88 = 0; c2_i88 < 3; c2_i88 = c2_i88 + 1) {
        c2_data.x[c2_i88] = c2_b_y2L1_new[c2_i88];
      }

      _SFD_EML_CALL(0,394);
      c2_x10.re = c2_data.x[0] - 5.0;
      c2_x10.im = 0.0;
      _SFD_EML_CALL(0,395);
      c2_x20.re = c2_data.x[1] - 5.0;
      c2_x20.im = 0.0;
      _SFD_EML_CALL(0,396);
      c2_x30.re = c2_data.x[2] + 172.0;
      c2_x30.im = 0.0;
      _SFD_EML_CALL(0,398);
      for (c2_i89 = 0; c2_i89 < 600; c2_i89 = c2_i89 + 1) {
        c2_lvm[c2_i89] = c2_dv2[c2_i89];
      }

      _SFD_EML_CALL(0,499);
      c2_h_a = c2_x10;
      c2_e_b = c2_x20;
      c2_ed_y.re = c2_h_a.re * c2_e_b.re - c2_h_a.im * c2_e_b.im;
      c2_ed_y.im = c2_h_a.re * c2_e_b.im + c2_h_a.im * c2_e_b.re;
      c2_i_a = c2_x10;
      c2_f_b = c2_x30;
      c2_fd_y.re = c2_i_a.re * c2_f_b.re - c2_i_a.im * c2_f_b.im;
      c2_fd_y.im = c2_i_a.re * c2_f_b.im + c2_i_a.im * c2_f_b.re;
      c2_j_a = c2_x20;
      c2_g_b = c2_x30;
      c2_gd_y.re = c2_j_a.re * c2_g_b.re - c2_j_a.im * c2_g_b.im;
      c2_gd_y.im = c2_j_a.re * c2_g_b.im + c2_j_a.im * c2_g_b.re;
      c2_dc5 = c2_mpower(chartInstance, c2_x10);
      c2_dc6 = c2_mpower(chartInstance, c2_x20);
      c2_dc7 = c2_mpower(chartInstance, c2_x30);
      c2_dcv1[0].re = c2_dc5.re;
      c2_dcv1[0].im = -c2_dc5.im;
      c2_dcv1[1].re = c2_ed_y.re;
      c2_dcv1[1].im = -c2_ed_y.im;
      c2_dcv1[2].re = c2_fd_y.re;
      c2_dcv1[2].im = -c2_fd_y.im;
      c2_dcv1[3].re = c2_dc6.re;
      c2_dcv1[3].im = -c2_dc6.im;
      c2_dcv1[4].re = c2_gd_y.re;
      c2_dcv1[4].im = -c2_gd_y.im;
      c2_dcv1[5].re = c2_dc7.re;
      c2_dcv1[5].im = -c2_dc7.im;
      for (c2_i90 = 0; c2_i90 < 6; c2_i90 = c2_i90 + 1) {
        c2_h_b[c2_i90] = c2_dcv1[c2_i90];
      }

      for (c2_i91 = 0; c2_i91 < 100; c2_i91 = c2_i91 + 1) {
        c2_temp17[c2_i91].re = 0.0;
        c2_temp17[c2_i91].im = 0.0;
        for (c2_i92 = 0; c2_i92 < 6; c2_i92 = c2_i92 + 1) {
          c2_b_hoistedExpr = c2_i91 + 100 * c2_i92;
          c2_dc8.re = c2_dv2[c2_b_hoistedExpr];
          c2_dc8.im = 0.0;
          c2_dc9.re = c2_dc8.re * c2_h_b[c2_i92].re - c2_dc8.im * c2_h_b[c2_i92]
            .im;
          c2_dc9.im = c2_dc8.re * c2_h_b[c2_i92].im + c2_dc8.im * c2_h_b[c2_i92]
            .re;
          c2_temp17[c2_i91].re = c2_temp17[c2_i91].re + c2_dc9.re;
          c2_temp17[c2_i91].im = c2_temp17[c2_i91].im + c2_dc9.im;
        }
      }

      _SFD_EML_CALL(0,500);
      c2_next_time = 0.0;
      c2_ii = 1.0;
      c2_c_ii = 1.0;
     label_4:
      ;
      if (c2_c_ii <= 100.0) {
        c2_ii = c2_c_ii;
        CV_EML_FOR(0, 2, 1);
        _SFD_EML_CALL(0,503);
        if (CV_EML_IF(0, 29, c2_temp17[_SFD_EML_ARRAY_BOUNDS_CHECK("temp17",
              (int32_T)_SFD_INTEGER_CHECK("ii", c2_ii), 1, 100, 1, 0) - 1].re
                      < 0.0)) {
          _SFD_EML_CALL(0,504);
          c2_next_time = c2_ii;
        } else {
          c2_c_ii = c2_c_ii + 1.0;
          sf_mex_listen_for_ctrl_c(chartInstance->S);
          goto label_4;
        }
      } else {
        CV_EML_FOR(0, 2, 0);
      }

      _SFD_EML_CALL(0,509);
      for (c2_i93 = 0; c2_i93 < 27; c2_i93 = c2_i93 + 1) {
        c2_xc_u[c2_i93] = c2_cv21[c2_i93];
      }

      c2_hd_y = NULL;
      sf_mex_assign(&c2_hd_y, sf_mex_create("y", &c2_xc_u, 10, 0U, 1U, 0U, 2, 1,
        27));
      c2_yc_u = c2_next_time;
      c2_id_y = NULL;
      sf_mex_assign(&c2_id_y, sf_mex_create("y", &c2_yc_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_hd_y, 14, c2_id_y);
      _SFD_EML_CALL(0,510);
      if (CV_EML_IF(0, 30, c2_next_time > 10.0)) {
        _SFD_EML_CALL(0,511);
        c2_next_time = 10.0;
      }

      _SFD_EML_CALL(0,514);
      if (CV_EML_IF(0, 31, c2_next_time == 0.0)) {
        _SFD_EML_CALL(0,515);
        c2_next_time = 10.0;
      }

      /* next_time = 1; */
      /*
         next_time = 1/1000*(x30^2+x10^2+x20^2)/(.1790e-3*x10*x20+.8050e-4*x20^2+.2220e-3*x10^2-.1000e-2*x20*x30+.2970e-4*x10*x30+(.8450e-7*x10^2*x30^2+.2100e-5*x10^2*x20^2-.4130e-6*x20^2*x10*x30+.1280e-6*x10*x30^2*x20+.2660e-6*x10^3*x20-.4940e-7*x20^3*x30+.2160e-6*x20^3*x10+.2950e-5*x20^2*x30^2+.1310e-6*x10^4+.1960e-5*x20^4+.1590e-8*x30^4-.4720e-7*x10^3*x30+.1120e-6*x30^3*x20-.6030e-7*x30^3*x10-.3210e-6*x10^2*x20*x30)^(1/2)); */
      /*
         next_time = .70000e-2/(-.11865e-1*x20*x30+.95396e-3*x20^2+.21198e-2*x10*x20+.35169e-3*x10*x30+.26262e-2*x10^2+.20604e-37*(-.13900e71*x30^3*x10+.25736e71*x30^3*x20-.13082e72*x10*x20^2*x30+.36513e69*x30^4+.69318e71*x10^3*x20-.95486e70*x10^3*x30+.35134e71*x10^4+.45169e72*x20^4+.78155e72*x20^2*x30^2+.49082e72*x10^2*x20^2-.11756e72*x10^2*x20*x30+.52617e71*x20^3*x10-.27592e71*x20^3*x30+.19544e71*x10^2*x30^2+.23431e71*x20*x30^2*x10)^(1/2))*(x10^2+x20^2+x30^2); */
      /* force low TAU for comparison purposes. */
      /* next_time = 1; % this is the lowest tau obtained running the regular program */
      /*      if imag(next_time)==0 */
      /*          if next_time < 0.9375 */
      /*              disp(next_time); */
      /*              next_time = next_time + 0.1809; */
      /*              disp('next_time < 0.9375'); */
      /*          end */
      /*      end */
      /*      if imag(next_time)~=0 || next_time>=10 || next_time<0 */
      /*          next_time=10; */
      /*      end */
      _SFD_EML_CALL(0,536);
      c2_time_schedule[2] = c2_time_schedule[2] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,537);
      c2_time_schedule[3] = c2_time_schedule[3] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,538);
      c2_TAU_K[1] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,540);
      for (c2_i94 = 0; c2_i94 < 71; c2_i94 = c2_i94 + 1) {
        c2_ad_u[c2_i94] = c2_cv22[c2_i94];
      }

      c2_jd_y = NULL;
      sf_mex_assign(&c2_jd_y, sf_mex_create("y", &c2_ad_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_bd_u = c2_TAU_K[1];
      c2_kd_y = NULL;
      sf_mex_assign(&c2_kd_y, sf_mex_create("y", &c2_bd_u, 0, 0U, 0U, 0U, 0));
      c2_cd_u = c2_time_schedule[2];
      c2_ld_y = NULL;
      sf_mex_assign(&c2_ld_y, sf_mex_create("y", &c2_cd_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_jd_y, 14, c2_kd_y, 14, c2_ld_y);
      _SFD_EML_CALL(0,541);
      for (c2_i95 = 0; c2_i95 < 71; c2_i95 = c2_i95 + 1) {
        c2_dd_u[c2_i95] = c2_cv23[c2_i95];
      }

      c2_md_y = NULL;
      sf_mex_assign(&c2_md_y, sf_mex_create("y", &c2_dd_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_ed_u = c2_TAU_K[1];
      c2_nd_y = NULL;
      sf_mex_assign(&c2_nd_y, sf_mex_create("y", &c2_ed_u, 0, 0U, 0U, 0U, 0));
      c2_fd_u = c2_time_schedule[3];
      c2_od_y = NULL;
      sf_mex_assign(&c2_od_y, sf_mex_create("y", &c2_fd_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_md_y, 14, c2_nd_y, 14, c2_od_y);
      goto label_5;
    }
  }

  CV_EML_MCDC(0, 1, FALSE);
  CV_EML_IF(0, 28, FALSE);
 label_5:
  ;
  _SFD_EML_CALL(0,544);
  if (CV_EML_COND(0, 4, c2_time_schedule[4] <= c2_ttStart_next + c2_deltaCAP)) {
    for (c2_i96 = 0; c2_i96 < 4; c2_i96 = c2_i96 + 1) {
      c2_o_lateIN[c2_i96] = (c2_lateIN[c2_i96] == 5.0);
    }

    if (!(CV_EML_COND(0, 5, c2_b_sum(chartInstance, c2_o_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 2, TRUE);
      CV_EML_IF(0, 32, TRUE);

      /*  control loops */
      /*  low priority loops */
      /*  we can set some conditions on the value of the measurement: */
      /*  e.g. if the temperature is higher than something then we sample */
      /*  faster! */
      _SFD_EML_CALL(0,550);
      c2_i_b = c2_b_rand(chartInstance);
      c2_pd_y = 2.0 * c2_i_b;
      c2_next_time = 1.0 + c2_pd_y;
      _SFD_EML_CALL(0,551);
      c2_time_schedule[4] = c2_time_schedule[4] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,552);
      c2_TAU_K[2] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,554);
      for (c2_i97 = 0; c2_i97 < 71; c2_i97 = c2_i97 + 1) {
        c2_gd_u[c2_i97] = c2_cv24[c2_i97];
      }

      c2_qd_y = NULL;
      sf_mex_assign(&c2_qd_y, sf_mex_create("y", &c2_gd_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_hd_u = c2_TAU_K[2];
      c2_rd_y = NULL;
      sf_mex_assign(&c2_rd_y, sf_mex_create("y", &c2_hd_u, 0, 0U, 0U, 0U, 0));
      c2_id_u = c2_time_schedule[6];
      c2_sd_y = NULL;
      sf_mex_assign(&c2_sd_y, sf_mex_create("y", &c2_id_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_qd_y, 14, c2_rd_y, 14, c2_sd_y);
      goto label_6;
    }
  }

  CV_EML_MCDC(0, 2, FALSE);
  CV_EML_IF(0, 32, FALSE);
 label_6:
  ;
  _SFD_EML_CALL(0,556);
  if (CV_EML_COND(0, 6, c2_time_schedule[5] <= c2_ttStart_next + c2_deltaCAP)) {
    for (c2_i98 = 0; c2_i98 < 4; c2_i98 = c2_i98 + 1) {
      c2_p_lateIN[c2_i98] = (c2_lateIN[c2_i98] == 6.0);
    }

    if (!(CV_EML_COND(0, 7, c2_b_sum(chartInstance, c2_p_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 3, TRUE);
      CV_EML_IF(0, 33, TRUE);

      /*  control loops */
      /*  low priority loops */
      /*  we can set some conditions on the value of the measurement: */
      /*  e.g. if the temperature is higher than something then we sample */
      /*  faster! */
      _SFD_EML_CALL(0,562);
      c2_j_b = c2_b_rand(chartInstance);
      c2_td_y = c2_j_b;
      c2_next_time = 1.0 + c2_td_y;
      _SFD_EML_CALL(0,563);
      c2_time_schedule[5] = c2_time_schedule[5] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,564);
      c2_TAU_K[3] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,566);
      for (c2_i99 = 0; c2_i99 < 71; c2_i99 = c2_i99 + 1) {
        c2_jd_u[c2_i99] = c2_cv25[c2_i99];
      }

      c2_ud_y = NULL;
      sf_mex_assign(&c2_ud_y, sf_mex_create("y", &c2_jd_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_kd_u = c2_TAU_K[3];
      c2_vd_y = NULL;
      sf_mex_assign(&c2_vd_y, sf_mex_create("y", &c2_kd_u, 0, 0U, 0U, 0U, 0));
      c2_ld_u = c2_time_schedule[6];
      c2_wd_y = NULL;
      sf_mex_assign(&c2_wd_y, sf_mex_create("y", &c2_ld_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_ud_y, 14, c2_vd_y, 14, c2_wd_y);
      goto label_7;
    }
  }

  CV_EML_MCDC(0, 3, FALSE);
  CV_EML_IF(0, 33, FALSE);
 label_7:
  ;
  _SFD_EML_CALL(0,568);
  if (CV_EML_COND(0, 8, c2_time_schedule[6] <= c2_ttStart_next + c2_deltaCAP)) {
    for (c2_i100 = 0; c2_i100 < 4; c2_i100 = c2_i100 + 1) {
      c2_q_lateIN[c2_i100] = (c2_lateIN[c2_i100] == 7.0);
    }

    if (!(CV_EML_COND(0, 9, c2_b_sum(chartInstance, c2_q_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 4, TRUE);
      CV_EML_IF(0, 34, TRUE);

      /*  control loops */
      /*  low priority loops */
      /*  we can set some conditions on the value of the measurement: */
      /*  e.g. if the temperature is higher than something then we sample */
      /*  faster! */
      _SFD_EML_CALL(0,574);
      c2_k_b = c2_b_rand(chartInstance);
      c2_xd_y = c2_k_b;
      c2_next_time = 1.0 + c2_xd_y;
      _SFD_EML_CALL(0,575);
      c2_time_schedule[6] = c2_time_schedule[6] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,576);
      c2_TAU_K[4] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,578);
      for (c2_i101 = 0; c2_i101 < 71; c2_i101 = c2_i101 + 1) {
        c2_md_u[c2_i101] = c2_cv26[c2_i101];
      }

      c2_yd_y = NULL;
      sf_mex_assign(&c2_yd_y, sf_mex_create("y", &c2_md_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_nd_u = c2_TAU_K[4];
      c2_ae_y = NULL;
      sf_mex_assign(&c2_ae_y, sf_mex_create("y", &c2_nd_u, 0, 0U, 0U, 0U, 0));
      c2_od_u = c2_time_schedule[6];
      c2_be_y = NULL;
      sf_mex_assign(&c2_be_y, sf_mex_create("y", &c2_od_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_yd_y, 14, c2_ae_y, 14, c2_be_y);
      goto label_8;
    }
  }

  CV_EML_MCDC(0, 4, FALSE);
  CV_EML_IF(0, 34, FALSE);
 label_8:
  ;
  _SFD_EML_CALL(0,580);
  if (CV_EML_COND(0, 10, c2_time_schedule[7] <= c2_ttStart_next + c2_deltaCAP))
  {
    for (c2_i102 = 0; c2_i102 < 4; c2_i102 = c2_i102 + 1) {
      c2_r_lateIN[c2_i102] = (c2_lateIN[c2_i102] == 8.0);
    }

    if (!(CV_EML_COND(0, 11, c2_b_sum(chartInstance, c2_r_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 5, TRUE);
      CV_EML_IF(0, 35, TRUE);

      /*  low priority loops */
      _SFD_EML_CALL(0,582);
      c2_l_b = c2_b_rand(chartInstance);
      c2_ce_y = 8.9999999999999991E-01 * c2_l_b;
      c2_next_time = 1.0 + c2_ce_y;
      _SFD_EML_CALL(0,583);
      c2_time_schedule[7] = c2_time_schedule[7] + c2_next_time;

      /*  time_schedule stores the value of tk+1 */
      _SFD_EML_CALL(0,584);
      c2_TAU_K[5] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,586);
      for (c2_i103 = 0; c2_i103 < 71; c2_i103 = c2_i103 + 1) {
        c2_pd_u[c2_i103] = c2_cv27[c2_i103];
      }

      c2_de_y = NULL;
      sf_mex_assign(&c2_de_y, sf_mex_create("y", &c2_pd_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_qd_u = c2_TAU_K[5];
      c2_ee_y = NULL;
      sf_mex_assign(&c2_ee_y, sf_mex_create("y", &c2_qd_u, 0, 0U, 0U, 0U, 0));
      c2_rd_u = c2_time_schedule[7];
      c2_fe_y = NULL;
      sf_mex_assign(&c2_fe_y, sf_mex_create("y", &c2_rd_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_de_y, 14, c2_ee_y, 14, c2_fe_y);
      goto label_9;
    }
  }

  CV_EML_MCDC(0, 5, FALSE);
  CV_EML_IF(0, 35, FALSE);
 label_9:
  ;
  _SFD_EML_CALL(0,588);
  if (CV_EML_COND(0, 12, c2_time_schedule[8] <= c2_ttStart_next + c2_deltaCAP))
  {
    for (c2_i104 = 0; c2_i104 < 4; c2_i104 = c2_i104 + 1) {
      c2_s_lateIN[c2_i104] = (c2_lateIN[c2_i104] == 9.0);
    }

    if (!(CV_EML_COND(0, 13, c2_b_sum(chartInstance, c2_s_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 6, TRUE);
      CV_EML_IF(0, 36, TRUE);

      /*  low priority loops */
      _SFD_EML_CALL(0,590);
      c2_m_b = c2_b_rand(chartInstance);
      c2_ge_y = 1.5 * c2_m_b;
      c2_next_time = 1.0 + c2_ge_y;
      _SFD_EML_CALL(0,591);
      c2_time_schedule[8] = c2_time_schedule[8] + c2_next_time;
      _SFD_EML_CALL(0,592);
      c2_TAU_K[6] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,594);
      for (c2_i105 = 0; c2_i105 < 71; c2_i105 = c2_i105 + 1) {
        c2_sd_u[c2_i105] = c2_cv28[c2_i105];
      }

      c2_he_y = NULL;
      sf_mex_assign(&c2_he_y, sf_mex_create("y", &c2_sd_u, 10, 0U, 1U, 0U, 2, 1,
        71));
      c2_td_u = c2_TAU_K[6];
      c2_ie_y = NULL;
      sf_mex_assign(&c2_ie_y, sf_mex_create("y", &c2_td_u, 0, 0U, 0U, 0U, 0));
      c2_ud_u = c2_time_schedule[8];
      c2_je_y = NULL;
      sf_mex_assign(&c2_je_y, sf_mex_create("y", &c2_ud_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_he_y, 14, c2_ie_y, 14, c2_je_y);
      goto label_10;
    }
  }

  CV_EML_MCDC(0, 6, FALSE);
  CV_EML_IF(0, 36, FALSE);
 label_10:
  ;
  _SFD_EML_CALL(0,596);
  if (CV_EML_COND(0, 14, c2_time_schedule[9] <= c2_ttStart_next + c2_deltaCAP))
  {
    for (c2_i106 = 0; c2_i106 < 4; c2_i106 = c2_i106 + 1) {
      c2_t_lateIN[c2_i106] = (c2_lateIN[c2_i106] == 10.0);
    }

    if (!(CV_EML_COND(0, 15, c2_b_sum(chartInstance, c2_t_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 7, TRUE);
      CV_EML_IF(0, 37, TRUE);

      /*  low priority loops */
      _SFD_EML_CALL(0,598);
      c2_n_b = c2_b_rand(chartInstance);
      c2_ke_y = 1.5 * c2_n_b;
      c2_next_time = 1.0 + c2_ke_y;
      _SFD_EML_CALL(0,599);
      c2_time_schedule[9] = c2_time_schedule[9] + c2_next_time;
      _SFD_EML_CALL(0,600);
      c2_TAU_K[7] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,602);
      for (c2_i107 = 0; c2_i107 < 72; c2_i107 = c2_i107 + 1) {
        c2_vd_u[c2_i107] = c2_cv29[c2_i107];
      }

      c2_le_y = NULL;
      sf_mex_assign(&c2_le_y, sf_mex_create("y", &c2_vd_u, 10, 0U, 1U, 0U, 2, 1,
        72));
      c2_wd_u = c2_TAU_K[7];
      c2_me_y = NULL;
      sf_mex_assign(&c2_me_y, sf_mex_create("y", &c2_wd_u, 0, 0U, 0U, 0U, 0));
      c2_xd_u = c2_time_schedule[9];
      c2_ne_y = NULL;
      sf_mex_assign(&c2_ne_y, sf_mex_create("y", &c2_xd_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_le_y, 14, c2_me_y, 14, c2_ne_y);
      goto label_11;
    }
  }

  CV_EML_MCDC(0, 7, FALSE);
  CV_EML_IF(0, 37, FALSE);
 label_11:
  ;
  _SFD_EML_CALL(0,604);
  if (CV_EML_COND(0, 16, c2_time_schedule[10] <= c2_ttStart_next + c2_deltaCAP))
  {
    for (c2_i108 = 0; c2_i108 < 4; c2_i108 = c2_i108 + 1) {
      c2_u_lateIN[c2_i108] = (c2_lateIN[c2_i108] == 11.0);
    }

    if (!(CV_EML_COND(0, 17, c2_b_sum(chartInstance, c2_u_lateIN) != 0.0) != 0.0))
    {
      CV_EML_MCDC(0, 8, TRUE);
      CV_EML_IF(0, 38, TRUE);

      /*  low priority loops */
      _SFD_EML_CALL(0,606);
      c2_o_b = c2_b_rand(chartInstance);
      c2_oe_y = 2.5 * c2_o_b;
      c2_next_time = 1.0 + c2_oe_y;
      _SFD_EML_CALL(0,607);
      c2_time_schedule[10] = c2_time_schedule[10] + c2_next_time;
      _SFD_EML_CALL(0,608);
      c2_TAU_K[8] = c2_next_time;

      /*  store all the periods */
      _SFD_EML_CALL(0,610);
      for (c2_i109 = 0; c2_i109 < 72; c2_i109 = c2_i109 + 1) {
        c2_yd_u[c2_i109] = c2_cv30[c2_i109];
      }

      c2_pe_y = NULL;
      sf_mex_assign(&c2_pe_y, sf_mex_create("y", &c2_yd_u, 10, 0U, 1U, 0U, 2, 1,
        72));
      c2_ae_u = c2_TAU_K[8];
      c2_qe_y = NULL;
      sf_mex_assign(&c2_qe_y, sf_mex_create("y", &c2_ae_u, 0, 0U, 0U, 0U, 0));
      c2_be_u = c2_time_schedule[10];
      c2_re_y = NULL;
      sf_mex_assign(&c2_re_y, sf_mex_create("y", &c2_be_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_pe_y, 14, c2_qe_y, 14, c2_re_y);
      goto label_12;
    }
  }

  CV_EML_MCDC(0, 8, FALSE);
  CV_EML_IF(0, 38, FALSE);
 label_12:
  ;
  _SFD_EML_CALL(0,614);
  c2_BO = 6.0;

  /*  Fixed BO!!! */
  /* % Scheduler */
  /*  2) if the node has a tk+1, then we check if it is between the next */
  /*  superframe which is ttStart_next+deltaCAP to ttStart_next+deltaBI */
  /*  2.a) if this is true, store the node in a new vector */
  /*  scheduled_next=[nodeID;time]; */
  _SFD_EML_CALL(0,622);
  c2_tBI = c2_BI[2];
  _SFD_EML_CALL(0,623);
  c2_j = 1.0;
  _SFD_EML_CALL(0,625);
  for (c2_i110 = 0; c2_i110 < 73; c2_i110 = c2_i110 + 1) {
    c2_ce_u[c2_i110] = c2_cv31[c2_i110];
  }

  c2_se_y = NULL;
  sf_mex_assign(&c2_se_y, sf_mex_create("y", &c2_ce_u, 10, 0U, 1U, 0U, 2, 1, 73));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_se_y);
  _SFD_EML_CALL(0,626);
  for (c2_i111 = 0; c2_i111 < 57; c2_i111 = c2_i111 + 1) {
    c2_de_u[c2_i111] = c2_cv32[c2_i111];
  }

  c2_te_y = NULL;
  sf_mex_assign(&c2_te_y, sf_mex_create("y", &c2_de_u, 10, 0U, 1U, 0U, 2, 1, 57));
  c2_ee_u = c2_ttStart_next;
  c2_ue_y = NULL;
  sf_mex_assign(&c2_ue_y, sf_mex_create("y", &c2_ee_u, 0, 0U, 0U, 0U, 0));
  c2_fe_u = c2_tBI;
  c2_ve_y = NULL;
  sf_mex_assign(&c2_ve_y, sf_mex_create("y", &c2_fe_u, 0, 0U, 0U, 0U, 0));
  c2_ge_u = c2_deltaCAP;
  c2_we_y = NULL;
  sf_mex_assign(&c2_we_y, sf_mex_create("y", &c2_ge_u, 0, 0U, 0U, 0U, 0));
  c2_he_u = (c2_ttStart_next + c2_tBI) + c2_deltaCAP;
  c2_xe_y = NULL;
  sf_mex_assign(&c2_xe_y, sf_mex_create("y", &c2_he_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 5U, 14, c2_te_y, 14, c2_ue_y, 14, c2_ve_y, 14,
                    c2_we_y, 14, c2_xe_y);
  _SFD_EML_CALL(0,627);
  for (c2_i112 = 0; c2_i112 < 41; c2_i112 = c2_i112 + 1) {
    c2_ie_u[c2_i112] = c2_cv33[c2_i112];
  }

  c2_ye_y = NULL;
  sf_mex_assign(&c2_ye_y, sf_mex_create("y", &c2_ie_u, 10, 0U, 1U, 0U, 2, 1, 41));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_ye_y);
  _SFD_EML_CALL(0,628);
  for (c2_i113 = 0; c2_i113 < 34; c2_i113 = c2_i113 + 1) {
    c2_je_u[c2_i113] = c2_cv34[c2_i113];
  }

  c2_af_y = NULL;
  sf_mex_assign(&c2_af_y, sf_mex_create("y", &c2_je_u, 10, 0U, 1U, 0U, 2, 1, 34));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_af_y);
  _SFD_EML_CALL(0,629);
  for (c2_i114 = 0; c2_i114 < 10; c2_i114 = c2_i114 + 1) {
    c2_ke_u[c2_i114] = c2_cv4[c2_i114];
  }

  c2_bf_y = NULL;
  sf_mex_assign(&c2_bf_y, sf_mex_create("y", &c2_ke_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_le_u = c2_time_schedule[0];
  c2_cf_y = NULL;
  sf_mex_assign(&c2_cf_y, sf_mex_create("y", &c2_le_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_bf_y, 14, c2_cf_y);
  _SFD_EML_CALL(0,630);
  if (CV_EML_IF(0, 39, c2_time_schedule[0] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,631);
    for (c2_i115 = 0; c2_i115 < 4; c2_i115 = c2_i115 + 1) {
      c2_me_u[c2_i115] = c2_cv6[c2_i115];
    }

    c2_df_y = NULL;
    sf_mex_assign(&c2_df_y, sf_mex_create("y", &c2_me_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_df_y);
  }

  _SFD_EML_CALL(0,633);
  for (c2_i116 = 0; c2_i116 < 2; c2_i116 = c2_i116 + 1) {
    c2_ne_u[c2_i116] = c2_cv7[c2_i116];
  }

  c2_ef_y = NULL;
  sf_mex_assign(&c2_ef_y, sf_mex_create("y", &c2_ne_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_ef_y);
  _SFD_EML_CALL(0,634);
  for (c2_i117 = 0; c2_i117 < 10; c2_i117 = c2_i117 + 1) {
    c2_oe_u[c2_i117] = c2_cv8[c2_i117];
  }

  c2_ff_y = NULL;
  sf_mex_assign(&c2_ff_y, sf_mex_create("y", &c2_oe_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_pe_u = c2_time_schedule[1];
  c2_gf_y = NULL;
  sf_mex_assign(&c2_gf_y, sf_mex_create("y", &c2_pe_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ff_y, 14, c2_gf_y);
  _SFD_EML_CALL(0,635);
  if (CV_EML_IF(0, 40, c2_time_schedule[1] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,636);
    for (c2_i118 = 0; c2_i118 < 4; c2_i118 = c2_i118 + 1) {
      c2_qe_u[c2_i118] = c2_cv6[c2_i118];
    }

    c2_hf_y = NULL;
    sf_mex_assign(&c2_hf_y, sf_mex_create("y", &c2_qe_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_hf_y);
  }

  _SFD_EML_CALL(0,638);
  for (c2_i119 = 0; c2_i119 < 2; c2_i119 = c2_i119 + 1) {
    c2_re_u[c2_i119] = c2_cv7[c2_i119];
  }

  c2_if_y = NULL;
  sf_mex_assign(&c2_if_y, sf_mex_create("y", &c2_re_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_if_y);
  _SFD_EML_CALL(0,639);
  for (c2_i120 = 0; c2_i120 < 10; c2_i120 = c2_i120 + 1) {
    c2_se_u[c2_i120] = c2_cv9[c2_i120];
  }

  c2_jf_y = NULL;
  sf_mex_assign(&c2_jf_y, sf_mex_create("y", &c2_se_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_te_u = c2_time_schedule[2];
  c2_kf_y = NULL;
  sf_mex_assign(&c2_kf_y, sf_mex_create("y", &c2_te_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_jf_y, 14, c2_kf_y);
  _SFD_EML_CALL(0,640);
  if (CV_EML_IF(0, 41, c2_time_schedule[2] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,641);
    for (c2_i121 = 0; c2_i121 < 4; c2_i121 = c2_i121 + 1) {
      c2_ue_u[c2_i121] = c2_cv6[c2_i121];
    }

    c2_lf_y = NULL;
    sf_mex_assign(&c2_lf_y, sf_mex_create("y", &c2_ue_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_lf_y);
  }

  _SFD_EML_CALL(0,643);
  for (c2_i122 = 0; c2_i122 < 2; c2_i122 = c2_i122 + 1) {
    c2_ve_u[c2_i122] = c2_cv7[c2_i122];
  }

  c2_mf_y = NULL;
  sf_mex_assign(&c2_mf_y, sf_mex_create("y", &c2_ve_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_mf_y);
  _SFD_EML_CALL(0,644);
  for (c2_i123 = 0; c2_i123 < 10; c2_i123 = c2_i123 + 1) {
    c2_we_u[c2_i123] = c2_cv10[c2_i123];
  }

  c2_nf_y = NULL;
  sf_mex_assign(&c2_nf_y, sf_mex_create("y", &c2_we_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_xe_u = c2_time_schedule[3];
  c2_of_y = NULL;
  sf_mex_assign(&c2_of_y, sf_mex_create("y", &c2_xe_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_nf_y, 14, c2_of_y);
  _SFD_EML_CALL(0,645);
  if (CV_EML_IF(0, 42, c2_time_schedule[3] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,646);
    for (c2_i124 = 0; c2_i124 < 4; c2_i124 = c2_i124 + 1) {
      c2_ye_u[c2_i124] = c2_cv6[c2_i124];
    }

    c2_pf_y = NULL;
    sf_mex_assign(&c2_pf_y, sf_mex_create("y", &c2_ye_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_pf_y);
  }

  _SFD_EML_CALL(0,648);
  for (c2_i125 = 0; c2_i125 < 2; c2_i125 = c2_i125 + 1) {
    c2_af_u[c2_i125] = c2_cv7[c2_i125];
  }

  c2_qf_y = NULL;
  sf_mex_assign(&c2_qf_y, sf_mex_create("y", &c2_af_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_qf_y);
  _SFD_EML_CALL(0,649);
  for (c2_i126 = 0; c2_i126 < 10; c2_i126 = c2_i126 + 1) {
    c2_bf_u[c2_i126] = c2_cv11[c2_i126];
  }

  c2_rf_y = NULL;
  sf_mex_assign(&c2_rf_y, sf_mex_create("y", &c2_bf_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_cf_u = c2_time_schedule[4];
  c2_sf_y = NULL;
  sf_mex_assign(&c2_sf_y, sf_mex_create("y", &c2_cf_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_rf_y, 14, c2_sf_y);
  _SFD_EML_CALL(0,650);
  if (CV_EML_IF(0, 43, c2_time_schedule[4] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,651);
    for (c2_i127 = 0; c2_i127 < 4; c2_i127 = c2_i127 + 1) {
      c2_df_u[c2_i127] = c2_cv6[c2_i127];
    }

    c2_tf_y = NULL;
    sf_mex_assign(&c2_tf_y, sf_mex_create("y", &c2_df_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_tf_y);
  }

  _SFD_EML_CALL(0,653);
  for (c2_i128 = 0; c2_i128 < 2; c2_i128 = c2_i128 + 1) {
    c2_ef_u[c2_i128] = c2_cv7[c2_i128];
  }

  c2_uf_y = NULL;
  sf_mex_assign(&c2_uf_y, sf_mex_create("y", &c2_ef_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_uf_y);
  _SFD_EML_CALL(0,654);
  for (c2_i129 = 0; c2_i129 < 10; c2_i129 = c2_i129 + 1) {
    c2_ff_u[c2_i129] = c2_cv12[c2_i129];
  }

  c2_vf_y = NULL;
  sf_mex_assign(&c2_vf_y, sf_mex_create("y", &c2_ff_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_gf_u = c2_time_schedule[5];
  c2_wf_y = NULL;
  sf_mex_assign(&c2_wf_y, sf_mex_create("y", &c2_gf_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_vf_y, 14, c2_wf_y);
  _SFD_EML_CALL(0,655);
  if (CV_EML_IF(0, 44, c2_time_schedule[5] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,656);
    for (c2_i130 = 0; c2_i130 < 4; c2_i130 = c2_i130 + 1) {
      c2_hf_u[c2_i130] = c2_cv6[c2_i130];
    }

    c2_xf_y = NULL;
    sf_mex_assign(&c2_xf_y, sf_mex_create("y", &c2_hf_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_xf_y);
  }

  _SFD_EML_CALL(0,658);
  for (c2_i131 = 0; c2_i131 < 2; c2_i131 = c2_i131 + 1) {
    c2_if_u[c2_i131] = c2_cv7[c2_i131];
  }

  c2_yf_y = NULL;
  sf_mex_assign(&c2_yf_y, sf_mex_create("y", &c2_if_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_yf_y);
  _SFD_EML_CALL(0,659);
  for (c2_i132 = 0; c2_i132 < 10; c2_i132 = c2_i132 + 1) {
    c2_jf_u[c2_i132] = c2_cv13[c2_i132];
  }

  c2_ag_y = NULL;
  sf_mex_assign(&c2_ag_y, sf_mex_create("y", &c2_jf_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_kf_u = c2_time_schedule[6];
  c2_bg_y = NULL;
  sf_mex_assign(&c2_bg_y, sf_mex_create("y", &c2_kf_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ag_y, 14, c2_bg_y);
  _SFD_EML_CALL(0,660);
  if (CV_EML_IF(0, 45, c2_time_schedule[6] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,661);
    for (c2_i133 = 0; c2_i133 < 4; c2_i133 = c2_i133 + 1) {
      c2_lf_u[c2_i133] = c2_cv6[c2_i133];
    }

    c2_cg_y = NULL;
    sf_mex_assign(&c2_cg_y, sf_mex_create("y", &c2_lf_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_cg_y);
  }

  _SFD_EML_CALL(0,663);
  for (c2_i134 = 0; c2_i134 < 2; c2_i134 = c2_i134 + 1) {
    c2_mf_u[c2_i134] = c2_cv7[c2_i134];
  }

  c2_dg_y = NULL;
  sf_mex_assign(&c2_dg_y, sf_mex_create("y", &c2_mf_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_dg_y);
  _SFD_EML_CALL(0,664);
  for (c2_i135 = 0; c2_i135 < 10; c2_i135 = c2_i135 + 1) {
    c2_nf_u[c2_i135] = c2_cv14[c2_i135];
  }

  c2_eg_y = NULL;
  sf_mex_assign(&c2_eg_y, sf_mex_create("y", &c2_nf_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_of_u = c2_time_schedule[7];
  c2_fg_y = NULL;
  sf_mex_assign(&c2_fg_y, sf_mex_create("y", &c2_of_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_eg_y, 14, c2_fg_y);
  _SFD_EML_CALL(0,665);
  if (CV_EML_IF(0, 46, c2_time_schedule[7] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,666);
    for (c2_i136 = 0; c2_i136 < 4; c2_i136 = c2_i136 + 1) {
      c2_pf_u[c2_i136] = c2_cv6[c2_i136];
    }

    c2_gg_y = NULL;
    sf_mex_assign(&c2_gg_y, sf_mex_create("y", &c2_pf_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_gg_y);
  }

  _SFD_EML_CALL(0,668);
  for (c2_i137 = 0; c2_i137 < 2; c2_i137 = c2_i137 + 1) {
    c2_qf_u[c2_i137] = c2_cv7[c2_i137];
  }

  c2_hg_y = NULL;
  sf_mex_assign(&c2_hg_y, sf_mex_create("y", &c2_qf_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_hg_y);
  _SFD_EML_CALL(0,669);
  for (c2_i138 = 0; c2_i138 < 10; c2_i138 = c2_i138 + 1) {
    c2_rf_u[c2_i138] = c2_cv15[c2_i138];
  }

  c2_ig_y = NULL;
  sf_mex_assign(&c2_ig_y, sf_mex_create("y", &c2_rf_u, 10, 0U, 1U, 0U, 2, 1, 10));
  c2_sf_u = c2_time_schedule[8];
  c2_jg_y = NULL;
  sf_mex_assign(&c2_jg_y, sf_mex_create("y", &c2_sf_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ig_y, 14, c2_jg_y);
  _SFD_EML_CALL(0,670);
  if (CV_EML_IF(0, 47, c2_time_schedule[8] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,671);
    for (c2_i139 = 0; c2_i139 < 4; c2_i139 = c2_i139 + 1) {
      c2_tf_u[c2_i139] = c2_cv6[c2_i139];
    }

    c2_kg_y = NULL;
    sf_mex_assign(&c2_kg_y, sf_mex_create("y", &c2_tf_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_kg_y);
  }

  _SFD_EML_CALL(0,673);
  for (c2_i140 = 0; c2_i140 < 2; c2_i140 = c2_i140 + 1) {
    c2_uf_u[c2_i140] = c2_cv7[c2_i140];
  }

  c2_lg_y = NULL;
  sf_mex_assign(&c2_lg_y, sf_mex_create("y", &c2_uf_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_lg_y);
  _SFD_EML_CALL(0,674);
  for (c2_i141 = 0; c2_i141 < 11; c2_i141 = c2_i141 + 1) {
    c2_vf_u[c2_i141] = c2_cv16[c2_i141];
  }

  c2_mg_y = NULL;
  sf_mex_assign(&c2_mg_y, sf_mex_create("y", &c2_vf_u, 10, 0U, 1U, 0U, 2, 1, 11));
  c2_wf_u = c2_time_schedule[9];
  c2_ng_y = NULL;
  sf_mex_assign(&c2_ng_y, sf_mex_create("y", &c2_wf_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_mg_y, 14, c2_ng_y);
  _SFD_EML_CALL(0,675);
  if (CV_EML_IF(0, 48, c2_time_schedule[9] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,676);
    for (c2_i142 = 0; c2_i142 < 4; c2_i142 = c2_i142 + 1) {
      c2_xf_u[c2_i142] = c2_cv6[c2_i142];
    }

    c2_og_y = NULL;
    sf_mex_assign(&c2_og_y, sf_mex_create("y", &c2_xf_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_og_y);
  }

  _SFD_EML_CALL(0,678);
  for (c2_i143 = 0; c2_i143 < 2; c2_i143 = c2_i143 + 1) {
    c2_yf_u[c2_i143] = c2_cv7[c2_i143];
  }

  c2_pg_y = NULL;
  sf_mex_assign(&c2_pg_y, sf_mex_create("y", &c2_yf_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_pg_y);
  _SFD_EML_CALL(0,679);
  for (c2_i144 = 0; c2_i144 < 11; c2_i144 = c2_i144 + 1) {
    c2_ag_u[c2_i144] = c2_cv17[c2_i144];
  }

  c2_qg_y = NULL;
  sf_mex_assign(&c2_qg_y, sf_mex_create("y", &c2_ag_u, 10, 0U, 1U, 0U, 2, 1, 11));
  c2_bg_u = c2_time_schedule[10];
  c2_rg_y = NULL;
  sf_mex_assign(&c2_rg_y, sf_mex_create("y", &c2_bg_u, 0, 0U, 0U, 0U, 0));
  sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_qg_y, 14, c2_rg_y);
  _SFD_EML_CALL(0,680);
  if (CV_EML_IF(0, 49, c2_time_schedule[10] <= (c2_ttStart_next + c2_tBI) +
                c2_deltaCAP)) {
    _SFD_EML_CALL(0,681);
    for (c2_i145 = 0; c2_i145 < 4; c2_i145 = c2_i145 + 1) {
      c2_cg_u[c2_i145] = c2_cv6[c2_i145];
    }

    c2_sg_y = NULL;
    sf_mex_assign(&c2_sg_y, sf_mex_create("y", &c2_cg_u, 10, 0U, 1U, 0U, 2, 1, 4));
    sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_sg_y);
  }

  _SFD_EML_CALL(0,683);
  for (c2_i146 = 0; c2_i146 < 2; c2_i146 = c2_i146 + 1) {
    c2_dg_u[c2_i146] = c2_cv7[c2_i146];
  }

  c2_tg_y = NULL;
  sf_mex_assign(&c2_tg_y, sf_mex_create("y", &c2_dg_u, 10, 0U, 1U, 0U, 2, 1, 2));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_tg_y);
  c2_i = 1.0;
  c2_c_i = 1.0;
  while (c2_c_i <= 11.0) {
    c2_i = c2_c_i;
    CV_EML_FOR(0, 3, 1);
    _SFD_EML_CALL(0,685);
    if (CV_EML_IF(0, 50, c2_time_schedule[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "time_schedule", (int32_T)_SFD_INTEGER_CHECK("i", c2_i), 1, 11, 1,
          0) - 1] <= (c2_ttStart_next + c2_tBI) + c2_deltaCAP)) {
      /* && time_schedule(i)>=ttStart_next+deltaCAP */
      _SFD_EML_CALL(0,686);
      for (c2_i147 = 0; c2_i147 < 60; c2_i147 = c2_i147 + 1) {
        c2_eg_u[c2_i147] = c2_cv35[c2_i147];
      }

      c2_ug_y = NULL;
      sf_mex_assign(&c2_ug_y, sf_mex_create("y", &c2_eg_u, 10, 0U, 1U, 0U, 2, 1,
        60));
      c2_fg_u = c2_i;
      c2_vg_y = NULL;
      sf_mex_assign(&c2_vg_y, sf_mex_create("y", &c2_fg_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_ug_y, 14, c2_vg_y);
      _SFD_EML_CALL(0,687);
      c2_scheduled_next[(_SFD_EML_ARRAY_BOUNDS_CHECK("scheduled_next", (int32_T)
        _SFD_INTEGER_CHECK("j", c2_j), 1, 11, 2, 0) - 1) << 1] =
        c2_i;
      _SFD_EML_CALL(0,688);
      c2_scheduled_next[1 + ((_SFD_EML_ARRAY_BOUNDS_CHECK("scheduled_next",
        (int32_T)_SFD_INTEGER_CHECK("j", c2_j), 1, 11, 2, 0) - 1) << 1
        )] = c2_time_schedule[_SFD_EML_ARRAY_BOUNDS_CHECK("time_schedule",
        (int32_T)_SFD_INTEGER_CHECK("i", c2_i), 1, 11, 1, 0) - 1];

      /*  this vector contains the time of nodes scheduled for next superframe */
      _SFD_EML_CALL(0,689);
      c2_j = c2_j + 1.0;
    }

    c2_c_i = c2_c_i + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 3, 0);

  /*  if j==1 % In case no one got a slot we should allow for a bigger BO */
  /*      BO=BO+1; */
  /*      tBI=BI(BO-3); */
  /*      disp('No one got slot, make BO bigger'); */
  /*      j=1; */
  /*      for i=1:7 */
  /*          if time_schedule(i)<=ttStart_next+tBI+deltaCAP %&& time_schedule(i)>=ttStart_next+deltaCAP */
  /*              scheduled_next(1,j)=i; */
  /*              scheduled_next(2,j)=time_schedule(i);% this vector contains the time of nodes scheduled for next superframe */
  /*              j=j+1; */
  /*          end */
  /*      end */
  /*  end */
  /*  3) After step 2 is run for all i, align the times of scheduled_next in */
  /*  order to get the Earliest deadline sensor first */
  _SFD_EML_CALL(0,709);
  c2_i148 = 0;
  for (c2_i149 = 0; c2_i149 < 11; c2_i149 = c2_i149 + 1) {
    c2_b_scheduled_next[c2_i149] = c2_scheduled_next[c2_i148 + 1];
    c2_i148 = c2_i148 + 2;
  }

  c2_sort(chartInstance, c2_b_scheduled_next, c2_b_B, c2_b_pos);
  for (c2_i150 = 0; c2_i150 < 11; c2_i150 = c2_i150 + 1) {
    c2_B[c2_i150] = c2_b_B[c2_i150];
  }

  for (c2_i151 = 0; c2_i151 < 11; c2_i151 = c2_i151 + 1) {
    c2_pos[c2_i151] = c2_b_pos[c2_i151];
  }

  /*  4) Assign priorities to all the nodes in scheduled_next and assign the slots in a separate */
  /*  vector slots=[Tk, etc]. slots has size 7. Assign also the actuator nodes */
  /*  IF they are a control loop */
  /*  pos_1=0; */
  /*  pos_3=0; */
  /*  fprintf('--\n'); */
  /*          for i=1:size(pos,2) */
  /*              fprintf('i=%u: pos(i)=%u  node=%u \n', i, pos(i), scheduled_next(1,pos(i))); */
  /*          end */
  /*  for i=1:size(pos,2) */
  /*      if scheduled_next(1,pos(i))==1 && scheduled_next(1,pos(i+1))~=2 */
  /*          pos_1=i; */
  /*      end */
  /*      if pos_1~=0 && scheduled_next(1,pos(i))==2 */
  /*          temp = pos(i); */
  /*          pos(i) = pos(pos_1+1); */
  /*          pos(pos_1+1) = temp; */
  /*      end */
  /*      if scheduled_next(1,pos(i))==3 && scheduled_next(1,pos(i+1))~=4 */
  /*          pos_3=i; */
  /*      end */
  /*      if pos_3~=0 && scheduled_next(1,pos(i))==4 */
  /*          temp = pos(i); */
  /*          pos(i) = pos(pos_3+1); */
  /*          pos(pos_3+1) = temp; */
  /*      end */
  /*  end */
  /*   */
  /*  % Enforce priority of tank nodes over sensor nodes! */
  /*  if size(pos,2) > 7 */
  /*      while 1 */
  /*          found=0; */
  /*          fprintf('--\n'); */
  /*          for i=1:size(pos,2) */
  /*              fprintf('i=%u: pos(i)=%u  node=%u \n', i, pos(i), scheduled_next(1,pos(i))); */
  /*          end */
  /*          for i=7:size(pos,2) */
  /*              if scheduled_next(1,pos(i))==1 */
  /*                  found=1; */
  /*                  for k=6:-1:1 */
  /*                      if scheduled_next(1,pos(k))~=3 && scheduled_next(1,pos(k))~=4 && scheduled_next(1,pos(k+1))~=3 */
  /*                          temp = pos(i); */
  /*                          pos(i) = pos(k); */
  /*                          pos(i+1) = pos(k+1); */
  /*                          pos(k) = temp; */
  /*                          pos(k+1) = temp+1; */
  /*                          fprintf('Tank 1s node took over!!!!!!!!!!!!!!!!!!!!!!!!!!\n'); */
  /*                          break; */
  /*                      end */
  /*                  end */
  /*              end */
  /*              if scheduled_next(1,pos(i))==3 */
  /*                  found=1; */
  /*                  for k=6:-1:1 */
  /*                      if scheduled_next(1,pos(k))~=1 && scheduled_next(1,pos(k))~=2 && scheduled_next(1,pos(k+1))~=1 */
  /*                          temp = pos(i); */
  /*                          pos(i) = pos(k); */
  /*                          pos(i+1) = pos(k+1); */
  /*                          pos(k) = temp; */
  /*                          pos(k+1) = temp+1; */
  /*                          fprintf('Tank 2s node took over!!!!!!!!!!!!!!!!!!!!!!!!!!\n'); */
  /*                          break; */
  /*                      end */
  /*                  end */
  /*              end */
  /*          end */
  /*          if found==0 */
  /*              break; */
  /*          end */
  /*      end */
  /*  end */
  /*  Assign slots in EDF fashion, Note: the actuator needs to be assigned */
  /*  after the measurement */
  _SFD_EML_CALL(0,787);
  for (c2_i152 = 0; c2_i152 < 24; c2_i152 = c2_i152 + 1) {
    c2_gg_u[c2_i152] = c2_cv36[c2_i152];
  }

  c2_wg_y = NULL;
  sf_mex_assign(&c2_wg_y, sf_mex_create("y", &c2_gg_u, 10, 0U, 1U, 0U, 2, 1, 24));
  sf_mex_call_debug("fprintf", 0U, 1U, 14, c2_wg_y);
  _SFD_EML_CALL(0,788);
  c2_slot_num = 1.0;
  _SFD_EML_CALL(0,789);
  c2_l = 1.0;
  c2_i = 1.0;
  c2_loop_ub = c2_j - 1.0;
  c2_d_i = 1.0;
  while (c2_d_i <= c2_loop_ub) {
    c2_i = c2_d_i;
    CV_EML_FOR(0, 4, 1);

    /*  we can also write slot=pos */
    _SFD_EML_CALL(0,791);
    c2_node = c2_scheduled_next[(_SFD_EML_ARRAY_BOUNDS_CHECK("scheduled_next",
      (int32_T)_SFD_INTEGER_CHECK("", c2_pos[
      _SFD_EML_ARRAY_BOUNDS_CHECK("pos", (int32_T)_SFD_INTEGER_CHECK("i", c2_i),
      1, 11, 1, 0) - 1]), 1, 11, 2, 0) - 1) << 1];
    _SFD_EML_CALL(0,792);
    if (CV_EML_IF(0, 51, c2_node == 1.0)) {
      _SFD_EML_CALL(0,793);
      c2_slot[0] = c2_slot_num;

      /*  slot stores the assigned slots */
      _SFD_EML_CALL(0,794);
      c2_k_a = c2_slot_num - 1.0;
      c2_xg_y = c2_k_a * 1920.0;
      c2_l_a = c2_xg_y;
      c2_yg_y = c2_l_a * 1.5259E-05;
      c2_time_schedule[0] = (c2_ttStart_next + c2_deltaCAP) + c2_yg_y;
      _SFD_EML_CALL(0,795);
      c2_slot[1] = c2_slot_num + 1.0;
      _SFD_EML_CALL(0,796);
      c2_m_a = c2_slot_num;
      c2_ah_y = c2_m_a * 1920.0;
      c2_n_a = c2_ah_y;
      c2_bh_y = c2_n_a * 1.5259E-05;
      c2_time_schedule[1] = (c2_ttStart_next + c2_deltaCAP) + c2_bh_y;
      _SFD_EML_CALL(0,797);
      for (c2_i153 = 0; c2_i153 < 26; c2_i153 = c2_i153 + 1) {
        c2_hg_u[c2_i153] = c2_cv37[c2_i153];
      }

      c2_ch_y = NULL;
      sf_mex_assign(&c2_ch_y, sf_mex_create("y", &c2_hg_u, 10, 0U, 1U, 0U, 2, 1,
        26));
      c2_ig_u = c2_node;
      c2_dh_y = NULL;
      sf_mex_assign(&c2_dh_y, sf_mex_create("y", &c2_ig_u, 0, 0U, 0U, 0U, 0));
      c2_jg_u = c2_slot_num;
      c2_eh_y = NULL;
      sf_mex_assign(&c2_eh_y, sf_mex_create("y", &c2_jg_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_ch_y, 14, c2_dh_y, 14, c2_eh_y);
      _SFD_EML_CALL(0,798);
      for (c2_i154 = 0; c2_i154 < 26; c2_i154 = c2_i154 + 1) {
        c2_kg_u[c2_i154] = c2_cv37[c2_i154];
      }

      c2_fh_y = NULL;
      sf_mex_assign(&c2_fh_y, sf_mex_create("y", &c2_kg_u, 10, 0U, 1U, 0U, 2, 1,
        26));
      c2_lg_u = c2_node + 1.0;
      c2_gh_y = NULL;
      sf_mex_assign(&c2_gh_y, sf_mex_create("y", &c2_lg_u, 0, 0U, 0U, 0U, 0));
      c2_mg_u = c2_slot_num + 1.0;
      c2_hh_y = NULL;
      sf_mex_assign(&c2_hh_y, sf_mex_create("y", &c2_mg_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_fh_y, 14, c2_gh_y, 14, c2_hh_y);
      _SFD_EML_CALL(0,799);
      c2_slot_num = c2_slot_num + 2.0;
    }

    _SFD_EML_CALL(0,801);
    if (CV_EML_IF(0, 52, c2_node == 3.0)) {
      _SFD_EML_CALL(0,802);
      c2_slot[2] = c2_slot_num;

      /*  slot stores the assigned slots */
      _SFD_EML_CALL(0,803);
      c2_o_a = c2_slot_num - 1.0;
      c2_ih_y = c2_o_a * 1920.0;
      c2_p_a = c2_ih_y;
      c2_jh_y = c2_p_a * 1.5259E-05;
      c2_time_schedule[2] = (c2_ttStart_next + c2_deltaCAP) + c2_jh_y;
      _SFD_EML_CALL(0,804);
      c2_slot[3] = c2_slot_num + 1.0;
      _SFD_EML_CALL(0,805);
      c2_q_a = c2_slot_num;
      c2_kh_y = c2_q_a * 1920.0;
      c2_r_a = c2_kh_y;
      c2_lh_y = c2_r_a * 1.5259E-05;
      c2_time_schedule[3] = (c2_ttStart_next + c2_deltaCAP) + c2_lh_y;
      _SFD_EML_CALL(0,806);
      for (c2_i155 = 0; c2_i155 < 26; c2_i155 = c2_i155 + 1) {
        c2_ng_u[c2_i155] = c2_cv37[c2_i155];
      }

      c2_mh_y = NULL;
      sf_mex_assign(&c2_mh_y, sf_mex_create("y", &c2_ng_u, 10, 0U, 1U, 0U, 2, 1,
        26));
      c2_og_u = c2_node;
      c2_nh_y = NULL;
      sf_mex_assign(&c2_nh_y, sf_mex_create("y", &c2_og_u, 0, 0U, 0U, 0U, 0));
      c2_pg_u = c2_slot_num;
      c2_oh_y = NULL;
      sf_mex_assign(&c2_oh_y, sf_mex_create("y", &c2_pg_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_mh_y, 14, c2_nh_y, 14, c2_oh_y);
      _SFD_EML_CALL(0,807);
      for (c2_i156 = 0; c2_i156 < 26; c2_i156 = c2_i156 + 1) {
        c2_qg_u[c2_i156] = c2_cv37[c2_i156];
      }

      c2_ph_y = NULL;
      sf_mex_assign(&c2_ph_y, sf_mex_create("y", &c2_qg_u, 10, 0U, 1U, 0U, 2, 1,
        26));
      c2_rg_u = c2_node + 1.0;
      c2_qh_y = NULL;
      sf_mex_assign(&c2_qh_y, sf_mex_create("y", &c2_rg_u, 0, 0U, 0U, 0U, 0));
      c2_sg_u = c2_slot_num + 1.0;
      c2_rh_y = NULL;
      sf_mex_assign(&c2_rh_y, sf_mex_create("y", &c2_sg_u, 0, 0U, 0U, 0U, 0));
      sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_ph_y, 14, c2_qh_y, 14, c2_rh_y);
      _SFD_EML_CALL(0,808);
      c2_slot_num = c2_slot_num + 2.0;
    }

    c2_d_i = c2_d_i + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 4, 0);
  _SFD_EML_CALL(0,812);
  c2_l = 1.0;
  c2_i = 1.0;
  c2_b_loop_ub = c2_j - 1.0;
  c2_e_i = 1.0;
  while (c2_e_i <= c2_b_loop_ub) {
    c2_i = c2_e_i;
    CV_EML_FOR(0, 5, 1);

    /*  we can also write slot=pos */
    _SFD_EML_CALL(0,814);
    c2_node = c2_scheduled_next[(_SFD_EML_ARRAY_BOUNDS_CHECK("scheduled_next",
      (int32_T)_SFD_INTEGER_CHECK("", c2_pos[
      _SFD_EML_ARRAY_BOUNDS_CHECK("pos", (int32_T)_SFD_INTEGER_CHECK("i", c2_i),
      1, 11, 1, 0) - 1]), 1, 11, 2, 0) - 1) << 1];
    _SFD_EML_CALL(0,815);
    if (CV_EML_COND(0, 18, c2_node != 1.0)) {
      if (CV_EML_COND(0, 19, c2_node != 2.0)) {
        if (CV_EML_COND(0, 20, c2_node != 3.0)) {
          if (CV_EML_COND(0, 21, c2_node != 4.0)) {
            CV_EML_MCDC(0, 9, TRUE);
            CV_EML_IF(0, 53, TRUE);
            _SFD_EML_CALL(0,816);
            if (CV_EML_IF(0, 54, c2_slot_num <= 7.0)) {
              _SFD_EML_CALL(0,817);
              if (CV_EML_COND(0, 22, c2_node == 5.0)) {
              } else if (CV_EML_COND(0, 23, c2_node == 6.0)) {
              } else if (CV_EML_COND(0, 24, c2_node == 7.0)) {
                goto label_13;
              } else if (CV_EML_COND(0, 25, c2_node == 8.0)) {
                goto label_14;
              } else if (CV_EML_COND(0, 26, c2_node == 9.0)) {
                goto label_15;
              } else if (CV_EML_COND(0, 27, c2_node == 10.0)) {
                goto label_16;
              } else if (CV_EML_COND(0, 28, c2_node == 11.0)) {
                goto label_17;
              } else {
                CV_EML_MCDC(0, 10, FALSE);
                CV_EML_IF(0, 55, FALSE);
                goto label_18;
              }

             label_13:
              ;
             label_14:
              ;
             label_15:
              ;
             label_16:
              ;
             label_17:
              ;
              CV_EML_MCDC(0, 10, TRUE);
              CV_EML_IF(0, 55, TRUE);
              _SFD_EML_CALL(0,818);
              for (c2_i157 = 0; c2_i157 < 26; c2_i157 = c2_i157 + 1) {
                c2_tg_u[c2_i157] = c2_cv37[c2_i157];
              }

              c2_sh_y = NULL;
              sf_mex_assign(&c2_sh_y, sf_mex_create("y", &c2_tg_u, 10, 0U, 1U,
                0U, 2, 1, 26));
              c2_ug_u = c2_node;
              c2_th_y = NULL;
              sf_mex_assign(&c2_th_y, sf_mex_create("y", &c2_ug_u, 0, 0U, 0U, 0U,
                0));
              c2_vg_u = c2_slot_num;
              c2_uh_y = NULL;
              sf_mex_assign(&c2_uh_y, sf_mex_create("y", &c2_vg_u, 0, 0U, 0U, 0U,
                0));
              sf_mex_call_debug("fprintf", 0U, 3U, 14, c2_sh_y, 14, c2_th_y, 14,
                                c2_uh_y);
              _SFD_EML_CALL(0,819);
              c2_slot[_SFD_EML_ARRAY_BOUNDS_CHECK("slot", (int32_T)
                _SFD_INTEGER_CHECK("node", c2_node), 1, 11, 1, 0) - 1] =
                c2_slot_num;
              _SFD_EML_CALL(0,820);
              c2_s_a = c2_slot_num - 1.0;
              c2_vh_y = c2_s_a * 1920.0;
              c2_t_a = c2_vh_y;
              c2_wh_y = c2_t_a * 1.5259E-05;
              c2_time_schedule[_SFD_EML_ARRAY_BOUNDS_CHECK("time_schedule",
                (int32_T)_SFD_INTEGER_CHECK("node", c2_node), 1, 11, 1, 0) - 1] =
                (
                 c2_ttStart_next + c2_deltaCAP) + c2_wh_y;
              _SFD_EML_CALL(0,821);
              c2_slot_num = c2_slot_num + 1.0;
             label_18:
              ;
            } else {
              /*  */
              _SFD_EML_CALL(0,824);
              c2_lateOUT[_SFD_EML_ARRAY_BOUNDS_CHECK("lateOUT", (int32_T)
                _SFD_INTEGER_CHECK("l", c2_l), 1, 4, 1, 0) - 1] = c2_node;
              _SFD_EML_CALL(0,825);
              c2_l = c2_l + 1.0;
              _SFD_EML_CALL(0,826);
              for (c2_i158 = 0; c2_i158 < 54; c2_i158 = c2_i158 + 1) {
                c2_wg_u[c2_i158] = c2_cv38[c2_i158];
              }

              c2_xh_y = NULL;
              sf_mex_assign(&c2_xh_y, sf_mex_create("y", &c2_wg_u, 10, 0U, 1U,
                0U, 2, 1, 54));
              c2_xg_u = c2_node;
              c2_yh_y = NULL;
              sf_mex_assign(&c2_yh_y, sf_mex_create("y", &c2_xg_u, 0, 0U, 0U, 0U,
                0));
              sf_mex_call_debug("fprintf", 0U, 2U, 14, c2_xh_y, 14, c2_yh_y);
            }

            goto label_19;
          } else {
            goto label_20;
          }
        } else {
          goto label_21;
        }
      }
    }

   label_21:
    ;
   label_20:
    ;
    CV_EML_MCDC(0, 9, FALSE);
    CV_EML_IF(0, 53, FALSE);
   label_19:
    ;
    c2_e_i = c2_e_i + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 5, 0);

  /*  the variable slot which is of size 7 will now have all the slots given to */
  /*  each of the nodes. */
  /* slot=[4 5 6 7 16 16 16 1 16 3 2]; */
  /*  5) send slots to the basestation and BO. */
  c2_i = 1.0;
  c2_f_i = 1.0;
  while (c2_f_i <= 11.0) {
    c2_i = c2_f_i;
    CV_EML_FOR(0, 6, 1);
    _SFD_EML_CALL(0,837);
    if (CV_EML_IF(0, 56, c2_slot[_SFD_EML_ARRAY_BOUNDS_CHECK("slot", (int32_T)
          _SFD_INTEGER_CHECK("i", c2_i), 1, 11, 1, 0) - 1] != 16.0)) {
      _SFD_EML_CALL(0,838);
      c2_slot[_SFD_EML_ARRAY_BOUNDS_CHECK("slot", (int32_T)_SFD_INTEGER_CHECK(
        "i", c2_i), 1, 11, 1, 0) - 1] = c2_slot[
        _SFD_EML_ARRAY_BOUNDS_CHECK("slot", (int32_T)_SFD_INTEGER_CHECK("i",
        c2_i), 1, 11, 1, 0) - 1] + 8.0;
    }

    c2_f_i = c2_f_i + 1.0;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 6, 0);
  _SFD_EML_CALL(0,841);
  c2_TM1 = c2_slot[0];
  _SFD_EML_CALL(0,842);
  c2_TA1 = c2_slot[1];
  _SFD_EML_CALL(0,843);
  c2_TM2 = c2_slot[2];
  _SFD_EML_CALL(0,844);
  c2_TA2 = c2_slot[3];
  _SFD_EML_CALL(0,845);
  c2_TS1 = c2_slot[4];
  _SFD_EML_CALL(0,846);
  c2_TS2 = c2_slot[5];
  _SFD_EML_CALL(0,847);
  c2_TS3 = c2_slot[6];
  _SFD_EML_CALL(0,848);
  c2_TS4 = c2_slot[7];
  _SFD_EML_CALL(0,849);
  c2_TS5 = c2_slot[8];
  _SFD_EML_CALL(0,850);
  c2_TS6 = c2_slot[9];
  _SFD_EML_CALL(0,851);
  c2_TS7 = c2_slot[10];
  _SFD_EML_CALL(0,853);
  c2_time_schedule_1 = c2_time_schedule[0];
  _SFD_EML_CALL(0,854);
  c2_time_schedule_2 = c2_time_schedule[1];
  _SFD_EML_CALL(0,855);
  c2_time_schedule_3 = c2_time_schedule[2];
  _SFD_EML_CALL(0,856);
  c2_time_schedule_4 = c2_time_schedule[3];
  _SFD_EML_CALL(0,857);
  c2_time_schedule_5 = c2_time_schedule[4];
  _SFD_EML_CALL(0,858);
  c2_time_schedule_6 = c2_time_schedule[5];
  _SFD_EML_CALL(0,859);
  c2_time_schedule_7 = c2_time_schedule[6];
  _SFD_EML_CALL(0,860);
  c2_time_schedule_8 = c2_time_schedule[7];
  _SFD_EML_CALL(0,861);
  c2_time_schedule_9 = c2_time_schedule[8];
  _SFD_EML_CALL(0,862);
  c2_time_schedule_10 = c2_time_schedule[9];
  _SFD_EML_CALL(0,863);
  c2_time_schedule_11 = c2_time_schedule[10];
  _SFD_EML_CALL(0,865);
  c2_lateOUT_1 = c2_lateOUT[0];
  _SFD_EML_CALL(0,866);
  c2_lateOUT_2 = c2_lateOUT[1];
  _SFD_EML_CALL(0,867);
  c2_lateOUT_3 = c2_lateOUT[2];
  _SFD_EML_CALL(0,868);
  c2_lateOUT_4 = c2_lateOUT[3];
  _SFD_EML_CALL(0,870);
  c2_c_rand(chartInstance, c2_uv1);
  for (c2_i159 = 0; c2_i159 < 625; c2_i159 = c2_i159 + 1) {
    c2_randState2[c2_i159] = c2_uv1[c2_i159];
  }

  _SFD_EML_CALL(0,871);
  c2_randStateOUT = (real_T)c2_randState2[0];
 label_1:
  ;

  /* fprintf('\n'); */
  _SFD_EML_CALL(0,-871);
  sf_debug_symbol_scope_pop();
  *c2_b_BO = c2_BO;
  *c2_b_TM1 = c2_TM1;
  *c2_b_TA1 = c2_TA1;
  *c2_b_TM2 = c2_TM2;
  *c2_b_TA2 = c2_TA2;
  *c2_b_TS1 = c2_TS1;
  *c2_b_TS2 = c2_TS2;
  *c2_b_TS3 = c2_TS3;
  *c2_b_TS4 = c2_TS4;
  *c2_b_TS5 = c2_TS5;
  *c2_b_TS6 = c2_TS6;
  *c2_b_TS7 = c2_TS7;
  *c2_b_time_schedule_1 = c2_time_schedule_1;
  *c2_b_time_schedule_2 = c2_time_schedule_2;
  *c2_b_time_schedule_3 = c2_time_schedule_3;
  *c2_b_time_schedule_4 = c2_time_schedule_4;
  *c2_b_time_schedule_5 = c2_time_schedule_5;
  *c2_b_time_schedule_6 = c2_time_schedule_6;
  *c2_b_time_schedule_7 = c2_time_schedule_7;
  *c2_b_time_schedule_8 = c2_time_schedule_8;
  *c2_b_time_schedule_9 = c2_time_schedule_9;
  *c2_b_time_schedule_10 = c2_time_schedule_10;
  *c2_b_time_schedule_11 = c2_time_schedule_11;
  for (c2_i160 = 0; c2_i160 < 9; c2_i160 = c2_i160 + 1) {
    (*c2_b_TAU_K)[c2_i160] = c2_TAU_K[c2_i160];
  }

  *c2_b_INIT = c2_INIT;
  *c2_b_lateOUT_1 = c2_lateOUT_1;
  *c2_b_lateOUT_2 = c2_lateOUT_2;
  *c2_b_lateOUT_3 = c2_lateOUT_3;
  *c2_b_lateOUT_4 = c2_lateOUT_4;
  *c2_b_randStateOUT = c2_randStateOUT;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static void c2_rand(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T
                    c2_varargin_2)
{
  int32_T c2_i161;
  int32_T c2_i162;
  uint32_T c2_hoistedGlobal[625];
  int32_T c2_i163;
  uint32_T c2_state[625];
  real_T c2_arg3;
  int32_T c2_i164;
  uint32_T c2_b_state[625];
  int32_T c2_i165;
  uint32_T c2_c_state[625];
  int32_T c2_i166;
  if (!chartInstance->c2_twister_state_not_empty) {
    for (c2_i161 = 0; c2_i161 < 625; c2_i161 = c2_i161 + 1) {
      chartInstance->c2_twister_state[c2_i161] = 0U;
    }

    chartInstance->c2_twister_state_not_empty = TRUE;
  }

  for (c2_i162 = 0; c2_i162 < 625; c2_i162 = c2_i162 + 1) {
    c2_hoistedGlobal[c2_i162] = chartInstance->c2_twister_state[c2_i162];
  }

  for (c2_i163 = 0; c2_i163 < 625; c2_i163 = c2_i163 + 1) {
    c2_state[c2_i163] = c2_hoistedGlobal[c2_i163];
  }

  c2_arg3 = c2_varargin_2;
  for (c2_i164 = 0; c2_i164 < 625; c2_i164 = c2_i164 + 1) {
    c2_b_state[c2_i164] = c2_state[c2_i164];
  }

  for (c2_i165 = 0; c2_i165 < 625; c2_i165 = c2_i165 + 1) {
    c2_c_state[c2_i165] = c2_b_state[c2_i165];
  }

  c2_twister_state_vector(chartInstance, c2_c_state, c2_arg3, c2_b_state);
  for (c2_i166 = 0; c2_i166 < 625; c2_i166 = c2_i166 + 1) {
    chartInstance->c2_twister_state[c2_i166] = c2_b_state[c2_i166];
  }

  chartInstance->c2_method = 2U;
  chartInstance->c2_method_not_empty = TRUE;
}

static void c2_twister_state_vector(SFc2_SendFetchS2InstanceStruct
  *chartInstance, uint32_T c2_mt[625], real_T c2_seed, uint32_T
  c2_b_mt[625])
{
  int32_T c2_i167;
  real_T c2_d0;
  real_T c2_d1;
  uint32_T c2_u0;
  uint32_T c2_r;
  real_T c2_mti;
  real_T c2_b_mti;
  real_T c2_d2;
  real_T c2_d3;
  real_T c2_d4;
  uint32_T c2_u1;
  for (c2_i167 = 0; c2_i167 < 625; c2_i167 = c2_i167 + 1) {
    c2_b_mt[c2_i167] = c2_mt[c2_i167];
  }

  c2_d0 = c2_seed;
  c2_d1 = c2_d0;
  if (c2_d1 < 4.294967296E+09) {
    if (c2_d1 >= 0.0) {
      c2_u0 = (uint32_T)c2_d1;
    } else {
      c2_u0 = 0U;
    }
  } else if (c2_d1 >= 4.294967296E+09) {
    c2_u0 = MAX_uint32_T;
  } else {
    c2_u0 = 0U;
  }

  c2_r = c2_u0;
  c2_b_mt[0] = c2_r;
  for (c2_mti = 2.0; c2_mti <= 624.0; c2_mti = c2_mti + 1.0) {
    c2_b_mti = c2_mti;
    c2_d2 = c2_b_mti - 1.0;
    c2_d3 = c2_d2;
    c2_d3 = c2_d3 < 0.0 ? muDoubleScalarCeil(c2_d3 - 0.5) : muDoubleScalarFloor
      (c2_d3 + 0.5);
    c2_d4 = c2_d3;
    if (c2_d4 < 4.294967296E+09) {
      if (c2_d4 >= 0.0) {
        c2_u1 = (uint32_T)c2_d4;
      } else {
        c2_u1 = 0U;
      }
    } else if (c2_d4 >= 4.294967296E+09) {
      c2_u1 = MAX_uint32_T;
    } else {
      c2_u1 = 0U;
    }

    c2_r = (c2_r ^ c2_r >> 30U) * 1812433253U + c2_u1;
    c2_b_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c2_b_mti), 1, 625, 1, 0) - 1] = c2_r;
  }

  c2_b_mt[624] = 624U;
}

static real_T c2_sum(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T c2_x
                     [11])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_y = c2_x[0];
  for (c2_k = 2; c2_k < 12; c2_k = c2_k + 1) {
    c2_b_k = c2_k;
    c2_y = c2_y + c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 11, 1, 0) - 1];
  }

  return c2_y;
}

static real_T c2_b_sum(SFc2_SendFetchS2InstanceStruct *chartInstance, boolean_T
  c2_x[4])
{
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  c2_y = (real_T)c2_x[0];
  for (c2_k = 2; c2_k < 5; c2_k = c2_k + 1) {
    c2_b_k = c2_k;
    c2_y = c2_y + (real_T)c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 4, 1, 0) - 1];
  }

  return c2_y;
}

static creal_T c2_mpower(SFc2_SendFetchS2InstanceStruct *chartInstance, creal_T
  c2_a)
{
  return c2_power(chartInstance, c2_a, 2.0);
}

static creal_T c2_power(SFc2_SendFetchS2InstanceStruct *chartInstance, creal_T
  c2_a, real_T c2_b)
{
  creal_T c2_y;
  creal_T c2_ak;
  real_T c2_bk;
  creal_T c2_b_a;
  real_T c2_b_b;
  real_T c2_ar;
  real_T c2_ai;
  real_T c2_br;
  real_T c2_bi;
  real_T c2_x;
  real_T c2_b_x;
  boolean_T c2_c_b;
  boolean_T c2_b0;
  real_T c2_c_x;
  boolean_T c2_d_b;
  boolean_T c2_b1;
  boolean_T c2_e_b;
  real_T c2_d_x;
  real_T c2_e_x;
  creal_T c2_t2;
  real_T c2_f_x;
  real_T c2_e;
  real_T c2_g_x;
  real_T c2_ed2;
  real_T c2_f_b;
  real_T c2_b_y;
  creal_T c2_c_a;
  creal_T c2_g_b;
  creal_T c2_d_a;
  creal_T c2_h_b;
  creal_T c2_c_y;
  creal_T c2_d_y;
  creal_T c2_yk;
  real_T c2_b_br;
  real_T c2_b_bi;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_e_y;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_f_y;
  real_T c2_s;
  real_T c2_e_a;
  real_T c2_i_b;
  real_T c2_g_y;
  real_T c2_d;
  real_T c2_f_a;
  real_T c2_h_y;
  real_T c2_g_a;
  real_T c2_i_y;
  real_T c2_h_a;
  real_T c2_j_b;
  real_T c2_j_y;
  real_T c2_i_a;
  real_T c2_k_y;
  real_T c2_j_a;
  real_T c2_l_y;
  creal_T c2_l_x;
  creal_T c2_m_x;
  creal_T c2_n_x;
  real_T c2_x1;
  real_T c2_x2;
  real_T c2_k_a;
  real_T c2_k_b;
  real_T c2_o_x;
  boolean_T c2_l_b;
  real_T c2_m_y;
  real_T c2_p_x;
  boolean_T c2_m_b;
  boolean_T c2_b2;
  real_T c2_n_y;
  real_T c2_q_x;
  real_T c2_r;
  real_T c2_l_a;
  creal_T c2_n_b;
  creal_T c2_t;
  real_T c2_tr;
  real_T c2_ti;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_m_a;
  real_T c2_o_b;
  real_T c2_o_y;
  real_T c2_t_x;
  real_T c2_u_x;
  real_T c2_n_a;
  real_T c2_p_b;
  real_T c2_p_y;
  c2_ak = c2_a;
  c2_bk = c2_b;
  c2_b_a = c2_ak;
  c2_b_b = c2_bk;
  c2_ar = c2_b_a.re;
  c2_ai = c2_b_a.im;
  c2_br = c2_b_b;
  c2_bi = 0.0;
  if (c2_ai == 0.0) {
    if (c2_bi == 0.0) {
      if (c2_ar >= 0.0) {
        c2_y.re = muDoubleScalarPower(c2_ar, c2_br);
        c2_y.im = 0.0;
        return c2_y;
      } else {
        goto label_1;
      }
    }
  }

 label_1:
  ;
  if (c2_bi == 0.0) {
    c2_x = c2_br;
    c2_b_x = c2_x;
    c2_c_b = muDoubleScalarIsInf(c2_b_x);
    c2_b0 = !c2_c_b;
    c2_c_x = c2_x;
    c2_d_b = muDoubleScalarIsNaN(c2_c_x);
    c2_b1 = !c2_d_b;
    c2_e_b = (c2_b0 && c2_b1);
    if (c2_e_b) {
      c2_d_x = c2_br;
      c2_e_x = c2_d_x;
      c2_e_x = muDoubleScalarFloor(c2_e_x);
      if (c2_e_x == c2_br) {
        c2_t2 = c2_b_a;
        c2_y.re = 1.0;
        c2_y.im = 0.0;
        c2_f_x = c2_br;
        for (c2_e = muDoubleScalarAbs(c2_f_x); c2_e > 0.0; c2_e = c2_ed2) {
          c2_g_x = c2_e / 2.0;
          c2_ed2 = c2_g_x;
          c2_ed2 = muDoubleScalarFloor(c2_ed2);
          c2_f_b = c2_ed2;
          c2_b_y = 2.0 * c2_f_b;
          if (c2_b_y != c2_e) {
            c2_c_a = c2_t2;
            c2_g_b = c2_y;
            c2_y.re = c2_c_a.re * c2_g_b.re - c2_c_a.im * c2_g_b.im;
            c2_y.im = c2_c_a.re * c2_g_b.im + c2_c_a.im * c2_g_b.re;
          }

          c2_d_a = c2_t2;
          c2_h_b = c2_t2;
          c2_t2.re = c2_d_a.re * c2_h_b.re - c2_d_a.im * c2_h_b.im;
          c2_t2.im = c2_d_a.re * c2_h_b.im + c2_d_a.im * c2_h_b.re;
        }

        if (c2_br < 0.0) {
          c2_c_y = c2_y;
          c2_d_y = c2_c_y;
          c2_yk = c2_d_y;
          c2_b_br = c2_yk.re;
          c2_b_bi = c2_yk.im;
          if (c2_b_bi == 0.0) {
            c2_y.re = 1.0 / c2_b_br;
            c2_y.im = 0.0;
          } else if (c2_b_br == 0.0) {
            c2_y.re = 0.0 / c2_b_bi;
            c2_y.im = -1.0 / c2_b_bi;
          } else {
            c2_h_x = c2_b_br;
            c2_i_x = c2_h_x;
            c2_e_y = muDoubleScalarAbs(c2_i_x);
            c2_j_x = c2_b_bi;
            c2_k_x = c2_j_x;
            c2_f_y = muDoubleScalarAbs(c2_k_x);
            if (c2_e_y > c2_f_y) {
              c2_s = c2_b_bi / c2_b_br;
              c2_e_a = c2_s;
              c2_i_b = c2_b_bi;
              c2_g_y = c2_e_a * c2_i_b;
              c2_d = c2_b_br + c2_g_y;
              c2_f_a = c2_s;
              c2_h_y = c2_f_a * 0.0;
              c2_g_a = c2_s;
              c2_i_y = c2_g_a;
              c2_y.re = (1.0 + c2_h_y) / c2_d;
              c2_y.im = (0.0 - c2_i_y) / c2_d;
            } else {
              c2_s = c2_b_br / c2_b_bi;
              c2_h_a = c2_s;
              c2_j_b = c2_b_br;
              c2_j_y = c2_h_a * c2_j_b;
              c2_d = c2_b_bi + c2_j_y;
              c2_i_a = c2_s;
              c2_k_y = c2_i_a;
              c2_j_a = c2_s;
              c2_l_y = c2_j_a * 0.0;
              c2_y.re = c2_k_y / c2_d;
              c2_y.im = (c2_l_y - 1.0) / c2_d;
            }
          }
        }

        return c2_y;
      } else {
        goto label_2;
      }
    }
  }

 label_2:
  ;
  c2_l_x = c2_b_a;
  c2_m_x = c2_l_x;
  c2_n_x = c2_m_x;
  c2_x1 = c2_n_x.re;
  c2_x2 = c2_n_x.im;
  c2_k_a = muDoubleScalarAbs(c2_x1);
  c2_k_b = muDoubleScalarAbs(c2_x2);
  if (c2_k_b > c2_k_a) {
  } else {
    c2_o_x = c2_k_b;
    c2_l_b = muDoubleScalarIsNaN(c2_o_x);
    if (c2_l_b) {
    } else {
      c2_m_y = c2_k_a;
      goto label_3;
    }
  }

  c2_m_y = c2_k_b;
  c2_k_b = c2_k_a;
 label_3:
  ;
  if (c2_m_y == 0.0) {
  } else {
    c2_p_x = c2_m_y;
    c2_m_b = muDoubleScalarIsInf(c2_p_x);
    if (c2_m_b) {
    } else {
      c2_b2 = FALSE;
      goto label_4;
    }
  }

  c2_b2 = TRUE;
 label_4:
  ;
  if (!c2_b2) {
    c2_k_b = c2_k_b / c2_m_y;
    c2_m_y = c2_m_y * muDoubleScalarSqrt(1.0 + c2_k_b * c2_k_b);
  }

  c2_n_y = c2_m_x.im;
  c2_q_x = c2_m_x.re;
  c2_r = muDoubleScalarAtan2(c2_n_y, c2_q_x);
  c2_m_x.re = muDoubleScalarLog(c2_m_y);
  c2_m_x.im = c2_r;
  c2_l_a = c2_b_b;
  c2_n_b = c2_m_x;
  c2_t.re = c2_l_a * c2_n_b.re;
  c2_t.im = c2_l_a * c2_n_b.im;
  c2_tr = muDoubleScalarExp(c2_t.re);
  c2_ti = c2_t.im;
  c2_r_x = c2_ti;
  c2_s_x = c2_r_x;
  c2_s_x = muDoubleScalarCos(c2_s_x);
  c2_m_a = c2_tr;
  c2_o_b = c2_s_x;
  c2_o_y = c2_m_a * c2_o_b;
  c2_t_x = c2_ti;
  c2_u_x = c2_t_x;
  c2_u_x = muDoubleScalarSin(c2_u_x);
  c2_n_a = c2_tr;
  c2_p_b = c2_u_x;
  c2_p_y = c2_n_a * c2_p_b;
  c2_y.re = c2_o_y;
  c2_y.im = c2_p_y;
  return c2_y;
}

static real_T c2_b_rand(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
  uint32_T c2_uv2[625];
  int32_T c2_i168;
  int32_T c2_i169;
  uint32_T c2_hoistedGlobal[625];
  int32_T c2_i170;
  uint32_T c2_state[625];
  int32_T c2_i171;
  uint32_T c2_b_twister_state[625];
  int32_T c2_i172;
  uint32_T c2_c_twister_state[625];
  real_T c2_b_r;
  uint32_T c2_b_state[625];
  int32_T c2_i173;
  real_T c2_d5;
  int32_T c2_i174;
  uint32_T c2_b_hoistedGlobal;
  uint32_T c2_c_state;
  uint32_T c2_b_v4_state;
  uint32_T c2_s;
  uint32_T c2_u2;
  uint32_T c2_hi;
  uint32_T c2_lo;
  uint32_T c2_test1;
  uint32_T c2_test2;
  uint32_T c2_d_state;
  real_T c2_a;
  real_T c2_c_r;
  real_T c2_d6;
  if (!chartInstance->c2_method_not_empty) {
    chartInstance->c2_method = 2U;
    chartInstance->c2_method_not_empty = TRUE;
  }

  if (chartInstance->c2_method == 2) {
    if (!chartInstance->c2_twister_state_not_empty) {
      c2_eml_rand_mt19937ar(chartInstance, c2_uv2);
      for (c2_i168 = 0; c2_i168 < 625; c2_i168 = c2_i168 + 1) {
        chartInstance->c2_twister_state[c2_i168] = c2_uv2[c2_i168];
      }

      chartInstance->c2_twister_state_not_empty = TRUE;
    }

    for (c2_i169 = 0; c2_i169 < 625; c2_i169 = c2_i169 + 1) {
      c2_hoistedGlobal[c2_i169] = chartInstance->c2_twister_state[c2_i169];
    }

    for (c2_i170 = 0; c2_i170 < 625; c2_i170 = c2_i170 + 1) {
      c2_state[c2_i170] = c2_hoistedGlobal[c2_i170];
    }

    for (c2_i171 = 0; c2_i171 < 625; c2_i171 = c2_i171 + 1) {
      c2_b_twister_state[c2_i171] = c2_state[c2_i171];
    }

    for (c2_i172 = 0; c2_i172 < 625; c2_i172 = c2_i172 + 1) {
      c2_c_twister_state[c2_i172] = c2_b_twister_state[c2_i172];
    }

    c2_genrandu(chartInstance, c2_c_twister_state, c2_b_state, &c2_b_r);
    for (c2_i173 = 0; c2_i173 < 625; c2_i173 = c2_i173 + 1) {
      c2_b_twister_state[c2_i173] = c2_b_state[c2_i173];
    }

    c2_d5 = c2_b_r;
    for (c2_i174 = 0; c2_i174 < 625; c2_i174 = c2_i174 + 1) {
      chartInstance->c2_twister_state[c2_i174] = c2_b_twister_state[c2_i174];
    }

    return c2_d5;
  } else {
    if (!chartInstance->c2_v4_state_not_empty) {
      chartInstance->c2_v4_state = 1144108930U;
      chartInstance->c2_v4_state_not_empty = TRUE;
    }

    c2_b_hoistedGlobal = chartInstance->c2_v4_state;
    c2_c_state = c2_b_hoistedGlobal;
    c2_b_v4_state = c2_c_state;
    c2_s = c2_b_v4_state;
    c2_u2 = 127773U;
    c2_hi = c2_u2 == (uint32_T)0 ? MAX_uint32_T : c2_s / c2_u2;
    c2_lo = c2_s - c2_hi * 127773U;
    c2_test1 = 16807U * c2_lo;
    c2_test2 = 2836U * c2_hi;
    if (c2_test1 < c2_test2) {
      c2_d_state = (2147483647U - c2_test2) + c2_test1;
    } else {
      c2_d_state = c2_test1 - c2_test2;
    }

    c2_a = (real_T)c2_d_state;
    c2_c_r = c2_a * 4.6566128752457969E-10;
    c2_b_v4_state = c2_d_state;
    c2_d6 = c2_c_r;
    chartInstance->c2_v4_state = c2_b_v4_state;
    return c2_d6;
  }
}

static void c2_eml_rand_mt19937ar(SFc2_SendFetchS2InstanceStruct *chartInstance,
  uint32_T c2_state[625])
{
  int32_T c2_i175;
  uint32_T c2_uv3[625];
  for (c2_i175 = 0; c2_i175 < 625; c2_i175 = c2_i175 + 1) {
    c2_uv3[c2_i175] = 0U;
  }

  c2_twister_state_vector(chartInstance, c2_uv3, 5489.0, c2_state);
}

static void c2_genrandu(SFc2_SendFetchS2InstanceStruct *chartInstance, uint32_T
  c2_mt[625], uint32_T c2_b_mt[625], real_T *c2_r)
{
  int32_T c2_i176;
  int32_T c2_i177;
  uint32_T c2_c_mt[625];
  int32_T c2_i178;
  uint32_T c2_d_mt[625];
  int32_T c2_i179;
  uint32_T c2_u[2];
  real_T c2_j;
  real_T c2_b_j;
  uint32_T c2_mti;
  real_T c2_kk;
  real_T c2_b_kk;
  uint32_T c2_y;
  uint32_T c2_b_y;
  uint32_T c2_c_y;
  real_T c2_c_kk;
  uint32_T c2_d_y;
  uint32_T c2_e_y;
  uint32_T c2_f_y;
  uint32_T c2_g_y;
  int32_T c2_i180;
  int32_T c2_i181;
  uint32_T c2_b_u[2];
  real_T c2_a;
  real_T c2_h_y;
  real_T c2_b;
  for (c2_i176 = 0; c2_i176 < 625; c2_i176 = c2_i176 + 1) {
    c2_b_mt[c2_i176] = c2_mt[c2_i176];
  }

  /*    This is a uniform (0,1) pseudorandom number generator based on: */
  /*  */
  /*     A C-program for MT19937, with initialization improved 2002/1/26. */
  /*     Coded by Takuji Nishimura and Makoto Matsumoto. */
  /*  */
  /*     Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
  /*     All rights reserved. */
  /*  */
  /*     Redistribution and use in source and binary forms, with or without */
  /*     modification, are permitted provided that the following conditions */
  /*     are met: */
  /*  */
  /*       1. Redistributions of source code must retain the above copyright */
  /*          notice, this list of conditions and the following disclaimer. */
  /*  */
  /*       2. Redistributions in binary form must reproduce the above copyright */
  /*          notice, this list of conditions and the following disclaimer in the */
  /*          documentation and/or other materials provided with the distribution. */
  /*  */
  /*       3. The names of its contributors may not be used to endorse or promote */
  /*          products derived from this software without specific prior written */
  /*          permission. */
  /*  */
  /*     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
  /*     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
  /*     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
  /*     A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR */
  /*     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, */
  /*     EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, */
  /*     PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR */
  /*     PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF */
  /*     LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING */
  /*     NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS */
  /*     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
  do {
    for (c2_i177 = 0; c2_i177 < 625; c2_i177 = c2_i177 + 1) {
      c2_c_mt[c2_i177] = c2_b_mt[c2_i177];
    }

    for (c2_i178 = 0; c2_i178 < 625; c2_i178 = c2_i178 + 1) {
      c2_d_mt[c2_i178] = c2_c_mt[c2_i178];
    }

    for (c2_i179 = 0; c2_i179 < 2; c2_i179 = c2_i179 + 1) {
      c2_u[c2_i179] = 0U;
    }

    for (c2_j = 1.0; c2_j <= 2.0; c2_j = c2_j + 1.0) {
      c2_b_j = c2_j;
      c2_mti = c2_d_mt[624] + 1U;
      if ((real_T)c2_mti >= 625.0) {
        for (c2_kk = 1.0; c2_kk <= 227.0; c2_kk = c2_kk + 1.0) {
          c2_b_kk = c2_kk;
          c2_y = (c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                   _SFD_INTEGER_CHECK("", c2_b_kk), 1, 625, 1, 0) - 1] &
                  2147483648U) | (
            c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", c2_b_kk + 1.0), 1, 625, 1, 0) - 1] & 2147483647U);
          c2_b_y = c2_y;
          c2_c_y = c2_b_y;
          if ((real_T)(c2_c_y & 1U) == 0.0) {
            c2_c_y = c2_c_y >> 1U;
          } else {
            c2_c_y = c2_c_y >> 1U ^ 2567483615U;
          }

          c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            c2_b_kk), 1, 625, 1, 0) - 1] = c2_d_mt[
            _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            c2_b_kk + 397.0), 1, 625, 1, 0) - 1] ^ c2_c_y;
        }

        for (c2_c_kk = 228.0; c2_c_kk <= 623.0; c2_c_kk = c2_c_kk + 1.0) {
          c2_b_kk = c2_c_kk;
          c2_y = (c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                   _SFD_INTEGER_CHECK("", c2_b_kk), 1, 625, 1, 0) - 1] &
                  2147483648U) | (
            c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
            "", c2_b_kk + 1.0), 1, 625, 1, 0) - 1] & 2147483647U);
          c2_d_y = c2_y;
          c2_e_y = c2_d_y;
          if ((real_T)(c2_e_y & 1U) == 0.0) {
            c2_e_y = c2_e_y >> 1U;
          } else {
            c2_e_y = c2_e_y >> 1U ^ 2567483615U;
          }

          c2_d_mt[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            c2_b_kk), 1, 625, 1, 0) - 1] = c2_d_mt[
            _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (c2_b_kk + 1.0) - 228.0), 1, 625, 1, 0) - 1] ^ c2_e_y;
        }

        c2_y = (c2_d_mt[623] & 2147483648U) | (c2_d_mt[0] & 2147483647U);
        c2_f_y = c2_y;
        c2_g_y = c2_f_y;
        if ((real_T)(c2_g_y & 1U) == 0.0) {
          c2_g_y = c2_g_y >> 1U;
        } else {
          c2_g_y = c2_g_y >> 1U ^ 2567483615U;
        }

        c2_d_mt[623] = c2_d_mt[396] ^ c2_g_y;
        c2_mti = 1U;
      }

      c2_y = c2_d_mt[(int32_T)(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        (uint32_T)_SFD_INTEGER_CHECK("", (real_T)c2_mti), 1, 625
        , 1, 0) - 1];
      c2_d_mt[624] = c2_mti;
      c2_y = c2_y ^ c2_y >> 11U;
      c2_y = c2_y ^ (c2_y << 7U & 2636928640U);
      c2_y = c2_y ^ (c2_y << 15U & 4022730752U);
      c2_y = c2_y ^ c2_y >> 18U;
      c2_u[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c2_b_j), 1, 2, 1, 0) - 1] = c2_y;
    }

    for (c2_i180 = 0; c2_i180 < 625; c2_i180 = c2_i180 + 1) {
      c2_b_mt[c2_i180] = c2_d_mt[c2_i180];
    }

    for (c2_i181 = 0; c2_i181 < 2; c2_i181 = c2_i181 + 1) {
      c2_b_u[c2_i181] = c2_u[c2_i181];
    }

    c2_b_u[0] = c2_b_u[0] >> 5U;
    c2_b_u[1] = c2_b_u[1] >> 6U;
    c2_a = (real_T)c2_b_u[0];
    c2_h_y = c2_a * 6.7108864E+07;
    c2_b = c2_h_y + (real_T)c2_b_u[1];
    *c2_r = 1.1102230246251565E-16 * c2_b;
  } while (!(*c2_r != 0.0));
}

static void c2_sort(SFc2_SendFetchS2InstanceStruct *chartInstance, real_T c2_x
                    [11], real_T c2_b_x[11], real_T c2_idx[11])
{
  int32_T c2_i182;
  int32_T c2_i183;
  real_T c2_c_x[11];
  int32_T c2_i184;
  real_T c2_d_x[11];
  int32_T c2_i185;
  real_T c2_e_x[11];
  int32_T c2_iidx[11];
  int32_T c2_i186;
  real_T c2_b_idx[11];
  int32_T c2_i187;
  real_T c2_f_x[11];
  int32_T c2_i188;
  int32_T c2_i189;
  int32_T c2_i190;
  for (c2_i182 = 0; c2_i182 < 11; c2_i182 = c2_i182 + 1) {
    c2_b_x[c2_i182] = c2_x[c2_i182];
  }

  for (c2_i183 = 0; c2_i183 < 11; c2_i183 = c2_i183 + 1) {
    c2_c_x[c2_i183] = c2_b_x[c2_i183];
  }

  for (c2_i184 = 0; c2_i184 < 11; c2_i184 = c2_i184 + 1) {
    c2_d_x[c2_i184] = c2_c_x[c2_i184];
  }

  for (c2_i185 = 0; c2_i185 < 11; c2_i185 = c2_i185 + 1) {
    c2_e_x[c2_i185] = c2_d_x[c2_i185];
  }

  c2_eml_sort_idx(chartInstance, c2_e_x, c2_iidx);
  for (c2_i186 = 0; c2_i186 < 11; c2_i186 = c2_i186 + 1) {
    c2_b_idx[c2_i186] = (real_T)c2_iidx[c2_i186];
  }

  for (c2_i187 = 0; c2_i187 < 11; c2_i187 = c2_i187 + 1) {
    c2_f_x[c2_i187] = c2_d_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c2_iidx[c2_i187]), 1, 11, 1, 0) - 1
      ];
  }

  for (c2_i188 = 0; c2_i188 < 11; c2_i188 = c2_i188 + 1) {
    c2_d_x[c2_i188] = c2_f_x[c2_i188];
  }

  for (c2_i189 = 0; c2_i189 < 11; c2_i189 = c2_i189 + 1) {
    c2_b_x[c2_i189] = c2_d_x[c2_i189];
  }

  for (c2_i190 = 0; c2_i190 < 11; c2_i190 = c2_i190 + 1) {
    c2_idx[c2_i190] = c2_b_idx[c2_i190];
  }
}

static void c2_eml_sort_idx(SFc2_SendFetchS2InstanceStruct *chartInstance,
  real_T c2_x[11], int32_T c2_idx[11])
{
  int32_T c2_i191;
  int32_T c2_idx0[11];
  int32_T c2_i192;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_a;
  int32_T c2_c;
  int32_T c2_i193;
  real_T c2_v[11];
  int32_T c2_irow1;
  int32_T c2_irow2;
  real_T c2_b_a;
  real_T c2_b;
  real_T c2_b_x;
  boolean_T c2_b_b;
  boolean_T c2_b3;
  boolean_T c2_p;
  int32_T c2_c_a;
  int32_T c2_b_c;
  int32_T c2_d_a;
  int32_T c2_c_c;
  int32_T c2_i;
  int32_T c2_e_a;
  int32_T c2_i2;
  int32_T c2_j;
  int32_T c2_c_b;
  int32_T c2_pEnd;
  int32_T c2_b_p;
  int32_T c2_q;
  int32_T c2_f_a;
  int32_T c2_d_b;
  int32_T c2_qEnd;
  int32_T c2_g_a;
  int32_T c2_e_b;
  int32_T c2_kEnd;
  int32_T c2_i194;
  real_T c2_b_v[11];
  int32_T c2_b_irow1;
  int32_T c2_b_irow2;
  real_T c2_h_a;
  real_T c2_f_b;
  real_T c2_c_x;
  boolean_T c2_g_b;
  boolean_T c2_b4;
  boolean_T c2_c_p;
  int32_T c2_i_a;
  int32_T c2_j_a;
  int32_T c2_k_a;
  int32_T c2_l_a;
  int32_T c2_m_a;
  int32_T c2_n_a;
  int32_T c2_o_a;
  int32_T c2_p_a;
  int32_T c2_c_k;
  int32_T c2_q_a;
  int32_T c2_h_b;
  int32_T c2_d_c;
  int32_T c2_r_a;
  int32_T c2_i_b;
  for (c2_i191 = 0; c2_i191 < 11; c2_i191 = c2_i191 + 1) {
    c2_idx0[c2_i191] = 1;
  }

  for (c2_i192 = 0; c2_i192 < 11; c2_i192 = c2_i192 + 1) {
    c2_idx[c2_i192] = 1 + c2_i192;
  }

  for (c2_k = 1; c2_k < 11; c2_k = c2_k + 2) {
    c2_b_k = c2_k;
    c2_a = c2_b_k;
    c2_c = c2_a + 1;
    for (c2_i193 = 0; c2_i193 < 11; c2_i193 = c2_i193 + 1) {
      c2_v[c2_i193] = c2_x[c2_i193];
    }

    c2_irow1 = c2_b_k;
    c2_irow2 = c2_c;
    c2_b_a = c2_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_irow1), 1, 11, 1, 0) - 1];
    c2_b = c2_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_irow2), 1, 11, 1, 0) - 1];
    if (c2_b_a <= c2_b) {
    } else {
      c2_b_x = c2_b;
      c2_b_b = muDoubleScalarIsNaN(c2_b_x);
      if (c2_b_b) {
      } else {
        c2_b3 = FALSE;
        goto label_1;
      }
    }

    c2_b3 = TRUE;
   label_1:
    ;
    c2_p = c2_b3;
    if (c2_p) {
    } else {
      c2_c_a = c2_b_k;
      c2_b_c = c2_c_a + 1;
      c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_k), 1, 11, 1, 0) - 1] = c2_b_c;
      c2_d_a = c2_b_k;
      c2_c_c = c2_d_a + 1;
      c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_c_c), 1, 11, 1, 0) - 1] = c2_b_k;
    }
  }

  for (c2_i = 2; c2_i <= 11; c2_i = c2_i2) {
    c2_e_a = c2_i;
    c2_i2 = c2_e_a << 1;
    c2_j = 1;
    c2_c_b = c2_i;
    for (c2_pEnd = 1 + c2_c_b; c2_pEnd < 12; c2_pEnd = c2_r_a + c2_i_b) {
      c2_b_p = c2_j;
      c2_q = c2_pEnd;
      c2_f_a = c2_j;
      c2_d_b = c2_i2;
      c2_qEnd = c2_f_a + c2_d_b;
      if (c2_qEnd > 12) {
        c2_qEnd = 12;
      }

      c2_b_k = 1;
      c2_g_a = c2_qEnd;
      c2_e_b = c2_j;
      c2_kEnd = c2_g_a - c2_e_b;
      while (c2_b_k <= c2_kEnd) {
        for (c2_i194 = 0; c2_i194 < 11; c2_i194 = c2_i194 + 1) {
          c2_b_v[c2_i194] = c2_x[c2_i194];
        }

        c2_b_irow1 = c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_p), 1, 11, 1, 0) - 1];
        c2_b_irow2 = c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_q), 1, 11, 1, 0) - 1];
        c2_h_a = c2_b_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_irow1), 1, 11, 1, 0) - 1];
        c2_f_b = c2_b_v[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c2_b_irow2), 1, 11, 1, 0) - 1];
        if (c2_h_a <= c2_f_b) {
        } else {
          c2_c_x = c2_f_b;
          c2_g_b = muDoubleScalarIsNaN(c2_c_x);
          if (c2_g_b) {
          } else {
            c2_b4 = FALSE;
            goto label_2;
          }
        }

        c2_b4 = TRUE;
       label_2:
        ;
        c2_c_p = c2_b4;
        if (c2_c_p) {
          c2_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 11, 1, 0) - 1] = c2_idx[
            _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_p), 1, 11, 1, 0) - 1];
          c2_i_a = c2_b_p;
          c2_b_p = c2_i_a + 1;
          if (c2_b_p == c2_pEnd) {
            while (c2_q < c2_qEnd) {
              c2_j_a = c2_b_k;
              c2_b_k = c2_j_a + 1;
              c2_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 11, 1, 0) - 1] =
                c2_idx[
                _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_q), 1, 11, 1, 0) - 1];
              c2_k_a = c2_q;
              c2_q = c2_k_a + 1;
            }
          }
        } else {
          c2_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_b_k), 1, 11, 1, 0) - 1] = c2_idx[
            _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c2_q), 1, 11, 1, 0) - 1];
          c2_l_a = c2_q;
          c2_q = c2_l_a + 1;
          if (c2_q == c2_qEnd) {
            while (c2_b_p < c2_pEnd) {
              c2_m_a = c2_b_k;
              c2_b_k = c2_m_a + 1;
              c2_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 11, 1, 0) - 1] =
                c2_idx[
                _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c2_b_p), 1, 11, 1, 0) - 1];
              c2_n_a = c2_b_p;
              c2_b_p = c2_n_a + 1;
            }
          }
        }

        c2_o_a = c2_b_k;
        c2_b_k = c2_o_a + 1;
      }

      c2_p_a = c2_j;
      c2_b_p = c2_p_a - 1;
      for (c2_c_k = 1; c2_c_k <= c2_kEnd; c2_c_k = c2_c_k + 1) {
        c2_b_k = c2_c_k;
        c2_q_a = c2_b_p;
        c2_h_b = c2_b_k;
        c2_d_c = c2_q_a + c2_h_b;
        c2_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_d_c), 1, 11, 1, 0) - 1] = c2_idx0[
          _SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c2_b_k), 1, 11, 1, 0) - 1];
      }

      c2_j = c2_qEnd;
      c2_r_a = c2_j;
      c2_i_b = c2_i;
    }
  }
}

static void c2_c_rand(SFc2_SendFetchS2InstanceStruct *chartInstance, uint32_T
                      c2_r[625])
{
  int32_T c2_i195;
  uint32_T c2_uv4[625];
  uint32_T c2_state[625];
  int32_T c2_i196;
  int32_T c2_i197;
  if (!chartInstance->c2_twister_state_not_empty) {
    for (c2_i195 = 0; c2_i195 < 625; c2_i195 = c2_i195 + 1) {
      c2_uv4[c2_i195] = 0U;
    }

    c2_twister_state_vector(chartInstance, c2_uv4, 5489.0, c2_state);
    for (c2_i196 = 0; c2_i196 < 625; c2_i196 = c2_i196 + 1) {
      chartInstance->c2_twister_state[c2_i196] = c2_state[c2_i196];
    }

    chartInstance->c2_twister_state_not_empty = TRUE;
  }

  for (c2_i197 = 0; c2_i197 < 625; c2_i197 = c2_i197 + 1) {
    c2_r[c2_i197] = chartInstance->c2_twister_state[c2_i197];
  }
}

static const mxArray *c2_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_b_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i198;
  real_T c2_b_u[9];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i198 = 0; c2_i198 < 9; c2_i198 = c2_i198 + 1) {
    c2_b_u[c2_i198] = (*((real_T (*)[9])c2_u))[c2_i198];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 1, 9));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_c_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i199;
  uint32_T c2_b_u[625];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i199 = 0; c2_i199 < 625; c2_i199 = c2_i199 + 1) {
    c2_b_u[c2_i199] = (*((uint32_T (*)[625])c2_u))[c2_i199];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 7, 0U, 1U, 0U, 1, 625));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_d_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i200;
  real_T c2_b_u[11];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i200 = 0; c2_i200 < 11; c2_i200 = c2_i200 + 1) {
    c2_b_u[c2_i200] = (*((real_T (*)[11])c2_u))[c2_i200];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 1, 11));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_e_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i201;
  creal_T c2_b_u[100];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i201 = 0; c2_i201 < 100; c2_i201 = c2_i201 + 1) {
    c2_b_u[c2_i201] = (*((creal_T (*)[100])c2_u))[c2_i201];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 1U, 1U, 0U, 1, 100));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_f_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i202;
  int32_T c2_i203;
  int32_T c2_i204;
  real_T c2_b_u[600];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_i202 = 0;
  for (c2_i203 = 0; c2_i203 < 6; c2_i203 = c2_i203 + 1) {
    for (c2_i204 = 0; c2_i204 < 100; c2_i204 = c2_i204 + 1) {
      c2_b_u[c2_i204 + c2_i202] = (*((real_T (*)[600])c2_u))[c2_i204 + c2_i202];
    }

    c2_i202 = c2_i202 + 100;
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 100, 6));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_g_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  creal_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((creal_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 1U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_h_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  struct smQvuE1V6fJPmdrAbGjhegG c2_b_u;
  const mxArray *c2_b_y = NULL;
  int32_T c2_i205;
  real_T c2_c_u[3];
  const mxArray *c2_c_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((struct smQvuE1V6fJPmdrAbGjhegG *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_createstruct("structure", 2, 1, 1));
  for (c2_i205 = 0; c2_i205 < 3; c2_i205 = c2_i205 + 1) {
    c2_c_u[c2_i205] = c2_b_u.x[c2_i205];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_c_u, 0, 0U, 1U, 0U, 1, 3));
  sf_mex_addfield(c2_b_y, c2_c_y, "x", "x", 0);
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_i_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i206;
  int32_T c2_i207;
  int32_T c2_i208;
  real_T c2_b_u[22];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_i206 = 0;
  for (c2_i207 = 0; c2_i207 < 11; c2_i207 = c2_i207 + 1) {
    for (c2_i208 = 0; c2_i208 < 2; c2_i208 = c2_i208 + 1) {
      c2_b_u[c2_i208 + c2_i206] = (*((real_T (*)[22])c2_u))[c2_i208 + c2_i206];
    }

    c2_i206 = c2_i206 + 2;
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 2, 11));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_j_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i209;
  real_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i209 = 0; c2_i209 < 4; c2_i209 = c2_i209 + 1) {
    c2_b_u[c2_i209] = (*((real_T (*)[4])c2_u))[c2_i209];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 2, 1, 4));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_k_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  int32_T c2_i210;
  real_T c2_b_u[11];
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  for (c2_i210 = 0; c2_i210 < 11; c2_i210 = c2_i210 + 1) {
    c2_b_u[c2_i210] = (*((real_T (*)[11])c2_u))[c2_i210];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 1U, 0U, 1, 11));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

const mxArray *sf_c2_SendFetchS2_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_ResolvedFunctionInfo c2_info[83];
  const mxArray *c2_m0 = NULL;
  int32_T c2_i211;
  c2_ResolvedFunctionInfo *c2_r0;
  c2_nameCaptureInfo = NULL;
  c2_info_helper(c2_info);
  c2_b_info_helper(c2_info);
  sf_mex_assign(&c2_m0, sf_mex_createstruct("nameCaptureInfo", 1, 83));
  for (c2_i211 = 0; c2_i211 < 83; c2_i211 = c2_i211 + 1) {
    c2_r0 = &c2_info[c2_i211];
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->context)), "context",
                    "nameCaptureInfo", c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c2_r0->name)), "name",
                    "nameCaptureInfo", c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c2_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", c2_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c2_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo",
                    c2_i211);
    sf_mex_addfield(c2_m0, sf_mex_create("nameCaptureInfo", &c2_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo",
                    c2_i211);
  }

  sf_mex_assign(&c2_nameCaptureInfo, c2_m0);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(c2_ResolvedFunctionInfo c2_info[83])
{
  c2_info[0].context = "";
  c2_info[0].name = "ctranspose";
  c2_info[0].dominantType = "double";
  c2_info[0].resolved = "[B]ctranspose";
  c2_info[0].fileLength = 0U;
  c2_info[0].fileTime1 = 0U;
  c2_info[0].fileTime2 = 0U;
  c2_info[1].context = "";
  c2_info[1].name = "double";
  c2_info[1].dominantType = "double";
  c2_info[1].resolved = "[B]double";
  c2_info[1].fileLength = 0U;
  c2_info[1].fileTime1 = 0U;
  c2_info[1].fileTime2 = 0U;
  c2_info[2].context = "";
  c2_info[2].name = "rand";
  c2_info[2].dominantType = "char";
  c2_info[2].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[2].fileLength = 4030U;
  c2_info[2].fileTime1 = 1244735552U;
  c2_info[2].fileTime2 = 0U;
  c2_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[3].name = "uint8";
  c2_info[3].dominantType = "double";
  c2_info[3].resolved = "[B]uint8";
  c2_info[3].fileLength = 0U;
  c2_info[3].fileTime1 = 0U;
  c2_info[3].fileTime2 = 0U;
  c2_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[4].name = "nargin";
  c2_info[4].dominantType = "";
  c2_info[4].resolved = "[B]nargin";
  c2_info[4].fileLength = 0U;
  c2_info[4].fileTime1 = 0U;
  c2_info[4].fileTime2 = 0U;
  c2_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[5].name = "gt";
  c2_info[5].dominantType = "double";
  c2_info[5].resolved = "[B]gt";
  c2_info[5].fileLength = 0U;
  c2_info[5].fileTime1 = 0U;
  c2_info[5].fileTime2 = 0U;
  c2_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[6].name = "ischar";
  c2_info[6].dominantType = "char";
  c2_info[6].resolved = "[B]ischar";
  c2_info[6].fileLength = 0U;
  c2_info[6].fileTime1 = 0U;
  c2_info[6].fileTime2 = 0U;
  c2_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[7].name = "eq";
  c2_info[7].dominantType = "double";
  c2_info[7].resolved = "[B]eq";
  c2_info[7].fileLength = 0U;
  c2_info[7].fileTime1 = 0U;
  c2_info[7].fileTime2 = 0U;
  c2_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[8].name = "le";
  c2_info[8].dominantType = "double";
  c2_info[8].resolved = "[B]le";
  c2_info[8].fileLength = 0U;
  c2_info[8].fileTime1 = 0U;
  c2_info[8].fileTime2 = 0U;
  c2_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[9].name = "nargout";
  c2_info[9].dominantType = "";
  c2_info[9].resolved = "[B]nargout";
  c2_info[9].fileLength = 0U;
  c2_info[9].fileTime1 = 0U;
  c2_info[9].fileTime2 = 0U;
  c2_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[10].name = "isa";
  c2_info[10].dominantType = "double";
  c2_info[10].resolved = "[B]isa";
  c2_info[10].fileLength = 0U;
  c2_info[10].fileTime1 = 0U;
  c2_info[10].fileTime2 = 0U;
  c2_info[11].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[11].name = "strcmp";
  c2_info[11].dominantType = "char";
  c2_info[11].resolved = "[B]strcmp";
  c2_info[11].fileLength = 0U;
  c2_info[11].fileTime1 = 0U;
  c2_info[11].fileTime2 = 0U;
  c2_info[12].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[12].name = "not";
  c2_info[12].dominantType = "logical";
  c2_info[12].resolved = "[B]not";
  c2_info[12].fileLength = 0U;
  c2_info[12].fileTime1 = 0U;
  c2_info[12].fileTime2 = 0U;
  c2_info[13].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[13].name = "size";
  c2_info[13].dominantType = "double";
  c2_info[13].resolved = "[B]size";
  c2_info[13].fileLength = 0U;
  c2_info[13].fileTime1 = 0U;
  c2_info[13].fileTime2 = 0U;
  c2_info[14].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[14].name = "isscalar";
  c2_info[14].dominantType = "double";
  c2_info[14].resolved = "[B]isscalar";
  c2_info[14].fileLength = 0U;
  c2_info[14].fileTime1 = 0U;
  c2_info[14].fileTime2 = 0U;
  c2_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[15].name = "eml_rand_mt19937ar";
  c2_info[15].dominantType = "char";
  c2_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mt19937ar.m";
  c2_info[15].fileLength = 24300U;
  c2_info[15].fileTime1 = 1246283388U;
  c2_info[15].fileTime2 = 0U;
  c2_info[16].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mt19937ar.m/twister_state_vector";
  c2_info[16].name = "zeros";
  c2_info[16].dominantType = "double";
  c2_info[16].resolved = "[B]zeros";
  c2_info[16].fileLength = 0U;
  c2_info[16].fileTime1 = 0U;
  c2_info[16].fileTime2 = 0U;
  c2_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mt19937ar.m/twister_state_vector";
  c2_info[17].name = "minus";
  c2_info[17].dominantType = "double";
  c2_info[17].resolved = "[B]minus";
  c2_info[17].fileLength = 0U;
  c2_info[17].fileTime1 = 0U;
  c2_info[17].fileTime2 = 0U;
  c2_info[18].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mt19937ar.m/twister_state_vector";
  c2_info[18].name = "uint32";
  c2_info[18].dominantType = "double";
  c2_info[18].resolved = "[B]uint32";
  c2_info[18].fileLength = 0U;
  c2_info[18].fileTime1 = 0U;
  c2_info[18].fileTime2 = 0U;
  c2_info[19].context = "";
  c2_info[19].name = "ones";
  c2_info[19].dominantType = "double";
  c2_info[19].resolved = "[B]ones";
  c2_info[19].fileLength = 0U;
  c2_info[19].fileTime1 = 0U;
  c2_info[19].fileTime2 = 0U;
  c2_info[20].context = "";
  c2_info[20].name = "mtimes";
  c2_info[20].dominantType = "double";
  c2_info[20].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[20].fileLength = 3425U;
  c2_info[20].fileTime1 = 1250672766U;
  c2_info[20].fileTime2 = 0U;
  c2_info[21].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[21].name = "isinteger";
  c2_info[21].dominantType = "double";
  c2_info[21].resolved = "[B]isinteger";
  c2_info[21].fileLength = 0U;
  c2_info[21].fileTime1 = 0U;
  c2_info[21].fileTime2 = 0U;
  c2_info[22].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c2_info[22].name = "class";
  c2_info[22].dominantType = "double";
  c2_info[22].resolved = "[B]class";
  c2_info[22].fileLength = 0U;
  c2_info[22].fileTime1 = 0U;
  c2_info[22].fileTime2 = 0U;
  c2_info[23].context = "";
  c2_info[23].name = "sum";
  c2_info[23].dominantType = "double";
  c2_info[23].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[23].fileLength = 3457U;
  c2_info[23].fileTime1 = 1244735552U;
  c2_info[23].fileTime2 = 0U;
  c2_info[24].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[24].name = "isequal";
  c2_info[24].dominantType = "double";
  c2_info[24].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[24].fileLength = 180U;
  c2_info[24].fileTime1 = 1226577270U;
  c2_info[24].fileTime2 = 0U;
  c2_info[25].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[25].name = "false";
  c2_info[25].dominantType = "";
  c2_info[25].resolved = "[B]false";
  c2_info[25].fileLength = 0U;
  c2_info[25].fileTime1 = 0U;
  c2_info[25].fileTime2 = 0U;
  c2_info[26].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m";
  c2_info[26].name = "eml_isequal_core";
  c2_info[26].dominantType = "double";
  c2_info[26].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[26].fileLength = 4192U;
  c2_info[26].fileTime1 = 1257783382U;
  c2_info[26].fileTime2 = 0U;
  c2_info[27].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[27].name = "ge";
  c2_info[27].dominantType = "double";
  c2_info[27].resolved = "[B]ge";
  c2_info[27].fileLength = 0U;
  c2_info[27].fileTime1 = 0U;
  c2_info[27].fileTime2 = 0U;
  c2_info[28].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[28].name = "islogical";
  c2_info[28].dominantType = "logical";
  c2_info[28].resolved = "[B]islogical";
  c2_info[28].fileLength = 0U;
  c2_info[28].fileTime1 = 0U;
  c2_info[28].fileTime2 = 0U;
  c2_info[29].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m";
  c2_info[29].name = "isnumeric";
  c2_info[29].dominantType = "double";
  c2_info[29].resolved = "[B]isnumeric";
  c2_info[29].fileLength = 0U;
  c2_info[29].fileTime1 = 0U;
  c2_info[29].fileTime2 = 0U;
  c2_info[30].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c2_info[30].name = "ndims";
  c2_info[30].dominantType = "double";
  c2_info[30].resolved = "[B]ndims";
  c2_info[30].fileLength = 0U;
  c2_info[30].fileTime1 = 0U;
  c2_info[30].fileTime2 = 0U;
  c2_info[31].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c2_info[31].name = "ne";
  c2_info[31].dominantType = "double";
  c2_info[31].resolved = "[B]ne";
  c2_info[31].fileLength = 0U;
  c2_info[31].fileTime1 = 0U;
  c2_info[31].fileTime2 = 0U;
  c2_info[32].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m/same_size";
  c2_info[32].name = "true";
  c2_info[32].dominantType = "";
  c2_info[32].resolved = "[B]true";
  c2_info[32].fileLength = 0U;
  c2_info[32].fileTime1 = 0U;
  c2_info[32].fileTime2 = 0U;
  c2_info[33].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[33].name = "eml_const_nonsingleton_dim";
  c2_info[33].dominantType = "double";
  c2_info[33].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c2_info[33].fileLength = 1473U;
  c2_info[33].fileTime1 = 1240262002U;
  c2_info[33].fileTime2 = 0U;
  c2_info[34].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[34].name = "cast";
  c2_info[34].dominantType = "double";
  c2_info[34].resolved = "[B]cast";
  c2_info[34].fileLength = 0U;
  c2_info[34].fileTime1 = 0U;
  c2_info[34].fileTime2 = 0U;
  c2_info[35].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[35].name = "eml_scalar_eg";
  c2_info[35].dominantType = "double";
  c2_info[35].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[35].fileLength = 3068U;
  c2_info[35].fileTime1 = 1240262010U;
  c2_info[35].fileTime2 = 0U;
  c2_info[36].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c2_info[36].name = "isstruct";
  c2_info[36].dominantType = "double";
  c2_info[36].resolved = "[B]isstruct";
  c2_info[36].fileLength = 0U;
  c2_info[36].fileTime1 = 0U;
  c2_info[36].fileTime2 = 0U;
  c2_info[37].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m/allreal";
  c2_info[37].name = "isreal";
  c2_info[37].dominantType = "double";
  c2_info[37].resolved = "[B]isreal";
  c2_info[37].fileLength = 0U;
  c2_info[37].fileTime1 = 0U;
  c2_info[37].fileTime2 = 0U;
  c2_info[38].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[38].name = "isempty";
  c2_info[38].dominantType = "double";
  c2_info[38].resolved = "[B]isempty";
  c2_info[38].fileLength = 0U;
  c2_info[38].fileTime1 = 0U;
  c2_info[38].fileTime2 = 0U;
  c2_info[39].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[39].name = "eml_index_class";
  c2_info[39].dominantType = "";
  c2_info[39].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c2_info[39].fileLength = 909U;
  c2_info[39].fileTime1 = 1192466782U;
  c2_info[39].fileTime2 = 0U;
  c2_info[40].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m";
  c2_info[40].name = "plus";
  c2_info[40].dominantType = "double";
  c2_info[40].resolved = "[B]plus";
  c2_info[40].fileLength = 0U;
  c2_info[40].fileTime1 = 0U;
  c2_info[40].fileTime2 = 0U;
  c2_info[41].context = "";
  c2_info[41].name = "complex";
  c2_info[41].dominantType = "double";
  c2_info[41].resolved = "[B]complex";
  c2_info[41].fileLength = 0U;
  c2_info[41].fileTime1 = 0U;
  c2_info[41].fileTime2 = 0U;
  c2_info[42].context = "";
  c2_info[42].name = "uminus";
  c2_info[42].dominantType = "double";
  c2_info[42].resolved = "[B]uminus";
  c2_info[42].fileLength = 0U;
  c2_info[42].fileTime1 = 0U;
  c2_info[42].fileTime2 = 0U;
  c2_info[43].context = "";
  c2_info[43].name = "mpower";
  c2_info[43].dominantType = "double";
  c2_info[43].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[43].fileLength = 3710U;
  c2_info[43].fileTime1 = 1238434288U;
  c2_info[43].fileTime2 = 0U;
  c2_info[44].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m";
  c2_info[44].name = "power";
  c2_info[44].dominantType = "double";
  c2_info[44].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[44].fileLength = 5380U;
  c2_info[44].fileTime1 = 1228093898U;
  c2_info[44].fileTime2 = 0U;
  c2_info[45].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[45].name = "eml_scalexp_alloc";
  c2_info[45].dominantType = "double";
  c2_info[45].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c2_info[45].fileLength = 808U;
  c2_info[45].fileTime1 = 1230494698U;
  c2_info[45].fileTime2 = 0U;
  c2_info[46].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[46].name = "real";
  c2_info[46].dominantType = "double";
  c2_info[46].resolved = "[B]real";
  c2_info[46].fileLength = 0U;
  c2_info[46].fileTime1 = 0U;
  c2_info[46].fileTime2 = 0U;
  c2_info[47].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[47].name = "imag";
  c2_info[47].dominantType = "double";
  c2_info[47].resolved = "[B]imag";
  c2_info[47].fileLength = 0U;
  c2_info[47].fileTime1 = 0U;
  c2_info[47].fileTime2 = 0U;
  c2_info[48].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[48].name = "isfinite";
  c2_info[48].dominantType = "double";
  c2_info[48].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[48].fileLength = 364U;
  c2_info[48].fileTime1 = 1226577272U;
  c2_info[48].fileTime2 = 0U;
  c2_info[49].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[49].name = "isinf";
  c2_info[49].dominantType = "double";
  c2_info[49].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m";
  c2_info[49].fileLength = 506U;
  c2_info[49].fileTime1 = 1228093808U;
  c2_info[49].fileTime2 = 0U;
  c2_info[50].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[50].name = "isnan";
  c2_info[50].dominantType = "double";
  c2_info[50].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m";
  c2_info[50].fileLength = 506U;
  c2_info[50].fileTime1 = 1228093810U;
  c2_info[50].fileTime2 = 0U;
  c2_info[51].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c2_info[51].name = "and";
  c2_info[51].dominantType = "logical";
  c2_info[51].resolved = "[B]and";
  c2_info[51].fileLength = 0U;
  c2_info[51].fileTime1 = 0U;
  c2_info[51].fileTime2 = 0U;
  c2_info[52].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[52].name = "eml_scalar_floor";
  c2_info[52].dominantType = "double";
  c2_info[52].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c2_info[52].fileLength = 260U;
  c2_info[52].fileTime1 = 1209330790U;
  c2_info[52].fileTime2 = 0U;
  c2_info[53].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[53].name = "eml_scalar_abs";
  c2_info[53].dominantType = "double";
  c2_info[53].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[53].fileLength = 461U;
  c2_info[53].fileTime1 = 1203447960U;
  c2_info[53].fileTime2 = 0U;
  c2_info[54].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[54].name = "lt";
  c2_info[54].dominantType = "double";
  c2_info[54].resolved = "[B]lt";
  c2_info[54].fileLength = 0U;
  c2_info[54].fileTime1 = 0U;
  c2_info[54].fileTime2 = 0U;
  c2_info[55].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[55].name = "eml_div";
  c2_info[55].dominantType = "double";
  c2_info[55].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c2_info[55].fileLength = 4269U;
  c2_info[55].fileTime1 = 1228093826U;
  c2_info[55].fileTime2 = 0U;
  c2_info[56].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c2_info[56].name = "abs";
  c2_info[56].dominantType = "double";
  c2_info[56].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m";
  c2_info[56].fileLength = 566U;
  c2_info[56].fileTime1 = 1221267132U;
  c2_info[56].fileTime2 = 0U;
  c2_info[57].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[57].name = "eml_scalar_log";
  c2_info[57].dominantType = "double";
  c2_info[57].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_log.m";
  c2_info[57].fileLength = 264U;
  c2_info[57].fileTime1 = 1203447996U;
  c2_info[57].fileTime2 = 0U;
  c2_info[58].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c2_info[58].name = "eml_dlapy2";
  c2_info[58].dominantType = "double";
  c2_info[58].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m";
  c2_info[58].fileLength = 1062U;
  c2_info[58].fileTime1 = 1209330826U;
  c2_info[58].fileTime2 = 0U;
  c2_info[59].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m";
  c2_info[59].name = "times";
  c2_info[59].dominantType = "double";
  c2_info[59].resolved = "[B]times";
  c2_info[59].fileLength = 0U;
  c2_info[59].fileTime1 = 0U;
  c2_info[59].fileTime2 = 0U;
  c2_info[60].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_log.m";
  c2_info[60].name = "eml_scalar_atan2";
  c2_info[60].dominantType = "double";
  c2_info[60].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m";
  c2_info[60].fileLength = 964U;
  c2_info[60].fileTime1 = 1209330786U;
  c2_info[60].fileTime2 = 0U;
  c2_info[61].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[61].name = "eml_scalar_cos";
  c2_info[61].dominantType = "double";
  c2_info[61].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c2_info[61].fileLength = 602U;
  c2_info[61].fileTime1 = 1209330786U;
  c2_info[61].fileTime2 = 0U;
  c2_info[62].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m/scalar_complex_power";
  c2_info[62].name = "eml_scalar_sin";
  c2_info[62].dominantType = "double";
  c2_info[62].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c2_info[62].fileLength = 601U;
  c2_info[62].fileTime1 = 1209330790U;
  c2_info[62].fileTime2 = 0U;
  c2_info[63].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mt19937ar.m/genrandu";
  c2_info[63].name = "eps";
  c2_info[63].dominantType = "";
  c2_info[63].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[63].fileLength = 1331U;
  c2_info[63].fileTime1 = 1246283386U;
  c2_info[63].fileTime2 = 0U;
}

static void c2_b_info_helper(c2_ResolvedFunctionInfo c2_info[83])
{
  c2_info[64].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m";
  c2_info[64].name = "eml_is_float_class";
  c2_info[64].dominantType = "char";
  c2_info[64].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c2_info[64].fileLength = 226U;
  c2_info[64].fileTime1 = 1197850440U;
  c2_info[64].fileTime2 = 0U;
  c2_info[65].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/rand.m";
  c2_info[65].name = "eml_rand_mcg16807";
  c2_info[65].dominantType = "char";
  c2_info[65].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_rand_mcg16807.m";
  c2_info[65].fileLength = 5995U;
  c2_info[65].fileTime1 = 1244735552U;
  c2_info[65].fileTime2 = 0U;
  c2_info[66].context = "";
  c2_info[66].name = "sort";
  c2_info[66].dominantType = "double";
  c2_info[66].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m";
  c2_info[66].fileLength = 749U;
  c2_info[66].fileTime1 = 1226577258U;
  c2_info[66].fileTime2 = 0U;
  c2_info[67].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m";
  c2_info[67].name = "eml_sort";
  c2_info[67].dominantType = "double";
  c2_info[67].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m";
  c2_info[67].fileLength = 2788U;
  c2_info[67].fileTime1 = 1240262012U;
  c2_info[67].fileTime2 = 0U;
  c2_info[68].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m";
  c2_info[68].name = "eml_assert_valid_dim";
  c2_info[68].dominantType = "double";
  c2_info[68].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[68].fileLength = 909U;
  c2_info[68].fileTime1 = 1240262002U;
  c2_info[68].fileTime2 = 0U;
  c2_info[69].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m";
  c2_info[69].name = "intmax";
  c2_info[69].dominantType = "char";
  c2_info[69].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[69].fileLength = 1535U;
  c2_info[69].fileTime1 = 1192466728U;
  c2_info[69].fileTime2 = 0U;
  c2_info[70].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m";
  c2_info[70].name = "int32";
  c2_info[70].dominantType = "double";
  c2_info[70].resolved = "[B]int32";
  c2_info[70].fileLength = 0U;
  c2_info[70].fileTime1 = 0U;
  c2_info[70].fileTime2 = 0U;
  c2_info[71].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m";
  c2_info[71].name = "isvector";
  c2_info[71].dominantType = "double";
  c2_info[71].resolved = "[B]isvector";
  c2_info[71].fileLength = 0U;
  c2_info[71].fileTime1 = 0U;
  c2_info[71].fileTime2 = 0U;
  c2_info[72].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m";
  c2_info[72].name = "eml_sort_idx";
  c2_info[72].dominantType = "double";
  c2_info[72].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[72].fileLength = 2402U;
  c2_info[72].fileTime1 = 1226577290U;
  c2_info[72].fileTime2 = 0U;
  c2_info[73].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[73].name = "eml_index_plus";
  c2_info[73].dominantType = "int32";
  c2_info[73].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c2_info[73].fileLength = 272U;
  c2_info[73].fileTime1 = 1192466784U;
  c2_info[73].fileTime2 = 0U;
  c2_info[74].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[74].name = "colon";
  c2_info[74].dominantType = "int32";
  c2_info[74].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[74].fileLength = 8592U;
  c2_info[74].fileTime1 = 1257783382U;
  c2_info[74].fileTime2 = 0U;
  c2_info[75].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m";
  c2_info[75].name = "floor";
  c2_info[75].dominantType = "double";
  c2_info[75].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m";
  c2_info[75].fileLength = 332U;
  c2_info[75].fileTime1 = 1203448022U;
  c2_info[75].fileTime2 = 0U;
  c2_info[76].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m/checkrange";
  c2_info[76].name = "intmin";
  c2_info[76].dominantType = "char";
  c2_info[76].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m";
  c2_info[76].fileLength = 1505U;
  c2_info[76].fileTime1 = 1192466728U;
  c2_info[76].fileTime2 = 0U;
  c2_info[77].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m/unrounded_npoints";
  c2_info[77].name = "realmax";
  c2_info[77].dominantType = "";
  c2_info[77].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmax.m";
  c2_info[77].fileLength = 771U;
  c2_info[77].fileTime1 = 1226577276U;
  c2_info[77].fileTime2 = 0U;
  c2_info[78].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m";
  c2_info[78].name = "eml_error";
  c2_info[78].dominantType = "char";
  c2_info[78].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m";
  c2_info[78].fileLength = 315U;
  c2_info[78].fileTime1 = 1213926744U;
  c2_info[78].fileTime2 = 0U;
  c2_info[79].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m/eml_integer_colon";
  c2_info[79].name = "transpose";
  c2_info[79].dominantType = "int32";
  c2_info[79].resolved = "[B]transpose";
  c2_info[79].fileLength = 0U;
  c2_info[79].fileTime1 = 0U;
  c2_info[79].fileTime2 = 0U;
  c2_info[80].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[80].name = "eml_index_minus";
  c2_info[80].dominantType = "int32";
  c2_info[80].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c2_info[80].fileLength = 277U;
  c2_info[80].fileTime1 = 1192466784U;
  c2_info[80].fileTime2 = 0U;
  c2_info[81].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[81].name = "eml_sort_le";
  c2_info[81].dominantType = "int32";
  c2_info[81].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m";
  c2_info[81].fileLength = 2752U;
  c2_info[81].fileTime1 = 1256045234U;
  c2_info[81].fileTime2 = 0U;
  c2_info[82].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m";
  c2_info[82].name = "eml_index_times";
  c2_info[82].dominantType = "int32";
  c2_info[82].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c2_info[82].fileLength = 280U;
  c2_info[82].fileTime1 = 1192466786U;
  c2_info[82].fileTime2 = 0U;
}

static const mxArray *c2_l_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  boolean_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((boolean_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static const mxArray *c2_m_sf_marshall(void *chartInstanceVoid, void *c2_u)
{
  const mxArray *c2_y = NULL;
  real_T c2_b_u;
  const mxArray *c2_b_y = NULL;
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)chartInstanceVoid;
  c2_y = NULL;
  c2_b_u = *((real_T *)c2_u);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c2_y, c2_b_y);
  return c2_y;
}

static real_T c2_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_BO, const char_T *c2_name)
{
  real_T c2_y;
  real_T c2_d7;
  sf_mex_import(c2_name, sf_mex_dup(c2_BO), &c2_d7, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d7;
  sf_mex_destroy(&c2_BO);
  return c2_y;
}

static void c2_b_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_TAU_K, const char_T *c2_name,
  real_T c2_y[9])
{
  real_T c2_dv3[9];
  int32_T c2_i212;
  sf_mex_import(c2_name, sf_mex_dup(c2_TAU_K), &c2_dv3, 1, 0, 0U, 1, 0U, 2, 1, 9);
  for (c2_i212 = 0; c2_i212 < 9; c2_i212 = c2_i212 + 1) {
    c2_y[c2_i212] = c2_dv3[c2_i212];
  }

  sf_mex_destroy(&c2_TAU_K);
}

static uint8_T c2_c_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_method, const char_T *
  c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u3;
  if (mxIsEmpty(c2_b_method)) {
    chartInstance->c2_method_not_empty = FALSE;
  } else {
    chartInstance->c2_method_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_method), &c2_u3, 1, 3, 0U, 0, 0U, 0);
    c2_y = c2_u3;
  }

  sf_mex_destroy(&c2_b_method);
  return c2_y;
}

static void c2_d_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct *chartInstance,
  const mxArray *c2_b_twister_state, const char_T *
  c2_name, uint32_T c2_y[625])
{
  uint32_T c2_uv5[625];
  int32_T c2_i213;
  if (mxIsEmpty(c2_b_twister_state)) {
    chartInstance->c2_twister_state_not_empty = FALSE;
  } else {
    chartInstance->c2_twister_state_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_twister_state), &c2_uv5, 1, 7, 0U, 1,
                  0U, 1, 625);
    for (c2_i213 = 0; c2_i213 < 625; c2_i213 = c2_i213 + 1) {
      c2_y[c2_i213] = c2_uv5[c2_i213];
    }
  }

  sf_mex_destroy(&c2_b_twister_state);
}

static uint32_T c2_e_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_v4_state, const char_T *
  c2_name)
{
  uint32_T c2_y;
  uint32_T c2_u4;
  if (mxIsEmpty(c2_b_v4_state)) {
    chartInstance->c2_v4_state_not_empty = FALSE;
  } else {
    chartInstance->c2_v4_state_not_empty = TRUE;
    sf_mex_import(c2_name, sf_mex_dup(c2_b_v4_state), &c2_u4, 1, 7, 0U, 0, 0U, 0);
    c2_y = c2_u4;
  }

  sf_mex_destroy(&c2_b_v4_state);
  return c2_y;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_SendFetchS2InstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_SendFetchS2,
  const char_T *c2_name)
{
  uint8_T c2_y;
  uint8_T c2_u5;
  sf_mex_import(c2_name, sf_mex_dup(c2_b_is_active_c2_SendFetchS2), &c2_u5, 1, 3,
                0U, 0, 0U, 0);
  c2_y = c2_u5;
  sf_mex_destroy(&c2_b_is_active_c2_SendFetchS2);
  return c2_y;
}

static void init_dsm_address_info(SFc2_SendFetchS2InstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_SendFetchS2_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(449438487U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1513382621U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2720957101U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1131482075U);
}

mxArray *sf_c2_SendFetchS2_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(2176485275U);
    pr[1] = (double)(1232737537U);
    pr[2] = (double)(680654269U);
    pr[3] = (double)(4088079713U);
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,30,3,dataFields);

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
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,15,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,15,"type",mxType);
    }

    mxSetField(mxData,15,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,16,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,16,"type",mxType);
    }

    mxSetField(mxData,16,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,17,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,17,"type",mxType);
    }

    mxSetField(mxData,17,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,18,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,18,"type",mxType);
    }

    mxSetField(mxData,18,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,19,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,19,"type",mxType);
    }

    mxSetField(mxData,19,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,20,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,20,"type",mxType);
    }

    mxSetField(mxData,20,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,21,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,21,"type",mxType);
    }

    mxSetField(mxData,21,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,22,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,22,"type",mxType);
    }

    mxSetField(mxData,22,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,23,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,23,"type",mxType);
    }

    mxSetField(mxData,23,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,24,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,24,"type",mxType);
    }

    mxSetField(mxData,24,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,25,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,25,"type",mxType);
    }

    mxSetField(mxData,25,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,26,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,26,"type",mxType);
    }

    mxSetField(mxData,26,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,27,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,27,"type",mxType);
    }

    mxSetField(mxData,27,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,28,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,28,"type",mxType);
    }

    mxSetField(mxData,28,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,29,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,29,"type",mxType);
    }

    mxSetField(mxData,29,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,30,3,dataFields);

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
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,14,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,14,"type",mxType);
    }

    mxSetField(mxData,14,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,15,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,15,"type",mxType);
    }

    mxSetField(mxData,15,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,16,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,16,"type",mxType);
    }

    mxSetField(mxData,16,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,17,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,17,"type",mxType);
    }

    mxSetField(mxData,17,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,18,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,18,"type",mxType);
    }

    mxSetField(mxData,18,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,19,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,19,"type",mxType);
    }

    mxSetField(mxData,19,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,20,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,20,"type",mxType);
    }

    mxSetField(mxData,20,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,21,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,21,"type",mxType);
    }

    mxSetField(mxData,21,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,22,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,22,"type",mxType);
    }

    mxSetField(mxData,22,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(9);
      mxSetField(mxData,23,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,23,"type",mxType);
    }

    mxSetField(mxData,23,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,24,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,24,"type",mxType);
    }

    mxSetField(mxData,24,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,25,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,25,"type",mxType);
    }

    mxSetField(mxData,25,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,26,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,26,"type",mxType);
    }

    mxSetField(mxData,26,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,27,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,27,"type",mxType);
    }

    mxSetField(mxData,27,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,28,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,28,"type",mxType);
    }

    mxSetField(mxData,28,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,29,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,29,"type",mxType);
    }

    mxSetField(mxData,29,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c2_SendFetchS2(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[5],T\"BO\",},{M[1],M[38],T\"INIT\",},{M[1],M[14],T\"TA1\",},{M[1],M[15],T\"TA2\",},{M[1],M[37],T\"TAU_K\",},{M[1],M[12],T\"TM1\",},{M[1],M[13],T\"TM2\",},{M[1],M[16],T\"TS1\",},{M[1],M[17],T\"TS2\",},{M[1],M[18],T\"TS3\",}}",
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[46],T\"TS4\",},{M[1],M[47],T\"TS5\",},{M[1],M[60],T\"TS6\",},{M[1],M[61],T\"TS7\",},{M[1],M[50],T\"lateOUT_1\",},{M[1],M[51],T\"lateOUT_2\",},{M[1],M[58],T\"lateOUT_3\",},{M[1],M[59],T\"lateOUT_4\",},{M[1],M[63],T\"randStateOUT\",},{M[1],M[22],T\"time_schedule_1\",}}",
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[56],T\"time_schedule_10\",},{M[1],M[57],T\"time_schedule_11\",},{M[1],M[31],T\"time_schedule_2\",},{M[1],M[32],T\"time_schedule_3\",},{M[1],M[33],T\"time_schedule_4\",},{M[1],M[34],T\"time_schedule_5\",},{M[1],M[35],T\"time_schedule_6\",},{M[1],M[36],T\"time_schedule_7\",},{M[1],M[48],T\"time_schedule_8\",},{M[1],M[49],T\"time_schedule_9\",}}",
    "100 S1x4'type','srcId','name','auxInfo'{{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[360 366],M[1],T\"/opt/matlab/toolbox/eml/lib/matlab/elmat/rand.m\"}}},{M[4],M[0],T\"twister_state\",S'l','i','p'{{M1x2[376 389],M[1],T\"/opt/matlab/toolbox/eml/lib/matlab/elmat/rand.m\"}}},{M[4],M[0],T\"v4_state\",S'l','i','p'{{M1x2[367 375],M[1],T\"/opt/matlab/toolbox/eml/lib/matlab/elmat/rand.m\"}}},{M[8],M[0],T\"is_active_c2_SendFetchS2\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 34, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_SendFetchS2_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_SendFetchS2InstanceStruct *chartInstance;
    chartInstance = (SFc2_SendFetchS2InstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_SendFetchS2MachineNumber_,
           2,
           1,
           1,
           60,
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
          init_script_number_translation(_SendFetchS2MachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_SendFetchS2MachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_SendFetchS2MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Y11",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(1,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"BO",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(2,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Y12",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(3,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Y21",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(4,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"Y22",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(5,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"S1",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(6,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"S2",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(7,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"S3",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(8,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"S4",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(9,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"S5",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(10,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TM1",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(11,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TA1",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(12,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TM2",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(13,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TA2",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(14,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS1",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(15,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS2",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(16,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS3",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(17,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS4",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(18,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS5",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(19,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS6",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(20,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TS7",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(21,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"TIME",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(22,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"I1",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(23,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"I2",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(24,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "current_BO",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(25,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_1",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(26,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_1",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(27,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_2",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(28,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_3",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(29,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_4",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(30,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_5",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(31,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_6",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(32,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_7",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(33,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_8",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(34,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_9",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(35,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_10",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(36,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule1_11",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(37,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_2",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(38,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_3",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(39,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_4",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(40,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_5",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(41,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_6",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(42,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_7",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(43,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_8",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(44,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_9",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(45,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_10",0,(MexFcnForType)
                              c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(46,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "time_schedule_11",0,(MexFcnForType)
                              c2_m_sf_marshall);

          {
            unsigned int dimVector[2];
            dimVector[0]= 1;
            dimVector[1]= 9;
            _SFD_SET_DATA_PROPS(47,2,0,1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
                                1.0,0,"TAU_K",0,(MexFcnForType)c2_b_sf_marshall);
          }

          _SFD_SET_DATA_PROPS(48,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,"INIT",0,
                              (MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(49,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "INIT_in",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(50,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateIN_1",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(51,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateIN_2",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(52,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateOUT_1",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(53,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateOUT_2",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(54,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateIN_3",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(55,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateIN_4",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(56,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateOUT_3",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(57,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "lateOUT_4",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(58,1,1,0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "randStateIN",0,(MexFcnForType)c2_m_sf_marshall);
          _SFD_SET_DATA_PROPS(59,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "randStateOUT",0,(MexFcnForType)c2_sf_marshall);
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
        _SFD_CV_INIT_EML(0,1,57,0,0,7,0,29,11);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,91193);
        _SFD_CV_INIT_EML_IF(0,0,2219,2246,3750,3792);
        _SFD_CV_INIT_EML_IF(0,1,2251,2266,3145,3749);
        _SFD_CV_INIT_EML_IF(0,2,4537,4579,-1,4695);
        _SFD_CV_INIT_EML_IF(0,3,4584,4605,4657,4691);
        _SFD_CV_INIT_EML_IF(0,4,4752,4794,-1,4910);
        _SFD_CV_INIT_EML_IF(0,5,4799,4820,4872,4906);
        _SFD_CV_INIT_EML_IF(0,6,4967,5009,-1,5125);
        _SFD_CV_INIT_EML_IF(0,7,5014,5035,5087,5121);
        _SFD_CV_INIT_EML_IF(0,8,5182,5224,-1,5340);
        _SFD_CV_INIT_EML_IF(0,9,5229,5250,5302,5336);
        _SFD_CV_INIT_EML_IF(0,10,5397,5439,-1,5555);
        _SFD_CV_INIT_EML_IF(0,11,5444,5465,5517,5551);
        _SFD_CV_INIT_EML_IF(0,12,5612,5654,-1,5770);
        _SFD_CV_INIT_EML_IF(0,13,5659,5680,5732,5766);
        _SFD_CV_INIT_EML_IF(0,14,5827,5869,-1,5985);
        _SFD_CV_INIT_EML_IF(0,15,5874,5895,5947,5981);
        _SFD_CV_INIT_EML_IF(0,16,6042,6084,-1,6200);
        _SFD_CV_INIT_EML_IF(0,17,6089,6110,6162,6196);
        _SFD_CV_INIT_EML_IF(0,18,6257,6299,-1,6415);
        _SFD_CV_INIT_EML_IF(0,19,6304,6325,6377,6411);
        _SFD_CV_INIT_EML_IF(0,20,6474,6517,-1,6634);
        _SFD_CV_INIT_EML_IF(0,21,6522,6544,6596,6630);
        _SFD_CV_INIT_EML_IF(0,22,6693,6736,-1,6853);
        _SFD_CV_INIT_EML_IF(0,23,6741,6763,6815,6849);
        _SFD_CV_INIT_EML_IF(0,24,6870,6935,-1,43318);
        _SFD_CV_INIT_EML_IF(0,25,41211,41228,-1,-2);
        _SFD_CV_INIT_EML_IF(0,26,41353,41370,-1,41402);
        _SFD_CV_INIT_EML_IF(0,27,41407,41422,-1,41452);
        _SFD_CV_INIT_EML_IF(0,28,43320,43385,-1,79749);
        _SFD_CV_INIT_EML_IF(0,29,77658,77675,-1,-2);
        _SFD_CV_INIT_EML_IF(0,30,77805,77822,-1,77854);
        _SFD_CV_INIT_EML_IF(0,31,77864,77879,-1,77909);
        _SFD_CV_INIT_EML_IF(0,32,79751,79816,-1,80323);
        _SFD_CV_INIT_EML_IF(0,33,80324,80389,-1,80896);
        _SFD_CV_INIT_EML_IF(0,34,80897,80962,-1,81469);
        _SFD_CV_INIT_EML_IF(0,35,81470,81535,-1,81876);
        _SFD_CV_INIT_EML_IF(0,36,81877,81942,-1,82242);
        _SFD_CV_INIT_EML_IF(0,37,82243,82310,-1,82614);
        _SFD_CV_INIT_EML_IF(0,38,82615,82682,-1,82986);
        _SFD_CV_INIT_EML_IF(0,39,83624,83670,-1,83695);
        _SFD_CV_INIT_EML_IF(0,40,83752,83798,-1,83823);
        _SFD_CV_INIT_EML_IF(0,41,83880,83926,-1,83951);
        _SFD_CV_INIT_EML_IF(0,42,84008,84054,-1,84079);
        _SFD_CV_INIT_EML_IF(0,43,84136,84182,-1,84207);
        _SFD_CV_INIT_EML_IF(0,44,84264,84310,-1,84335);
        _SFD_CV_INIT_EML_IF(0,45,84392,84438,-1,84463);
        _SFD_CV_INIT_EML_IF(0,46,84520,84566,-1,84591);
        _SFD_CV_INIT_EML_IF(0,47,84648,84694,-1,84719);
        _SFD_CV_INIT_EML_IF(0,48,84778,84825,-1,84850);
        _SFD_CV_INIT_EML_IF(0,49,84909,84956,-1,84981);
        _SFD_CV_INIT_EML_IF(0,50,85012,85058,-1,85356);
        _SFD_CV_INIT_EML_IF(0,51,88783,88793,-1,89201);
        _SFD_CV_INIT_EML_IF(0,52,89206,89216,-1,89624);
        _SFD_CV_INIT_EML_IF(0,53,89717,89768,-1,90258);
        _SFD_CV_INIT_EML_IF(0,54,89773,89789,90113,90254);
        _SFD_CV_INIT_EML_IF(0,55,89798,89876,-1,90106);
        _SFD_CV_INIT_EML_IF(0,56,90455,90471,-1,90506);
        _SFD_CV_INIT_EML_FOR(0,0,3348,3358,3489);
        _SFD_CV_INIT_EML_FOR(0,1,41190,41203,41293);
        _SFD_CV_INIT_EML_FOR(0,2,77637,77650,77740);
        _SFD_CV_INIT_EML_FOR(0,3,84997,85008,85360);
        _SFD_CV_INIT_EML_FOR(0,4,88701,88713,89628);
        _SFD_CV_INIT_EML_FOR(0,5,89635,89647,90262);
        _SFD_CV_INIT_EML_FOR(0,6,90440,90451,90510);

        {
          static int condStart[] = { 6874, 6919 };

          static int condEnd[] = { 6913, 6935 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,0,6873,6935,2,0,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 43324, 43369 };

          static int condEnd[] = { 43363, 43385 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,43323,43385,2,2,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 79755, 79800 };

          static int condEnd[] = { 79794, 79816 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,2,79754,79816,2,4,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 80328, 80373 };

          static int condEnd[] = { 80367, 80389 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,3,80327,80389,2,6,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 80901, 80946 };

          static int condEnd[] = { 80940, 80962 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,4,80900,80962,2,8,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 81474, 81519 };

          static int condEnd[] = { 81513, 81535 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,5,81473,81535,2,10,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 81881, 81926 };

          static int condEnd[] = { 81920, 81942 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,6,81880,81942,2,12,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 82247, 82293 };

          static int condEnd[] = { 82287, 82310 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,7,82246,82310,2,14,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 82619, 82665 };

          static int condEnd[] = { 82659, 82682 };

          static int pfixExpr[] = { 0, 1, -1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,8,82618,82682,2,16,&(condStart[0]),&(condEnd[0]),
                                4,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 89720, 89733, 89746, 89759 };

          static int condEnd[] = { 89729, 89742, 89755, 89768 };

          static int pfixExpr[] = { 0, 1, -3, 2, -3, 3, -3 };

          _SFD_CV_INIT_EML_MCDC(0,9,89720,89768,4,18,&(condStart[0]),&(condEnd[0]),
                                7,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 89801, 89812, 89823, 89834, 89845, 89856,
            89868 };

          static int condEnd[] = { 89808, 89819, 89830, 89841, 89852, 89864,
            89876 };

          static int pfixExpr[] = { 0, 1, -2, 2, -2, 3, -2, 4, -2, 5, -2, 6, -2
          };

          _SFD_CV_INIT_EML_MCDC(0,10,89801,89876,7,22,&(condStart[0]),&(condEnd
            [0]),13,&(pfixExpr[0]));
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
          real_T *c2_Y11;
          real_T *c2_BO;
          real_T *c2_Y12;
          real_T *c2_Y21;
          real_T *c2_Y22;
          real_T *c2_S1;
          real_T *c2_S2;
          real_T *c2_S3;
          real_T *c2_S4;
          real_T *c2_S5;
          real_T *c2_TM1;
          real_T *c2_TA1;
          real_T *c2_TM2;
          real_T *c2_TA2;
          real_T *c2_TS1;
          real_T *c2_TS2;
          real_T *c2_TS3;
          real_T *c2_TS4;
          real_T *c2_TS5;
          real_T *c2_TS6;
          real_T *c2_TS7;
          real_T *c2_TIME;
          real_T *c2_I1;
          real_T *c2_I2;
          real_T *c2_current_BO;
          real_T *c2_time_schedule_1;
          real_T *c2_time_schedule1_1;
          real_T *c2_time_schedule1_2;
          real_T *c2_time_schedule1_3;
          real_T *c2_time_schedule1_4;
          real_T *c2_time_schedule1_5;
          real_T *c2_time_schedule1_6;
          real_T *c2_time_schedule1_7;
          real_T *c2_time_schedule1_8;
          real_T *c2_time_schedule1_9;
          real_T *c2_time_schedule1_10;
          real_T *c2_time_schedule1_11;
          real_T *c2_time_schedule_2;
          real_T *c2_time_schedule_3;
          real_T *c2_time_schedule_4;
          real_T *c2_time_schedule_5;
          real_T *c2_time_schedule_6;
          real_T *c2_time_schedule_7;
          real_T *c2_time_schedule_8;
          real_T *c2_time_schedule_9;
          real_T *c2_time_schedule_10;
          real_T *c2_time_schedule_11;
          real_T (*c2_TAU_K)[9];
          real_T *c2_INIT;
          real_T *c2_INIT_in;
          real_T *c2_lateIN_1;
          real_T *c2_lateIN_2;
          real_T *c2_lateOUT_1;
          real_T *c2_lateOUT_2;
          real_T *c2_lateIN_3;
          real_T *c2_lateIN_4;
          real_T *c2_lateOUT_3;
          real_T *c2_lateOUT_4;
          real_T *c2_randStateIN;
          real_T *c2_randStateOUT;
          c2_randStateOUT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 30);
          c2_randStateIN = (real_T *)ssGetInputPortSignal(chartInstance->S, 29);
          c2_lateOUT_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 29);
          c2_lateOUT_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 28);
          c2_lateIN_4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 28);
          c2_lateIN_3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 27);
          c2_lateOUT_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 27);
          c2_lateOUT_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 26);
          c2_lateIN_2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 26);
          c2_lateIN_1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 25);
          c2_INIT_in = (real_T *)ssGetInputPortSignal(chartInstance->S, 24);
          c2_INIT = (real_T *)ssGetOutputPortSignal(chartInstance->S, 25);
          c2_TAU_K = (real_T (*)[9])ssGetOutputPortSignal(chartInstance->S, 24);
          c2_time_schedule_11 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            23);
          c2_time_schedule_10 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            22);
          c2_time_schedule_9 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            21);
          c2_time_schedule_8 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            20);
          c2_time_schedule_7 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            19);
          c2_time_schedule_6 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            18);
          c2_time_schedule_5 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            17);
          c2_time_schedule_4 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            16);
          c2_time_schedule_3 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            15);
          c2_time_schedule_2 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            14);
          c2_time_schedule1_11 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            23);
          c2_time_schedule1_10 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            22);
          c2_time_schedule1_9 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            21);
          c2_time_schedule1_8 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            20);
          c2_time_schedule1_7 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            19);
          c2_time_schedule1_6 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            18);
          c2_time_schedule1_5 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            17);
          c2_time_schedule1_4 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            16);
          c2_time_schedule1_3 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            15);
          c2_time_schedule1_2 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            14);
          c2_time_schedule1_1 = (real_T *)ssGetInputPortSignal(chartInstance->S,
            13);
          c2_time_schedule_1 = (real_T *)ssGetOutputPortSignal(chartInstance->S,
            13);
          c2_current_BO = (real_T *)ssGetInputPortSignal(chartInstance->S, 12);
          c2_I2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 11);
          c2_I1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 10);
          c2_TIME = (real_T *)ssGetInputPortSignal(chartInstance->S, 9);
          c2_TS7 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 12);
          c2_TS6 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 11);
          c2_TS5 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 10);
          c2_TS4 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 9);
          c2_TS3 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
          c2_TS2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
          c2_TS1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c2_TA2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c2_TM2 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c2_TA1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_TM1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_S5 = (real_T *)ssGetInputPortSignal(chartInstance->S, 8);
          c2_S4 = (real_T *)ssGetInputPortSignal(chartInstance->S, 7);
          c2_S3 = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c2_S2 = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c2_S1 = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_Y22 = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c2_Y21 = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_Y12 = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_BO = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_Y11 = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_Y11);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_BO);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_Y12);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_Y21);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_Y22);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_S1);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_S2);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_S3);
          _SFD_SET_DATA_VALUE_PTR(8U, c2_S4);
          _SFD_SET_DATA_VALUE_PTR(9U, c2_S5);
          _SFD_SET_DATA_VALUE_PTR(10U, c2_TM1);
          _SFD_SET_DATA_VALUE_PTR(11U, c2_TA1);
          _SFD_SET_DATA_VALUE_PTR(12U, c2_TM2);
          _SFD_SET_DATA_VALUE_PTR(13U, c2_TA2);
          _SFD_SET_DATA_VALUE_PTR(14U, c2_TS1);
          _SFD_SET_DATA_VALUE_PTR(15U, c2_TS2);
          _SFD_SET_DATA_VALUE_PTR(16U, c2_TS3);
          _SFD_SET_DATA_VALUE_PTR(17U, c2_TS4);
          _SFD_SET_DATA_VALUE_PTR(18U, c2_TS5);
          _SFD_SET_DATA_VALUE_PTR(19U, c2_TS6);
          _SFD_SET_DATA_VALUE_PTR(20U, c2_TS7);
          _SFD_SET_DATA_VALUE_PTR(21U, c2_TIME);
          _SFD_SET_DATA_VALUE_PTR(22U, c2_I1);
          _SFD_SET_DATA_VALUE_PTR(23U, c2_I2);
          _SFD_SET_DATA_VALUE_PTR(24U, c2_current_BO);
          _SFD_SET_DATA_VALUE_PTR(25U, c2_time_schedule_1);
          _SFD_SET_DATA_VALUE_PTR(26U, c2_time_schedule1_1);
          _SFD_SET_DATA_VALUE_PTR(27U, c2_time_schedule1_2);
          _SFD_SET_DATA_VALUE_PTR(28U, c2_time_schedule1_3);
          _SFD_SET_DATA_VALUE_PTR(29U, c2_time_schedule1_4);
          _SFD_SET_DATA_VALUE_PTR(30U, c2_time_schedule1_5);
          _SFD_SET_DATA_VALUE_PTR(31U, c2_time_schedule1_6);
          _SFD_SET_DATA_VALUE_PTR(32U, c2_time_schedule1_7);
          _SFD_SET_DATA_VALUE_PTR(33U, c2_time_schedule1_8);
          _SFD_SET_DATA_VALUE_PTR(34U, c2_time_schedule1_9);
          _SFD_SET_DATA_VALUE_PTR(35U, c2_time_schedule1_10);
          _SFD_SET_DATA_VALUE_PTR(36U, c2_time_schedule1_11);
          _SFD_SET_DATA_VALUE_PTR(37U, c2_time_schedule_2);
          _SFD_SET_DATA_VALUE_PTR(38U, c2_time_schedule_3);
          _SFD_SET_DATA_VALUE_PTR(39U, c2_time_schedule_4);
          _SFD_SET_DATA_VALUE_PTR(40U, c2_time_schedule_5);
          _SFD_SET_DATA_VALUE_PTR(41U, c2_time_schedule_6);
          _SFD_SET_DATA_VALUE_PTR(42U, c2_time_schedule_7);
          _SFD_SET_DATA_VALUE_PTR(43U, c2_time_schedule_8);
          _SFD_SET_DATA_VALUE_PTR(44U, c2_time_schedule_9);
          _SFD_SET_DATA_VALUE_PTR(45U, c2_time_schedule_10);
          _SFD_SET_DATA_VALUE_PTR(46U, c2_time_schedule_11);
          _SFD_SET_DATA_VALUE_PTR(47U, c2_TAU_K);
          _SFD_SET_DATA_VALUE_PTR(48U, c2_INIT);
          _SFD_SET_DATA_VALUE_PTR(49U, c2_INIT_in);
          _SFD_SET_DATA_VALUE_PTR(50U, c2_lateIN_1);
          _SFD_SET_DATA_VALUE_PTR(51U, c2_lateIN_2);
          _SFD_SET_DATA_VALUE_PTR(52U, c2_lateOUT_1);
          _SFD_SET_DATA_VALUE_PTR(53U, c2_lateOUT_2);
          _SFD_SET_DATA_VALUE_PTR(54U, c2_lateIN_3);
          _SFD_SET_DATA_VALUE_PTR(55U, c2_lateIN_4);
          _SFD_SET_DATA_VALUE_PTR(56U, c2_lateOUT_3);
          _SFD_SET_DATA_VALUE_PTR(57U, c2_lateOUT_4);
          _SFD_SET_DATA_VALUE_PTR(58U, c2_randStateIN);
          _SFD_SET_DATA_VALUE_PTR(59U, c2_randStateOUT);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_SendFetchS2MachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c2_SendFetchS2(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*)
    chartInstanceVar);
  initialize_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_SendFetchS2(void *chartInstanceVar)
{
  enable_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_SendFetchS2(void *chartInstanceVar)
{
  disable_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_SendFetchS2(void *chartInstanceVar)
{
  sf_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar);
}

static mxArray* sf_internal_get_sim_state_c2_SendFetchS2(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_SendFetchS2
    ((SFc2_SendFetchS2InstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = sf_get_sim_state_info_c2_SendFetchS2();/* state var info */
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

static void sf_internal_set_sim_state_c2_SendFetchS2(SimStruct* S, const mxArray
  *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_SendFetchS2();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static mxArray* sf_opaque_get_sim_state_c2_SendFetchS2(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_SendFetchS2(S);
}

static void sf_opaque_set_sim_state_c2_SendFetchS2(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c2_SendFetchS2(S, st);
}

static void sf_opaque_terminate_c2_SendFetchS2(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_SendFetchS2(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_SendFetchS2((SFc2_SendFetchS2InstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_SendFetchS2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("SendFetchS2","SendFetchS2",2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("SendFetchS2","SendFetchS2",2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("SendFetchS2",
      "SendFetchS2",2,"gatewayCannotBeInlinedMultipleTimes"));
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
      ssSetInputPortOptimOpts(S, 10, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 11, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 12, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 13, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 14, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 15, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 16, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 17, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 18, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 19, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 20, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 21, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 22, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 23, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 24, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 25, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 26, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 27, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 28, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 29, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"SendFetchS2","SendFetchS2",2,30);
      sf_mark_chart_reusable_outputs(S,"SendFetchS2","SendFetchS2",2,30);
    }

    sf_set_rtw_dwork_info(S,"SendFetchS2","SendFetchS2",2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(4238352618U));
  ssSetChecksum1(S,(2348492374U));
  ssSetChecksum2(S,(3853389688U));
  ssSetChecksum3(S,(2611259409U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_SendFetchS2(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "SendFetchS2", "SendFetchS2",2);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_SendFetchS2(SimStruct *S)
{
  SFc2_SendFetchS2InstanceStruct *chartInstance;
  chartInstance = (SFc2_SendFetchS2InstanceStruct *)malloc(sizeof
    (SFc2_SendFetchS2InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_SendFetchS2InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_SendFetchS2;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_SendFetchS2;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_SendFetchS2;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_SendFetchS2;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_SendFetchS2;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_SendFetchS2;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_SendFetchS2;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_SendFetchS2;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_SendFetchS2;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_SendFetchS2;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_SendFetchS2;
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

void c2_SendFetchS2_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_SendFetchS2(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_SendFetchS2(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_SendFetchS2(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_SendFetchS2_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
