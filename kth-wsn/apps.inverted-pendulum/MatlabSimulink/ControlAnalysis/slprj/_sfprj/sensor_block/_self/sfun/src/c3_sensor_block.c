/* Include files */

#include "blascompat32.h"
#include "sensor_block_sfun.h"
#include "c3_sensor_block.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance.chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance.instanceNumber)
#include "sensor_block_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c3_IN_NO_ACTIVE_CHILD          (0)

/* Variable Declarations */

/* Variable Definitions */
static SFc3_sensor_blockInstanceStruct chartInstance;

/* Function Declarations */
static void initialize_c3_sensor_block(void);
static void initialize_params_c3_sensor_block(void);
static void enable_c3_sensor_block(void);
static void disable_c3_sensor_block(void);
static void c3_update_debugger_state_c3_sensor_block(void);
static const mxArray *get_sim_state_c3_sensor_block(void);
static void set_sim_state_c3_sensor_block(const mxArray *c3_st);
static void finalize_c3_sensor_block(void);
static void sf_c3_sensor_block(void);
static void c3_c3_sensor_block(void);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber);
static const mxArray *c3_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_b_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_c_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_d_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_e_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_f_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_g_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_h_sf_marshall(void *c3_chartInstance, void *c3_u);
static const mxArray *c3_i_sf_marshall(void *c3_chartInstance, void *c3_u);
static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[24]);
static void init_dsm_address_info(void);

/* Function Definitions */
static void initialize_c3_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  chartInstance.c3_xc_value_p_not_empty = false;
  chartInstance.c3_xc_delay_p_not_empty = false;
  chartInstance.c3_theta_value_p_not_empty = false;
  chartInstance.c3_theta_delay_p_not_empty = false;
  chartInstance.c3_actuator_value_p_not_empty = false;
  chartInstance.c3_actuator_delay_p_not_empty = false;
  chartInstance.c3_is_active_c3_sensor_block = 0U;
}

static void initialize_params_c3_sensor_block(void)
{
}

static void enable_c3_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void disable_c3_sensor_block(void)
{
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
}

static void c3_update_debugger_state_c3_sensor_block(void)
{
}

static const mxArray *get_sim_state_c3_sensor_block(void)
{
  const mxArray *c3_st = NULL;
  const mxArray *c3_y = NULL;
  real_T c3_u;
  const mxArray *c3_b_y = NULL;
  boolean_T c3_b_u;
  const mxArray *c3_c_y = NULL;
  real_T c3_c_u;
  const mxArray *c3_d_y = NULL;
  real_T c3_d_u;
  const mxArray *c3_e_y = NULL;
  boolean_T c3_e_u;
  const mxArray *c3_f_y = NULL;
  real_T c3_f_u;
  const mxArray *c3_g_y = NULL;
  real_T c3_g_u;
  const mxArray *c3_h_y = NULL;
  boolean_T c3_h_u;
  const mxArray *c3_i_y = NULL;
  real_T c3_i_u;
  const mxArray *c3_j_y = NULL;
  real_T c3_j_u;
  const mxArray *c3_k_y = NULL;
  real_T c3_k_u;
  const mxArray *c3_l_y = NULL;
  real_T c3_l_u;
  const mxArray *c3_m_y = NULL;
  real_T c3_m_u;
  const mxArray *c3_n_y = NULL;
  real_T c3_n_u;
  const mxArray *c3_o_y = NULL;
  real_T c3_o_u;
  const mxArray *c3_p_y = NULL;
  uint8_T c3_p_u;
  const mxArray *c3_q_y = NULL;
  real_T *c3_actuator_delay;
  boolean_T *c3_actuator_update;
  real_T *c3_actuator_value;
  real_T *c3_theta_delay;
  boolean_T *c3_theta_update;
  real_T *c3_theta_value;
  real_T *c3_xc_delay;
  boolean_T *c3_xc_update;
  real_T *c3_xc_value;
  c3_theta_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  c3_actuator_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 7);
  c3_xc_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c3_theta_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c3_actuator_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 9);
  c3_theta_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c3_xc_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c3_xc_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c3_actuator_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 8);
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellarray(16));
  c3_u = *c3_actuator_delay;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_b_u = *c3_actuator_update;
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", &c3_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_c_u = *c3_actuator_value;
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", &c3_c_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_d_u = *c3_theta_delay;
  c3_e_y = NULL;
  sf_mex_assign(&c3_e_y, sf_mex_create("y", &c3_d_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_e_u = *c3_theta_update;
  c3_f_y = NULL;
  sf_mex_assign(&c3_f_y, sf_mex_create("y", &c3_e_u, 11, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_f_u = *c3_theta_value;
  c3_g_y = NULL;
  sf_mex_assign(&c3_g_y, sf_mex_create("y", &c3_f_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 5, c3_g_y);
  c3_g_u = *c3_xc_delay;
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y", &c3_g_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 6, c3_h_y);
  c3_h_u = *c3_xc_update;
  c3_i_y = NULL;
  sf_mex_assign(&c3_i_y, sf_mex_create("y", &c3_h_u, 11, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 7, c3_i_y);
  c3_i_u = *c3_xc_value;
  c3_j_y = NULL;
  sf_mex_assign(&c3_j_y, sf_mex_create("y", &c3_i_u, 0, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 8, c3_j_y);
  c3_j_u = chartInstance.c3_actuator_delay_p;
  c3_k_y = NULL;
  if (!chartInstance.c3_actuator_delay_p_not_empty) {
    sf_mex_assign(&c3_k_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_k_y, sf_mex_create("y", &c3_j_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 9, c3_k_y);
  c3_k_u = chartInstance.c3_actuator_value_p;
  c3_l_y = NULL;
  if (!chartInstance.c3_actuator_value_p_not_empty) {
    sf_mex_assign(&c3_l_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_l_y, sf_mex_create("y", &c3_k_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 10, c3_l_y);
  c3_l_u = chartInstance.c3_theta_delay_p;
  c3_m_y = NULL;
  if (!chartInstance.c3_theta_delay_p_not_empty) {
    sf_mex_assign(&c3_m_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_m_y, sf_mex_create("y", &c3_l_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 11, c3_m_y);
  c3_m_u = chartInstance.c3_theta_value_p;
  c3_n_y = NULL;
  if (!chartInstance.c3_theta_value_p_not_empty) {
    sf_mex_assign(&c3_n_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_n_y, sf_mex_create("y", &c3_m_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 12, c3_n_y);
  c3_n_u = chartInstance.c3_xc_delay_p;
  c3_o_y = NULL;
  if (!chartInstance.c3_xc_delay_p_not_empty) {
    sf_mex_assign(&c3_o_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_o_y, sf_mex_create("y", &c3_n_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 13, c3_o_y);
  c3_o_u = chartInstance.c3_xc_value_p;
  c3_p_y = NULL;
  if (!chartInstance.c3_xc_value_p_not_empty) {
    sf_mex_assign(&c3_p_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_p_y, sf_mex_create("y", &c3_o_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_setcell(c3_y, 14, c3_p_y);
  c3_p_u = chartInstance.c3_is_active_c3_sensor_block;
  c3_q_y = NULL;
  sf_mex_assign(&c3_q_y, sf_mex_create("y", &c3_p_u, 3, 0U, 0U, 0U, 0));
  sf_mex_setcell(c3_y, 15, c3_q_y);
  sf_mex_assign(&c3_st, c3_y);
  return c3_st;
}

static void set_sim_state_c3_sensor_block(const mxArray *c3_st)
{
  const mxArray *c3_u;
  const mxArray *c3_actuator_delay;
  real_T c3_d0;
  real_T c3_y;
  const mxArray *c3_actuator_update;
  boolean_T c3_b0;
  boolean_T c3_b_y;
  const mxArray *c3_b_actuator_delay;
  real_T c3_d1;
  real_T c3_c_y;
  const mxArray *c3_c_actuator_delay;
  real_T c3_d2;
  real_T c3_d_y;
  const mxArray *c3_b_actuator_update;
  boolean_T c3_b1;
  boolean_T c3_e_y;
  const mxArray *c3_d_actuator_delay;
  real_T c3_d3;
  real_T c3_f_y;
  const mxArray *c3_e_actuator_delay;
  real_T c3_d4;
  real_T c3_g_y;
  const mxArray *c3_c_actuator_update;
  boolean_T c3_b2;
  boolean_T c3_h_y;
  const mxArray *c3_f_actuator_delay;
  real_T c3_d5;
  real_T c3_i_y;
  const mxArray *c3_b_actuator_delay_p;
  real_T c3_d6;
  real_T c3_j_y;
  const mxArray *c3_b_actuator_value_p;
  real_T c3_d7;
  real_T c3_k_y;
  const mxArray *c3_b_theta_delay_p;
  real_T c3_d8;
  real_T c3_l_y;
  const mxArray *c3_b_theta_value_p;
  real_T c3_d9;
  real_T c3_m_y;
  const mxArray *c3_b_xc_delay_p;
  real_T c3_d10;
  real_T c3_n_y;
  const mxArray *c3_b_xc_value_p;
  real_T c3_d11;
  real_T c3_o_y;
  const mxArray *c3_b_is_active_c3_sensor_block;
  uint8_T c3_u0;
  uint8_T c3_p_y;
  real_T *c3_g_actuator_delay;
  boolean_T *c3_d_actuator_update;
  real_T *c3_actuator_value;
  real_T *c3_theta_delay;
  boolean_T *c3_theta_update;
  real_T *c3_theta_value;
  real_T *c3_xc_delay;
  boolean_T *c3_xc_update;
  real_T *c3_xc_value;
  c3_theta_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  c3_d_actuator_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 7);
  c3_xc_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c3_theta_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c3_g_actuator_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 9);
  c3_theta_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c3_xc_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c3_xc_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c3_actuator_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 8);
  chartInstance.c3_doneDoubleBufferReInit = true;
  c3_u = sf_mex_dup(c3_st);
  c3_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 0));
  sf_mex_import("actuator_delay", sf_mex_dup(c3_actuator_delay), &c3_d0, 1, 0,
                0U, 0, 0U, 0);
  c3_y = c3_d0;
  sf_mex_destroy(&c3_actuator_delay);
  *c3_g_actuator_delay = c3_y;
  c3_actuator_update = sf_mex_dup(sf_mex_getcell(c3_u, 1));
  sf_mex_import("actuator_update", sf_mex_dup(c3_actuator_update), &c3_b0, 1, 11,
                0U, 0, 0U, 0);
  c3_b_y = c3_b0;
  sf_mex_destroy(&c3_actuator_update);
  *c3_d_actuator_update = c3_b_y;
  c3_b_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 2));
  sf_mex_import("actuator_value", sf_mex_dup(c3_b_actuator_delay), &c3_d1, 1, 0,
                0U, 0, 0U, 0);
  c3_c_y = c3_d1;
  sf_mex_destroy(&c3_b_actuator_delay);
  *c3_actuator_value = c3_c_y;
  c3_c_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 3));
  sf_mex_import("theta_delay", sf_mex_dup(c3_c_actuator_delay), &c3_d2, 1, 0, 0U,
                0, 0U, 0);
  c3_d_y = c3_d2;
  sf_mex_destroy(&c3_c_actuator_delay);
  *c3_theta_delay = c3_d_y;
  c3_b_actuator_update = sf_mex_dup(sf_mex_getcell(c3_u, 4));
  sf_mex_import("theta_update", sf_mex_dup(c3_b_actuator_update), &c3_b1, 1, 11,
                0U, 0, 0U, 0);
  c3_e_y = c3_b1;
  sf_mex_destroy(&c3_b_actuator_update);
  *c3_theta_update = c3_e_y;
  c3_d_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 5));
  sf_mex_import("theta_value", sf_mex_dup(c3_d_actuator_delay), &c3_d3, 1, 0, 0U,
                0, 0U, 0);
  c3_f_y = c3_d3;
  sf_mex_destroy(&c3_d_actuator_delay);
  *c3_theta_value = c3_f_y;
  c3_e_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 6));
  sf_mex_import("xc_delay", sf_mex_dup(c3_e_actuator_delay), &c3_d4, 1, 0, 0U, 0,
                0U, 0);
  c3_g_y = c3_d4;
  sf_mex_destroy(&c3_e_actuator_delay);
  *c3_xc_delay = c3_g_y;
  c3_c_actuator_update = sf_mex_dup(sf_mex_getcell(c3_u, 7));
  sf_mex_import("xc_update", sf_mex_dup(c3_c_actuator_update), &c3_b2, 1, 11, 0U,
                0, 0U, 0);
  c3_h_y = c3_b2;
  sf_mex_destroy(&c3_c_actuator_update);
  *c3_xc_update = c3_h_y;
  c3_f_actuator_delay = sf_mex_dup(sf_mex_getcell(c3_u, 8));
  sf_mex_import("xc_value", sf_mex_dup(c3_f_actuator_delay), &c3_d5, 1, 0, 0U, 0,
                0U, 0);
  c3_i_y = c3_d5;
  sf_mex_destroy(&c3_f_actuator_delay);
  *c3_xc_value = c3_i_y;
  c3_b_actuator_delay_p = sf_mex_dup(sf_mex_getcell(c3_u, 9));
  if (mxIsEmpty(c3_b_actuator_delay_p)) {
    chartInstance.c3_actuator_delay_p_not_empty = false;
  } else {
    chartInstance.c3_actuator_delay_p_not_empty = true;
    sf_mex_import("actuator_delay_p", sf_mex_dup(c3_b_actuator_delay_p), &c3_d6,
                  1, 0, 0U, 0, 0U, 0);
    c3_j_y = c3_d6;
  }

  sf_mex_destroy(&c3_b_actuator_delay_p);
  chartInstance.c3_actuator_delay_p = c3_j_y;
  c3_b_actuator_value_p = sf_mex_dup(sf_mex_getcell(c3_u, 10));
  if (mxIsEmpty(c3_b_actuator_value_p)) {
    chartInstance.c3_actuator_value_p_not_empty = false;
  } else {
    chartInstance.c3_actuator_value_p_not_empty = true;
    sf_mex_import("actuator_value_p", sf_mex_dup(c3_b_actuator_value_p), &c3_d7,
                  1, 0, 0U, 0, 0U, 0);
    c3_k_y = c3_d7;
  }

  sf_mex_destroy(&c3_b_actuator_value_p);
  chartInstance.c3_actuator_value_p = c3_k_y;
  c3_b_theta_delay_p = sf_mex_dup(sf_mex_getcell(c3_u, 11));
  if (mxIsEmpty(c3_b_theta_delay_p)) {
    chartInstance.c3_theta_delay_p_not_empty = false;
  } else {
    chartInstance.c3_theta_delay_p_not_empty = true;
    sf_mex_import("theta_delay_p", sf_mex_dup(c3_b_theta_delay_p), &c3_d8, 1, 0,
                  0U, 0, 0U, 0);
    c3_l_y = c3_d8;
  }

  sf_mex_destroy(&c3_b_theta_delay_p);
  chartInstance.c3_theta_delay_p = c3_l_y;
  c3_b_theta_value_p = sf_mex_dup(sf_mex_getcell(c3_u, 12));
  if (mxIsEmpty(c3_b_theta_value_p)) {
    chartInstance.c3_theta_value_p_not_empty = false;
  } else {
    chartInstance.c3_theta_value_p_not_empty = true;
    sf_mex_import("theta_value_p", sf_mex_dup(c3_b_theta_value_p), &c3_d9, 1, 0,
                  0U, 0, 0U, 0);
    c3_m_y = c3_d9;
  }

  sf_mex_destroy(&c3_b_theta_value_p);
  chartInstance.c3_theta_value_p = c3_m_y;
  c3_b_xc_delay_p = sf_mex_dup(sf_mex_getcell(c3_u, 13));
  if (mxIsEmpty(c3_b_xc_delay_p)) {
    chartInstance.c3_xc_delay_p_not_empty = false;
  } else {
    chartInstance.c3_xc_delay_p_not_empty = true;
    sf_mex_import("xc_delay_p", sf_mex_dup(c3_b_xc_delay_p), &c3_d10, 1, 0, 0U,
                  0, 0U, 0);
    c3_n_y = c3_d10;
  }

  sf_mex_destroy(&c3_b_xc_delay_p);
  chartInstance.c3_xc_delay_p = c3_n_y;
  c3_b_xc_value_p = sf_mex_dup(sf_mex_getcell(c3_u, 14));
  if (mxIsEmpty(c3_b_xc_value_p)) {
    chartInstance.c3_xc_value_p_not_empty = false;
  } else {
    chartInstance.c3_xc_value_p_not_empty = true;
    sf_mex_import("xc_value_p", sf_mex_dup(c3_b_xc_value_p), &c3_d11, 1, 0, 0U,
                  0, 0U, 0);
    c3_o_y = c3_d11;
  }

  sf_mex_destroy(&c3_b_xc_value_p);
  chartInstance.c3_xc_value_p = c3_o_y;
  c3_b_is_active_c3_sensor_block = sf_mex_dup(sf_mex_getcell(c3_u, 15));
  sf_mex_import("is_active_c3_sensor_block", sf_mex_dup
                (c3_b_is_active_c3_sensor_block), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_p_y = c3_u0;
  sf_mex_destroy(&c3_b_is_active_c3_sensor_block);
  chartInstance.c3_is_active_c3_sensor_block = c3_p_y;
  sf_mex_destroy(&c3_u);
  c3_update_debugger_state_c3_sensor_block();
  sf_mex_destroy(&c3_st);
}

static void finalize_c3_sensor_block(void)
{
}

static void sf_c3_sensor_block(void)
{
  int32_T c3_previousEvent;
  boolean_T *c3_xc_update;
  uint16_T *c3_src_addr;
  real_T *c3_xc_value;
  real_T *c3_xc_delay;
  uint16_T *c3_sensor_v;
  uint16_T *c3_delay;
  boolean_T *c3_theta_update;
  real_T *c3_theta_value;
  real_T *c3_theta_delay;
  boolean_T *c3_actuator_update;
  real_T *c3_actuator_value;
  real_T *c3_actuator_delay;
  c3_theta_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  c3_actuator_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 7);
  c3_xc_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c3_sensor_v = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
  c3_theta_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c3_actuator_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 9);
  c3_theta_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c3_delay = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
  c3_xc_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c3_xc_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c3_src_addr = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c3_actuator_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 8);
  _sfTime_ = (real_T)ssGetT(chartInstance.S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG,2);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_xc_update, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_src_addr, 1U);
  _SFD_DATA_RANGE_CHECK(*c3_xc_value, 2U);
  _SFD_DATA_RANGE_CHECK(*c3_xc_delay, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_sensor_v, 4U);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_delay, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_theta_update, 6U);
  _SFD_DATA_RANGE_CHECK(*c3_theta_value, 7U);
  _SFD_DATA_RANGE_CHECK(*c3_theta_delay, 8U);
  _SFD_DATA_RANGE_CHECK((real_T)*c3_actuator_update, 9U);
  _SFD_DATA_RANGE_CHECK(*c3_actuator_value, 10U);
  _SFD_DATA_RANGE_CHECK(*c3_actuator_delay, 11U);
  c3_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  c3_c3_sensor_block();
  _sfEvent_ = c3_previousEvent;
  sf_debug_check_for_state_inconsistency(_sensor_blockMachineNumber_,
    chartInstance.chartNumber, chartInstance.instanceNumber);
}

static void c3_c3_sensor_block(void)
{
  uint16_T c3_src_addr;
  uint16_T c3_sensor_v;
  uint16_T c3_delay;
  real_T c3_nargout = 9.0;
  real_T c3_nargin = 3.0;
  real_T c3_NUMBER_SENSORS;
  real_T c3_NUMBER_LINKS;
  real_T c3_TIME_SYMBOL;
  real_T c3_TPENDULUM_ADDRESS;
  real_T c3_XCART_ADDRESS;
  real_T c3_COORDINATOR_ADDRESS;
  real_T c3_actuator_delay;
  real_T c3_actuator_value;
  boolean_T c3_actuator_update;
  real_T c3_theta_delay;
  real_T c3_theta_value;
  boolean_T c3_theta_update;
  real_T c3_xc_delay;
  real_T c3_xc_value;
  boolean_T c3_xc_update;
  real_T c3_a;
  real_T c3_b_a;
  real_T c3_A;
  real_T c3_x;
  real_T c3_b_x;
  real_T c3_c_x;
  real_T c3_c_a;
  real_T c3_d_a;
  real_T c3_e_a;
  real_T c3_f_a;
  real_T *c3_b_actuator_delay;
  real_T *c3_b_actuator_value;
  boolean_T *c3_b_actuator_update;
  real_T *c3_b_theta_delay;
  real_T *c3_b_theta_value;
  boolean_T *c3_b_theta_update;
  real_T *c3_b_xc_delay;
  real_T *c3_b_xc_value;
  boolean_T *c3_b_xc_update;
  uint16_T *c3_b_delay;
  uint16_T *c3_b_sensor_v;
  uint16_T *c3_b_src_addr;
  c3_b_theta_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 5);
  c3_b_actuator_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 7);
  c3_b_xc_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
  c3_b_sensor_v = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
  c3_b_theta_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 4);
  c3_b_actuator_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 9);
  c3_b_theta_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 6);
  c3_b_delay = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
  c3_b_xc_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
  c3_b_xc_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 1);
  c3_b_src_addr = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 0);
  c3_b_actuator_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 8);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,2);
  c3_src_addr = *c3_b_src_addr;
  c3_sensor_v = *c3_b_sensor_v;
  c3_delay = *c3_b_delay;
  sf_debug_symbol_scope_push(26U, 0U);
  sf_debug_symbol_scope_add("actuator_delay_p",
    &chartInstance.c3_actuator_delay_p, c3_i_sf_marshall);
  sf_debug_symbol_scope_add("actuator_value_p",
    &chartInstance.c3_actuator_value_p, c3_h_sf_marshall);
  sf_debug_symbol_scope_add("theta_delay_p", &chartInstance.c3_theta_delay_p,
    c3_g_sf_marshall);
  sf_debug_symbol_scope_add("theta_value_p", &chartInstance.c3_theta_value_p,
    c3_f_sf_marshall);
  sf_debug_symbol_scope_add("xc_delay_p", &chartInstance.c3_xc_delay_p,
    c3_e_sf_marshall);
  sf_debug_symbol_scope_add("xc_value_p", &chartInstance.c3_xc_value_p,
    c3_d_sf_marshall);
  sf_debug_symbol_scope_add("nargout", &c3_nargout, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("nargin", &c3_nargin, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("NUMBER_SENSORS", &c3_NUMBER_SENSORS,
    c3_c_sf_marshall);
  sf_debug_symbol_scope_add("NUMBER_LINKS", &c3_NUMBER_LINKS, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("TIME_SYMBOL", &c3_TIME_SYMBOL, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("TPENDULUM_ADDRESS", &c3_TPENDULUM_ADDRESS,
    c3_c_sf_marshall);
  sf_debug_symbol_scope_add("XCART_ADDRESS", &c3_XCART_ADDRESS, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("COORDINATOR_ADDRESS", &c3_COORDINATOR_ADDRESS,
    c3_c_sf_marshall);
  sf_debug_symbol_scope_add("actuator_delay", &c3_actuator_delay,
    c3_c_sf_marshall);
  sf_debug_symbol_scope_add("actuator_value", &c3_actuator_value,
    c3_c_sf_marshall);
  sf_debug_symbol_scope_add("actuator_update", &c3_actuator_update,
    c3_b_sf_marshall);
  sf_debug_symbol_scope_add("theta_delay", &c3_theta_delay, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("theta_value", &c3_theta_value, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("theta_update", &c3_theta_update, c3_b_sf_marshall);
  sf_debug_symbol_scope_add("xc_delay", &c3_xc_delay, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("xc_value", &c3_xc_value, c3_c_sf_marshall);
  sf_debug_symbol_scope_add("xc_update", &c3_xc_update, c3_b_sf_marshall);
  sf_debug_symbol_scope_add("delay", &c3_delay, c3_sf_marshall);
  sf_debug_symbol_scope_add("sensor_v", &c3_sensor_v, c3_sf_marshall);
  sf_debug_symbol_scope_add("src_addr", &c3_src_addr, c3_sf_marshall);
  CV_EML_FCN(0, 0);

  /*   */
  _SFD_EML_CALL(0,9);
  c3_COORDINATOR_ADDRESS = 0.0;
  _SFD_EML_CALL(0,10);
  c3_XCART_ADDRESS = 1.0;
  _SFD_EML_CALL(0,11);
  c3_TPENDULUM_ADDRESS = 2.0;
  _SFD_EML_CALL(0,13);
  c3_TIME_SYMBOL = 1.5300814003304975E-02;
  _SFD_EML_CALL(0,15);
  c3_NUMBER_LINKS = 2.0;
  _SFD_EML_CALL(0,16);
  c3_NUMBER_SENSORS = 2.0;

  /*  First initialization of the variables */
  _SFD_EML_CALL(0,20);
  if (CV_EML_IF(0, 0, !chartInstance.c3_xc_value_p_not_empty)) {
    _SFD_EML_CALL(0,22);
    chartInstance.c3_xc_value_p = 0.0;
    chartInstance.c3_xc_value_p_not_empty = true;
    _SFD_EML_CALL(0,23);
    chartInstance.c3_xc_delay_p = 0.0;
    chartInstance.c3_xc_delay_p_not_empty = true;
    _SFD_EML_CALL(0,25);
    chartInstance.c3_theta_value_p = 0.0;
    chartInstance.c3_theta_value_p_not_empty = true;
    _SFD_EML_CALL(0,26);
    chartInstance.c3_theta_delay_p = 0.0;
    chartInstance.c3_theta_delay_p_not_empty = true;
    _SFD_EML_CALL(0,28);
    chartInstance.c3_actuator_value_p = 0.0;
    chartInstance.c3_actuator_value_p_not_empty = true;
    _SFD_EML_CALL(0,29);
    chartInstance.c3_actuator_delay_p = 0.0;
    chartInstance.c3_actuator_delay_p_not_empty = true;
  }

  _SFD_EML_CALL(0,32);
  c3_xc_update = false;
  _SFD_EML_CALL(0,33);
  c3_theta_update = false;
  _SFD_EML_CALL(0,34);
  c3_actuator_update = false;
  _SFD_EML_CALL(0,35);
  if (CV_EML_COND(0, 0, (real_T)c3_src_addr <= (c3_NUMBER_SENSORS +
        c3_NUMBER_LINKS) - 1.0)) {
    if (CV_EML_COND(0, 1, (real_T)c3_src_addr > 0.0)) {
      CV_EML_MCDC(0, 0, true);
      CV_EML_IF(0, 1, true);

      /*  */
      /*  SENSOR ---- > COORDINATOR */
      /*  */
      /*  Set the new values for the desired sensors */
      _SFD_EML_CALL(0,41);
      if (CV_EML_IF(0, 2, (real_T)c3_src_addr == c3_XCART_ADDRESS)) {
        _SFD_EML_CALL(0,43);
        chartInstance.c3_xc_value_p = (real_T)c3_sensor_v;
        _SFD_EML_CALL(0,44);
        chartInstance.c3_xc_delay_p = (real_T)c3_delay;
        _SFD_EML_CALL(0,45);
        c3_xc_update = true;
      } else {
        _SFD_EML_CALL(0,47);
        if (CV_EML_IF(0, 3, (real_T)c3_src_addr == c3_TPENDULUM_ADDRESS)) {
          _SFD_EML_CALL(0,49);
          chartInstance.c3_theta_value_p = (real_T)c3_sensor_v;
          _SFD_EML_CALL(0,50);
          chartInstance.c3_theta_delay_p = (real_T)c3_delay;
          _SFD_EML_CALL(0,51);
          c3_theta_update = true;
        } else {
          _SFD_EML_CALL(0,53);
          if (CV_EML_IF(0, 4, (real_T)c3_src_addr == c3_COORDINATOR_ADDRESS)) {
            /*  */
            /*  COORDIANTOR ---- > ACTUATOR */
            /*  */
            _SFD_EML_CALL(0,57);
            chartInstance.c3_actuator_value_p = (real_T)c3_sensor_v;
            _SFD_EML_CALL(0,58);
            chartInstance.c3_actuator_delay_p = (real_T)c3_delay;
            _SFD_EML_CALL(0,59);
            c3_actuator_update = true;
          }
        }
      }

      goto label_1;
    }
  }

  CV_EML_MCDC(0, 0, false);
  CV_EML_IF(0, 1, false);
 label_1:
  ;
  _SFD_EML_CALL(0,63);
  c3_a = chartInstance.c3_xc_value_p;
  c3_xc_value = c3_a * 9.2E-05;
  _SFD_EML_CALL(0,64);
  c3_b_a = chartInstance.c3_xc_delay_p;
  c3_xc_delay = c3_b_a * 1.5300814003304975E-02;
  _SFD_EML_CALL(0,66);
  c3_A = chartInstance.c3_theta_value_p;
  c3_x = c3_A;
  c3_b_x = c3_x;
  c3_c_x = c3_b_x;
  c3_c_a = c3_c_x / 1024.0;
  c3_d_a = c3_c_a * 2.0;
  c3_theta_value = c3_d_a * 3.14159265;
  _SFD_EML_CALL(0,67);
  c3_e_a = chartInstance.c3_theta_delay_p;
  c3_theta_delay = c3_e_a * 1.5300814003304975E-02;
  _SFD_EML_CALL(0,69);
  c3_actuator_value = chartInstance.c3_actuator_value_p;
  _SFD_EML_CALL(0,70);
  c3_f_a = chartInstance.c3_actuator_delay_p;
  c3_actuator_delay = c3_f_a * 1.5300814003304975E-02;
  _SFD_EML_CALL(0,-70);
  sf_debug_symbol_scope_pop();
  *c3_b_xc_update = c3_xc_update;
  *c3_b_xc_value = c3_xc_value;
  *c3_b_xc_delay = c3_xc_delay;
  *c3_b_theta_update = c3_theta_update;
  *c3_b_theta_value = c3_theta_value;
  *c3_b_theta_delay = c3_theta_delay;
  *c3_b_actuator_update = c3_actuator_update;
  *c3_b_actuator_value = c3_actuator_value;
  *c3_b_actuator_delay = c3_actuator_delay;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,2);
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber)
{
}

static const mxArray *c3_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  uint16_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((uint16_T *)c3_u);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 5, 0U, 0U, 0U, 0));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_b_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  boolean_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((boolean_T *)c3_u);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 11, 0U, 0U, 0U, 0));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_c_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_d_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_xc_value_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_e_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_xc_delay_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_f_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_theta_value_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_g_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_theta_delay_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_h_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_actuator_value_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

static const mxArray *c3_i_sf_marshall(void *c3_chartInstance, void *c3_u)
{
  const mxArray *c3_y = NULL;
  real_T c3_b_u;
  const mxArray *c3_b_y = NULL;
  c3_y = NULL;
  c3_b_u = *((real_T *)c3_u);
  c3_b_y = NULL;
  if (!chartInstance.c3_actuator_delay_p_not_empty) {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0));
  } else {
    sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0));
  }

  sf_mex_assign(&c3_y, c3_b_y);
  return c3_y;
}

const mxArray *sf_c3_sensor_block_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_ResolvedFunctionInfo c3_info[24];
  const mxArray *c3_m0 = NULL;
  int32_T c3_i0;
  c3_ResolvedFunctionInfo *c3_r0;
  c3_nameCaptureInfo = NULL;
  c3_info_helper(c3_info);
  sf_mex_assign(&c3_m0, sf_mex_createstruct("nameCaptureInfo", 1, 24));
  for (c3_i0 = 0; c3_i0 < 24; c3_i0 = c3_i0 + 1) {
    c3_r0 = &c3_info[c3_i0];
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->context)), "context",
                    "nameCaptureInfo", c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c3_r0->name)), "name",
                    "nameCaptureInfo", c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c3_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", c3_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c3_r0->resolved)), "resolved"
                    , "nameCaptureInfo", c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileLength,
      7, 0U, 0U, 0U, 0), "fileLength", "nameCaptureInfo",
                    c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTime1, 7,
      0U, 0U, 0U, 0), "fileTime1", "nameCaptureInfo", c3_i0);
    sf_mex_addfield(c3_m0, sf_mex_create("nameCaptureInfo", &c3_r0->fileTime2, 7,
      0U, 0U, 0U, 0), "fileTime2", "nameCaptureInfo", c3_i0);
  }

  sf_mex_assign(&c3_nameCaptureInfo, c3_m0);
  return c3_nameCaptureInfo;
}

static void c3_info_helper(c3_ResolvedFunctionInfo c3_info[24])
{
  c3_info[0].context = "";
  c3_info[0].name = "mtimes";
  c3_info[0].dominantType = "double";
  c3_info[0].resolved = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[0].fileLength = 3302U;
  c3_info[0].fileTime1 = 1242772494U;
  c3_info[0].fileTime2 = 0U;
  c3_info[1].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[1].name = "nargin";
  c3_info[1].dominantType = "";
  c3_info[1].resolved = "[B]nargin";
  c3_info[1].fileLength = 0U;
  c3_info[1].fileTime1 = 0U;
  c3_info[1].fileTime2 = 0U;
  c3_info[2].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[2].name = "gt";
  c3_info[2].dominantType = "double";
  c3_info[2].resolved = "[B]gt";
  c3_info[2].fileLength = 0U;
  c3_info[2].fileTime1 = 0U;
  c3_info[2].fileTime2 = 0U;
  c3_info[3].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[3].name = "isa";
  c3_info[3].dominantType = "double";
  c3_info[3].resolved = "[B]isa";
  c3_info[3].fileLength = 0U;
  c3_info[3].fileTime1 = 0U;
  c3_info[3].fileTime2 = 0U;
  c3_info[4].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[4].name = "isinteger";
  c3_info[4].dominantType = "double";
  c3_info[4].resolved = "[B]isinteger";
  c3_info[4].fileLength = 0U;
  c3_info[4].fileTime1 = 0U;
  c3_info[4].fileTime2 = 0U;
  c3_info[5].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[5].name = "isscalar";
  c3_info[5].dominantType = "double";
  c3_info[5].resolved = "[B]isscalar";
  c3_info[5].fileLength = 0U;
  c3_info[5].fileTime1 = 0U;
  c3_info[5].fileTime2 = 0U;
  c3_info[6].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[6].name = "strcmp";
  c3_info[6].dominantType = "char";
  c3_info[6].resolved = "[B]strcmp";
  c3_info[6].fileLength = 0U;
  c3_info[6].fileTime1 = 0U;
  c3_info[6].fileTime2 = 0U;
  c3_info[7].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[7].name = "size";
  c3_info[7].dominantType = "double";
  c3_info[7].resolved = "[B]size";
  c3_info[7].fileLength = 0U;
  c3_info[7].fileTime1 = 0U;
  c3_info[7].fileTime2 = 0U;
  c3_info[8].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[8].name = "eq";
  c3_info[8].dominantType = "double";
  c3_info[8].resolved = "[B]eq";
  c3_info[8].fileLength = 0U;
  c3_info[8].fileTime1 = 0U;
  c3_info[8].fileTime2 = 0U;
  c3_info[9].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[9].name = "class";
  c3_info[9].dominantType = "double";
  c3_info[9].resolved = "[B]class";
  c3_info[9].fileLength = 0U;
  c3_info[9].fileTime1 = 0U;
  c3_info[9].fileTime2 = 0U;
  c3_info[10].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m";
  c3_info[10].name = "not";
  c3_info[10].dominantType = "logical";
  c3_info[10].resolved = "[B]not";
  c3_info[10].fileLength = 0U;
  c3_info[10].fileTime1 = 0U;
  c3_info[10].fileTime2 = 0U;
  c3_info[11].context = "";
  c3_info[11].name = "mrdivide";
  c3_info[11].dominantType = "double";
  c3_info[11].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[11].fileLength = 800U;
  c3_info[11].fileTime1 = 1238455891U;
  c3_info[11].fileTime2 = 0U;
  c3_info[12].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[12].name = "ge";
  c3_info[12].dominantType = "double";
  c3_info[12].resolved = "[B]ge";
  c3_info[12].fileLength = 0U;
  c3_info[12].fileTime1 = 0U;
  c3_info[12].fileTime2 = 0U;
  c3_info[13].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.m";
  c3_info[13].name = "rdivide";
  c3_info[13].dominantType = "double";
  c3_info[13].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[13].fileLength = 620U;
  c3_info[13].fileTime1 = 1213948366U;
  c3_info[13].fileTime2 = 0U;
  c3_info[14].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[14].name = "isempty";
  c3_info[14].dominantType = "double";
  c3_info[14].resolved = "[B]isempty";
  c3_info[14].fileLength = 0U;
  c3_info[14].fileTime1 = 0U;
  c3_info[14].fileTime2 = 0U;
  c3_info[15].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[15].name = "eml_warning";
  c3_info[15].dominantType = "char";
  c3_info[15].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c3_info[15].fileLength = 262U;
  c3_info[15].fileTime1 = 1236278878U;
  c3_info[15].fileTime2 = 0U;
  c3_info[16].context = "[ILX]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m";
  c3_info[16].name = "eml_div";
  c3_info[16].dominantType = "double";
  c3_info[16].resolved =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m";
  c3_info[16].fileLength = 4269U;
  c3_info[16].fileTime1 = 1228115426U;
  c3_info[16].fileTime2 = 0U;
  c3_info[17].context =
    "[ILX]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m/eml_fldiv";
  c3_info[17].name = "isreal";
  c3_info[17].dominantType = "double";
  c3_info[17].resolved = "[B]isreal";
  c3_info[17].fileLength = 0U;
  c3_info[17].fileTime1 = 0U;
  c3_info[17].fileTime2 = 0U;
  c3_info[18].context = "";
  c3_info[18].name = "double";
  c3_info[18].dominantType = "double";
  c3_info[18].resolved = "[B]double";
  c3_info[18].fileLength = 0U;
  c3_info[18].fileTime1 = 0U;
  c3_info[18].fileTime2 = 0U;
  c3_info[19].context = "";
  c3_info[19].name = "false";
  c3_info[19].dominantType = "";
  c3_info[19].resolved = "[B]false";
  c3_info[19].fileLength = 0U;
  c3_info[19].fileTime1 = 0U;
  c3_info[19].fileTime2 = 0U;
  c3_info[20].context = "";
  c3_info[20].name = "plus";
  c3_info[20].dominantType = "double";
  c3_info[20].resolved = "[B]plus";
  c3_info[20].fileLength = 0U;
  c3_info[20].fileTime1 = 0U;
  c3_info[20].fileTime2 = 0U;
  c3_info[21].context = "";
  c3_info[21].name = "minus";
  c3_info[21].dominantType = "double";
  c3_info[21].resolved = "[B]minus";
  c3_info[21].fileLength = 0U;
  c3_info[21].fileTime1 = 0U;
  c3_info[21].fileTime2 = 0U;
  c3_info[22].context = "";
  c3_info[22].name = "le";
  c3_info[22].dominantType = "uint16";
  c3_info[22].resolved = "[B]le";
  c3_info[22].fileLength = 0U;
  c3_info[22].fileTime1 = 0U;
  c3_info[22].fileTime2 = 0U;
  c3_info[23].context = "";
  c3_info[23].name = "true";
  c3_info[23].dominantType = "";
  c3_info[23].resolved = "[B]true";
  c3_info[23].fileLength = 0U;
  c3_info[23].fileTime1 = 0U;
  c3_info[23].fileTime2 = 0U;
}

static void init_dsm_address_info(void)
{
}

/* SFunction Glue Code */
void sf_c3_sensor_block_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4275111201U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3700921421U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2442353154U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1051433000U);
}

mxArray *sf_c3_sensor_block_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,4,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
    double *pr = mxGetPr(mxChecksum);
    pr[0] = (double)(798216253U);
    pr[1] = (double)(4254533296U);
    pr[2] = (double)(3451626212U);
    pr[3] = (double)(2941162221U);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,9,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  return(mxAutoinheritanceInfo);
}

static mxArray *sf_get_sim_state_info_c3_sensor_block(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[23],T\"actuator_delay\",},{M[1],M[14],T\"actuator_update\",},{M[1],M[20],T\"actuator_value\",},{M[1],M[16],T\"theta_delay\",},{M[1],M[11],T\"theta_update\",},{M[1],M[13],T\"theta_value\",},{M[1],M[12],T\"xc_delay\",},{M[1],M[10],T\"xc_update\",},{M[1],M[5],T\"xc_value\",},{M[4],M[0],T\"actuator_delay_p\",S'l','i','p'{{M1x2[301 317],M[0],}}}}",
    "100 S1x6'type','srcId','name','auxInfo'{{M[4],M[0],T\"actuator_value_p\",S'l','i','p'{{M1x2[284 300],M[0],}}},{M[4],M[0],T\"theta_delay_p\",S'l','i','p'{{M1x2[261 274],M[0],}}},{M[4],M[0],T\"theta_value_p\",S'l','i','p'{{M1x2[247 260],M[0],}}},{M[4],M[0],T\"xc_delay_p\",S'l','i','p'{{M1x2[227 237],M[0],}}},{M[4],M[0],T\"xc_value_p\",S'l','i','p'{{M1x2[216 226],M[0],}}},{M[8],M[0],T\"is_active_c3_sensor_block\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 16, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_sensor_block_get_check_sum(&mxChecksum);
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
           3,
           1,
           1,
           12,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_update",0,(MexFcnForType)c3_b_sf_marshall);
          _SFD_SET_DATA_PROPS(1,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "src_addr",0,(MexFcnForType)c3_sf_marshall);
          _SFD_SET_DATA_PROPS(2,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_value",0,(MexFcnForType)c3_c_sf_marshall);
          _SFD_SET_DATA_PROPS(3,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "xc_delay",0,(MexFcnForType)c3_c_sf_marshall);
          _SFD_SET_DATA_PROPS(4,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,
                              "sensor_v",0,(MexFcnForType)c3_sf_marshall);
          _SFD_SET_DATA_PROPS(5,1,1,0,SF_UINT16,0,NULL,0,0,0,0.0,1.0,0,"delay",0,
                              (MexFcnForType)c3_sf_marshall);
          _SFD_SET_DATA_PROPS(6,2,0,1,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_update",0,(MexFcnForType)c3_b_sf_marshall);
          _SFD_SET_DATA_PROPS(7,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_value",0,(MexFcnForType)c3_c_sf_marshall);
          _SFD_SET_DATA_PROPS(8,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "theta_delay",0,(MexFcnForType)c3_c_sf_marshall);
          _SFD_SET_DATA_PROPS(9,2,0,1,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_update",0,(MexFcnForType)
                              c3_b_sf_marshall);
          _SFD_SET_DATA_PROPS(10,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_value",0,(MexFcnForType)c3_c_sf_marshall);
          _SFD_SET_DATA_PROPS(11,2,0,1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,
                              "actuator_delay",0,(MexFcnForType)c3_c_sf_marshall);
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1882);
        _SFD_CV_INIT_EML_IF(0,0,501,523,-1,729);
        _SFD_CV_INIT_EML_IF(0,1,800,869,-1,1625);
        _SFD_CV_INIT_EML_IF(0,2,998,1026,1174,1617);
        _SFD_CV_INIT_EML_IF(0,3,1174,1210,1366,1617);
        _SFD_CV_INIT_EML_IF(0,4,1366,1404,-1,1404);

        {
          static int condStart[] = { 805, 857 };

          static int condEnd[] = { 852, 869 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,0,805,869,2,0,&(condStart[0]),&(condEnd[0]),3,
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
          boolean_T *c3_xc_update;
          uint16_T *c3_src_addr;
          real_T *c3_xc_value;
          real_T *c3_xc_delay;
          uint16_T *c3_sensor_v;
          uint16_T *c3_delay;
          boolean_T *c3_theta_update;
          real_T *c3_theta_value;
          real_T *c3_theta_delay;
          boolean_T *c3_actuator_update;
          real_T *c3_actuator_value;
          real_T *c3_actuator_delay;
          c3_theta_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 5);
          c3_actuator_update = (boolean_T *)ssGetOutputPortSignal
            (chartInstance.S, 7);
          c3_xc_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 3);
          c3_sensor_v = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 1);
          c3_theta_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S,
            4);
          c3_actuator_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 9);
          c3_theta_delay = (real_T *)ssGetOutputPortSignal(chartInstance.S, 6);
          c3_delay = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 2);
          c3_xc_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 2);
          c3_xc_update = (boolean_T *)ssGetOutputPortSignal(chartInstance.S, 1);
          c3_src_addr = (uint16_T *)ssGetInputPortSignal(chartInstance.S, 0);
          c3_actuator_value = (real_T *)ssGetOutputPortSignal(chartInstance.S, 8);
          _SFD_SET_DATA_VALUE_PTR(0U, c3_xc_update);
          _SFD_SET_DATA_VALUE_PTR(1U, c3_src_addr);
          _SFD_SET_DATA_VALUE_PTR(2U, c3_xc_value);
          _SFD_SET_DATA_VALUE_PTR(3U, c3_xc_delay);
          _SFD_SET_DATA_VALUE_PTR(4U, c3_sensor_v);
          _SFD_SET_DATA_VALUE_PTR(5U, c3_delay);
          _SFD_SET_DATA_VALUE_PTR(6U, c3_theta_update);
          _SFD_SET_DATA_VALUE_PTR(7U, c3_theta_value);
          _SFD_SET_DATA_VALUE_PTR(8U, c3_theta_delay);
          _SFD_SET_DATA_VALUE_PTR(9U, c3_actuator_update);
          _SFD_SET_DATA_VALUE_PTR(10U, c3_actuator_value);
          _SFD_SET_DATA_VALUE_PTR(11U, c3_actuator_delay);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_sensor_blockMachineNumber_,
        chartInstance.chartNumber,chartInstance.instanceNumber);
    }
  }
}

static void sf_opaque_initialize_c3_sensor_block(void *chartInstanceVar)
{
  chart_debug_initialization(chartInstance.S,0);
  initialize_params_c3_sensor_block();
  initialize_c3_sensor_block();
}

static void sf_opaque_enable_c3_sensor_block(void *chartInstanceVar)
{
  enable_c3_sensor_block();
}

static void sf_opaque_disable_c3_sensor_block(void *chartInstanceVar)
{
  disable_c3_sensor_block();
}

static void sf_opaque_gateway_c3_sensor_block(void *chartInstanceVar)
{
  sf_c3_sensor_block();
}

static mxArray* sf_opaque_get_sim_state_c3_sensor_block(void *chartInstanceVar)
{
  mxArray *st = (mxArray *) get_sim_state_c3_sensor_block();
  return st;
}

static void sf_opaque_set_sim_state_c3_sensor_block(void *chartInstanceVar,
  const mxArray *st)
{
  set_sim_state_c3_sensor_block(sf_mex_dup(st));
}

static void sf_opaque_terminate_c3_sensor_block(void *chartInstanceVar)
{
  if (sim_mode_is_rtw_gen(chartInstance.S) || sim_mode_is_external
      (chartInstance.S)) {
    sf_clear_rtw_identifier(chartInstance.S);
  }

  finalize_c3_sensor_block();
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_sensor_block(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_sensor_block();
  }
}

static void mdlSetWorkWidths_c3_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("sensor_block","sensor_block",3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop("sensor_block","sensor_block",3,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop("sensor_block",
      "sensor_block",3,"gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"sensor_block","sensor_block",3,3);
      sf_mark_chart_reusable_outputs(S,"sensor_block","sensor_block",3,9);
    }

    sf_set_rtw_dwork_info(S,"sensor_block","sensor_block",3);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(71223634U));
  ssSetChecksum1(S,(818764129U));
  ssSetChecksum2(S,(2037748263U));
  ssSetChecksum3(S,(2133599336U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c3_sensor_block(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    sf_write_symbol_mapping(S, "sensor_block", "sensor_block",3);
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c3_sensor_block(SimStruct *S)
{
  chartInstance.chartInfo.chartInstance = NULL;
  chartInstance.chartInfo.isEMLChart = 1;
  chartInstance.chartInfo.chartInitialized = 0;
  chartInstance.chartInfo.sFunctionGateway = sf_opaque_gateway_c3_sensor_block;
  chartInstance.chartInfo.initializeChart = sf_opaque_initialize_c3_sensor_block;
  chartInstance.chartInfo.terminateChart = sf_opaque_terminate_c3_sensor_block;
  chartInstance.chartInfo.enableChart = sf_opaque_enable_c3_sensor_block;
  chartInstance.chartInfo.disableChart = sf_opaque_disable_c3_sensor_block;
  chartInstance.chartInfo.getSimState = sf_opaque_get_sim_state_c3_sensor_block;
  chartInstance.chartInfo.setSimState = sf_opaque_set_sim_state_c3_sensor_block;
  chartInstance.chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_sensor_block;
  chartInstance.chartInfo.zeroCrossings = NULL;
  chartInstance.chartInfo.outputs = NULL;
  chartInstance.chartInfo.derivatives = NULL;
  chartInstance.chartInfo.mdlRTW = mdlRTW_c3_sensor_block;
  chartInstance.chartInfo.mdlStart = mdlStart_c3_sensor_block;
  chartInstance.chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_sensor_block;
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

void c3_sensor_block_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_sensor_block(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_sensor_block(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_sensor_block(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_sensor_block_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
