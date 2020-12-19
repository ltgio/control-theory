/* Include files */

#include "magLev_sim_sfun.h"
#include "c1_magLev_sim.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "magLev_sim_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[11] = { "Ac", "Bc", "nargin",
  "nargout", "DataGS", "error", "x", "yeq", "ueq", "Cc", "xdot" };

/* Function Declarations */
static void initialize_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance);
static void initialize_params_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance);
static void enable_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance);
static void disable_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct *
  chartInstance);
static void set_sim_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance);
static void sf_gateway_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance);
static void mdl_start_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance);
static void initSimStructsc1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_xdot, const char_T *c1_identifier, real_T c1_y[3]);
static void c1_b_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[3]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_c_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_Cc, const char_T *c1_identifier, real_T c1_y[3]);
static void c1_d_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[3]);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_e_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_ueq, const char_T *c1_identifier);
static real_T c1_f_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_g_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[9]);
static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_h_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_DataGS_bus_io(void *chartInstanceVoid, void *c1_pData);
static uint8_T c1_i_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_magLev_sim, const char_T
  *c1_identifier);
static uint8_T c1_j_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_magLev_simInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc1_magLev_simInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_magLev_sim(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_magLev_sim = 0U;
}

static void initialize_params_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct *
  chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  const mxArray *c1_b_y = NULL;
  real_T c1_hoistedGlobal;
  const mxArray *c1_c_y = NULL;
  const mxArray *c1_d_y = NULL;
  real_T c1_b_hoistedGlobal;
  const mxArray *c1_e_y = NULL;
  uint8_T c1_c_hoistedGlobal;
  const mxArray *c1_f_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(5, 1), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", *chartInstance->c1_Cc, 0, 0U, 1U, 0U,
    2, 1, 3), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_hoistedGlobal = *chartInstance->c1_ueq;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_hoistedGlobal, 0, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", *chartInstance->c1_xdot, 0, 0U, 1U,
    0U, 1, 3), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_b_hoistedGlobal = *chartInstance->c1_yeq;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_b_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_c_hoistedGlobal = chartInstance->c1_is_active_c1_magLev_sim;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_c_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[3];
  int32_T c1_i0;
  real_T c1_dv1[3];
  int32_T c1_i1;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("Cc", c1_u, 0)),
                        "Cc", c1_dv0);
  for (c1_i0 = 0; c1_i0 < 3; c1_i0++) {
    (*chartInstance->c1_Cc)[c1_i0] = c1_dv0[c1_i0];
  }

  *chartInstance->c1_ueq = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("ueq", c1_u, 1)), "ueq");
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("xdot", c1_u, 2)),
                      "xdot", c1_dv1);
  for (c1_i1 = 0; c1_i1 < 3; c1_i1++) {
    (*chartInstance->c1_xdot)[c1_i1] = c1_dv1[c1_i1];
  }

  *chartInstance->c1_yeq = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("yeq", c1_u, 3)), "yeq");
  chartInstance->c1_is_active_c1_magLev_sim = c1_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c1_magLev_sim", c1_u, 4)),
     "is_active_c1_magLev_sim");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_magLev_sim(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  int32_T c1_i2;
  real_T c1_hoistedGlobal;
  c1_GainSchedulingStruct c1_b_DataGS;
  int32_T c1_i3;
  int32_T c1_i4;
  int32_T c1_i5;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_b_error;
  int32_T c1_i8;
  uint32_T c1_debug_family_var_map[11];
  real_T c1_b_x[3];
  real_T c1_Ac[9];
  real_T c1_Bc[3];
  real_T c1_nargin = 3.0;
  real_T c1_nargout = 4.0;
  real_T c1_b_yeq;
  real_T c1_b_ueq;
  real_T c1_b_Cc[3];
  real_T c1_b_xdot[3];
  int32_T c1_i9;
  int32_T c1_i10;
  int32_T c1_i11;
  int32_T c1_i12;
  int32_T c1_i13;
  real_T c1_a[9];
  int32_T c1_i14;
  real_T c1_b[3];
  int32_T c1_i15;
  real_T c1_y[3];
  int32_T c1_i16;
  int32_T c1_i17;
  real_T c1_b_b;
  int32_T c1_i18;
  int32_T c1_i19;
  int32_T c1_i20;
  int32_T c1_i21;
  int32_T c1_i22;
  int32_T c1_i23;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i2 = 0; c1_i2 < 3; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_x)[c1_i2], 2U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_error, 1U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_error;
  c1_b_DataGS.parameter = *(real_T *)&((char_T *)chartInstance->c1_DataGS)[0];
  c1_b_DataGS.ueq = *(real_T *)&((char_T *)chartInstance->c1_DataGS)[8];
  c1_b_DataGS.yeq = *(real_T *)&((char_T *)chartInstance->c1_DataGS)[16];
  c1_i3 = 0;
  for (c1_i4 = 0; c1_i4 < 3; c1_i4++) {
    for (c1_i6 = 0; c1_i6 < 3; c1_i6++) {
      c1_b_DataGS.Ac[c1_i6 + c1_i3] = ((real_T *)&((char_T *)
        chartInstance->c1_DataGS)[24])[c1_i6 + c1_i3];
    }

    c1_i3 += 3;
  }

  for (c1_i5 = 0; c1_i5 < 3; c1_i5++) {
    c1_b_DataGS.Bc[c1_i5] = ((real_T *)&((char_T *)chartInstance->c1_DataGS)[96])
      [c1_i5];
  }

  for (c1_i7 = 0; c1_i7 < 3; c1_i7++) {
    c1_b_DataGS.Cc[c1_i7] = ((real_T *)&((char_T *)chartInstance->c1_DataGS)[120])
      [c1_i7];
  }

  c1_b_DataGS.Dc = *(real_T *)&((char_T *)chartInstance->c1_DataGS)[144];
  c1_b_error = c1_hoistedGlobal;
  for (c1_i8 = 0; c1_i8 < 3; c1_i8++) {
    c1_b_x[c1_i8] = (*chartInstance->c1_x)[c1_i8];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_Ac, 0U, c1_e_sf_marshallOut,
    c1_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_Bc, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 2U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 3U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_DataGS, 4U, c1_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_error, 5U, c1_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_x, 6U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_yeq, 7U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_ueq, 8U, c1_c_sf_marshallOut,
    c1_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_Cc, 9U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_xdot, 10U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 2);
  c1_b_ueq = c1_b_DataGS.ueq;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  c1_b_yeq = c1_b_DataGS.yeq;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  for (c1_i9 = 0; c1_i9 < 9; c1_i9++) {
    c1_Ac[c1_i9] = c1_b_DataGS.Ac[c1_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  for (c1_i10 = 0; c1_i10 < 3; c1_i10++) {
    c1_Bc[c1_i10] = c1_b_DataGS.Bc[c1_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  for (c1_i11 = 0; c1_i11 < 3; c1_i11++) {
    c1_b_Cc[c1_i11] = c1_b_DataGS.Cc[c1_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  for (c1_i12 = 0; c1_i12 < 9; c1_i12++) {
    c1_a[c1_i12] = c1_Ac[c1_i12];
  }

  for (c1_i13 = 0; c1_i13 < 3; c1_i13++) {
    c1_b[c1_i13] = c1_b_x[c1_i13];
  }

  for (c1_i14 = 0; c1_i14 < 3; c1_i14++) {
    c1_y[c1_i14] = 0.0;
    c1_i16 = 0;
    for (c1_i17 = 0; c1_i17 < 3; c1_i17++) {
      c1_y[c1_i14] += c1_a[c1_i16 + c1_i14] * c1_b[c1_i17];
      c1_i16 += 3;
    }
  }

  for (c1_i15 = 0; c1_i15 < 3; c1_i15++) {
    c1_b[c1_i15] = c1_Bc[c1_i15];
  }

  c1_b_b = c1_b_error;
  for (c1_i18 = 0; c1_i18 < 3; c1_i18++) {
    c1_b[c1_i18] *= c1_b_b;
  }

  for (c1_i19 = 0; c1_i19 < 3; c1_i19++) {
    c1_b_xdot[c1_i19] = c1_y[c1_i19] + c1_b[c1_i19];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c1_yeq = c1_b_yeq;
  *chartInstance->c1_ueq = c1_b_ueq;
  for (c1_i20 = 0; c1_i20 < 3; c1_i20++) {
    (*chartInstance->c1_Cc)[c1_i20] = c1_b_Cc[c1_i20];
  }

  for (c1_i21 = 0; c1_i21 < 3; c1_i21++) {
    (*chartInstance->c1_xdot)[c1_i21] = c1_b_xdot[c1_i21];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_magLev_simMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_yeq, 3U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_ueq, 4U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  for (c1_i22 = 0; c1_i22 < 3; c1_i22++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_Cc)[c1_i22], 5U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  for (c1_i23 = 0; c1_i23 < 3; c1_i23++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_xdot)[c1_i23], 6U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }
}

static void mdl_start_c1_magLev_sim(SFc1_magLev_simInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc1_magLev_sim(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i24;
  const mxArray *c1_y = NULL;
  real_T c1_u[3];
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i24 = 0; c1_i24 < 3; c1_i24++) {
    c1_u[c1_i24] = (*(real_T (*)[3])c1_inData)[c1_i24];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_xdot, const char_T *c1_identifier, real_T c1_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xdot), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_xdot);
}

static void c1_b_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[3])
{
  real_T c1_dv2[3];
  int32_T c1_i25;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv2, 1, 0, 0U, 1, 0U, 1, 3);
  for (c1_i25 = 0; c1_i25 < 3; c1_i25++) {
    c1_y[c1_i25] = c1_dv2[c1_i25];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_xdot;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[3];
  int32_T c1_i26;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_b_xdot = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_xdot), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_xdot);
  for (c1_i26 = 0; c1_i26 < 3; c1_i26++) {
    (*(real_T (*)[3])c1_outData)[c1_i26] = c1_y[c1_i26];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i27;
  const mxArray *c1_y = NULL;
  real_T c1_u[3];
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i27 = 0; c1_i27 < 3; c1_i27++) {
    c1_u[c1_i27] = (*(real_T (*)[3])c1_inData)[c1_i27];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_c_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_Cc, const char_T *c1_identifier, real_T c1_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Cc), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_Cc);
}

static void c1_d_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[3])
{
  real_T c1_dv3[3];
  int32_T c1_i28;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv3, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c1_i28 = 0; c1_i28 < 3; c1_i28++) {
    c1_y[c1_i28] = c1_dv3[c1_i28];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_Cc;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[3];
  int32_T c1_i29;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_b_Cc = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_Cc), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_b_Cc);
  for (c1_i29 = 0; c1_i29 < 3; c1_i29++) {
    (*(real_T (*)[3])c1_outData)[c1_i29] = c1_y[c1_i29];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_e_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_b_ueq, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_ueq), &c1_thisId);
  sf_mex_destroy(&c1_b_ueq);
  return c1_y;
}

static real_T c1_f_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_ueq;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_b_ueq = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_ueq), &c1_thisId);
  sf_mex_destroy(&c1_b_ueq);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  c1_GainSchedulingStruct c1_u;
  const mxArray *c1_y = NULL;
  real_T c1_b_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_c_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_d_u;
  const mxArray *c1_d_y = NULL;
  int32_T c1_i30;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_u[9];
  int32_T c1_i31;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_u[3];
  int32_T c1_i32;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_u[3];
  real_T c1_h_u;
  const mxArray *c1_h_y = NULL;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(c1_GainSchedulingStruct *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c1_b_u = c1_u.parameter;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_y, c1_b_y, "parameter", "parameter", 0);
  c1_c_u = c1_u.ueq;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_y, c1_c_y, "ueq", "ueq", 0);
  c1_d_u = c1_u.yeq;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_y, c1_d_y, "yeq", "yeq", 0);
  for (c1_i30 = 0; c1_i30 < 9; c1_i30++) {
    c1_e_u[c1_i30] = c1_u.Ac[c1_i30];
  }

  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", c1_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_addfield(c1_y, c1_e_y, "Ac", "Ac", 0);
  for (c1_i31 = 0; c1_i31 < 3; c1_i31++) {
    c1_f_u[c1_i31] = c1_u.Bc[c1_i31];
  }

  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", c1_f_u, 0, 0U, 1U, 0U, 2, 3, 1),
                false);
  sf_mex_addfield(c1_y, c1_f_y, "Bc", "Bc", 0);
  for (c1_i32 = 0; c1_i32 < 3; c1_i32++) {
    c1_g_u[c1_i32] = c1_u.Cc[c1_i32];
  }

  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", c1_g_u, 0, 0U, 1U, 0U, 2, 1, 3),
                false);
  sf_mex_addfield(c1_y, c1_g_y, "Cc", "Cc", 0);
  c1_h_u = c1_u.Dc;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c1_y, c1_h_y, "Dc", "Dc", 0);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i33;
  int32_T c1_i34;
  const mxArray *c1_y = NULL;
  int32_T c1_i35;
  real_T c1_u[9];
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_i33 = 0;
  for (c1_i34 = 0; c1_i34 < 3; c1_i34++) {
    for (c1_i35 = 0; c1_i35 < 3; c1_i35++) {
      c1_u[c1_i35 + c1_i33] = (*(real_T (*)[9])c1_inData)[c1_i35 + c1_i33];
    }

    c1_i33 += 3;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static void c1_g_emlrt_marshallIn(SFc1_magLev_simInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[9])
{
  real_T c1_dv4[9];
  int32_T c1_i36;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv4, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c1_i36 = 0; c1_i36 < 9; c1_i36++) {
    c1_y[c1_i36] = c1_dv4[c1_i36];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_Ac;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[9];
  int32_T c1_i37;
  int32_T c1_i38;
  int32_T c1_i39;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_Ac = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_Ac), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_Ac);
  c1_i37 = 0;
  for (c1_i38 = 0; c1_i38 < 3; c1_i38++) {
    for (c1_i39 = 0; c1_i39 < 3; c1_i39++) {
      (*(real_T (*)[9])c1_outData)[c1_i39 + c1_i37] = c1_y[c1_i39 + c1_i37];
    }

    c1_i37 += 3;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_magLev_sim_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static const mxArray *c1_f_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_h_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i40;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i40, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i40;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_DataGS_bus_io(void *chartInstanceVoid, void *c1_pData)
{
  const mxArray *c1_mxVal;
  c1_GainSchedulingStruct c1_tmp;
  int32_T c1_i41;
  int32_T c1_i42;
  int32_T c1_i43;
  int32_T c1_i44;
  int32_T c1_i45;
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)chartInstanceVoid;
  c1_mxVal = NULL;
  c1_mxVal = NULL;
  c1_tmp.parameter = *(real_T *)&((char_T *)(c1_GainSchedulingStruct *)c1_pData)
    [0];
  c1_tmp.ueq = *(real_T *)&((char_T *)(c1_GainSchedulingStruct *)c1_pData)[8];
  c1_tmp.yeq = *(real_T *)&((char_T *)(c1_GainSchedulingStruct *)c1_pData)[16];
  c1_i41 = 0;
  for (c1_i42 = 0; c1_i42 < 3; c1_i42++) {
    for (c1_i44 = 0; c1_i44 < 3; c1_i44++) {
      c1_tmp.Ac[c1_i44 + c1_i41] = ((real_T *)&((char_T *)
        (c1_GainSchedulingStruct *)c1_pData)[24])[c1_i44 + c1_i41];
    }

    c1_i41 += 3;
  }

  for (c1_i43 = 0; c1_i43 < 3; c1_i43++) {
    c1_tmp.Bc[c1_i43] = ((real_T *)&((char_T *)(c1_GainSchedulingStruct *)
      c1_pData)[96])[c1_i43];
  }

  for (c1_i45 = 0; c1_i45 < 3; c1_i45++) {
    c1_tmp.Cc[c1_i45] = ((real_T *)&((char_T *)(c1_GainSchedulingStruct *)
      c1_pData)[120])[c1_i45];
  }

  c1_tmp.Dc = *(real_T *)&((char_T *)(c1_GainSchedulingStruct *)c1_pData)[144];
  sf_mex_assign(&c1_mxVal, c1_d_sf_marshallOut(chartInstance, &c1_tmp), false);
  return c1_mxVal;
}

static uint8_T c1_i_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_magLev_sim, const char_T
  *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_magLev_sim), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_magLev_sim);
  return c1_y;
}

static uint8_T c1_j_emlrt_marshallIn(SFc1_magLev_simInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_magLev_simInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_magLev_simInstanceStruct
  *chartInstance)
{
  chartInstance->c1_yeq = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_ueq = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_Cc = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_DataGS = (c1_GainSchedulingStruct *)
    ssGetInputPortSignal_wrapper(chartInstance->S, 0);
  chartInstance->c1_xdot = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_error = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_x = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_magLev_sim_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(667598878U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3900361432U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1753208488U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3729562807U);
}

mxArray* sf_c1_magLev_sim_get_post_codegen_info(void);
mxArray *sf_c1_magLev_sim_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("NdbPN2upyG7SP10OmVUqJH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(3);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_magLev_sim_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_magLev_sim_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_magLev_sim_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_magLev_sim_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_magLev_sim_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_magLev_sim(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[11],T\"Cc\",},{M[1],M[8],T\"ueq\",},{M[1],M[5],T\"xdot\",},{M[1],M[13],T\"yeq\",},{M[8],M[0],T\"is_active_c1_magLev_sim\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_magLev_sim_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_magLev_simInstanceStruct *chartInstance =
      (SFc1_magLev_simInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _magLev_simMachineNumber_,
           1,
           1,
           1,
           0,
           7,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_magLev_simMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_magLev_simMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _magLev_simMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"DataGS");
          _SFD_SET_DATA_PROPS(1,1,1,0,"error");
          _SFD_SET_DATA_PROPS(2,1,1,0,"x");
          _SFD_SET_DATA_PROPS(3,2,0,1,"yeq");
          _SFD_SET_DATA_PROPS(4,2,0,1,"ueq");
          _SFD_SET_DATA_PROPS(5,2,0,1,"Cc");
          _SFD_SET_DATA_PROPS(6,2,0,1,"xdot");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,205);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_DataGS_bus_io,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_c_sf_marshallOut,(MexInFcnForType)c1_c_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1U;
          dimVector[1]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)
            c1_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _magLev_simMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_magLev_simInstanceStruct *chartInstance =
      (SFc1_magLev_simInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_yeq);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_ueq);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c1_Cc);
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_DataGS);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c1_xdot);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c1_error);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c1_x);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s8O5T7cnZ7DVXEdtKhXLGNC";
}

static void sf_opaque_initialize_c1_magLev_sim(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_magLev_simInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c1_magLev_sim((SFc1_magLev_simInstanceStruct*)
    chartInstanceVar);
  initialize_c1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_magLev_sim(void *chartInstanceVar)
{
  enable_c1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_magLev_sim(void *chartInstanceVar)
{
  disable_c1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_magLev_sim(void *chartInstanceVar)
{
  sf_gateway_c1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_magLev_sim(SimStruct* S)
{
  return get_sim_state_c1_magLev_sim((SFc1_magLev_simInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_magLev_sim(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c1_magLev_sim((SFc1_magLev_simInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_magLev_sim(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_magLev_simInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_magLev_sim_optimization_info();
    }

    finalize_c1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_magLev_sim((SFc1_magLev_simInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_magLev_sim(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_magLev_sim((SFc1_magLev_simInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_magLev_sim(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetStatesModifiedOnlyInUpdate(S, 1);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_magLev_sim_optimization_info(sim_mode_is_rtw_gen
      (S), sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2340857100U));
  ssSetChecksum1(S,(3403023610U));
  ssSetChecksum2(S,(3438501171U));
  ssSetChecksum3(S,(3083059938U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_magLev_sim(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_magLev_sim(SimStruct *S)
{
  SFc1_magLev_simInstanceStruct *chartInstance;
  chartInstance = (SFc1_magLev_simInstanceStruct *)utMalloc(sizeof
    (SFc1_magLev_simInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_magLev_simInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_magLev_sim;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_magLev_sim;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_magLev_sim;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_magLev_sim;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_magLev_sim;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_magLev_sim;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_magLev_sim;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_magLev_sim;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_magLev_sim;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_magLev_sim;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_magLev_sim;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_magLev_sim(chartInstance);
}

void c1_magLev_sim_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_magLev_sim(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_magLev_sim(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_magLev_sim(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_magLev_sim_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
