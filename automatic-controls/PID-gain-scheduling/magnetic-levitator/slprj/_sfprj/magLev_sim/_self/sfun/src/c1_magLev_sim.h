#ifndef __c1_magLev_sim_h__
#define __c1_magLev_sim_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_GainSchedulingStruct_tag
#define struct_GainSchedulingStruct_tag

struct GainSchedulingStruct_tag
{
  real_T parameter;
  real_T ueq;
  real_T yeq;
  real_T Ac[9];
  real_T Bc[3];
  real_T Cc[3];
  real_T Dc;
};

#endif                                 /*struct_GainSchedulingStruct_tag*/

#ifndef typedef_c1_GainSchedulingStruct
#define typedef_c1_GainSchedulingStruct

typedef struct GainSchedulingStruct_tag c1_GainSchedulingStruct;

#endif                                 /*typedef_c1_GainSchedulingStruct*/

#ifndef typedef_SFc1_magLev_simInstanceStruct
#define typedef_SFc1_magLev_simInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_magLev_sim;
  real_T *c1_yeq;
  real_T *c1_ueq;
  real_T (*c1_Cc)[3];
  c1_GainSchedulingStruct *c1_DataGS;
  real_T (*c1_xdot)[3];
  real_T *c1_error;
  real_T (*c1_x)[3];
} SFc1_magLev_simInstanceStruct;

#endif                                 /*typedef_SFc1_magLev_simInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_magLev_sim_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_magLev_sim_get_check_sum(mxArray *plhs[]);
extern void c1_magLev_sim_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
