#ifndef __c1_Chua_MPC_h__
#define __c1_Chua_MPC_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_Chua_MPCInstanceStruct
#define typedef_SFc1_Chua_MPCInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_isStable;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_Chua_MPC;
  real_T (*c1_x)[3];
  real_T (*c1_dx)[3];
  real_T (*c1_u)[3];
  real_T *c1_np;
  real_T *c1_nq;
  real_T *c1_p;
  real_T *c1_q;
} SFc1_Chua_MPCInstanceStruct;

#endif                                 /*typedef_SFc1_Chua_MPCInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_Chua_MPC_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_Chua_MPC_get_check_sum(mxArray *plhs[]);
extern void c1_Chua_MPC_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
