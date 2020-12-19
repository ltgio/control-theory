#ifndef __c3_Chua_MPC_h__
#define __c3_Chua_MPC_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_Chua_MPCInstanceStruct
#define typedef_SFc3_Chua_MPCInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_Chua_MPC;
  real_T (*c3_x)[3];
  real_T (*c3_dx)[3];
  real_T (*c3_u)[3];
  real_T *c3_np;
  real_T *c3_nq;
  real_T *c3_p;
  real_T *c3_q;
} SFc3_Chua_MPCInstanceStruct;

#endif                                 /*typedef_SFc3_Chua_MPCInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_Chua_MPC_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_Chua_MPC_get_check_sum(mxArray *plhs[]);
extern void c3_Chua_MPC_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
