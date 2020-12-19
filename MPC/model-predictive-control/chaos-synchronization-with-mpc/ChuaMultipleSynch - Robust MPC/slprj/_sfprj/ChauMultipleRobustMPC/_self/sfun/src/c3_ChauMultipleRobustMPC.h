#ifndef __c3_ChauMultipleRobustMPC_h__
#define __c3_ChauMultipleRobustMPC_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_ChauMultipleRobustMPCInstanceStruct
#define typedef_SFc3_ChauMultipleRobustMPCInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_ChauMultipleRobustMPC;
  real_T (*c3_x)[3];
  real_T (*c3_dx)[3];
  real_T (*c3_u)[3];
  real_T *c3_np;
  real_T *c3_nq;
  real_T *c3_p;
  real_T *c3_q;
} SFc3_ChauMultipleRobustMPCInstanceStruct;

#endif                                 /*typedef_SFc3_ChauMultipleRobustMPCInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c3_ChauMultipleRobustMPC_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_ChauMultipleRobustMPC_get_check_sum(mxArray *plhs[]);
extern void c3_ChauMultipleRobustMPC_method_dispatcher(SimStruct *S, int_T
  method, void *data);

#endif
