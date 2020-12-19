#ifndef __c2_Chua_LQr_h__
#define __c2_Chua_LQr_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_Chua_LQrInstanceStruct
#define typedef_SFc2_Chua_LQrInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_Chua_LQr;
  real_T (*c2_x)[3];
  real_T (*c2_dx)[3];
} SFc2_Chua_LQrInstanceStruct;

#endif                                 /*typedef_SFc2_Chua_LQrInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_Chua_LQr_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_Chua_LQr_get_check_sum(mxArray *plhs[]);
extern void c2_Chua_LQr_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
