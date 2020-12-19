/* Include files */

#include "ChauMultipleRobustMPC_sfun.h"
#include "ChauMultipleRobustMPC_sfun_debug_macros.h"
#include "c2_ChauMultipleRobustMPC.h"
#include "c3_ChauMultipleRobustMPC.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _ChauMultipleRobustMPCMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void ChauMultipleRobustMPC_initializer(void)
{
}

void ChauMultipleRobustMPC_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_ChauMultipleRobustMPC_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_ChauMultipleRobustMPC_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_ChauMultipleRobustMPC_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

extern void sf_ChauMultipleRobustMPC_uses_exported_functions(int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[])
{
  plhs[0] = mxCreateLogicalScalar(0);
}

unsigned int sf_ChauMultipleRobustMPC_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4255005166U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4020400167U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1770895285U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(759061498U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4269290707U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1042101317U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1891826753U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1094698596U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_ChauMultipleRobustMPC_get_check_sum(mxArray *plhs[]);
          sf_c2_ChauMultipleRobustMPC_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_ChauMultipleRobustMPC_get_check_sum(mxArray *plhs[]);
          sf_c3_ChauMultipleRobustMPC_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3061339410U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1991824845U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3599338742U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2357874978U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2684362106U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2646930400U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2792260839U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3638964041U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_ChauMultipleRobustMPC_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "cQdfeT9FJtC4wXhlCxZYp") == 0) {
          extern mxArray *sf_c2_ChauMultipleRobustMPC_get_autoinheritance_info
            (void);
          plhs[0] = sf_c2_ChauMultipleRobustMPC_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "Ww0ayGEvKowCrF7f2mfNP") == 0) {
          extern mxArray *sf_c3_ChauMultipleRobustMPC_get_autoinheritance_info
            (void);
          plhs[0] = sf_c3_ChauMultipleRobustMPC_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_ChauMultipleRobustMPC_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray
          *sf_c2_ChauMultipleRobustMPC_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_ChauMultipleRobustMPC_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_ChauMultipleRobustMPC_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_ChauMultipleRobustMPC_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_ChauMultipleRobustMPC_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Ixkke317Nk4G6ol1pxsh1B") == 0) {
          extern mxArray *sf_c2_ChauMultipleRobustMPC_third_party_uses_info(void);
          plhs[0] = sf_c2_ChauMultipleRobustMPC_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "KN1LlbzORKUw2asDPd0DbH") == 0) {
          extern mxArray *sf_c3_ChauMultipleRobustMPC_third_party_uses_info(void);
          plhs[0] = sf_c3_ChauMultipleRobustMPC_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_ChauMultipleRobustMPC_jit_fallback_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Ixkke317Nk4G6ol1pxsh1B") == 0) {
          extern mxArray *sf_c2_ChauMultipleRobustMPC_jit_fallback_info(void);
          plhs[0] = sf_c2_ChauMultipleRobustMPC_jit_fallback_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "KN1LlbzORKUw2asDPd0DbH") == 0) {
          extern mxArray *sf_c3_ChauMultipleRobustMPC_jit_fallback_info(void);
          plhs[0] = sf_c3_ChauMultipleRobustMPC_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_ChauMultipleRobustMPC_updateBuildInfo_args_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Ixkke317Nk4G6ol1pxsh1B") == 0) {
          extern mxArray *sf_c2_ChauMultipleRobustMPC_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c2_ChauMultipleRobustMPC_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "KN1LlbzORKUw2asDPd0DbH") == 0) {
          extern mxArray *sf_c3_ChauMultipleRobustMPC_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c3_ChauMultipleRobustMPC_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sf_ChauMultipleRobustMPC_get_post_codegen_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  unsigned int chartFileNumber = (unsigned int) mxGetScalar(prhs[0]);
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  switch (chartFileNumber) {
   case 2:
    {
      if (strcmp(tpChksum, "Ixkke317Nk4G6ol1pxsh1B") == 0) {
        extern mxArray *sf_c2_ChauMultipleRobustMPC_get_post_codegen_info(void);
        plhs[0] = sf_c2_ChauMultipleRobustMPC_get_post_codegen_info();
        return;
      }
    }
    break;

   case 3:
    {
      if (strcmp(tpChksum, "KN1LlbzORKUw2asDPd0DbH") == 0) {
        extern mxArray *sf_c3_ChauMultipleRobustMPC_get_post_codegen_info(void);
        plhs[0] = sf_c3_ChauMultipleRobustMPC_get_post_codegen_info();
        return;
      }
    }
    break;

   default:
    break;
  }

  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void ChauMultipleRobustMPC_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _ChauMultipleRobustMPCMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"ChauMultipleRobustMPC","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _ChauMultipleRobustMPCMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _ChauMultipleRobustMPCMachineNumber_,0);
}

void ChauMultipleRobustMPC_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_ChauMultipleRobustMPC_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "ChauMultipleRobustMPC", "ChauMultipleRobustMPC");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_ChauMultipleRobustMPC_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
