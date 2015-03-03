/*
 * _coder_buildBiDirectionalRRTWrapper_mex.c
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_buildBiDirectionalRRTWrapper_api.h"
#include "buildBiDirectionalRRTWrapper_initialize.h"
#include "buildBiDirectionalRRTWrapper_terminate.h"

/* Function Declarations */
static void buildBiDirectionalRRTWrapper_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "buildBiDirectionalRRTWrapper", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void buildBiDirectionalRRTWrapper_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[5];
  const mxArray *inputs[9];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  c_buildBiDirectionalRRTWrapperS* d_buildBiDirectionalRRTWrapperS = (c_buildBiDirectionalRRTWrapperS*)mxCalloc(1,sizeof(c_buildBiDirectionalRRTWrapperS));
  /* Module initialization. */
  buildBiDirectionalRRTWrapper_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 9) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 9, mxCHAR_CLASS, 28, "buildBiDirectionalRRTWrapper");
  } else if (nlhs > 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 28, "buildBiDirectionalRRTWrapper");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  buildBiDirectionalRRTWrapper_api(d_buildBiDirectionalRRTWrapperS, inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  buildBiDirectionalRRTWrapper_terminate();
  mxFree(d_buildBiDirectionalRRTWrapperS);
}

void buildBiDirectionalRRTWrapper_atexit_wrapper(void)
{
   buildBiDirectionalRRTWrapper_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(buildBiDirectionalRRTWrapper_atexit_wrapper);
  /* Dispatch the entry-point. */
  buildBiDirectionalRRTWrapper_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_buildBiDirectionalRRTWrapper_mex.c) */
