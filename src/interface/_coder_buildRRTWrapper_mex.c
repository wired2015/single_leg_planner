/*
 * _coder_buildRRTWrapper_mex.c
 *
 * Code generation for function 'buildRRTWrapper'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_buildRRTWrapper_api.h"

/* Function Declarations */
static void buildRRTWrapper_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "buildRRTWrapper", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void buildRRTWrapper_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[4];
  const mxArray *inputs[12];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  buildRRTWrapper_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 12) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 12, mxCHAR_CLASS, 15, "buildRRTWrapper");
  } else if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 15, "buildRRTWrapper");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  buildRRTWrapper_api(inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  buildRRTWrapper_terminate();
}

void buildRRTWrapper_atexit_wrapper(void)
{
   buildRRTWrapper_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(buildRRTWrapper_atexit_wrapper);
  /* Dispatch the entry-point. */
  buildRRTWrapper_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_buildRRTWrapper_mex.c) */
