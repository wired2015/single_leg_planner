/*
 * _coder_buildRRT_mex.c
 *
 * Code generation for function 'buildRRT'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_buildRRT_api.h"
#include "buildRRT_initialize.h"
#include "buildRRT_terminate.h"

/* Function Declarations */
static void buildRRT_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "buildRRT", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void buildRRT_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[2];
  const mxArray *inputs[16];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  buildRRT_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 16) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 16, mxCHAR_CLASS, 8, "buildRRT");
  } else if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 8, "buildRRT");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  buildRRT_api(inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  buildRRT_terminate();
}

void buildRRT_atexit_wrapper(void)
{
   buildRRT_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(buildRRT_atexit_wrapper);
  /* Dispatch the entry-point. */
  buildRRT_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_buildRRT_mex.c) */
