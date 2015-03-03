/*
 * buildRRTWrapper_mex_initialize.c
 *
 * Code generation for function 'buildRRTWrapper_mex_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "buildRRTWrapper_mex_initialize.h"
#include "buildRRTWrapper_mex_data.h"
#include <stdio.h>

/* Function Definitions */
void buildRRTWrapper_mex_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (buildRRTWrapper_mex_initialize.c) */
