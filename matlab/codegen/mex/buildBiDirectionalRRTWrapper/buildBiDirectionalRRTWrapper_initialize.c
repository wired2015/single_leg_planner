/*
 * buildBiDirectionalRRTWrapper_initialize.c
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRTWrapper_initialize.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Function Definitions */
void buildBiDirectionalRRTWrapper_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (buildBiDirectionalRRTWrapper_initialize.c) */
