/*
 * buildRRT_terminate.c
 *
 * Code generation for function 'buildRRT_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "buildRRT_terminate.h"

/* Function Definitions */
void buildRRT_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void buildRRT_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (buildRRT_terminate.c) */
