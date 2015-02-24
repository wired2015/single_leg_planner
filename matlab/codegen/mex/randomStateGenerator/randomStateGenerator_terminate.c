/*
 * randomStateGenerator_terminate.c
 *
 * Code generation for function 'randomStateGenerator_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "randomStateGenerator.h"
#include "randomStateGenerator_terminate.h"
#include <stdio.h>

/* Function Definitions */
void randomStateGenerator_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void randomStateGenerator_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (randomStateGenerator_terminate.c) */
