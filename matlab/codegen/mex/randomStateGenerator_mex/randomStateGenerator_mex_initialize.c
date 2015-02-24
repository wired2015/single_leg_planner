/*
 * randomStateGenerator_mex_initialize.c
 *
 * Code generation for function 'randomStateGenerator_mex_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "randomStateGenerator_mex_initialize.h"
#include "randomStateGenerator_mex_data.h"
#include <stdio.h>

/* Function Declarations */
static void randomStateGenerator_mex_once(void);

/* Function Definitions */
static void randomStateGenerator_mex_once(void)
{
  cartesianLimits_not_empty_init();
  goalSeedFreq_not_empty_init();
  exhaustive_not_empty_init();
  ankleThreshold_not_empty_init();
  U_SIZE_not_empty_init();
  NODE_SIZE_not_empty_init();
  buildRRTWrapper_init();
}

void randomStateGenerator_mex_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    randomStateGenerator_mex_once();
  }
}

/* End of code generation (randomStateGenerator_mex_initialize.c) */
