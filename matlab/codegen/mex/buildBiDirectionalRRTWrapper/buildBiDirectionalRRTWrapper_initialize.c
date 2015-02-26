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

/* Function Declarations */
static void c_buildBiDirectionalRRTWrapper_(void);

/* Function Definitions */
static void c_buildBiDirectionalRRTWrapper_(void)
{
  HGAINS_not_empty_init();
  cartesianLimits_not_empty_init();
  goalSeedFreq_not_empty_init();
  exhaustive_not_empty_init();
  ankleThreshold_not_empty_init();
  U_SIZE_not_empty_init();
  NODE_SIZE_not_empty_init();
  NUM_NODES_not_empty_init();
  d_buildBiDirectionalRRTWrapper_();
}

void buildBiDirectionalRRTWrapper_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    c_buildBiDirectionalRRTWrapper_();
  }
}

/* End of code generation (buildBiDirectionalRRTWrapper_initialize.c) */
