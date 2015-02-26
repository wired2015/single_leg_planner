/*
 * buildRRTWrapper_initialize.c
 *
 * Code generation for function 'buildRRTWrapper_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_initialize.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Function Declarations */
static void buildRRTWrapper_once(void);

/* Function Definitions */
static void buildRRTWrapper_once(void)
{
  threshold_not_empty_init();
  HGAINS_not_empty_init();
  cartesianLimits_not_empty_init();
  goalSeedFreq_not_empty_init();
  exhaustive_not_empty_init();
  ankleThreshold_not_empty_init();
  U_SIZE_not_empty_init();
  NODE_SIZE_not_empty_init();
  NUM_NODES_not_empty_init();
  buildRRTWrapper_init();
}

void buildRRTWrapper_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    buildRRTWrapper_once();
  }
}

/* End of code generation (buildRRTWrapper_initialize.c) */
