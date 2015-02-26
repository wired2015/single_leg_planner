/*
 * buildBiDirectionalRRTWrapper_mexutil.c
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper_mexutil'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRTWrapper_mexutil.h"
#include <stdio.h>

/* Function Definitions */
void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

/* End of code generation (buildBiDirectionalRRTWrapper_mexutil.c) */
