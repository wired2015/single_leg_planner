/*
 * sherpaTTPlanner_mex_mexutil.c
 *
 * Code generation for function 'sherpaTTPlanner_mex_mexutil'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include <stdio.h>

/* Function Definitions */
void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

const mxArray *message(const emlrtStack *sp, const mxArray *b, emlrtMCInfo
  *location)
{
  const mxArray *pArray;
  const mxArray *m13;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m13, 1, &pArray, "message", true,
    location);
}

/* End of code generation (sherpaTTPlanner_mex_mexutil.c) */
