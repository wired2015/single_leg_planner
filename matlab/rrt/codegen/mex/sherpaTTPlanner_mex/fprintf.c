/*
 * fprintf.c
 *
 * Code generation for function 'fprintf'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "fprintf.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = { 35, "fprintf",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/iofun/fprintf.m" };

static emlrtMCInfo emlrtMCI = { 69, 14, "fprintf",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/iofun/fprintf.m" };

static emlrtRSInfo lc_emlrtRSI = { 69, "fprintf",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/iofun/fprintf.m" };

/* Function Declarations */
static real_T c_fprintf(const emlrtStack *sp);
static const mxArray *feval(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, const mxArray *d, emlrtMCInfo *location);

/* Function Definitions */
static real_T c_fprintf(const emlrtStack *sp)
{
  const mxArray *y;
  static const int32_T iv2[2] = { 1, 7 };

  const mxArray *m0;
  char_T cv0[7];
  int32_T i;
  static const char_T cv1[7] = { 'f', 'p', 'r', 'i', 'n', 't', 'f' };

  const mxArray *b_y;
  const mxArray *c_y;
  static const int32_T iv3[2] = { 1, 36 };

  char_T cv2[36];
  static const char_T cv3[36] = { 'S', 'o', 'l', 'u', 't', 'i', 'o', 'n', ' ',
    'o', 'u', 't', ' ', 'o', 'f', ' ', 'a', 'n', 'g', 'u', 'l', 'a', 'r', ' ',
    'l', 'i', 'm', 'i', 't', ' ', 'r', 'a', 'n', 'g', 'e', '.' };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m0 = emlrtCreateCharArray(2, iv2);
  for (i = 0; i < 7; i++) {
    cv0[i] = cv1[i];
  }

  emlrtInitCharArrayR2013a(sp, 7, m0, cv0);
  emlrtAssign(&y, m0);
  b_y = NULL;
  m0 = emlrtCreateDoubleScalar(1.0);
  emlrtAssign(&b_y, m0);
  c_y = NULL;
  m0 = emlrtCreateCharArray(2, iv3);
  for (i = 0; i < 36; i++) {
    cv2[i] = cv3[i];
  }

  emlrtInitCharArrayR2013a(sp, 36, m0, cv2);
  emlrtAssign(&c_y, m0);
  st.site = &lc_emlrtRSI;
  return emlrt_marshallIn(&st, feval(&st, y, b_y, c_y, &emlrtMCI), "feval");
}

static const mxArray *feval(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, const mxArray *d, emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  const mxArray *m14;
  pArrays[0] = b;
  pArrays[1] = c;
  pArrays[2] = d;
  return emlrtCallMATLABR2012b(sp, 1, &m14, 3, pArrays, "feval", true, location);
}

void b_fprintf(const emlrtStack *sp)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &h_emlrtRSI;
  c_fprintf(&st);
}

/* End of code generation (fprintf.c) */
