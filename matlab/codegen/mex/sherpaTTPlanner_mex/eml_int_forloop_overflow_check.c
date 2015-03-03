/*
 * eml_int_forloop_overflow_check.c
 *
 * Code generation for function 'eml_int_forloop_overflow_check'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildBiDirectionalRRT.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtMCInfo d_emlrtMCI = { 87, 9, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtMCInfo e_emlrtMCI = { 86, 15, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtRSInfo kc_emlrtRSI = { 86, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtRSInfo mc_emlrtRSI = { 87, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

/* Function Declarations */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b, const
  mxArray *c, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m15;
  pArrays[0] = b;
  pArrays[1] = c;
  return emlrtCallMATLABR2012b(sp, 1, &m15, 2, pArrays, "message", true,
    location);
}

void b_check_forloop_overflow_error(const emlrtStack *sp)
{
  const mxArray *y;
  static const int32_T iv16[2] = { 1, 34 };

  const mxArray *m5;
  char_T cv12[34];
  int32_T i;
  static const char_T cv13[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o',
    'p', '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *b_y;
  static const int32_T iv17[2] = { 1, 5 };

  char_T cv14[5];
  static const char_T cv15[5] = { 'i', 'n', 't', '3', '2' };

  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = sp;
  b_st.tls = sp->tls;
  y = NULL;
  m5 = emlrtCreateCharArray(2, iv16);
  for (i = 0; i < 34; i++) {
    cv12[i] = cv13[i];
  }

  emlrtInitCharArrayR2013a(sp, 34, m5, cv12);
  emlrtAssign(&y, m5);
  b_y = NULL;
  m5 = emlrtCreateCharArray(2, iv17);
  for (i = 0; i < 5; i++) {
    cv14[i] = cv15[i];
  }

  emlrtInitCharArrayR2013a(sp, 5, m5, cv14);
  emlrtAssign(&b_y, m5);
  st.site = &kc_emlrtRSI;
  b_st.site = &mc_emlrtRSI;
  error(&st, b_message(&b_st, y, b_y, &d_emlrtMCI), &e_emlrtMCI);
}

void check_forloop_overflow_error(const emlrtStack *sp)
{
  const mxArray *y;
  static const int32_T iv4[2] = { 1, 34 };

  const mxArray *m2;
  char_T cv4[34];
  int32_T i;
  static const char_T cv5[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o',
    'p', '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *b_y;
  static const int32_T iv5[2] = { 1, 5 };

  char_T cv6[5];
  static const char_T cv7[5] = { 'i', 'n', 't', '3', '2' };

  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = sp;
  b_st.tls = sp->tls;
  y = NULL;
  m2 = emlrtCreateCharArray(2, iv4);
  for (i = 0; i < 34; i++) {
    cv4[i] = cv5[i];
  }

  emlrtInitCharArrayR2013a(sp, 34, m2, cv4);
  emlrtAssign(&y, m2);
  b_y = NULL;
  m2 = emlrtCreateCharArray(2, iv5);
  for (i = 0; i < 5; i++) {
    cv6[i] = cv7[i];
  }

  emlrtInitCharArrayR2013a(sp, 5, m2, cv6);
  emlrtAssign(&b_y, m2);
  st.site = &kc_emlrtRSI;
  b_st.site = &mc_emlrtRSI;
  error(&st, b_message(&b_st, y, b_y, &d_emlrtMCI), &e_emlrtMCI);
}

/* End of code generation (eml_int_forloop_overflow_check.c) */
