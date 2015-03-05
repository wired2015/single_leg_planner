/*
 * sherpaTTPlanner_mex_mexutil.h
 *
 * Code generation for function 'sherpaTTPlanner_mex_mexutil'
 *
 */

#ifndef __SHERPATTPLANNER_MEX_MEXUTIL_H__
#define __SHERPATTPLANNER_MEX_MEXUTIL_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "sherpaTTPlanner_mex_types.h"

/* Function Declarations */
extern real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);

#ifdef __WATCOMC__

#pragma aux b_emlrt_marshallIn value [8087];

#endif

extern void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
extern real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *b_feval,
  const char_T *identifier);

#ifdef __WATCOMC__

#pragma aux emlrt_marshallIn value [8087];

#endif

extern void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
extern const mxArray *message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
extern real_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

#ifdef __WATCOMC__

#pragma aux n_emlrt_marshallIn value [8087];

#endif
#endif

/* End of code generation (sherpaTTPlanner_mex_mexutil.h) */
