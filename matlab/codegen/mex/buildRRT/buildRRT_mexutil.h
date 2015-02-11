/*
 * buildRRT_mexutil.h
 *
 * Code generation for function 'buildRRT_mexutil'
 *
 */

#ifndef __BUILDRRT_MEXUTIL_H__
#define __BUILDRRT_MEXUTIL_H__

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
#include "buildRRT_types.h"

/* Function Declarations */
extern const mxArray *b_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
extern void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);

#endif

/* End of code generation (buildRRT_mexutil.h) */
