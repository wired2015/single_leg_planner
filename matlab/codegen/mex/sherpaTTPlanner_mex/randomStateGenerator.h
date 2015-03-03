/*
 * randomStateGenerator.h
 *
 * Code generation for function 'randomStateGenerator'
 *
 */

#ifndef __RANDOMSTATEGENERATOR_H__
#define __RANDOMSTATEGENERATOR_H__

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
extern void randomStateGenerator(const emlrtStack *sp, int32_T NUM_POINTS, const
  real_T jointLimits[20], const struct0_T *kC, real_T panHeight, int32_T legNum,
  emxArray_real_T *states);

#endif

/* End of code generation (randomStateGenerator.h) */
