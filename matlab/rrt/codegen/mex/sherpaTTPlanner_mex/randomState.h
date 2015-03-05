/*
 * randomState.h
 *
 * Code generation for function 'randomState'
 *
 */

#ifndef __RANDOMSTATE_H__
#define __RANDOMSTATE_H__

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
extern void randomState(const emlrtStack *sp, const real_T jointLimits[20],
  real_T panHeight, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r, real_T xRand[13]);

#endif

/* End of code generation (randomState.h) */
