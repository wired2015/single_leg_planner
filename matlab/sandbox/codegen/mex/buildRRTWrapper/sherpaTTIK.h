/*
 * sherpaTTIK.h
 *
 * Code generation for function 'sherpaTTIK'
 *
 */

#ifndef __SHERPATTIK_H__
#define __SHERPATTIK_H__

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
#include "buildRRTWrapper_types.h"

/* Function Declarations */
extern void b_sherpaTTIK(const emlrtStack *sp, const real_T u[3], real_T kC_l1,
  real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T
  kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const real_T jointLimits[20],
  real_T q[3]);
extern void sherpaTTIK(const emlrtStack *sp, const real_T u[3], real_T kC_l1,
  real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T
  kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const real_T jointLimits[20],
  real_T q[3]);

#endif

/* End of code generation (sherpaTTIK.h) */
