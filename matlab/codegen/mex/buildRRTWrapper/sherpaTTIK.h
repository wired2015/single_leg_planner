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
extern void sherpaTTIK(const emlrtStack *sp, real_T xC, real_T yC, real_T zC,
  const real_T kinematicConst[16], const real_T jointLimits[12], real_T *alpha,
  real_T *beta, real_T *b_gamma);

#endif

/* End of code generation (sherpaTTIK.h) */
