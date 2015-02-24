/*
 * getXStar.h
 *
 * Code generation for function 'getXStar'
 *
 */

#ifndef __GETXSTAR_H__
#define __GETXSTAR_H__

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
#include "randomStateGenerator_types.h"

/* Function Declarations */
extern real_T getXStar(const emlrtStack *sp, real_T z, real_T angle, boolean_T
  selector, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5,
  real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r);

#ifdef __WATCOMC__

#pragma aux getXStar value [8087];

#endif
#endif

/* End of code generation (getXStar.h) */
