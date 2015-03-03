/*
 * rk4.h
 *
 * Code generation for function 'rk4'
 *
 */

#ifndef __RK4_H__
#define __RK4_H__

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
#include "buildRRTWrapper_mex_types.h"

/* Function Declarations */
extern void rk4(const emlrtStack *sp, const real_T uIn[2], const real_T uBDot[6],
                const real_T xInit[13], const real_T jointLimits[20], const
                struct0_T *kC, int32_T legNum, real_T xNewFull[13], real_T
                transitionArray[80]);

#endif

/* End of code generation (rk4.h) */
