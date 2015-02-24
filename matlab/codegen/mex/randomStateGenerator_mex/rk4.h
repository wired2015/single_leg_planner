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
#include "randomStateGenerator_mex_types.h"

/* Function Declarations */
extern void rk4(const emlrtStack *sp, const real_T uIn[2], const real_T uBDot[6],
                real_T dt, real_T Dt, real_T xInit_data[], const real_T
                jointLimits[20], const struct0_T *kC, int32_T legNum, real_T
                xNew_data[], int32_T xNew_size[2], emxArray_real_T
                *transitionArray);

#endif

/* End of code generation (rk4.h) */
