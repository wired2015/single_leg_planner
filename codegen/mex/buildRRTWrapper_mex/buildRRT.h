/*
 * buildRRT.h
 *
 * Code generation for function 'buildRRT'
 *
 */

#ifndef __BUILDRRT_H__
#define __BUILDRRT_H__

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
extern void buildRRT(const emlrtStack *sp, const real_T nInit[13], const real_T
                     nGoal[13], const real_T jointLimits[20], real_T panHeight,
                     const real_T U[18], real_T dt, real_T Dt, const struct0_T
                     *kC, const real_T uBDot[6], int32_T legNum, emxArray_real_T
                     *T, emxArray_real_T *path);

#endif

/* End of code generation (buildRRT.h) */
