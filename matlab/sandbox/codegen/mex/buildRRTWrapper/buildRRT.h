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
#include "buildRRTWrapper_types.h"

/* Function Declarations */
extern void buildRRT(const emlrtStack *sp, const real_T nInit[13], const real_T
                     nGoal[13], int32_T NUM_NODES, const real_T jointLimits[20],
                     const real_T cartesianLimits[4], real_T panHeight, const
                     real_T HGAINS[3], const real_T U[10], real_T dt, real_T Dt,
                     const struct0_T *kC, real_T ankleThreshold, boolean_T
                     exhaustive, real_T threshold, const real_T uBDot[6],
                     int32_T legNum, emxArray_real_T *T, emxArray_real_T *path);

#endif

/* End of code generation (buildRRT.h) */
