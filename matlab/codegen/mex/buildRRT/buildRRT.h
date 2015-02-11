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
#include "buildRRT_types.h"

/* Function Declarations */
extern void buildRRT(const emlrtStack *sp, const real_T nInit[11], const real_T
                     nGoal[11], int32_T NUM_NODES, const real_T jointLimits[12],
                     real_T K, const real_T HGAINS[3], int32_T NODE_SIZE, const
                     real_T U[10], int32_T U_SIZE, real_T dt, real_T Dt, const
                     real_T kinematicConst[12], real_T ankleThreshold, boolean_T
                     exhaustive, real_T threshold, int32_T goalSeedFreq,
                     emxArray_real_T *T, emxArray_real_T *path);

#endif

/* End of code generation (buildRRT.h) */
