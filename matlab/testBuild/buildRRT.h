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
extern void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointRange[6], const real_T jointLimits[12], const real_T
                    kinematicConst[16], real_T panHeight, const real_T U[10],
                    real_T Dt, real_T dt, real_T *nodeIDCount, const real_T
                    nGoal[11]);

#endif

/* End of code generation (buildRRT.h) */
