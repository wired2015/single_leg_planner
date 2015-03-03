/*
 * buildBiDirectionalRRTWrapper.h
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper'
 *
 */

#ifndef __BUILDBIDIRECTIONALRRTWRAPPER_H__
#define __BUILDBIDIRECTIONALRRTWRAPPER_H__

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
extern void buildBiDirectionalRRTWrapper(sherpaTTPlanner_mexStackData *SD, const
  emlrtStack *sp, const real_T nInitCartesianB[6], const real_T nGoalCartesianB
  [6], real_T phiInit, real_T omegaInit, const real_T jointLimits[20], real_T
  bodyHeight, const struct0_T *kC, int32_T legNum, const real_T uBDot[6],
  emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T *pathC,
  emxArray_real_T *pathJ, boolean_T *success);

#endif

/* End of code generation (buildBiDirectionalRRTWrapper.h) */
