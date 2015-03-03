/*
 * buildBiDirectionalRRT.h
 *
 * Code generation for function 'buildBiDirectionalRRT'
 *
 */

#ifndef __BUILDBIDIRECTIONALRRT_H__
#define __BUILDBIDIRECTIONALRRT_H__

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
extern void buildBiDirectionalRRT(sherpaTTPlanner_mexStackData *SD, const
  emlrtStack *sp, const real_T nInit[13], const real_T nGoal[13], const real_T
  jointLimits[20], real_T panHeight, const struct0_T *kC, const real_T uBDot[6],
  int32_T legNum, const real_T TP2B[16], real_T T1[69750], real_T T2[69750],
  emxArray_real_T *pathJ, emxArray_real_T *pathC);

#endif

/* End of code generation (buildBiDirectionalRRT.h) */
