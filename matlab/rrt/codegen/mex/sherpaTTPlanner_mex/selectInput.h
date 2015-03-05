/*
 * selectInput.h
 *
 * Code generation for function 'selectInput'
 *
 */

#ifndef __SELECTINPUT_H__
#define __SELECTINPUT_H__

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
extern void b_selectInput(const emlrtStack *sp, const real_T xNear[13], const
  real_T xRand[13], const struct0_T *kC, const real_T jointLimits[20], const
  real_T uBDot[6], int32_T legNum, real_T xNew[13], real_T transitionArray[80]);
extern void selectInput(const emlrtStack *sp, const real_T xNear[13], const
  real_T xRand[13], const struct0_T *kC, const real_T jointLimits[20], const
  real_T uBDot[6], int32_T legNum, real_T xNew[13], real_T transitionArray[80]);

#endif

/* End of code generation (selectInput.h) */
