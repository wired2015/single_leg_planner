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
#include "buildBiDirectionalRRTWrapper_types.h"

/* Function Declarations */
extern void selectInput(const emlrtStack *sp, const real_T xNear[13], const
  real_T xRand[13], real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r, real_T kC_B2PXOffset, real_T kC_B2PZOffset, const real_T
  kC_legAngleOffset[4], const real_T jointLimits[20], const real_T uBDot[6],
  int32_T legNum, real_T xNew[13], real_T transitionArray[80]);

#endif

/* End of code generation (selectInput.h) */
