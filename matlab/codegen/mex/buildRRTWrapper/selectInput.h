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
#include "buildRRTWrapper_types.h"

/* Function Declarations */
extern void selectInput(const emlrtStack *sp, const real_T xNear_data[], const
  real_T xRand[13], const real_T U[18], real_T dt, real_T Dt, const struct0_T
  *kC, const real_T jointLimits[20], const real_T uBDot[6], int32_T legNum,
  real_T xNew_data[], int32_T xNew_size[2], emxArray_real_T *transitionArray);

#endif

/* End of code generation (selectInput.h) */
