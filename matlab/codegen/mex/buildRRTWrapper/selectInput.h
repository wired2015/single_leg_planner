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
  int32_T xNear_size[2], const real_T xRand_data[], const int32_T xRand_size[2],
  const real_T U[10], real_T dt, real_T Dt, real_T kC_l2, real_T kC_l3, real_T
  kC_l4, real_T kC_l5, real_T kC_l7, real_T kC_zeta, const real_T jointLimits[12],
  real_T xNew_data[], int32_T xNew_size[2], emxArray_real_T *transitionArray);

#endif

/* End of code generation (selectInput.h) */
