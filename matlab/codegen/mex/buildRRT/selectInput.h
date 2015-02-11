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
#include "buildRRT_types.h"

/* Function Declarations */
extern void selectInput(const emlrtStack *sp, const emxArray_real_T *xNear,
  const real_T xRand_data[], const int32_T xRand_size[2], const real_T U[10],
  real_T dt, real_T Dt, int32_T NODE_SIZE, int32_T U_SIZE, const real_T HGAINS[3],
  const real_T kinematicConst[12], real_T ankleThreshold, const real_T
  jointLimits[12], emxArray_real_T *xNew, emxArray_real_T *transitionArray);

#endif

/* End of code generation (selectInput.h) */
