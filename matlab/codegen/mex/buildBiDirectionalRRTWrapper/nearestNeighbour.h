/*
 * nearestNeighbour.h
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

#ifndef __NEARESTNEIGHBOUR_H__
#define __NEARESTNEIGHBOUR_H__

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
extern void b_nearestNeighbour(const emlrtStack *sp, const real_T x_data[],
  const emxArray_real_T *T, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T
  kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta,
  real_T kC_r, int32_T NODE_SIZE, real_T xNear_data[], int32_T xNear_size[2],
  emxArray_real_T *transitionArray, real_T *d);
extern void nearestNeighbour(const emlrtStack *sp, const real_T x[13], const
  emxArray_real_T *T, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r, real_T nodeIDCount, int32_T NODE_SIZE, real_T xNear_data[], int32_T
  xNear_size[2], emxArray_real_T *transitionArray, real_T *d);

#endif

/* End of code generation (nearestNeighbour.h) */
