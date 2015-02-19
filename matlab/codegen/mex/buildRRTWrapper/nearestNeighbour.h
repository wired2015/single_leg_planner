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
#include "buildRRTWrapper_types.h"

/* Function Declarations */
extern void nearestNeighbour(const emlrtStack *sp, real_T s_alpha, real_T s_beta,
  real_T s_gamma, const emxArray_struct1_T *T, const real_T jointLimits[14],
  real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l7, real_T
  kC_zeta, real_T nodeIDCount, struct1_T *xNear, real_T *d);

#endif

/* End of code generation (nearestNeighbour.h) */
