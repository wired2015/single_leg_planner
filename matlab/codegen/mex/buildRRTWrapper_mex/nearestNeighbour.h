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
#include "buildRRTWrapper_mex_types.h"

/* Function Declarations */
extern void nearestNeighbour(const emlrtStack *sp, const real_T x[13], const
  real_T T[139500], real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r, real_T nodeIDCount, real_T xNear[13], real_T transitionArray[80], real_T
  *d);

#endif

/* End of code generation (nearestNeighbour.h) */
