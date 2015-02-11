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
extern void nearestNeighbour(const emlrtStack *sp, const real_T x[11], const
  emxArray_real_T *T, const real_T jointLimits[12], const real_T kinematicConst
  [16], real_T nodeIDCount, int32_T NODE_SIZE, real_T xNear_data[], int32_T
  xNear_size[2], emxArray_real_T *transitionArray, real_T *d);

#endif

/* End of code generation (nearestNeighbour.h) */
