/*
 * generateTrMatrices.h
 *
 * Code generation for function 'generateTrMatrices'
 *
 */

#ifndef __GENERATETRMATRICES_H__
#define __GENERATETRMATRICES_H__

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
extern void generateTrMatrices(const emlrtStack *sp, const real_T q[4], real_T
  kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6,
  real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, real_T kC_B2PXOffset,
  real_T kC_B2PZOffset, const real_T kC_legAngleOffset[4], int32_T legNum,
  real_T TB2G[16], real_T TP2B[16], real_T TI2P[16], real_T TJ2I[16], real_T
  TO2J[16], real_T TQ2O[16], real_T TR2Q[16], real_T TS2R[16], real_T TW2S[16],
  real_T TC2W[16]);

#endif

/* End of code generation (generateTrMatrices.h) */
