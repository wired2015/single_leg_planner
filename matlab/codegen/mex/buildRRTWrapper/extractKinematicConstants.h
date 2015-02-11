/*
 * extractKinematicConstants.h
 *
 * Code generation for function 'extractKinematicConstants'
 *
 */

#ifndef __EXTRACTKINEMATICCONSTANTS_H__
#define __EXTRACTKINEMATICCONSTANTS_H__

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
extern void extractKinematicConstants(const real_T kinematicConst[16], real_T
  *L1, real_T *L2, real_T *L3, real_T *L4, real_T *L5, real_T *L6, real_T *L7,
  real_T *L8, real_T *zeta, real_T *r, real_T *B2PXOffset, real_T *B2PZOffset,
  real_T *leg1AngleOffset, real_T *leg2AngleOffset, real_T *leg3AngleOffset,
  real_T *leg4AngleOffset);

#endif

/* End of code generation (extractKinematicConstants.h) */
