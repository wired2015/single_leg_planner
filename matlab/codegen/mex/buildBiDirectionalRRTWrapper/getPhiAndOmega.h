/*
 * getPhiAndOmega.h
 *
 * Code generation for function 'getPhiAndOmega'
 *
 */

#ifndef __GETPHIANDOMEGA_H__
#define __GETPHIANDOMEGA_H__

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
extern void getPhiAndOmega(const emlrtStack *sp, const real_T uBDot[6], const
  real_T qDot[4], const real_T q[4], const struct0_T *kC, int32_T legNum, real_T
  *phi, real_T *omega);

#endif

/* End of code generation (getPhiAndOmega.h) */
