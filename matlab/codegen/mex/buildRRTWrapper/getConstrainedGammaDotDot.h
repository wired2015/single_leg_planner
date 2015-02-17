/*
 * getConstrainedGammaDotDot.h
 *
 * Code generation for function 'getConstrainedGammaDotDot'
 *
 */

#ifndef __GETCONSTRAINEDGAMMADOTDOT_H__
#define __GETCONSTRAINEDGAMMADOTDOT_H__

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
extern real_T getConstrainedGammaDotDot(real_T kC_l3, real_T kC_l5, real_T
  kC_zeta, const real_T qDotDot[2], const real_T qDot[3], const real_T q[3]);

#ifdef __WATCOMC__

#pragma aux getConstrainedGammaDotDot value [8087];

#endif
#endif

/* End of code generation (getConstrainedGammaDotDot.h) */
