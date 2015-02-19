/*
 * makeState.h
 *
 * Code generation for function 'makeState'
 *
 */

#ifndef __MAKESTATE_H__
#define __MAKESTATE_H__

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
extern void makeState(const emlrtStack *sp, real_T alpha, real_T beta, real_T
                      b_gamma, real_T alphaDot, real_T betaDot, real_T gammaDot,
                      real_T phi, real_T omega, real_T dt, real_T Dt, struct1_T *
                      s);

#endif

/* End of code generation (makeState.h) */
