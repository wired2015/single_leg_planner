/*
 * validJointState.h
 *
 * Code generation for function 'validJointState'
 *
 */

#ifndef __VALIDJOINTSTATE_H__
#define __VALIDJOINTSTATE_H__

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
#include "randomStateGenerator_mex_types.h"

/* Function Declarations */
extern boolean_T validJointState(const real_T state[10], const real_T
  jointLimits[20]);

#endif

/* End of code generation (validJointState.h) */
