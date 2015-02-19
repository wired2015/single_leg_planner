/*
 * makeEmptyState.h
 *
 * Code generation for function 'makeEmptyState'
 *
 */

#ifndef __MAKEEMPTYSTATE_H__
#define __MAKEEMPTYSTATE_H__

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
extern void makeEmptyState(const emlrtStack *sp, real_T dt, real_T Dt, real_T
  *s_id, real_T *s_parentID, real_T *s_cost, real_T *s_alpha, real_T *s_beta,
  real_T *s_gamma, real_T *s_alphaDot, real_T *s_betaDot, real_T *s_gammaDot,
  real_T *s_phi, real_T *s_omega, emxArray_struct2_T *s_transitionArray);

#endif

/* End of code generation (makeEmptyState.h) */
