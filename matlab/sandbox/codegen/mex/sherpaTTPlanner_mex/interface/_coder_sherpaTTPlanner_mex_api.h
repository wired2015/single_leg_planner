/*
 * _coder_sherpaTTPlanner_mex_api.h
 *
 * Code generation for function '_coder_sherpaTTPlanner_mex_api'
 *
 */

#ifndef ___CODER_SHERPATTPLANNER_MEX_API_H__
#define ___CODER_SHERPATTPLANNER_MEX_API_H__

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
#include "sherpaTTPlanner_mex_types.h"

/* Function Declarations */
extern void buildBiDirectionalRRTWrapper_api(sherpaTTPlanner_mexStackData *SD,
  const mxArray * const prhs[8], const mxArray *plhs[5]);
extern void buildRRTWrapper_api(sherpaTTPlanner_mexStackData *SD, const mxArray *
  const prhs[8], const mxArray *plhs[4]);
extern void randomStateGenerator_api(const mxArray * const prhs[5], const
  mxArray *plhs[1]);

#endif

/* End of code generation (_coder_sherpaTTPlanner_mex_api.h) */
