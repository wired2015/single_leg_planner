/*
 * buildBiDirectionalRRTWrapper.h
 *
 * Code generation for function 'buildBiDirectionalRRTWrapper'
 *
 */

#ifndef __BUILDBIDIRECTIONALRRTWRAPPER_H__
#define __BUILDBIDIRECTIONALRRTWRAPPER_H__

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
extern void HGAINS_not_empty_init(void);
extern void NODE_SIZE_not_empty_init(void);
extern void NUM_NODES_not_empty_init(void);
extern void U_SIZE_not_empty_init(void);
extern void ankleThreshold_not_empty_init(void);
extern void buildBiDirectionalRRTWrapper(const emlrtStack *sp, const real_T
  nInitCartesianB[6], const real_T nGoalCartesianB[6], real_T phiInit, real_T
  omegaInit, const real_T jointLimits[20], real_T bodyHeight, const real_T U[18],
  real_T dt, real_T Dt, const struct0_T *kC, real_T threshold, int32_T legNum,
  const real_T uBDot[6], emxArray_real_T *T1, emxArray_real_T *T2,
  emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T *success);
extern void cartesianLimits_not_empty_init(void);
extern void d_buildBiDirectionalRRTWrapper_(void);
extern void exhaustive_not_empty_init(void);
extern void goalSeedFreq_not_empty_init(void);

#endif

/* End of code generation (buildBiDirectionalRRTWrapper.h) */
