//
// File: buildRRTWrapper.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 14:11:02
//
#ifndef __BUILDRRTWRAPPER_H__
#define __BUILDRRTWRAPPER_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildRRTWrapper_types.h"

// Function Declarations
extern void buildRRTWrapper(const double nInitCartesianB[6], const double
  nGoalCartesianB[6], const double jointLimits[12], double bodyHeight, const
  double U[10], double dt, double Dt, const struct0_T *kC, double threshold, int
  legNum, emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T *pathJ,
  boolean_T *success);
extern void buildRRTWrapper_init();

#endif

//
// File trailer for buildRRTWrapper.h
//
// [EOF]
//
