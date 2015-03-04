//
// File: buildBiDirectionalRRTWrapper.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:32:33
//
#ifndef __BUILDBIDIRECTIONALRRTWRAPPER_H__
#define __BUILDBIDIRECTIONALRRTWRAPPER_H__

// Include Files
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "sherpaTTPlanner_types.h"

// Function Declarations
extern void buildBiDirectionalRRTWrapper(const double nInitCartesianB[6], const
  double nGoalCartesianB[6], double phiInit, double omegaInit, const double
  jointLimits[20], const struct0_T *kC, int legNum, const double uBDot[6],
  emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T *pathC,
  emxArray_real_T *pathJ, boolean_T *success);

#endif

//
// File trailer for buildBiDirectionalRRTWrapper.h
//
// [EOF]
//
