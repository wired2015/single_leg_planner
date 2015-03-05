//
// File: buildBiDirectionalRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 15:01:21
//
#ifndef __BUILDBIDIRECTIONALRRT_H__
#define __BUILDBIDIRECTIONALRRT_H__

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
extern void buildBiDirectionalRRT(const double nInit[13], const double nGoal[13],
  const double jointLimits[20], double panHeight, const struct0_T *kC, const
  double uBDot[6], int legNum, const double TP2B[16], double T1[46500], double
  T2[46500], emxArray_real_T *pathJ, emxArray_real_T *pathC);

#endif

//
// File trailer for buildBiDirectionalRRT.h
//
// [EOF]
//
