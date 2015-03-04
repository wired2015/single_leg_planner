//
// File: randomStateGenerator.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:32:33
//
#ifndef __RANDOMSTATEGENERATOR_H__
#define __RANDOMSTATEGENERATOR_H__

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
extern void randomStateGenerator(int NUM_POINTS, const double jointLimits[20],
  const struct0_T *kC, double panHeight, int legNum, emxArray_real_T *states);

#endif

//
// File trailer for randomStateGenerator.h
//
// [EOF]
//
