//
// File: randomStateGenerator.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 27-Feb-2015 15:48:27
//
#ifndef __RANDOMSTATEGENERATOR_H__
#define __RANDOMSTATEGENERATOR_H__

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
extern void randomStateGenerator(int NUM_POINTS, const double jointLimits[20],
  const struct0_T *kC, double panHeight, int legNum, emxArray_real_T *states);

#endif

//
// File trailer for randomStateGenerator.h
//
// [EOF]
//
