//
// File: randomState.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 27-Feb-2015 15:48:27
//
#ifndef __RANDOMSTATE_H__
#define __RANDOMSTATE_H__

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
extern void randomState(const double jointLimits[20], double panHeight, double
  kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double kC_l6,
  double kC_l7, double kC_l8, double kC_zeta, double kC_r, double xRand[13]);

#endif

//
// File trailer for randomState.h
//
// [EOF]
//
