//
// File: rk4.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:32:33
//
#ifndef __RK4_H__
#define __RK4_H__

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
extern void rk4(const double uIn[2], const double uBDot[6], const double xInit
                [13], const double jointLimits[20], const struct0_T *kC, int
                legNum, double xNewFull[13], double transitionArray[80]);

#endif

//
// File trailer for rk4.h
//
// [EOF]
//
