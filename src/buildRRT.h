//
// File: buildRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//
#ifndef __BUILDRRT_H__
#define __BUILDRRT_H__

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
extern void rrtLoop(emxArray_real_T *T, const double jointLimits[12], const
                    double b_cartesianLimits[4], const struct0_T *kC, double
                    panHeight, const double U[10], double Dt, double dt, double *
                    nodeIDCount, const double nGoal[11], const double uBDot[6],
                    int legNum);

#endif

//
// File trailer for buildRRT.h
//
// [EOF]
//
