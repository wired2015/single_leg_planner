//
// File: buildRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 15:29:21
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
extern void buildRRT(const double nInit[11], const double nGoal[11], const
                     double jointLimits[12], double panHeight, const double U[10],
                     double dt, double Dt, const struct0_T *kC, emxArray_real_T *
                     T, emxArray_real_T *path);

#endif

//
// File trailer for buildRRT.h
//
// [EOF]
//
