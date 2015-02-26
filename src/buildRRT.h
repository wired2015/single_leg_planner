//
// File: buildRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 26-Feb-2015 11:03:31
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
extern void rrtLoop(emxArray_real_T *T, const double jointLimits[20], const
                    struct0_T *kC, double panHeight, const double U[18], double
                    Dt, double dt, double *nodeIDCount, const double uBDot[6],
                    int legNum);

#endif

//
// File trailer for buildRRT.h
//
// [EOF]
//
