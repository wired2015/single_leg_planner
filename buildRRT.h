//
// File: buildRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Feb-2015 15:38:22
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
extern void rrtLoop(emxArray_real_T *T, const double jointRange[6], const double
                    jointLimits[12], const double kinematicConst[12], double K,
                    const double U[10], double Dt, double dt, int NODE_SIZE, int
                    U_SIZE, const double HGAINS[3], double ankleThreshold,
                    double *nodeIDCount, const double nGoal[11], int
                    goalSeedFreq);

#endif

//
// File trailer for buildRRT.h
//
// [EOF]
//
