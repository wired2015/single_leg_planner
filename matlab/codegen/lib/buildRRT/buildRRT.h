//
// File: buildRRT.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//
#ifndef __BUILDRRT_H__
#define __BUILDRRT_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildRRT_types.h"

// Function Declarations
extern void buildRRT(const double nInit[11], const double nGoal[11], int
                     NUM_NODES, const double jointLimits[12], double K, const
                     double HGAINS[3], int NODE_SIZE, const double U[10], int
                     U_SIZE, double dt, double Dt, const double kinematicConst
                     [12], double ankleThreshold, boolean_T exhaustive, double
                     threshold, int goalSeedFreq, emxArray_real_T *T,
                     emxArray_real_T *path);

#endif

//
// File trailer for buildRRT.h
//
// [EOF]
//
