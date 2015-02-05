//
// File: buildRRTWrapper.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Feb-2015 15:38:22
//
#ifndef __BUILDRRTWRAPPER_H__
#define __BUILDRRTWRAPPER_H__

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
extern void buildRRTWrapper(const double nInit[6], const double nGoal[6], int
  NUM_NODES, const double jointLimits[12], double K, const double HGAINS[3], int
  NODE_SIZE, const double U[10], int U_SIZE, double dt, double Dt, const double
  kinematicConst[12], double ankleThreshold, boolean_T exhaustive, double
  threshold, int goalSeedFreq, emxArray_real_T *T, emxArray_real_T *path,
  emxArray_real_T *pathJoint);

#endif

//
// File trailer for buildRRTWrapper.h
//
// [EOF]
//
