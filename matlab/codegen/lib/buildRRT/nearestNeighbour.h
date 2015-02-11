//
// File: nearestNeighbour.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//
#ifndef __NEARESTNEIGHBOUR_H__
#define __NEARESTNEIGHBOUR_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildRRT_types.h"

// Function Declarations
extern void nearestNeighbour(const double x[11], const emxArray_real_T *T, const
  double kinematicConst[12], double nodeIDCount, int NODE_SIZE, emxArray_real_T *
  xNear, emxArray_real_T *transitionArray, double *d);

#endif

//
// File trailer for nearestNeighbour.h
//
// [EOF]
//
