//
// File: selectInput.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Feb-2015 15:38:22
//
#ifndef __SELECTINPUT_H__
#define __SELECTINPUT_H__

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
extern void selectInput(const emxArray_real_T *xNear, const double xRand_data[],
  const int xRand_size[2], const double U[10], double dt, double Dt, int
  NODE_SIZE, int U_SIZE, const double HGAINS[3], const double kinematicConst[12],
  double ankleThreshold, const double jointLimits[12], emxArray_real_T *xNew,
  emxArray_real_T *transitionArray);

#endif

//
// File trailer for selectInput.h
//
// [EOF]
//
