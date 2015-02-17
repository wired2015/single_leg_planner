//
// File: selectInput.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 13:54:41
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
extern void selectInput(const double xNear_data[], const int xNear_size[2],
  const double xRand_data[], const int xRand_size[2], const double U[10], double
  dt, double Dt, const struct0_T *kC, const double jointLimits[12], const double
  uBDot[6], int legNum, double xNew_data[], int xNew_size[2], emxArray_real_T
  *transitionArray);

#endif

//
// File trailer for selectInput.h
//
// [EOF]
//
