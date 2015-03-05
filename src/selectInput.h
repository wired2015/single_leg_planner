//
// File: selectInput.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 11:17:25
//
#ifndef __SELECTINPUT_H__
#define __SELECTINPUT_H__

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
extern void b_selectInput(const double xNear[13], const double xRand[13], const
  struct0_T *kC, const double jointLimits[20], const double uBDot[6], int legNum,
  double xNew[13], double transitionArray[80]);
extern void selectInput(const double xNear[13], const double xRand[13], const
  struct0_T *kC, const double jointLimits[20], const double uBDot[6], int legNum,
  double xNew[13], double transitionArray[80]);

#endif

//
// File trailer for selectInput.h
//
// [EOF]
//
