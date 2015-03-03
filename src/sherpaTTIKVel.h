//
// File: sherpaTTIKVel.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 03-Mar-2015 11:19:40
//
#ifndef __SHERPATTIKVEL_H__
#define __SHERPATTIKVEL_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "sherpaTTPlanner_types.h"

// Function Declarations
extern void sherpaTTIKVel(const double uDot[3], const double q[3], double kC_l2,
  double kC_l3, double kC_l4, double kC_l5, double kC_l7, double kC_zeta, double
  qDot[3]);

#endif

//
// File trailer for sherpaTTIKVel.h
//
// [EOF]
//
