//
// File: sherpaTTIK.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 24-Feb-2015 15:18:00
//
#ifndef __SHERPATTIK_H__
#define __SHERPATTIK_H__

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
extern void b_sherpaTTIK(const double u[3], double kC_l1, double kC_l2, double
  kC_l3, double kC_l4, double kC_l5, double kC_l6, double kC_l7, double kC_l8,
  double kC_zeta, double kC_r, const double jointLimits[20], double q[3]);
extern void sherpaTTIK(const double u[3], double kC_l1, double kC_l2, double
  kC_l3, double kC_l4, double kC_l5, double kC_l6, double kC_l7, double kC_l8,
  double kC_zeta, double kC_r, const double jointLimits[20], double q[3]);

#endif

//
// File trailer for sherpaTTIK.h
//
// [EOF]
//
