//
// File: nearestNeighbour.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//
#ifndef __NEARESTNEIGHBOUR_H__
#define __NEARESTNEIGHBOUR_H__

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
extern void nearestNeighbour(const double x[13], const double T[139500], double
  kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double kC_l6,
  double kC_l7, double kC_l8, double kC_zeta, double kC_r, double nodeIDCount,
  double xNear[13], double transitionArray[80], double *d);

#endif

//
// File trailer for nearestNeighbour.h
//
// [EOF]
//
