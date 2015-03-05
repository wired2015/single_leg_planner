//
// File: generateTrMatrices.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 15:01:21
//
#ifndef __GENERATETRMATRICES_H__
#define __GENERATETRMATRICES_H__

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
extern void generateTrMatrices(const double q[4], double kC_l1, double kC_l2,
  double kC_l3, double kC_l4, double kC_l5, double kC_l6, double kC_l7, double
  kC_l8, double kC_zeta, double kC_r, double kC_B2PXOffset, double kC_B2PZOffset,
  const double kC_legAngleOffset[4], int legNum, double TB2G[16], double TP2B[16],
  double TI2P[16], double TJ2I[16], double TO2J[16], double TQ2O[16], double
  TR2Q[16], double TS2R[16], double TW2S[16], double TC2W[16]);

#endif

//
// File trailer for generateTrMatrices.h
//
// [EOF]
//
