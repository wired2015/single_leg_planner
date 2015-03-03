//
// File: getPhiAndOmega.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 03-Mar-2015 11:19:40
//
#ifndef __GETPHIANDOMEGA_H__
#define __GETPHIANDOMEGA_H__

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
extern void getPhiAndOmega(const double uBDot[6], const double qDot[4], const
  double q[4], const struct0_T *kC, int legNum, double *phi, double *omega);

#endif

//
// File trailer for getPhiAndOmega.h
//
// [EOF]
//
