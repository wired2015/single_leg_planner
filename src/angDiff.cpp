//
// File: angDiff.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 03-Mar-2015 11:19:40
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "angDiff.h"
#include "sherpaTTPlanner_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// angDiff Finds the angular difference between th1 and th2.
// Arguments    : double th1
//                double th2
// Return Type  : double
//
double angDiff(double th1, double th2)
{
  double r;
  r = ((th1 - th2) + 3.1415926535897931) / 6.2831853071795862;
  if (fabs(r - rt_roundd_snf(r)) <= 2.2204460492503131E-16 * fabs(r)) {
    r = 0.0;
  } else {
    r = (r - floor(r)) * 6.2831853071795862;
  }

  return fabs(r - 3.1415926535897931);
}

//
// File trailer for angDiff.cpp
//
// [EOF]
//
