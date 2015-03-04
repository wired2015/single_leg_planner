//
// File: angDiff.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:16:20
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
  if (std::abs(r - rt_roundd_snf(r)) <= 2.2204460492503131E-16 * std::abs(r)) {
    r = 0.0;
  } else {
    r = (r - std::floor(r)) * 6.2831853071795862;
  }

  return std::abs(r - 3.1415926535897931);
}

//
// File trailer for angDiff.cpp
//
// [EOF]
//
