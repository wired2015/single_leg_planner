//
// File: heuristicSingleLeg.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 09-Feb-2015 13:36:11
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// heuristic Calculates the distance between states x1 and x2.
// Arguments    : const double xA_data[]
//                const emxArray_real_T *xB
//                const double kinematicConst[15]
// Return Type  : double
//
double heuristicSingleLeg(const double xA_data[], const emxArray_real_T *xB,
  const double kinematicConst[15])
{
  double d;
  double xStarA;
  double dxStar;
  double dAlpha;

  // heuristicSingleLeg.m
  // author: wreid
  // date: 20150107
  // Calculate the distance between angular positions.
  xStarA = (((kinematicConst[1] + kinematicConst[2] * cos(xA_data[4])) +
             kinematicConst[3] * cos(kinematicConst[8])) + kinematicConst[4] *
            cos(kinematicConst[8] + xA_data[5])) - kinematicConst[6];
  dxStar = ((((kinematicConst[1] + kinematicConst[2] * cos(xB->data[4])) +
              kinematicConst[3] * cos(kinematicConst[8])) + kinematicConst[4] *
             cos(kinematicConst[8] + xB->data[5])) - kinematicConst[6]) - xStarA;

  // angDiff Finds the angular difference between th1 and th2.
  dAlpha = ((xA_data[3] - xB->data[3]) + 3.1415926535897931) /
    6.2831853071795862;
  if (fabs(dAlpha - rt_roundd_snf(dAlpha)) <= 2.2204460492503131E-16 * fabs
      (dAlpha)) {
    dAlpha = 0.0;
  } else {
    dAlpha = (dAlpha - floor(dAlpha)) * 6.2831853071795862;
  }

  dAlpha = fabs(dAlpha - 3.1415926535897931);

  // Calculate the total distance.
  d = sqrt(dxStar * dxStar + xStarA * xStarA * (dAlpha * dAlpha));

  // dPosNorm+dVelNorm
  return d;
}

//
// File trailer for heuristicSingleLeg.cpp
//
// [EOF]
//
