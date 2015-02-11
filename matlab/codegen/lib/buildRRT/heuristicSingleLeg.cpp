//
// File: heuristicSingleLeg.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "heuristicSingleLeg.h"
#include "buildRRT_rtwutil.h"

// Function Definitions

//
// heuristic Calculates the distance between states x1 and x2.
// Arguments    : const emxArray_real_T *xA
//                const emxArray_real_T *xB
//                const double kinematicConst[12]
// Return Type  : double
//
double heuristicSingleLeg(const emxArray_real_T *xA, const emxArray_real_T *xB,
  const double kinematicConst[12])
{
  double xStarA;
  double dxStar;
  double dAlpha;

  // heuristicSingleLeg.m
  // author: wreid
  // date: 20150107
  // [x1,y1,z1] = fk(alpha1,beta1,gamma1,kinematicConst);
  // [x2,y2,z2] = fk(alpha2,beta2,gamma2,kinematicConst);
  // distMAX = [sqrt(range(1)^2+range(2)+range(3)^2) sqrt(range(4)^2+range(5)^2+range(6)^2)]; 
  // distMAX = [10 1];
  // d = HGAINS(1)*cartDist(xA(4:6),xB(4:6))/distMAX(1) +...
  //     HGAINS(2)*abs(z1-z2);
  // d = HGAINS(1)*cartDist([x1 y1 z1],[x2 y2 z2])/distMAX(1);
  //     HGAINS(2)*cartDist(x1(7:9),x2(7:9))/distMAX(2);
  xStarA = (((kinematicConst[1] + kinematicConst[2] * cos(xA->data[4])) +
             kinematicConst[3] * cos(kinematicConst[8])) + kinematicConst[4] *
            cos(kinematicConst[8] + xA->data[5])) - kinematicConst[6];
  dxStar = ((((kinematicConst[1] + kinematicConst[2] * cos(xB->data[4])) +
              kinematicConst[3] * cos(kinematicConst[8])) + kinematicConst[4] *
             cos(kinematicConst[8] + xB->data[5])) - kinematicConst[6]) - xStarA;

  // angDiff Finds the angular difference between th1 and th2.
  dAlpha = ((xA->data[3] - xB->data[3]) + 3.1415926535897931) /
    6.2831853071795862;
  if (fabs(dAlpha - rt_roundd_snf(dAlpha)) <= 2.2204460492503131E-16 * fabs
      (dAlpha)) {
    dAlpha = 0.0;
  } else {
    dAlpha = (dAlpha - floor(dAlpha)) * 6.2831853071795862;
  }

  dAlpha = fabs(dAlpha - 3.1415926535897931);
  return sqrt(dxStar * dxStar + xStarA * xStarA * (dAlpha * dAlpha));
}

//
// File trailer for heuristicSingleLeg.cpp
//
// [EOF]
//
