//
// File: extractKinematicConstants.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 12-Feb-2015 09:24:14
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "extractKinematicConstants.h"
#include <stdio.h>

// Function Definitions

//
// EXTRACTKINEMATICCONSTANTS Gets individual kinematic constants.
//    Takes the kinematicConst array and returns each of the individual
//    kinematic constants.
// Arguments    : const double kinematicConst[16]
//                double *L1
//                double *L2
//                double *L3
//                double *L4
//                double *L5
//                double *L6
//                double *L7
//                double *L8
//                double *zeta
//                double *r
//                double *B2PXOffset
//                double *B2PZOffset
//                double *leg1AngleOffset
//                double *leg2AngleOffset
//                double *leg3AngleOffset
//                double *leg4AngleOffset
// Return Type  : void
//
void extractKinematicConstants(const double kinematicConst[16], double *L1,
  double *L2, double *L3, double *L4, double *L5, double *L6, double *L7, double
  *L8, double *zeta, double *r, double *B2PXOffset, double *B2PZOffset, double
  *leg1AngleOffset, double *leg2AngleOffset, double *leg3AngleOffset, double
  *leg4AngleOffset)
{
  *L1 = kinematicConst[0];
  *L2 = kinematicConst[1];
  *L3 = kinematicConst[2];
  *L4 = kinematicConst[3];
  *L5 = kinematicConst[4];
  *L6 = kinematicConst[5];
  *L7 = kinematicConst[6];
  *L8 = kinematicConst[7];
  *zeta = kinematicConst[8];
  *r = kinematicConst[9];
  *B2PXOffset = kinematicConst[10];
  *B2PZOffset = kinematicConst[11];
  *leg1AngleOffset = kinematicConst[12];
  *leg2AngleOffset = kinematicConst[13];
  *leg3AngleOffset = kinematicConst[14];
  *leg4AngleOffset = kinematicConst[15];
}

//
// File trailer for extractKinematicConstants.cpp
//
// [EOF]
//
