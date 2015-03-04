//
// File: buildBiDirectionalRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:16:20
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_emxutil.h"
#include "buildBiDirectionalRRT.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double nInitCartesianB[6]
//                const double nGoalCartesianB[6]
//                double phiInit
//                double omegaInit
//                const double jointLimits[20]
//                const struct0_T *kC
//                int legNum
//                const double uBDot[6]
//                emxArray_real_T *T1
//                emxArray_real_T *T2
//                emxArray_real_T *pathC
//                emxArray_real_T *pathJ
//                boolean_T *success
// Return Type  : void
//
void buildBiDirectionalRRTWrapper(const double nInitCartesianB[6], const double
  nGoalCartesianB[6], double phiInit, double omegaInit, const double
  jointLimits[20], const struct0_T *kC, int legNum, const double uBDot[6],
  emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T *pathC,
  emxArray_real_T *pathJ, boolean_T *success)
{
  double TP2B[16];
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  double b_TP2B[9];
  int i1;
  double c_TP2B[3];
  double TB2P[16];
  double b_TB2P[3];
  double d0;
  double qInit[3];
  double qGoal[3];
  double nInitJoint[10];
  double c_TB2P[3];
  double nGoalJoint[10];
  double dv0[13];
  double dv1[13];
  static double b_T2[69750];
  static double b_T1[69750];

  // BUILDRRTWRAPPER This function acts as a wrapper for the buildRRT function.
  // Code generation for the singleLegPlanner is performed using this function
  // as an entry point.
  //
  // Inputs:
  // -nInitCartesianB: the
  // -nGoalCartesianB:
  // -jointLimits:
  // -bodyHeight:
  // -U:
  // -dt:
  // -Dt:
  // -kC:
  // -threshold:
  // -legNum:
  //
  // Outputs:
  // -T:
  // -pathC:
  // -pathJ:
  // -success:
  //
  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  // GETPANHEIGHT Summary of this function goes here
  //    Detailed explanation goes here
  // Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame 
  // to the pan coordinate frame.
  // TRP2B Generates the homogeneous transformation matrix between the body
  // and pan coordinate frames. kC is a struct containing the kinematic
  // constants of the leg. legNum indicates the number of the leg that is being
  // considered.
  //
  // Inputs:
  // -kC: Struct of kinematic constants of the Sherpa_TT leg
  // -legNum: The leg number identification.
  // Outputs:
  // -TP2B: The homogeneous transformation matrix that is used to transform
  // coordinates from the pan frame to the body frame.
  //
  // trP2B.m
  // author:    wreid
  // date:      20150214
  // TRDH Generates the homogeneous transformation matrix A using the
  // Denavit-Hartenberg parameters theta, d, a and alpha.
  //
  // trDH.m
  // author:    wreid
  // date:      20150214
  TP2B[0] = std::cos(kC->legAngleOffset[legNum - 1]);
  TP2B[4] = -std::sin(kC->legAngleOffset[legNum - 1]);
  TP2B[8] = std::sin(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[12] = kC->B2PXOffset * std::cos(kC->legAngleOffset[legNum - 1]);
  TP2B[1] = std::sin(kC->legAngleOffset[legNum - 1]);
  TP2B[5] = std::cos(kC->legAngleOffset[legNum - 1]);
  TP2B[9] = -std::cos(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[13] = kC->B2PXOffset * std::sin(kC->legAngleOffset[legNum - 1]);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = kC->B2PZOffset;
  for (i0 = 0; i0 < 4; i0++) {
    TP2B[3 + (i0 << 2)] = iv0[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_TP2B[i1 + 3 * i0] = -TP2B[i0 + (i1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * i1] * TP2B[12 + i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      TB2P[i1 + (i0 << 2)] = TP2B[i0 + (i1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    TB2P[12 + i0] = c_TP2B[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    TB2P[3 + (i0 << 2)] = iv0[i0];
  }

  // inv(TP2B);%
  // Transform the Cartesian goal and final positions in the pan coordinate
  // frame to the joint space.
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += TB2P[i0 + (i1 << 2)] * nInitCartesianB[i1];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[i1];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += TB2P[i0 + (i1 << 2)] * nInitCartesianB[3 + i1];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nInitJoint[i0] = qInit[i0];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                c_TP2B);
  nInitJoint[3] = phiInit;
  nInitJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nInitJoint[i0 + 5] = c_TP2B[i0];
    c_TB2P[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TB2P[i0] += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[3 + i1];
    }

    b_TB2P[i0] = c_TB2P[i0];
    nGoalJoint[i0] = qGoal[i0];
  }

  nInitJoint[8] = 0.0;
  nInitJoint[9] = omegaInit;
  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                c_TP2B);
  nGoalJoint[3] = 0.0;
  nGoalJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nGoalJoint[i0 + 5] = c_TP2B[i0];
  }

  nGoalJoint[8] = 0.0;
  nGoalJoint[9] = 0.0;

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    // Run buildRRT.
    dv0[0] = 1.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    dv1[0] = 1.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    for (i0 = 0; i0 < 10; i0++) {
      dv0[i0 + 3] = nInitJoint[i0];
      dv1[i0 + 3] = nGoalJoint[i0];
    }

    buildBiDirectionalRRT(dv0, dv1, jointLimits, -(-nInitCartesianB[2] +
      kC->B2PZOffset), kC, uBDot, legNum, TP2B, b_T1, b_T2, pathJ, pathC);
    i0 = T1->size[0] * T1->size[1];
    T1->size[0] = 750;
    T1->size[1] = 93;
    emxEnsureCapacity((emxArray__common *)T1, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 69750; i0++) {
      T1->data[i0] = b_T1[i0];
    }

    i0 = T2->size[0] * T2->size[1];
    T2->size[0] = 750;
    T2->size[1] = 93;
    emxEnsureCapacity((emxArray__common *)T2, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 69750; i0++) {
      T2->data[i0] = b_T2[i0];
    }

    // Transform path back to the Cartesian space.
    //          for i = 1:length(pathsJ)
    //              pathC = transformPath(pathsJ(i).path,kC,TP2B);
    //              pathLength = pathC(end,2);
    //              if i <= 1
    //                  pathLengthMin = pathLength;
    //                  pathCMin = pathC;
    //                  pathIndexMin = i;
    //              elseif pathLength < pathLengthMin
    //                  pathLengthMin = pathLength;
    //                  pathCMin = pathC;
    //                  pathIndexMin = i;
    //              end
    //          end
    //          pathC = pathCMin;
    //          pathJ = pathsJ(pathIndexMin).path;
  } else {
    *success = false;
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 0;
    pathC->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathC, i0, (int)sizeof(double));
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    i0 = T1->size[0] * T1->size[1];
    T1->size[0] = 0;
    T1->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T1, i0, (int)sizeof(double));
    i0 = T2->size[0] * T2->size[1];
    T2->size[0] = 0;
    T2->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T2, i0, (int)sizeof(double));
  }

  // Linearly interpolate to the goal state from the final state.
  //      sFinalC = pathC(end,:);
  //      sGoalC = [0 0 nGoalCartesianB true];
  //      pathCorrection = linInterp(sFinalC,sGoalC,10);
  //      pathC = [pathC; pathCorrection];
  //      [h,~] = size(pathCorrection);
  //      for i = 1:h
  //          uB = TB2P(1:3,1:3)*pathCorrection(i,3:5)' + TB2P(1:3,4);
  //          q = sherpaTTIK(uB',kC,jointLimits);
  //          pathJ = [pathJ; [pathCorrection(i,1) q 0 0 0 0 0 0 0]];
  //      end
  //  function pathC = transformPath(pathJ,kC,TP2B)
  //      [pathH,~] = size(pathJ);
  //      pathC = zeros(pathH,9);
  //      dist2Go = 0;
  //      for i = 1:pathH
  //          uP = sherpaTTFK(pathJ(i,2:4),kC);
  //          uPDot = sherpaTTFKVel(pathJ(i,7:9),pathJ(i,2:4),kC);
  //          uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
  //          uBDot = TP2B(1:3,1:3)*uPDot;
  //          if i ~= 1
  //              dist2Go = dist2Go + norm(uB'-pathC(i-1,3:5));
  //          end
  //          pathC(i,:) = [pathJ(i,1) dist2Go uB' uBDot' false];
  //      end
  //  end
}

//
// File trailer for buildBiDirectionalRRTWrapper.cpp
//
// [EOF]
//
