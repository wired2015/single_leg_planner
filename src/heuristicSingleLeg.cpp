//
// File: heuristicSingleLeg.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "heuristicSingleLeg.h"
#include "norm.h"
#include <stdio.h>

// Function Definitions

//
// heuristic Calculates the distance between states x1 and x2.
// Arguments    : const double xA[13]
//                const double xB[13]
//                double kC_l1
//                double kC_l2
//                double kC_l3
//                double kC_l4
//                double kC_l5
//                double kC_l6
//                double kC_l7
//                double kC_l8
//                double kC_zeta
//                double kC_r
// Return Type  : double
//
double b_heuristicSingleLeg(const double xA[13], const double xB[13], double
  kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double kC_l6,
  double kC_l7, double kC_l8, double kC_zeta, double kC_r)
{
  double d;
  double uA[3];
  double uB[3];
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int i16;

  // heuristicSingleLeg.m
  // author: wreid
  // date: 20150107
  // Calculate the distance between angular positions.
  //      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC);
  //      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC);
  //
  //      dxStarMax = xStarMax-xStarMin;
  //      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));
  //
  //      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax);
  //
  //      xStarA = legRadius(betaA,gammaA,kC);
  //      xStarB = legRadius(betaB,gammaB,kC);
  //
  //      dxStar = xStarB-xStarA;
  //      dAlpha = angDiff(alphaA,alphaB);
  //
  //      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2);
  //
  //      dPosNorm = dPos/dPosMax;
  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uA[0] = ((((kC_l2 + kC_l3 * std::cos(-xA[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xA[5] + kC_zeta)) - kC_l7) * std::cos(xA[3]);
  uA[1] = ((((kC_l2 + kC_l3 * std::cos(-xA[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xA[5] + kC_zeta)) - kC_l7) * std::sin(xA[3]);
  uA[2] = ((((kC_l1 + kC_l3 * std::sin(-xA[4])) - kC_l4 * std::sin(kC_zeta)) -
            kC_l5 * std::sin(xA[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uB[0] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::cos(xB[3]);
  uB[1] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::sin(xB[3]);
  uB[2] = ((((kC_l1 + kC_l3 * std::sin(-xB[4])) - kC_l4 * std::sin(kC_zeta)) -
            kC_l5 * std::sin(xB[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  qDot[0] = (-xB[8] * std::sin(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xB[5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[9] * kC_l3 * std::cos(xB[3]) * std::sin(xB[4])) - xB[10] * kC_l5
    * std::sin(xB[5] + kC_zeta) * std::cos(xB[3]);
  qDot[1] = (xB[8] * std::cos(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos(xB
    [5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[10] * kC_l5 * std::sin(xB[5] + kC_zeta) * std::sin(xB[3])) - xB
    [9] * kC_l3 * std::sin(xB[3]) * std::sin(xB[4]);
  qDot[2] = -xB[9] * kC_l3 * std::cos(xB[4]) - kC_l5 * xB[10] * std::cos(kC_zeta
    + xB[5]);
  b_qDot[0] = (-xA[8] * std::sin(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xA[5] + kC_zeta)) + kC_l3 * std::cos(xA[4])) + kC_l4 * std::cos(kC_zeta)) -
               xA[9] * kC_l3 * std::cos(xA[3]) * std::sin(xA[4])) - xA[10] *
    kC_l5 * std::sin(xA[5] + kC_zeta) * std::cos(xA[3]);
  b_qDot[1] = (xA[8] * std::cos(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xA[5] + kC_zeta)) + kC_l3 * std::cos(xA[4])) + kC_l4 * std::cos(kC_zeta)) -
               xA[10] * kC_l5 * std::sin(xA[5] + kC_zeta) * std::sin(xA[3])) -
    xA[9] * kC_l3 * std::sin(xA[3]) * std::sin(xA[4]);
  b_qDot[2] = -xA[9] * kC_l3 * std::cos(xA[4]) - kC_l5 * xA[10] * std::cos
    (kC_zeta + xA[5]);
  for (i16 = 0; i16 < 3; i16++) {
    b_uB[i16] = uB[i16] - uA[i16];
    c_qDot[i16] = qDot[i16] - b_qDot[i16];
  }

  d = norm(b_uB) + 0.0 * b_norm(c_qDot);

  // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
  // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
  //     uA = sherpaTTFK(xA(4:6),kC);
  //     uB = sherpaTTFK(xB(4:6),kC);
  // dPos = norm(uA-uB);
  // Calculate the total distance.
  // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
  return d;
}

//
// heuristic Calculates the distance between states x1 and x2.
// Arguments    : const double xA_data[]
//                const double xB[93]
//                double kC_l1
//                double kC_l2
//                double kC_l3
//                double kC_l4
//                double kC_l5
//                double kC_l6
//                double kC_l7
//                double kC_l8
//                double kC_zeta
//                double kC_r
// Return Type  : double
//
double c_heuristicSingleLeg(const double xA_data[], const double xB[93], double
  kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double kC_l6,
  double kC_l7, double kC_l8, double kC_zeta, double kC_r)
{
  double d;
  double uA[3];
  double uB[3];
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int i17;

  // heuristicSingleLeg.m
  // author: wreid
  // date: 20150107
  // Calculate the distance between angular positions.
  //      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC);
  //      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC);
  //
  //      dxStarMax = xStarMax-xStarMin;
  //      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));
  //
  //      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax);
  //
  //      xStarA = legRadius(betaA,gammaA,kC);
  //      xStarB = legRadius(betaB,gammaB,kC);
  //
  //      dxStar = xStarB-xStarA;
  //      dAlpha = angDiff(alphaA,alphaB);
  //
  //      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2);
  //
  //      dPosNorm = dPos/dPosMax;
  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uA[0] = ((((kC_l2 + kC_l3 * std::cos(-xA_data[4])) + kC_l4 * std::cos(kC_zeta))
            + kC_l5 * std::cos(xA_data[5] + kC_zeta)) - kC_l7) * std::cos
    (xA_data[3]);
  uA[1] = ((((kC_l2 + kC_l3 * std::cos(-xA_data[4])) + kC_l4 * std::cos(kC_zeta))
            + kC_l5 * std::cos(xA_data[5] + kC_zeta)) - kC_l7) * std::sin
    (xA_data[3]);
  uA[2] = ((((kC_l1 + kC_l3 * std::sin(-xA_data[4])) - kC_l4 * std::sin(kC_zeta))
            - kC_l5 * std::sin(xA_data[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uB[0] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::cos(xB[3]);
  uB[1] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::sin(xB[3]);
  uB[2] = ((((kC_l1 + kC_l3 * std::sin(-xB[4])) - kC_l4 * std::sin(kC_zeta)) -
            kC_l5 * std::sin(xB[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  qDot[0] = (-xB[8] * std::sin(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xB[5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[9] * kC_l3 * std::cos(xB[3]) * std::sin(xB[4])) - xB[10] * kC_l5
    * std::sin(xB[5] + kC_zeta) * std::cos(xB[3]);
  qDot[1] = (xB[8] * std::cos(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos(xB
    [5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[10] * kC_l5 * std::sin(xB[5] + kC_zeta) * std::sin(xB[3])) - xB
    [9] * kC_l3 * std::sin(xB[3]) * std::sin(xB[4]);
  qDot[2] = -xB[9] * kC_l3 * std::cos(xB[4]) - kC_l5 * xB[10] * std::cos(kC_zeta
    + xB[5]);
  b_qDot[0] = (-xA_data[8] * std::sin(xA_data[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    std::cos(xA_data[5] + kC_zeta)) + kC_l3 * std::cos(xA_data[4])) + kC_l4 *
    std::cos(kC_zeta)) - xA_data[9] * kC_l3 * std::cos(xA_data[3]) * std::sin
               (xA_data[4])) - xA_data[10] * kC_l5 * std::sin(xA_data[5] +
    kC_zeta) * std::cos(xA_data[3]);
  b_qDot[1] = (xA_data[8] * std::cos(xA_data[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    std::cos(xA_data[5] + kC_zeta)) + kC_l3 * std::cos(xA_data[4])) + kC_l4 *
    std::cos(kC_zeta)) - xA_data[10] * kC_l5 * std::sin(xA_data[5] + kC_zeta) *
               std::sin(xA_data[3])) - xA_data[9] * kC_l3 * std::sin(xA_data[3])
    * std::sin(xA_data[4]);
  b_qDot[2] = -xA_data[9] * kC_l3 * std::cos(xA_data[4]) - kC_l5 * xA_data[10] *
    std::cos(kC_zeta + xA_data[5]);
  for (i17 = 0; i17 < 3; i17++) {
    b_uB[i17] = uB[i17] - uA[i17];
    c_qDot[i17] = qDot[i17] - b_qDot[i17];
  }

  d = 0.9 * norm(b_uB) + 0.1 * b_norm(c_qDot);

  // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
  // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
  //     uA = sherpaTTFK(xA(4:6),kC);
  //     uB = sherpaTTFK(xB(4:6),kC);
  // dPos = norm(uA-uB);
  // Calculate the total distance.
  // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
  return d;
}

//
// heuristic Calculates the distance between states x1 and x2.
// Arguments    : const double xA[13]
//                const double xB[93]
//                double kC_l1
//                double kC_l2
//                double kC_l3
//                double kC_l4
//                double kC_l5
//                double kC_l6
//                double kC_l7
//                double kC_l8
//                double kC_zeta
//                double kC_r
// Return Type  : double
//
double heuristicSingleLeg(const double xA[13], const double xB[93], double kC_l1,
  double kC_l2, double kC_l3, double kC_l4, double kC_l5, double kC_l6, double
  kC_l7, double kC_l8, double kC_zeta, double kC_r)
{
  double d;
  double uA[3];
  double uB[3];
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int i6;

  // heuristicSingleLeg.m
  // author: wreid
  // date: 20150107
  // Calculate the distance between angular positions.
  //      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC);
  //      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC);
  //
  //      dxStarMax = xStarMax-xStarMin;
  //      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));
  //
  //      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax);
  //
  //      xStarA = legRadius(betaA,gammaA,kC);
  //      xStarB = legRadius(betaB,gammaB,kC);
  //
  //      dxStar = xStarB-xStarA;
  //      dAlpha = angDiff(alphaA,alphaB);
  //
  //      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2);
  //
  //      dPosNorm = dPos/dPosMax;
  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uA[0] = ((((kC_l2 + kC_l3 * std::cos(-xA[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xA[5] + kC_zeta)) - kC_l7) * std::cos(xA[3]);
  uA[1] = ((((kC_l2 + kC_l3 * std::cos(-xA[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xA[5] + kC_zeta)) - kC_l7) * std::sin(xA[3]);
  uA[2] = ((((kC_l1 + kC_l3 * std::sin(-xA[4])) - kC_l4 * std::sin(kC_zeta)) -
            kC_l5 * std::sin(xA[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFK Sherpa_TT Forward Kinematics
  //    Calculates the x,y,z position of the contact point given the alpha,
  //    beta and gamma joint values.
  // SHERPATTFK Calcluates the Cartesian position of the wheel contact point
  // relative to the pan coordinate frame for the SherpaTT Leg.
  //
  // Inputs:
  // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma]
  // -kC: A struct containing the kinematic constants of the SherpaTT leg.
  // Outputs:
  //
  // sherpaTTFK.m
  // author: wreid
  // date: 20150122
  uB[0] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::cos(xB[3]);
  uB[1] = ((((kC_l2 + kC_l3 * std::cos(-xB[4])) + kC_l4 * std::cos(kC_zeta)) +
            kC_l5 * std::cos(xB[5] + kC_zeta)) - kC_l7) * std::sin(xB[3]);
  uB[2] = ((((kC_l1 + kC_l3 * std::sin(-xB[4])) - kC_l4 * std::sin(kC_zeta)) -
            kC_l5 * std::sin(xB[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  qDot[0] = (-xB[8] * std::sin(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xB[5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[9] * kC_l3 * std::cos(xB[3]) * std::sin(xB[4])) - xB[10] * kC_l5
    * std::sin(xB[5] + kC_zeta) * std::cos(xB[3]);
  qDot[1] = (xB[8] * std::cos(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos(xB
    [5] + kC_zeta)) + kC_l3 * std::cos(xB[4])) + kC_l4 * std::cos(kC_zeta)) -
             xB[10] * kC_l5 * std::sin(xB[5] + kC_zeta) * std::sin(xB[3])) - xB
    [9] * kC_l3 * std::sin(xB[3]) * std::sin(xB[4]);
  qDot[2] = -xB[9] * kC_l3 * std::cos(xB[4]) - kC_l5 * xB[10] * std::cos(kC_zeta
    + xB[5]);
  b_qDot[0] = (-xA[8] * std::sin(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xA[5] + kC_zeta)) + kC_l3 * std::cos(xA[4])) + kC_l4 * std::cos(kC_zeta)) -
               xA[9] * kC_l3 * std::cos(xA[3]) * std::sin(xA[4])) - xA[10] *
    kC_l5 * std::sin(xA[5] + kC_zeta) * std::cos(xA[3]);
  b_qDot[1] = (xA[8] * std::cos(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 * std::cos
    (xA[5] + kC_zeta)) + kC_l3 * std::cos(xA[4])) + kC_l4 * std::cos(kC_zeta)) -
               xA[10] * kC_l5 * std::sin(xA[5] + kC_zeta) * std::sin(xA[3])) -
    xA[9] * kC_l3 * std::sin(xA[3]) * std::sin(xA[4]);
  b_qDot[2] = -xA[9] * kC_l3 * std::cos(xA[4]) - kC_l5 * xA[10] * std::cos
    (kC_zeta + xA[5]);
  for (i6 = 0; i6 < 3; i6++) {
    b_uB[i6] = uB[i6] - uA[i6];
    c_qDot[i6] = qDot[i6] - b_qDot[i6];
  }

  d = norm(b_uB) + 0.0 * b_norm(c_qDot);

  // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
  // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
  //     uA = sherpaTTFK(xA(4:6),kC);
  //     uB = sherpaTTFK(xB(4:6),kC);
  // dPos = norm(uA-uB);
  // Calculate the total distance.
  // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
  return d;
}

//
// File trailer for heuristicSingleLeg.cpp
//
// [EOF]
//
