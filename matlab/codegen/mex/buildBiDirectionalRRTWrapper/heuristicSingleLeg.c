/*
 * heuristicSingleLeg.c
 *
 * Code generation for function 'heuristicSingleLeg'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "heuristicSingleLeg.h"
#include "norm.h"
#include <stdio.h>

/* Function Definitions */
real_T b_heuristicSingleLeg(const real_T xA_data[], const real_T xB[93], real_T
  kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6,
  real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r)
{
  real_T d;
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];
  int32_T i13;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  /* Calculate the distance between angular positions. */
  /*      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC); */
  /*      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC); */
  /*       */
  /*      dxStarMax = xStarMax-xStarMin; */
  /*      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2)); */
  /*       */
  /*      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax); */
  /*       */
  /*      xStarA = legRadius(betaA,gammaA,kC); */
  /*      xStarB = legRadius(betaB,gammaB,kC); */
  /*       */
  /*      dxStar = xStarB-xStarA; */
  /*      dAlpha = angDiff(alphaA,alphaB); */
  /*       */
  /*      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2); */
  /*       */
  /*      dPosNorm = dPos/dPosMax; */
  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  uA[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xA_data[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xA_data[5]
             + kC_zeta)) - kC_l7) * muDoubleScalarCos(xA_data[3]);
  uA[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xA_data[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xA_data[5]
             + kC_zeta)) - kC_l7) * muDoubleScalarSin(xA_data[3]);
  uA[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-xA_data[4])) - kC_l4 *
             muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(xA_data[5]
             + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xB[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xB[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarCos(xB[3]);
  uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xB[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xB[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarSin(xB[3]);
  uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-xB[4])) - kC_l4 *
             muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(xB[5] +
             kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  qDot[0] = (-xB[8] * muDoubleScalarSin(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xB[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xB[4])) +
              kC_l4 * muDoubleScalarCos(kC_zeta)) - xB[9] * kC_l3 *
             muDoubleScalarCos(xB[3]) * muDoubleScalarSin(xB[4])) - xB[10] *
    kC_l5 * muDoubleScalarSin(xB[5] + kC_zeta) * muDoubleScalarCos(xB[3]);
  qDot[1] = (xB[8] * muDoubleScalarCos(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xB[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xB[4])) +
              kC_l4 * muDoubleScalarCos(kC_zeta)) - xB[10] * kC_l5 *
             muDoubleScalarSin(xB[5] + kC_zeta) * muDoubleScalarSin(xB[3])) -
    xB[9] * kC_l3 * muDoubleScalarSin(xB[3]) * muDoubleScalarSin(xB[4]);
  qDot[2] = -xB[9] * kC_l3 * muDoubleScalarCos(xB[4]) - kC_l5 * xB[10] *
    muDoubleScalarCos(kC_zeta + xB[5]);
  b_qDot[0] = (-xA_data[8] * muDoubleScalarSin(xA_data[3]) * ((((kC_l2 - kC_l7)
    + kC_l5 * muDoubleScalarCos(xA_data[5] + kC_zeta)) + kC_l3 *
    muDoubleScalarCos(xA_data[4])) + kC_l4 * muDoubleScalarCos(kC_zeta)) -
               xA_data[9] * kC_l3 * muDoubleScalarCos(xA_data[3]) *
               muDoubleScalarSin(xA_data[4])) - xA_data[10] * kC_l5 *
    muDoubleScalarSin(xA_data[5] + kC_zeta) * muDoubleScalarCos(xA_data[3]);
  b_qDot[1] = (xA_data[8] * muDoubleScalarCos(xA_data[3]) * ((((kC_l2 - kC_l7) +
    kC_l5 * muDoubleScalarCos(xA_data[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos
                 (xA_data[4])) + kC_l4 * muDoubleScalarCos(kC_zeta)) - xA_data
               [10] * kC_l5 * muDoubleScalarSin(xA_data[5] + kC_zeta) *
               muDoubleScalarSin(xA_data[3])) - xA_data[9] * kC_l3 *
    muDoubleScalarSin(xA_data[3]) * muDoubleScalarSin(xA_data[4]);
  b_qDot[2] = -xA_data[9] * kC_l3 * muDoubleScalarCos(xA_data[4]) - kC_l5 *
    xA_data[10] * muDoubleScalarCos(kC_zeta + xA_data[5]);
  for (i13 = 0; i13 < 3; i13++) {
    b_uB[i13] = uB[i13] - uA[i13];
    c_qDot[i13] = qDot[i13] - b_qDot[i13];
  }

  d = 0.9 * norm(b_uB) + 0.1 * b_norm(c_qDot);

  /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
  /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
  /*     uA = sherpaTTFK(xA(4:6),kC); */
  /*     uB = sherpaTTFK(xB(4:6),kC); */
  /* dPos = norm(uA-uB); */
  /* Calculate the total distance. */
  /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
  return d;
}

real_T heuristicSingleLeg(const real_T xA[13], const real_T xB[13], real_T kC_l1,
  real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T
  kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r)
{
  real_T d;
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];
  int32_T i12;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  /* Calculate the distance between angular positions. */
  /*      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC); */
  /*      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC); */
  /*       */
  /*      dxStarMax = xStarMax-xStarMin; */
  /*      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2)); */
  /*       */
  /*      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax); */
  /*       */
  /*      xStarA = legRadius(betaA,gammaA,kC); */
  /*      xStarB = legRadius(betaB,gammaB,kC); */
  /*       */
  /*      dxStar = xStarB-xStarA; */
  /*      dAlpha = angDiff(alphaA,alphaB); */
  /*       */
  /*      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2); */
  /*       */
  /*      dPosNorm = dPos/dPosMax; */
  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  uA[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xA[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xA[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarCos(xA[3]);
  uA[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xA[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xA[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarSin(xA[3]);
  uA[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-xA[4])) - kC_l4 *
             muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(xA[5] +
             kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xB[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xB[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarCos(xB[3]);
  uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xB[4])) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xB[5] +
             kC_zeta)) - kC_l7) * muDoubleScalarSin(xB[3]);
  uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-xB[4])) - kC_l4 *
             muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(xB[5] +
             kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  qDot[0] = (-xB[8] * muDoubleScalarSin(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xB[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xB[4])) +
              kC_l4 * muDoubleScalarCos(kC_zeta)) - xB[9] * kC_l3 *
             muDoubleScalarCos(xB[3]) * muDoubleScalarSin(xB[4])) - xB[10] *
    kC_l5 * muDoubleScalarSin(xB[5] + kC_zeta) * muDoubleScalarCos(xB[3]);
  qDot[1] = (xB[8] * muDoubleScalarCos(xB[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xB[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xB[4])) +
              kC_l4 * muDoubleScalarCos(kC_zeta)) - xB[10] * kC_l5 *
             muDoubleScalarSin(xB[5] + kC_zeta) * muDoubleScalarSin(xB[3])) -
    xB[9] * kC_l3 * muDoubleScalarSin(xB[3]) * muDoubleScalarSin(xB[4]);
  qDot[2] = -xB[9] * kC_l3 * muDoubleScalarCos(xB[4]) - kC_l5 * xB[10] *
    muDoubleScalarCos(kC_zeta + xB[5]);
  b_qDot[0] = (-xA[8] * muDoubleScalarSin(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xA[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xA[4])) +
    kC_l4 * muDoubleScalarCos(kC_zeta)) - xA[9] * kC_l3 * muDoubleScalarCos(xA[3])
               * muDoubleScalarSin(xA[4])) - xA[10] * kC_l5 * muDoubleScalarSin
    (xA[5] + kC_zeta) * muDoubleScalarCos(xA[3]);
  b_qDot[1] = (xA[8] * muDoubleScalarCos(xA[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
    muDoubleScalarCos(xA[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(xA[4])) +
    kC_l4 * muDoubleScalarCos(kC_zeta)) - xA[10] * kC_l5 * muDoubleScalarSin(xA
    [5] + kC_zeta) * muDoubleScalarSin(xA[3])) - xA[9] * kC_l3 *
    muDoubleScalarSin(xA[3]) * muDoubleScalarSin(xA[4]);
  b_qDot[2] = -xA[9] * kC_l3 * muDoubleScalarCos(xA[4]) - kC_l5 * xA[10] *
    muDoubleScalarCos(kC_zeta + xA[5]);
  for (i12 = 0; i12 < 3; i12++) {
    b_uB[i12] = uB[i12] - uA[i12];
    c_qDot[i12] = qDot[i12] - b_qDot[i12];
  }

  d = norm(b_uB) + 0.0 * b_norm(c_qDot);

  /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
  /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
  /*     uA = sherpaTTFK(xA(4:6),kC); */
  /*     uB = sherpaTTFK(xB(4:6),kC); */
  /* dPos = norm(uA-uB); */
  /* Calculate the total distance. */
  /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
  return d;
}

/* End of code generation (heuristicSingleLeg.c) */
