/*
 * File: heuristicSingleLeg.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "heuristicSingleLeg.h"
#include "norm.h"
#include <stdio.h>

/* Function Definitions */

/*
 * heuristic Calculates the distance between states x1 and x2.
 * Arguments    : const double xA_data[]
 *                const emxArray_real_T *xB
 *                double kC_l1
 *                double kC_l2
 *                double kC_l3
 *                double kC_l4
 *                double kC_l5
 *                double kC_l6
 *                double kC_l7
 *                double kC_l8
 *                double kC_zeta
 *                double kC_r
 * Return Type  : double
 */
double heuristicSingleLeg(const double xA_data[], const emxArray_real_T *xB,
  double kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
  kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r)
{
  double d;
  double uA[3];
  double q_idx_0;
  double q_idx_1;
  double q_idx_2;
  double uB[3];
  double qDot_idx_0;
  double qDot_idx_1;
  double qDot_idx_2;
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int i10;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
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
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
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
  uA[0] = ((((kC_l2 + kC_l3 * cos(-xA_data[4])) + kC_l4 * cos(kC_zeta)) + kC_l5 *
            cos(xA_data[5] + kC_zeta)) - kC_l7) * cos(xA_data[3]);
  uA[1] = ((((kC_l2 + kC_l3 * cos(-xA_data[4])) + kC_l4 * cos(kC_zeta)) + kC_l5 *
            cos(xA_data[5] + kC_zeta)) - kC_l7) * sin(xA_data[3]);
  uA[2] = ((((kC_l1 + kC_l3 * sin(-xA_data[4])) - kC_l4 * sin(kC_zeta)) - kC_l5 *
            sin(xA_data[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);
  q_idx_0 = xB->data[3];
  q_idx_1 = xB->data[4];
  q_idx_2 = xB->data[5];

  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
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
  uB[0] = ((((kC_l2 + kC_l3 * cos(-q_idx_1)) + kC_l4 * cos(kC_zeta)) + kC_l5 *
            cos(q_idx_2 + kC_zeta)) - kC_l7) * cos(q_idx_0);
  uB[1] = ((((kC_l2 + kC_l3 * cos(-q_idx_1)) + kC_l4 * cos(kC_zeta)) + kC_l5 *
            cos(q_idx_2 + kC_zeta)) - kC_l7) * sin(q_idx_0);
  uB[2] = ((((kC_l1 + kC_l3 * sin(-q_idx_1)) - kC_l4 * sin(kC_zeta)) - kC_l5 *
            sin(q_idx_2 + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  qDot_idx_0 = xB->data[8];
  qDot_idx_1 = xB->data[9];
  qDot_idx_2 = xB->data[10];
  q_idx_0 = xB->data[3];
  q_idx_1 = xB->data[4];
  q_idx_2 = xB->data[5];

  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  qDot[0] = (-qDot_idx_0 * sin(q_idx_0) * ((((kC_l2 - kC_l7) + kC_l5 * cos
    (q_idx_2 + kC_zeta)) + kC_l3 * cos(q_idx_1)) + kC_l4 * cos(kC_zeta)) -
             qDot_idx_1 * kC_l3 * cos(q_idx_0) * sin(q_idx_1)) - qDot_idx_2 *
    kC_l5 * sin(q_idx_2 + kC_zeta) * cos(q_idx_0);
  qDot[1] = (qDot_idx_0 * cos(q_idx_0) * ((((kC_l2 - kC_l7) + kC_l5 * cos
    (q_idx_2 + kC_zeta)) + kC_l3 * cos(q_idx_1)) + kC_l4 * cos(kC_zeta)) -
             qDot_idx_2 * kC_l5 * sin(q_idx_2 + kC_zeta) * sin(q_idx_0)) -
    qDot_idx_1 * kC_l3 * sin(q_idx_0) * sin(q_idx_1);
  qDot[2] = -qDot_idx_1 * kC_l3 * cos(q_idx_1) - kC_l5 * qDot_idx_2 * cos
    (kC_zeta + q_idx_2);
  b_qDot[0] = (-xA_data[8] * sin(xA_data[3]) * ((((kC_l2 - kC_l7) + kC_l5 * cos
    (xA_data[5] + kC_zeta)) + kC_l3 * cos(xA_data[4])) + kC_l4 * cos(kC_zeta)) -
               xA_data[9] * kC_l3 * cos(xA_data[3]) * sin(xA_data[4])) -
    xA_data[10] * kC_l5 * sin(xA_data[5] + kC_zeta) * cos(xA_data[3]);
  b_qDot[1] = (xA_data[8] * cos(xA_data[3]) * ((((kC_l2 - kC_l7) + kC_l5 * cos
    (xA_data[5] + kC_zeta)) + kC_l3 * cos(xA_data[4])) + kC_l4 * cos(kC_zeta)) -
               xA_data[10] * kC_l5 * sin(xA_data[5] + kC_zeta) * sin(xA_data[3]))
    - xA_data[9] * kC_l3 * sin(xA_data[3]) * sin(xA_data[4]);
  b_qDot[2] = -xA_data[9] * kC_l3 * cos(xA_data[4]) - kC_l5 * xA_data[10] * cos
    (kC_zeta + xA_data[5]);
  for (i10 = 0; i10 < 3; i10++) {
    b_uB[i10] = uB[i10] - uA[i10];
    c_qDot[i10] = qDot[i10] - b_qDot[i10];
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

/*
 * File trailer for heuristicSingleLeg.c
 *
 * [EOF]
 */
