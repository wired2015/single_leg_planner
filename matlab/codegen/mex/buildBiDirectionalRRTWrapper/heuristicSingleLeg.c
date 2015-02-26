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
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Function Definitions */
real_T heuristicSingleLeg(const emlrtStack *sp, const real_T xA_data[], const
  emxArray_real_T *xB, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r)
{
  real_T d;
  int32_T i9;
  real_T uA[3];
  real_T q_idx_0;
  real_T q_idx_1;
  real_T q_idx_2;
  real_T uB[3];
  real_T qDot_idx_0;
  real_T qDot_idx_1;
  real_T qDot_idx_2;
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(4, 1, i9, &m_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(5, 1, i9, &n_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(6, 1, i9, &o_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(7, 1, i9, &p_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(8, 1, i9, &q_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(9, 1, i9, &r_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(10, 1, i9, &s_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(11, 1, i9, &t_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(12, 1, i9, &u_emlrtBCI, sp);
  i9 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(13, 1, i9, &v_emlrtBCI, sp);

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
  q_idx_0 = xB->data[3];
  q_idx_1 = xB->data[4];
  q_idx_2 = xB->data[5];

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
  uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q_idx_1)) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q_idx_2 +
             kC_zeta)) - kC_l7) * muDoubleScalarCos(q_idx_0);
  uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q_idx_1)) + kC_l4 *
             muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q_idx_2 +
             kC_zeta)) - kC_l7) * muDoubleScalarSin(q_idx_0);
  uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-q_idx_1)) - kC_l4 *
             muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(q_idx_2 +
             kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  qDot_idx_0 = xB->data[8];
  qDot_idx_1 = xB->data[9];
  qDot_idx_2 = xB->data[10];
  q_idx_0 = xB->data[3];
  q_idx_1 = xB->data[4];
  q_idx_2 = xB->data[5];

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  qDot[0] = (-qDot_idx_0 * muDoubleScalarSin(q_idx_0) * ((((kC_l2 - kC_l7) +
    kC_l5 * muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos
    (q_idx_1)) + kC_l4 * muDoubleScalarCos(kC_zeta)) - qDot_idx_1 * kC_l3 *
             muDoubleScalarCos(q_idx_0) * muDoubleScalarSin(q_idx_1)) -
    qDot_idx_2 * kC_l5 * muDoubleScalarSin(q_idx_2 + kC_zeta) *
    muDoubleScalarCos(q_idx_0);
  qDot[1] = (qDot_idx_0 * muDoubleScalarCos(q_idx_0) * ((((kC_l2 - kC_l7) +
    kC_l5 * muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos
    (q_idx_1)) + kC_l4 * muDoubleScalarCos(kC_zeta)) - qDot_idx_2 * kC_l5 *
             muDoubleScalarSin(q_idx_2 + kC_zeta) * muDoubleScalarSin(q_idx_0))
    - qDot_idx_1 * kC_l3 * muDoubleScalarSin(q_idx_0) * muDoubleScalarSin
    (q_idx_1);
  qDot[2] = -qDot_idx_1 * kC_l3 * muDoubleScalarCos(q_idx_1) - kC_l5 *
    qDot_idx_2 * muDoubleScalarCos(kC_zeta + q_idx_2);
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
  for (i9 = 0; i9 < 3; i9++) {
    b_uB[i9] = uB[i9] - uA[i9];
    c_qDot[i9] = qDot[i9] - b_qDot[i9];
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

/* End of code generation (heuristicSingleLeg.c) */
