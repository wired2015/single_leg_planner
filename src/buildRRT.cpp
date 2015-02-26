//
// File: buildRRT.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 26-Feb-2015 11:03:31
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "buildRRT.h"
#include "norm.h"
#include "rk4.h"
#include "buildRRTWrapper_emxutil.h"
#include "nearestNeighbour.h"
#include "randomState.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : emxArray_real_T *T
//                const double jointLimits[20]
//                const struct0_T *kC
//                double panHeight
//                const double U[18]
//                double Dt
//                double dt
//                double *nodeIDCount
//                const double uBDot[6]
//                int legNum
// Return Type  : void
//
void rrtLoop(emxArray_real_T *T, const double jointLimits[20], const struct0_T
             *kC, double panHeight, const double U[18], double Dt, double dt,
             double *nodeIDCount, const double uBDot[6], int legNum)
{
  emxArray_real_T *unusedU4;
  double xRand[13];
  double unusedU5;
  int xNear_size[2];
  double xNear_data[13];
  emxArray_real_T *candTransArrays;
  int i12;
  int loop_ub;
  double candStates_data[117];
  double distance_data[9];
  int i;
  double b_U[2];
  double b_xNear_data[13];
  int tmp_size[2];
  double tmp_data[16];
  double uA[3];
  double uB[3];
  int ixstart;
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int itmp;
  boolean_T exitg1;
  double xNew_data[13];
  double b_uA[3];
  double c_uB[3];
  double d_qDot[3];
  double e_qDot[3];
  emxInit_real_T(&unusedU4, 2);
  randomState(jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
              kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);
  nearestNeighbour(xRand, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                   kC->l7, kC->l8, kC->zeta, kC->r, *nodeIDCount, xNear_data,
                   xNear_size, unusedU4, &unusedU5);

  // selectInput Selects the most appropriate control input.
  //    A control input is selected from a set of control inputs, U. An input
  //    is selected by applying each of the inputs to to state xNear, which
  //    results in p candidate states, where p is the size of the input set.
  //    The control input corresponding to candidate state that is closest to
  //    x1 is returned as u.
  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  emxInit_real_T(&candTransArrays, 2);
  unusedU5 = rt_roundd_snf(Dt / dt);
  i12 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 9;
  candTransArrays->size[1] = (int)((unusedU5 + 1.0) * 10.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i12, (int)sizeof(double));
  loop_ub = 9 * (int)((unusedU5 + 1.0) * 10.0);
  for (i12 = 0; i12 < loop_ub; i12++) {
    candTransArrays->data[i12] = 0.0;
  }

  // UJoint = zeros(U_SIZE,3);
  // Transform the control inputs to joint space.
  // for i = 1:U_SIZE
  // gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); 
  // gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6));
  // UJoint(i,:) = [U(i,:) gammaDotDot];
  // end
  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  for (i = 0; i < 9; i++) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i12 = 0; i12 < 2; i12++) {
      b_U[i12] = U[i + 9 * i12];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 13U * sizeof(double));
    rk4(b_U, uBDot, dt, Dt, b_xNear_data, jointLimits, kC, legNum, tmp_data,
        tmp_size, unusedU4);
    loop_ub = tmp_size[1];
    for (i12 = 0; i12 < loop_ub; i12++) {
      candStates_data[i + 9 * i12] = tmp_data[tmp_size[0] * i12];
    }

    loop_ub = unusedU4->size[1];
    for (i12 = 0; i12 < loop_ub; i12++) {
      candTransArrays->data[i + candTransArrays->size[0] * i12] = unusedU4->
        data[unusedU4->size[0] * i12];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); 
    // Calculate the distance between the candidate state and the random
    // state.
    // heuristic Calculates the distance between states x1 and x2.
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
    uA[0] = ((((kC->l2 + kC->l3 * cos(-candStates_data[i + 36])) + kC->l4 * cos
               (kC->zeta)) + kC->l5 * cos(candStates_data[i + 45] + kC->zeta)) -
             kC->l7) * cos(candStates_data[i + 27]);
    uA[1] = ((((kC->l2 + kC->l3 * cos(-candStates_data[i + 36])) + kC->l4 * cos
               (kC->zeta)) + kC->l5 * cos(candStates_data[i + 45] + kC->zeta)) -
             kC->l7) * sin(candStates_data[i + 27]);
    uA[2] = ((((kC->l1 + kC->l3 * sin(-candStates_data[i + 36])) - kC->l4 * sin
               (kC->zeta)) - kC->l5 * sin(candStates_data[i + 45] + kC->zeta)) -
             kC->l6) - (kC->l8 + kC->r);

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
    uB[0] = ((((kC->l2 + kC->l3 * cos(-xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(xRand[5] + kC->zeta)) - kC->l7) * cos(xRand[3]);
    uB[1] = ((((kC->l2 + kC->l3 * cos(-xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(xRand[5] + kC->zeta)) - kC->l7) * sin(xRand[3]);
    uB[2] = ((((kC->l1 + kC->l3 * sin(-xRand[4])) - kC->l4 * sin(kC->zeta)) -
              kC->l5 * sin(xRand[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

    // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
    // sherpaTTFKVel.m
    // author: wreid
    // date: 20150122
    // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
    // sherpaTTFKVel.m
    // author: wreid
    // date: 20150122
    // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
    // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
    //     uA = sherpaTTFK(xA(4:6),kC);
    //     uB = sherpaTTFK(xB(4:6),kC);
    // dPos = norm(uA-uB);
    // Calculate the total distance.
    // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    // angDiff Finds the angular difference between th1 and th2.
    unusedU5 = ((xNear_data[3] - candStates_data[i + 27]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(unusedU5 - rt_roundd_snf(unusedU5)) <= 2.2204460492503131E-16 *
        fabs(unusedU5)) {
      unusedU5 = 0.0;
    } else {
      unusedU5 = (unusedU5 - floor(unusedU5)) * 6.2831853071795862;
    }

    if (fabs(unusedU5 - 3.1415926535897931) > 0.087266462599716474) {
      ixstart = 1;
    } else {
      // aDiff = abs(aDiff/ankleDiffMax);
      ixstart = 0;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    qDot[0] = (-xRand[8] * sin(xRand[3]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
      (xRand[5] + kC->zeta)) + kC->l3 * cos(xRand[4])) + kC->l4 * cos(kC->zeta))
               - xRand[9] * kC->l3 * cos(xRand[3]) * sin(xRand[4])) - xRand[10] *
      kC->l5 * sin(xRand[5] + kC->zeta) * cos(xRand[3]);
    qDot[1] = (xRand[8] * cos(xRand[3]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
      (xRand[5] + kC->zeta)) + kC->l3 * cos(xRand[4])) + kC->l4 * cos(kC->zeta))
               - xRand[10] * kC->l5 * sin(xRand[5] + kC->zeta) * sin(xRand[3]))
      - xRand[9] * kC->l3 * sin(xRand[3]) * sin(xRand[4]);
    qDot[2] = -xRand[9] * kC->l3 * cos(xRand[4]) - kC->l5 * xRand[10] * cos
      (kC->zeta + xRand[5]);
    b_qDot[0] = (-candStates_data[i + 72] * sin(candStates_data[i + 27]) *
                 ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[i + 45] +
      kC->zeta)) + kC->l3 * cos(candStates_data[i + 36])) + kC->l4 * cos
                  (kC->zeta)) - candStates_data[i + 81] * kC->l3 * cos
                 (candStates_data[i + 27]) * sin(candStates_data[i + 36])) -
      candStates_data[i + 90] * kC->l5 * sin(candStates_data[i + 45] + kC->zeta)
      * cos(candStates_data[i + 27]);
    b_qDot[1] = (candStates_data[i + 72] * cos(candStates_data[i + 27]) *
                 ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[i + 45] +
      kC->zeta)) + kC->l3 * cos(candStates_data[i + 36])) + kC->l4 * cos
                  (kC->zeta)) - candStates_data[i + 90] * kC->l5 * sin
                 (candStates_data[i + 45] + kC->zeta) * sin(candStates_data[i +
      27])) - candStates_data[i + 81] * kC->l3 * sin(candStates_data[i + 27]) *
      sin(candStates_data[i + 36]);
    b_qDot[2] = -candStates_data[i + 81] * kC->l3 * cos(candStates_data[i + 36])
      - kC->l5 * candStates_data[i + 90] * cos(kC->zeta + candStates_data[i + 45]);
    for (i12 = 0; i12 < 3; i12++) {
      b_uB[i12] = uB[i12] - uA[i12];
      c_qDot[i12] = qDot[i12] - b_qDot[i12];
    }

    distance_data[i] = 0.5 * (norm(b_uB) + 0.0 * b_norm(c_qDot)) + 0.5 * (double)
      ixstart;

    // distance(i) = hDiff;
  }

  emxFree_real_T(&unusedU4);
  ixstart = 1;
  unusedU5 = distance_data[0];
  itmp = 0;
  if (rtIsNaN(distance_data[0])) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= 9)) {
      ixstart = i + 1;
      if (!rtIsNaN(distance_data[i])) {
        unusedU5 = distance_data[i];
        itmp = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (ixstart < 9) {
    while (ixstart + 1 <= 9) {
      if (distance_data[ixstart] < unusedU5) {
        unusedU5 = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
  for (i12 = 0; i12 < 13; i12++) {
    xNew_data[i12] = candStates_data[itmp + 9 * i12];
  }

  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  // Node ID
  xNew_data[1] = xNear_data[0];

  // Parent ID
  // heuristic Calculates the distance between states x1 and x2.
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
  b_uA[0] = ((((kC->l2 + kC->l3 * cos(-candStates_data[itmp + 36])) + kC->l4 *
               cos(kC->zeta)) + kC->l5 * cos(candStates_data[itmp + 45] +
    kC->zeta)) - kC->l7) * cos(candStates_data[itmp + 27]);
  b_uA[1] = ((((kC->l2 + kC->l3 * cos(-candStates_data[itmp + 36])) + kC->l4 *
               cos(kC->zeta)) + kC->l5 * cos(candStates_data[itmp + 45] +
    kC->zeta)) - kC->l7) * sin(candStates_data[itmp + 27]);
  b_uA[2] = ((((kC->l1 + kC->l3 * sin(-candStates_data[itmp + 36])) - kC->l4 *
               sin(kC->zeta)) - kC->l5 * sin(candStates_data[itmp + 45] +
    kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
  c_uB[0] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * cos
    (xNear_data[3]);
  c_uB[1] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * sin
    (xNear_data[3]);
  c_uB[2] = ((((kC->l1 + kC->l3 * sin(-xNear_data[4])) - kC->l4 * sin(kC->zeta))
              - kC->l5 * sin(xNear_data[5] + kC->zeta)) - kC->l6) - (kC->l8 +
    kC->r);

  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
  // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
  //     uA = sherpaTTFK(xA(4:6),kC);
  //     uB = sherpaTTFK(xB(4:6),kC);
  // dPos = norm(uA-uB);
  // Calculate the total distance.
  // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
  d_qDot[0] = (-xNear_data[8] * sin(xNear_data[3]) * ((((kC->l2 - kC->l7) +
    kC->l5 * cos(xNear_data[5] + kC->zeta)) + kC->l3 * cos(xNear_data[4])) +
    kC->l4 * cos(kC->zeta)) - xNear_data[9] * kC->l3 * cos(xNear_data[3]) * sin
               (xNear_data[4])) - xNear_data[10] * kC->l5 * sin(xNear_data[5] +
    kC->zeta) * cos(xNear_data[3]);
  d_qDot[1] = (xNear_data[8] * cos(xNear_data[3]) * ((((kC->l2 - kC->l7) +
    kC->l5 * cos(xNear_data[5] + kC->zeta)) + kC->l3 * cos(xNear_data[4])) +
    kC->l4 * cos(kC->zeta)) - xNear_data[10] * kC->l5 * sin(xNear_data[5] +
    kC->zeta) * sin(xNear_data[3])) - xNear_data[9] * kC->l3 * sin(xNear_data[3])
    * sin(xNear_data[4]);
  d_qDot[2] = -xNear_data[9] * kC->l3 * cos(xNear_data[4]) - kC->l5 *
    xNear_data[10] * cos(kC->zeta + xNear_data[5]);
  e_qDot[0] = (-candStates_data[itmp + 72] * sin(candStates_data[itmp + 27]) *
               ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[itmp + 45] +
    kC->zeta)) + kC->l3 * cos(candStates_data[itmp + 36])) + kC->l4 * cos
                (kC->zeta)) - candStates_data[itmp + 81] * kC->l3 * cos
               (candStates_data[itmp + 27]) * sin(candStates_data[itmp + 36])) -
    candStates_data[itmp + 90] * kC->l5 * sin(candStates_data[itmp + 45] +
    kC->zeta) * cos(candStates_data[itmp + 27]);
  e_qDot[1] = (candStates_data[itmp + 72] * cos(candStates_data[itmp + 27]) *
               ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[itmp + 45] +
    kC->zeta)) + kC->l3 * cos(candStates_data[itmp + 36])) + kC->l4 * cos
                (kC->zeta)) - candStates_data[itmp + 90] * kC->l5 * sin
               (candStates_data[itmp + 45] + kC->zeta) * sin
               (candStates_data[itmp + 27])) - candStates_data[itmp + 81] *
    kC->l3 * sin(candStates_data[itmp + 27]) * sin(candStates_data[itmp + 36]);
  e_qDot[2] = -candStates_data[itmp + 81] * kC->l3 * cos(candStates_data[itmp +
    36]) - kC->l5 * candStates_data[itmp + 90] * cos(kC->zeta +
    candStates_data[itmp + 45]);
  for (i12 = 0; i12 < 3; i12++) {
    uB[i12] = c_uB[i12] - b_uA[i12];
    qDot[i12] = d_qDot[i12] - e_qDot[i12];
  }

  xNew_data[2] = xNear_data[2] + (norm(uB) + 0.0 * b_norm(qDot));

  // Cost
  ixstart = (int)*nodeIDCount - 1;
  loop_ub = candTransArrays->size[1] - 1;
  for (i12 = 0; i12 < 13; i12++) {
    T->data[ixstart + T->size[0] * i12] = xNew_data[i12];
  }

  for (i12 = 0; i12 <= loop_ub; i12++) {
    T->data[ixstart + T->size[0] * (i12 + 13)] = candTransArrays->data[itmp +
      candTransArrays->size[0] * i12];
  }

  emxFree_real_T(&candTransArrays);

  // Append the new node to the tree.
  // if mod(nodeIDCount,100) == 0
  // fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
  // end
}

//
// File trailer for buildRRT.cpp
//
// [EOF]
//
