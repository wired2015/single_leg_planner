//
// File: buildRRT.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 25-Feb-2015 17:06:16
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "norm.h"
#include "rk4.h"
#include "buildRRTWrapper_emxutil.h"
#include "nearestNeighbour.h"
#include "rand.h"
#include "sherpaTTIK.h"
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
  double alphaRand;
  double xMax;
  double xMin;
  double xRand[3];
  double q[3];
  double gammaDotRand;
  int i9;
  double b_xRand[13];
  emxArray_real_T *unusedU4;
  double unusedU5;
  int xNear_size[2];
  double xNear_data[13];
  emxArray_real_T *candTransArrays;
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
  int itmp;
  boolean_T exitg1;
  double xNew_data[13];
  double b_uA[3];
  double b_uB[3];

  // randomState Picks a random state from the state space.
  //    A random state is selected from the state space within the boundaries of 
  //    the state space as defined by the MIN and MAX vectors. The state space has 
  //    a dimension n.
  //    Inputs:
  //        MIN:    The 1xn vector containing the minimum boundaries for the state 
  //                space.
  //        MAX:    The 1xn vector containing the maximum boundaries for the state 
  //                space.
  //    Outputs:
  //        xRand:  The 1xn vector describing the selected random state.
  // randomState.m
  // author: wreid
  // date: 20150107
  // [~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); 
  alphaRand = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * b_rand();
  if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[4])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[4]);

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos
            (jointLimits[2])) + kC->l5 * cos(asin(((((((kC->l1 - kC->l4 * sin
      (kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * sin(jointLimits[2])) -
      panHeight) / kC->l5));
  } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[4])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[4]);

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[5])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[5]);
  } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos
            (jointLimits[3])) + kC->l5 * cos(asin(((((((kC->l1 - kC->l4 * sin
      (kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * sin(jointLimits[3])) -
      panHeight) / kC->l5));

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[5])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[5]);
  } else {
    xMax = 0.0;
    xMin = 0.0;
  }

  xMax = xMin + (xMax - xMin) * b_rand();
  xRand[0] = xMax;
  xRand[1] = 0.0;
  xRand[2] = panHeight;
  b_sherpaTTIK(xRand, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
               kC->l8, kC->zeta, kC->r, jointLimits, q);
  xMax = (jointLimits[9] - jointLimits[8]) * b_rand() + jointLimits[8];
  xMin = (jointLimits[11] - jointLimits[10]) * b_rand() + jointLimits[10];
  gammaDotRand = (jointLimits[13] - jointLimits[12]) * b_rand() + jointLimits[12];

  // betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); 
  for (i9 = 0; i9 < 3; i9++) {
    b_xRand[i9] = 0.0;
  }

  emxInit_real_T(&unusedU4, 2);
  b_xRand[3] = alphaRand;
  b_xRand[4] = q[1];
  b_xRand[5] = q[2];
  b_xRand[6] = 0.0;
  b_xRand[7] = 0.0;
  b_xRand[8] = xMax;
  b_xRand[9] = xMin;
  b_xRand[10] = gammaDotRand;
  b_xRand[11] = 0.0;
  b_xRand[12] = 0.0;

  // if mod(nodeIDCount,goalSeedFreq) == 0
  //     xRand = nGoal;
  // end
  nearestNeighbour(b_xRand, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
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
  xMax = rt_roundd_snf(Dt / dt);
  i9 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 9;
  candTransArrays->size[1] = (int)((xMax + 1.0) * 10.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i9, (int)sizeof(double));
  loop_ub = 9 * (int)((xMax + 1.0) * 10.0);
  for (i9 = 0; i9 < loop_ub; i9++) {
    candTransArrays->data[i9] = 0.0;
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
    for (i9 = 0; i9 < 2; i9++) {
      b_U[i9] = U[i + 9 * i9];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 13U * sizeof(double));
    rk4(b_U, uBDot, dt, Dt, b_xNear_data, jointLimits, kC, legNum, tmp_data,
        tmp_size, unusedU4);
    loop_ub = tmp_size[1];
    for (i9 = 0; i9 < loop_ub; i9++) {
      candStates_data[i + 9 * i9] = tmp_data[tmp_size[0] * i9];
    }

    loop_ub = unusedU4->size[1];
    for (i9 = 0; i9 < loop_ub; i9++) {
      candTransArrays->data[i + candTransArrays->size[0] * i9] = unusedU4->
        data[unusedU4->size[0] * i9];
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
    uB[0] = ((((kC->l2 + kC->l3 * cos(-b_xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(b_xRand[5] + kC->zeta)) - kC->l7) * cos(alphaRand);
    uB[1] = ((((kC->l2 + kC->l3 * cos(-b_xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(b_xRand[5] + kC->zeta)) - kC->l7) * sin(alphaRand);
    uB[2] = ((((kC->l1 + kC->l3 * sin(-b_xRand[4])) - kC->l4 * sin(kC->zeta)) -
              kC->l5 * sin(b_xRand[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    xMax = ((xNear_data[3] - candStates_data[i + 27]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(xMax - rt_roundd_snf(xMax)) <= 2.2204460492503131E-16 * fabs(xMax))
    {
      xMax = 0.0;
    } else {
      xMax = (xMax - floor(xMax)) * 6.2831853071795862;
    }

    if (fabs(xMax - 3.1415926535897931) > 0.087266462599716474) {
      ixstart = 1;
    } else {
      // aDiff = abs(aDiff/ankleDiffMax);
      ixstart = 0;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    for (i9 = 0; i9 < 3; i9++) {
      xRand[i9] = uB[i9] - uA[i9];
    }

    distance_data[i] = 0.5 * norm(xRand) + 0.5 * (double)ixstart;

    // distance(i) = hDiff;
  }

  emxFree_real_T(&unusedU4);
  ixstart = 1;
  xMax = distance_data[0];
  itmp = 0;
  if (rtIsNaN(distance_data[0])) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= 9)) {
      ixstart = i + 1;
      if (!rtIsNaN(distance_data[i])) {
        xMax = distance_data[i];
        itmp = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (ixstart < 9) {
    while (ixstart + 1 <= 9) {
      if (distance_data[ixstart] < xMax) {
        xMax = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
  for (i9 = 0; i9 < 13; i9++) {
    xNew_data[i9] = candStates_data[itmp + 9 * i9];
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
  b_uB[0] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * cos
    (xNear_data[3]);
  b_uB[1] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * sin
    (xNear_data[3]);
  b_uB[2] = ((((kC->l1 + kC->l3 * sin(-xNear_data[4])) - kC->l4 * sin(kC->zeta))
              - kC->l5 * sin(xNear_data[5] + kC->zeta)) - kC->l6) - (kC->l8 +
    kC->r);

  // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
  // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
  //     uA = sherpaTTFK(xA(4:6),kC);
  //     uB = sherpaTTFK(xB(4:6),kC);
  // dPos = norm(uA-uB);
  // Calculate the total distance.
  // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
  for (i9 = 0; i9 < 3; i9++) {
    uB[i9] = b_uB[i9] - b_uA[i9];
  }

  xNew_data[2] = xNear_data[2] + norm(uB);

  // Cost
  ixstart = (int)*nodeIDCount - 1;
  loop_ub = candTransArrays->size[1] - 1;
  for (i9 = 0; i9 < 13; i9++) {
    T->data[ixstart + T->size[0] * i9] = xNew_data[i9];
  }

  for (i9 = 0; i9 <= loop_ub; i9++) {
    T->data[ixstart + T->size[0] * (i9 + 13)] = candTransArrays->data[itmp +
      candTransArrays->size[0] * i9];
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
