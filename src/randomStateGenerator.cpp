//
// File: randomStateGenerator.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 11:17:25
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "rand.h"
#include "sherpaTTIK.h"
#include "sherpaTTPlanner_emxutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : int N
//                const double jointLimits[20]
//                const struct0_T *kC
//                double panHeight
//                int legNum
//                emxArray_real_T *states
// Return Type  : void
//
void randomStateGenerator(int N, const double jointLimits[20], const struct0_T
  *kC, double panHeight, int legNum, emxArray_real_T *states)
{
  long i22;
  int i23;
  int loop_ub;
  double u2_idx_2;
  double cartesianLimits_idx_0;
  double cartesianLimits_idx_2;
  double cartesianLimits_idx_3;
  double alphaRand;
  double xMax;
  double xMin;
  double xRand[3];
  double u1[3];
  double gammaDotRand;
  double dv18[13];
  double uP[3];
  double TP2B[16];
  static const signed char iv7[4] = { 0, 0, 0, 1 };

  int i24;

  // RANDOMSTATEGENERATOR Generates N number of random states that are valid in
  // the Sherpa_TT leg's configuration space.
  //
  // Inputs:
  // -N: The number of random states to be generated.
  // -jointLimits: A 2x10 array of joint angular limits and angular positions.
  // -kC: A structure containing the kinematic constants of the Sherpa TT leg.
  // -panHeight: The height of the pan coordinate frame relative to the the
  // wheel contact point.
  // -legNum: The leg identifier.
  //
  // Outputs:
  // -states: A Nx6 array. Each row of the array describes a single state. A
  // state has the structure [x y z xDot yDot zDot]. The state coordinates are
  // realtive to the body frame.
  //
  // randomStateGenerator.m
  // author: wreid
  // date: 20150305
  i22 = legNum + 1L;
  if (i22 > 2147483647L) {
    i22 = 2147483647L;
  } else {
    if (i22 < -2147483648L) {
      i22 = -2147483648L;
    }
  }

  legNum = (int)i22;
  if (legNum > 4) {
    legNum = 4;
  } else {
    if (legNum < 1) {
      legNum = 1;
    }
  }

  i23 = states->size[0] * states->size[1];
  states->size[0] = N;
  states->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)states, i23, (int)sizeof(double));
  loop_ub = N * 6;
  for (i23 = 0; i23 < loop_ub; i23++) {
    states->data[i23] = 0.0;
  }

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
  u2_idx_2 = ((((kC->l1 + kC->l3 * std::sin(-jointLimits[3])) - kC->l4 * std::
                sin(kC->zeta)) - kC->l5 * std::sin(jointLimits[5] + kC->zeta)) -
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
  cartesianLimits_idx_0 = ((((kC->l1 + kC->l3 * std::sin(-jointLimits[2])) -
    kC->l4 * std::sin(kC->zeta)) - kC->l5 * std::sin(jointLimits[4] + kC->zeta))
    - kC->l6) - (kC->l8 + kC->r);
  cartesianLimits_idx_2 = ((((kC->l1 + kC->l3 * std::sin(-jointLimits[2])) -
    kC->l4 * std::sin(kC->zeta)) - kC->l5 * std::sin(jointLimits[5] + kC->zeta))
    - kC->l6) - (kC->l8 + kC->r);
  cartesianLimits_idx_3 = ((((kC->l1 + kC->l3 * std::sin(-jointLimits[3])) -
    kC->l4 * std::sin(kC->zeta)) - kC->l5 * std::sin(jointLimits[4] + kC->zeta))
    - kC->l6) - (kC->l8 + kC->r);
  for (loop_ub = 0; loop_ub + 1 <= N; loop_ub++) {
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
    if ((panHeight <= cartesianLimits_idx_0) && (panHeight >=
         cartesianLimits_idx_2)) {
      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMax = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(std::asin(((-panHeight + ((((kC->l1 - kC->l4 * std::sin
        (kC->zeta)) - kC->l6) - kC->l8) - kC->r)) - kC->l5 * std::sin(kC->zeta +
                  jointLimits[4])) / kC->l3))) + kC->l5 * std::cos(kC->zeta +
        jointLimits[4]);

      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMin = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(jointLimits[2])) + kC->l5 * std::cos(std::asin(((((((kC->l1 -
        kC->l4 * std::sin(kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * std::
        sin(jointLimits[2])) - panHeight) / kC->l5));
    } else if ((panHeight < cartesianLimits_idx_2) && (panHeight >=
                cartesianLimits_idx_3)) {
      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMax = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(std::asin(((-panHeight + ((((kC->l1 - kC->l4 * std::sin
        (kC->zeta)) - kC->l6) - kC->l8) - kC->r)) - kC->l5 * std::sin(kC->zeta +
                  jointLimits[4])) / kC->l3))) + kC->l5 * std::cos(kC->zeta +
        jointLimits[4]);

      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMin = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(std::asin(((-panHeight + ((((kC->l1 - kC->l4 * std::sin
        (kC->zeta)) - kC->l6) - kC->l8) - kC->r)) - kC->l5 * std::sin(kC->zeta +
                  jointLimits[5])) / kC->l3))) + kC->l5 * std::cos(kC->zeta +
        jointLimits[5]);
    } else if ((panHeight < cartesianLimits_idx_3) && (panHeight >= u2_idx_2)) {
      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMax = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(jointLimits[3])) + kC->l5 * std::cos(std::asin(((((((kC->l1 -
        kC->l4 * std::sin(kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * std::
        sin(jointLimits[3])) - panHeight) / kC->l5));

      // GETXSTAR Returns the xStar value given either the height of the wheel
      // contact
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
      //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
      // if selector=true, or gamma if selector=false.
      // Outputs:
      // -xStar: The radius in a cylindrical coordinate representation that
      // connects the pan coordinate frame to the wheel contact coordinate frame. 
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150216
      xMin = (((kC->l2 + kC->l4 * std::cos(kC->zeta)) - kC->l7) + kC->l3 * std::
              cos(std::asin(((-panHeight + ((((kC->l1 - kC->l4 * std::sin
        (kC->zeta)) - kC->l6) - kC->l8) - kC->r)) - kC->l5 * std::sin(kC->zeta +
                  jointLimits[5])) / kC->l3))) + kC->l5 * std::cos(kC->zeta +
        jointLimits[5]);
    } else {
      xMax = 0.0;
      xMin = 0.0;
    }

    xMax = xMin + (xMax - xMin) * b_rand();
    xRand[0] = xMax;
    xRand[1] = 0.0;
    xRand[2] = panHeight;
    b_sherpaTTIK(xRand, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
                 kC->l8, kC->zeta, kC->r, jointLimits, u1);
    xMax = (jointLimits[9] - jointLimits[8]) * b_rand() + jointLimits[8];
    xMin = (jointLimits[11] - jointLimits[10]) * b_rand() + jointLimits[10];
    gammaDotRand = (jointLimits[13] - jointLimits[12]) * b_rand() + jointLimits
      [12];

    // betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); 
    // if mod(nodeIDCount,goalSeedFreq) == 0
    //     xRand = nGoal;
    // end
    for (i23 = 0; i23 < 3; i23++) {
      dv18[i23] = 0.0;
    }

    dv18[3] = alphaRand;
    dv18[4] = u1[1];
    dv18[5] = u1[2];
    dv18[6] = 0.0;
    dv18[7] = 0.0;
    dv18[8] = xMax;
    dv18[9] = xMin;
    dv18[10] = gammaDotRand;
    dv18[11] = 0.0;
    dv18[12] = 0.0;
    for (i23 = 0; i23 < 3; i23++) {
      u1[i23] = dv18[3 + i23];
    }

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
    uP[0] = ((((kC->l2 + kC->l3 * std::cos(-u1[1])) + kC->l4 * std::cos(kC->zeta))
              + kC->l5 * std::cos(u1[2] + kC->zeta)) - kC->l7) * std::cos
      (alphaRand);
    uP[1] = ((((kC->l2 + kC->l3 * std::cos(-u1[1])) + kC->l4 * std::cos(kC->zeta))
              + kC->l5 * std::cos(u1[2] + kC->zeta)) - kC->l7) * std::sin
      (alphaRand);
    uP[2] = ((((kC->l1 + kC->l3 * std::sin(-u1[1])) - kC->l4 * std::sin(kC->zeta))
              - kC->l5 * std::sin(u1[2] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    for (i23 = 0; i23 < 4; i23++) {
      TP2B[3 + (i23 << 2)] = iv7[i23];
    }

    for (i23 = 0; i23 < 3; i23++) {
      xMax = 0.0;
      for (i24 = 0; i24 < 3; i24++) {
        xMax += TP2B[i23 + (i24 << 2)] * uP[i24];
      }

      u1[i23] = xMax + TP2B[12 + i23];
    }

    for (i23 = 0; i23 < 3; i23++) {
      states->data[loop_ub + states->size[0] * i23] = u1[i23];
    }

    states->data[loop_ub + states->size[0] * 3] = 0.0;
    states->data[loop_ub + (states->size[0] << 2)] = 0.0;
    states->data[loop_ub + states->size[0] * 5] = 0.0;
  }

  // save('experimentStates','states')
}

//
// File trailer for randomStateGenerator.cpp
//
// [EOF]
//
