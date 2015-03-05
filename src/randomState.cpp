//
// File: randomState.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "randomState.h"
#include "rand.h"
#include "sherpaTTIK.h"
#include <stdio.h>

// Function Definitions

//
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
// Arguments    : const double jointLimits[20]
//                double panHeight
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
//                double xRand[13]
// Return Type  : void
//
void randomState(const double jointLimits[20], double panHeight, double kC_l1,
                 double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
                 kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r,
                 double xRand[13])
{
  double alphaRand;
  double xMax;
  double xMin;
  double b_xRand[3];
  double q[3];
  double gammaDotRand;
  int i4;

  // randomState.m
  // author: wreid
  // date: 20150107
  // [~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); 
  alphaRand = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * b_rand();
  if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
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
    xMax = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos(std::
             asin(((-panHeight + ((((kC_l1 - kC_l4 * std::sin(kC_zeta)) - kC_l6)
      - kC_l8) - kC_r)) - kC_l5 * std::sin(kC_zeta + jointLimits[4])) / kC_l3)))
      + kC_l5 * std::cos(kC_zeta + jointLimits[4]);

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
    xMin = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos
            (jointLimits[2])) + kC_l5 * std::cos(std::asin(((((((kC_l1 - kC_l4 *
      std::sin(kC_zeta)) - kC_l6) - kC_l8) - kC_r) - kC_l3 * std::sin
      (jointLimits[2])) - panHeight) / kC_l5));
  } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
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
    xMax = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos(std::
             asin(((-panHeight + ((((kC_l1 - kC_l4 * std::sin(kC_zeta)) - kC_l6)
      - kC_l8) - kC_r)) - kC_l5 * std::sin(kC_zeta + jointLimits[4])) / kC_l3)))
      + kC_l5 * std::cos(kC_zeta + jointLimits[4]);

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
    xMin = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos(std::
             asin(((-panHeight + ((((kC_l1 - kC_l4 * std::sin(kC_zeta)) - kC_l6)
      - kC_l8) - kC_r)) - kC_l5 * std::sin(kC_zeta + jointLimits[5])) / kC_l3)))
      + kC_l5 * std::cos(kC_zeta + jointLimits[5]);
  } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
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
    xMax = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos
            (jointLimits[3])) + kC_l5 * std::cos(std::asin(((((((kC_l1 - kC_l4 *
      std::sin(kC_zeta)) - kC_l6) - kC_l8) - kC_r) - kC_l3 * std::sin
      (jointLimits[3])) - panHeight) / kC_l5));

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
    xMin = (((kC_l2 + kC_l4 * std::cos(kC_zeta)) - kC_l7) + kC_l3 * std::cos(std::
             asin(((-panHeight + ((((kC_l1 - kC_l4 * std::sin(kC_zeta)) - kC_l6)
      - kC_l8) - kC_r)) - kC_l5 * std::sin(kC_zeta + jointLimits[5])) / kC_l3)))
      + kC_l5 * std::cos(kC_zeta + jointLimits[5]);
  } else {
    xMax = 0.0;
    xMin = 0.0;
  }

  xMax = xMin + (xMax - xMin) * b_rand();
  b_xRand[0] = xMax;
  b_xRand[1] = 0.0;
  b_xRand[2] = panHeight;
  b_sherpaTTIK(b_xRand, kC_l1, kC_l2, kC_l3, kC_l4, kC_l5, kC_l6, kC_l7, kC_l8,
               kC_zeta, kC_r, jointLimits, q);
  xMax = (jointLimits[9] - jointLimits[8]) * b_rand() + jointLimits[8];
  xMin = (jointLimits[11] - jointLimits[10]) * b_rand() + jointLimits[10];
  gammaDotRand = (jointLimits[13] - jointLimits[12]) * b_rand() + jointLimits[12];

  // betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); 
  for (i4 = 0; i4 < 3; i4++) {
    xRand[i4] = 0.0;
  }

  xRand[3] = alphaRand;
  xRand[4] = q[1];
  xRand[5] = q[2];
  xRand[6] = 0.0;
  xRand[7] = 0.0;
  xRand[8] = xMax;
  xRand[9] = xMin;
  xRand[10] = gammaDotRand;
  xRand[11] = 0.0;
  xRand[12] = 0.0;

  // if mod(nodeIDCount,goalSeedFreq) == 0
  //     xRand = nGoal;
  // end
}

//
// File trailer for randomState.cpp
//
// [EOF]
//
