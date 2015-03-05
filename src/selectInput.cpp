//
// File: selectInput.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 11:17:25
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "selectInput.h"
#include "heuristicSingleLeg.h"
#include "angDiff.h"
#include "rk4.h"
#include <stdio.h>

// Function Definitions

//
// selectInput Selects the most appropriate control input.
//    A control input is selected from a set of control inputs, U. An input
//    is selected by applying each of the inputs to to state xNear, which
//    results in p candidate states, where p is the size of the input set.
//    The control input corresponding to candidate state that is closest to
//    x1 is returned as u.
// Arguments    : const double xNear[13]
//                const double xRand[13]
//                const struct0_T *kC
//                const double jointLimits[20]
//                const double uBDot[6]
//                int legNum
//                double xNew[13]
//                double transitionArray[80]
// Return Type  : void
//
void b_selectInput(const double xNear[13], const double xRand[13], const
                   struct0_T *kC, const double jointLimits[20], const double
                   uBDot[6], int legNum, double xNew[13], double
                   transitionArray[80])
{
  double candStates[65];
  double distance[5];
  double candTransArrays[400];
  int ixstart;
  double U[2];
  int i21;
  static const double b_U[10] = { 0.049999999999999996, -0.049999999999999996,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.049999999999999996, -0.049999999999999996, 0.0 };

  double dv17[80];
  double b_candStates[13];
  int aDiff;
  double mtmp;
  int itmp;
  boolean_T exitg1;

  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  memset(&candStates[0], 0, 65U * sizeof(double));

  // UJoint = zeros(U_SIZE,3);
  // Transform the control inputs to joint space.
  // for i = 1:U_SIZE
  // gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); 
  // gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6));
  // UJoint(i,:) = [U(i,:) gammaDotDot];
  // end
  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  for (ixstart = 0; ixstart < 5; ixstart++) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i21 = 0; i21 < 2; i21++) {
      U[i21] = b_U[ixstart + 5 * i21];
    }

    rk4(U, uBDot, xNear, jointLimits, kC, legNum, b_candStates, dv17);
    for (i21 = 0; i21 < 80; i21++) {
      candTransArrays[ixstart + 5 * i21] = dv17[i21];
    }

    for (i21 = 0; i21 < 13; i21++) {
      candStates[ixstart + 5 * i21] = b_candStates[i21];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); 
    // Calculate the distance between the candidate state and the random
    // state.
    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    if (angDiff(xNear[3], candStates[15 + ixstart]) > 0.087266462599716474) {
      aDiff = 1;
    } else {
      // aDiff = abs(aDiff/ankleDiffMax);
      aDiff = 0;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    for (i21 = 0; i21 < 13; i21++) {
      b_candStates[i21] = candStates[ixstart + 5 * i21];
    }

    distance[ixstart] = 0.5 * b_heuristicSingleLeg(b_candStates, xRand, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r) +
      0.5 * (double)aDiff;

    // distance(i) = hDiff;
  }

  ixstart = 1;
  mtmp = distance[0];
  itmp = 0;
  if (rtIsNaN(distance[0])) {
    aDiff = 1;
    exitg1 = false;
    while ((!exitg1) && (aDiff + 1 < 6)) {
      ixstart = aDiff + 1;
      if (!rtIsNaN(distance[aDiff])) {
        mtmp = distance[aDiff];
        itmp = aDiff;
        exitg1 = true;
      } else {
        aDiff++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 < 6) {
      if (distance[ixstart] < mtmp) {
        mtmp = distance[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  for (i21 = 0; i21 < 13; i21++) {
    xNew[i21] = candStates[itmp + 5 * i21];
  }

  for (i21 = 0; i21 < 80; i21++) {
    transitionArray[i21] = candTransArrays[itmp + 5 * i21];
  }

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
}

//
// selectInput Selects the most appropriate control input.
//    A control input is selected from a set of control inputs, U. An input
//    is selected by applying each of the inputs to to state xNear, which
//    results in p candidate states, where p is the size of the input set.
//    The control input corresponding to candidate state that is closest to
//    x1 is returned as u.
// Arguments    : const double xNear[13]
//                const double xRand[13]
//                const struct0_T *kC
//                const double jointLimits[20]
//                const double uBDot[6]
//                int legNum
//                double xNew[13]
//                double transitionArray[80]
// Return Type  : void
//
void selectInput(const double xNear[13], const double xRand[13], const struct0_T
                 *kC, const double jointLimits[20], const double uBDot[6], int
                 legNum, double xNew[13], double transitionArray[80])
{
  double candStates[65];
  double distance[5];
  double candTransArrays[400];
  int ixstart;
  double U[2];
  int i12;
  static const double b_U[10] = { 0.049999999999999996, -0.049999999999999996,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.012499999999999999, -0.012499999999999999, 0.0 };

  double dv14[80];
  double b_candStates[13];
  int aDiff;
  double mtmp;
  int itmp;
  boolean_T exitg1;

  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  memset(&candStates[0], 0, 65U * sizeof(double));

  // UJoint = zeros(U_SIZE,3);
  // Transform the control inputs to joint space.
  // for i = 1:U_SIZE
  // gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); 
  // gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6));
  // UJoint(i,:) = [U(i,:) gammaDotDot];
  // end
  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  for (ixstart = 0; ixstart < 5; ixstart++) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i12 = 0; i12 < 2; i12++) {
      U[i12] = b_U[ixstart + 5 * i12];
    }

    rk4(U, uBDot, xNear, jointLimits, kC, legNum, b_candStates, dv14);
    for (i12 = 0; i12 < 80; i12++) {
      candTransArrays[ixstart + 5 * i12] = dv14[i12];
    }

    for (i12 = 0; i12 < 13; i12++) {
      candStates[ixstart + 5 * i12] = b_candStates[i12];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); 
    // Calculate the distance between the candidate state and the random
    // state.
    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    if (angDiff(xNear[3], candStates[15 + ixstart]) > 0.39269908169872414) {
      aDiff = 1;
    } else {
      // aDiff = abs(aDiff/ankleDiffMax);
      aDiff = 0;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    for (i12 = 0; i12 < 13; i12++) {
      b_candStates[i12] = candStates[ixstart + 5 * i12];
    }

    distance[ixstart] = 0.5 * b_heuristicSingleLeg(b_candStates, xRand, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r) +
      0.5 * (double)aDiff;

    // distance(i) = hDiff;
  }

  ixstart = 1;
  mtmp = distance[0];
  itmp = 0;
  if (rtIsNaN(distance[0])) {
    aDiff = 1;
    exitg1 = false;
    while ((!exitg1) && (aDiff + 1 < 6)) {
      ixstart = aDiff + 1;
      if (!rtIsNaN(distance[aDiff])) {
        mtmp = distance[aDiff];
        itmp = aDiff;
        exitg1 = true;
      } else {
        aDiff++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 < 6) {
      if (distance[ixstart] < mtmp) {
        mtmp = distance[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  for (i12 = 0; i12 < 13; i12++) {
    xNew[i12] = candStates[itmp + 5 * i12];
  }

  for (i12 = 0; i12 < 80; i12++) {
    transitionArray[i12] = candTransArrays[itmp + 5 * i12];
  }

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
}

//
// File trailer for selectInput.cpp
//
// [EOF]
//
