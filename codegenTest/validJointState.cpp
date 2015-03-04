//
// File: validJointState.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:16:20
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "validJointState.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double b_state[10]
//                const double jointLimits[20]
// Return Type  : boolean_T
//
boolean_T validJointState(const double b_state[10], const double jointLimits[20])
{
  boolean_T valid;

  // VALIDJOINTSTATE Checks if a leg's joint state complies with the leg's angular 
  // position and rate limits.
  //
  // Inputs:
  // -state:       A 1x6 vector containing the joint state to be checked.
  // -jointLimits: A 2x6 vector containing the angular position and rate
  //               limits.
  // Outputs:
  // -valid:       A logical value indicating whether or not state is valid.
  //
  // validJointState.m
  // author: wreid
  // date:   20151402
  if ((b_state[0] < jointLimits[0]) || (b_state[0] > jointLimits[1]) ||
      (b_state[1] < jointLimits[2]) || (b_state[1] > jointLimits[3]) ||
      (b_state[2] < jointLimits[4]) || (b_state[2] > jointLimits[5]) ||
      (b_state[5] < jointLimits[10]) || (b_state[3] > jointLimits[11]) ||
      (b_state[6] < jointLimits[12]) || (0.0 > jointLimits[13]) || (b_state[7] <
       jointLimits[14]) || (b_state[5] > jointLimits[15])) {
    valid = false;
  } else {
    valid = true;
  }

  return valid;
}

//
// File trailer for validJointState.cpp
//
// [EOF]
//
