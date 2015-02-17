//
// File: validJointState.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "validJointState.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double b_state[6]
//                const double jointLimits[12]
// Return Type  : boolean_T
//
boolean_T validJointState(const double b_state[6], const double jointLimits[12])
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
      (b_state[3] < jointLimits[6]) || (b_state[3] > jointLimits[7]) ||
      (b_state[4] < jointLimits[8]) || (b_state[4] > jointLimits[9]) ||
      (b_state[5] < jointLimits[10]) || (b_state[5] > jointLimits[11])) {
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
