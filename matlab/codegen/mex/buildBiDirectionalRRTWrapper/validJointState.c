/*
 * validJointState.c
 *
 * Code generation for function 'validJointState'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "validJointState.h"
#include <stdio.h>

/* Function Definitions */
boolean_T validJointState(const real_T state[10], const real_T jointLimits[20])
{
  boolean_T valid;

  /* VALIDJOINTSTATE Checks if a leg's joint state complies with the leg's angular */
  /* position and rate limits. */
  /*  */
  /* Inputs: */
  /* -state:       A 1x6 vector containing the joint state to be checked. */
  /* -jointLimits: A 2x6 vector containing the angular position and rate */
  /*               limits. */
  /* Outputs: */
  /* -valid:       A logical value indicating whether or not state is valid. */
  /*  */
  /* validJointState.m */
  /* author: wreid */
  /* date:   20151402 */
  if ((state[0] < jointLimits[0]) || (state[0] > jointLimits[1]) || (state[1] <
       jointLimits[2]) || (state[1] > jointLimits[3]) || (state[2] <
       jointLimits[4]) || (state[2] > jointLimits[5]) || (state[5] <
       jointLimits[10]) || (state[3] > jointLimits[11]) || (state[6] <
       jointLimits[12]) || (0.0 > jointLimits[13]) || (state[7] < jointLimits[14])
      || (state[5] > jointLimits[15])) {
    valid = false;
  } else {
    valid = true;
  }

  return valid;
}

/* End of code generation (validJointState.c) */
