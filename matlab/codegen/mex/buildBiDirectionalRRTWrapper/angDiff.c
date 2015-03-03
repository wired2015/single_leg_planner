/*
 * angDiff.c
 *
 * Code generation for function 'angDiff'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "angDiff.h"
#include <stdio.h>

/* Function Definitions */
real_T angDiff(real_T th1, real_T th2)
{
  real_T r;

  /* angDiff Finds the angular difference between th1 and th2. */
  r = ((th1 - th2) + 3.1415926535897931) / 6.2831853071795862;
  if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
      muDoubleScalarAbs(r)) {
    r = 0.0;
  } else {
    r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
  }

  return muDoubleScalarAbs(r - 3.1415926535897931);
}

/* End of code generation (angDiff.c) */
