/*
 * File: validJointState.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

#ifndef __VALIDJOINTSTATE_H__
#define __VALIDJOINTSTATE_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildBiDirectionalRRTWrapper_types.h"

/* Function Declarations */
extern boolean_T validJointState(const double b_state[10], const double
  jointLimits[20]);

#endif

/*
 * File trailer for validJointState.h
 *
 * [EOF]
 */
