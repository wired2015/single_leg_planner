//
// File: rk4.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 26-Feb-2015 11:03:31
//
#ifndef __RK4_H__
#define __RK4_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildRRTWrapper_types.h"

// Function Declarations
extern void rk4(const double uIn[2], const double uBDot[6], double dt, double Dt,
                double xInit_data[], const double jointLimits[20], const
                struct0_T *kC, int legNum, double xNew_data[], int xNew_size[2],
                emxArray_real_T *transitionArray);

#endif

//
// File trailer for rk4.h
//
// [EOF]
//
