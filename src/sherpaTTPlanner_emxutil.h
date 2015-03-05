//
// File: sherpaTTPlanner_emxutil.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//
#ifndef __SHERPATTPLANNER_EMXUTIL_H__
#define __SHERPATTPLANNER_EMXUTIL_H__

// Include Files
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "sherpaTTPlanner_types.h"

// Function Declarations
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for sherpaTTPlanner_emxutil.h
//
// [EOF]
//
