//
// File: buildRRTWrapper_emxutil.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 25-Feb-2015 17:06:16
//
#ifndef __BUILDRRTWRAPPER_EMXUTIL_H__
#define __BUILDRRTWRAPPER_EMXUTIL_H__

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
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for buildRRTWrapper_emxutil.h
//
// [EOF]
//
