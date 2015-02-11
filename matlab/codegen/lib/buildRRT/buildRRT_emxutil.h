//
// File: buildRRT_emxutil.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//
#ifndef __BUILDRRT_EMXUTIL_H__
#define __BUILDRRT_EMXUTIL_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "buildRRT_types.h"

// Function Declarations
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for buildRRT_emxutil.h
//
// [EOF]
//
