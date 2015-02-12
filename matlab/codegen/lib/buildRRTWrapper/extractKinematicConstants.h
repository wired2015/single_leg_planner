//
// File: extractKinematicConstants.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 12-Feb-2015 09:24:14
//
#ifndef __EXTRACTKINEMATICCONSTANTS_H__
#define __EXTRACTKINEMATICCONSTANTS_H__

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
extern void extractKinematicConstants(const double kinematicConst[16], double
  *L1, double *L2, double *L3, double *L4, double *L5, double *L6, double *L7,
  double *L8, double *zeta, double *r, double *B2PXOffset, double *B2PZOffset,
  double *leg1AngleOffset, double *leg2AngleOffset, double *leg3AngleOffset,
  double *leg4AngleOffset);

#endif

//
// File trailer for extractKinematicConstants.h
//
// [EOF]
//
