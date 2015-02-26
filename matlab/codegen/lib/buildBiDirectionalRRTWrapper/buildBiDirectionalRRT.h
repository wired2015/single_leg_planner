/*
 * File: buildBiDirectionalRRT.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

#ifndef __BUILDBIDIRECTIONALRRT_H__
#define __BUILDBIDIRECTIONALRRT_H__

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
extern void buildBiDirectionalRRT(const double nInit[13], const double nGoal[13],
  const double jointLimits[20], double panHeight, const double U[18], double dt,
  double Dt, const struct0_T *kC, const double uBDot[6], int legNum, const
  double TP2B[16], emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T
  *pathJ, emxArray_real_T *pathC);

#endif

/*
 * File trailer for buildBiDirectionalRRT.h
 *
 * [EOF]
 */
