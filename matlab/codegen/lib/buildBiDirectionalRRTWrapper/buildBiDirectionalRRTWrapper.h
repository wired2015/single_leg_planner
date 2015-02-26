/*
 * File: buildBiDirectionalRRTWrapper.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

#ifndef __BUILDBIDIRECTIONALRRTWRAPPER_H__
#define __BUILDBIDIRECTIONALRRTWRAPPER_H__

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
extern void buildBiDirectionalRRTWrapper(const double nInitCartesianB[6], const
  double nGoalCartesianB[6], double phiInit, double omegaInit, const double
  jointLimits[20], double bodyHeight, const double U[18], double dt, double Dt,
  const struct0_T *kC, double threshold, int legNum, const double uBDot[6],
  emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T *pathC,
  emxArray_real_T *pathJ, boolean_T *success);
extern void c_buildBiDirectionalRRTWrapper_(void);

#endif

/*
 * File trailer for buildBiDirectionalRRTWrapper.h
 *
 * [EOF]
 */
