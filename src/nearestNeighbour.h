//
// File: nearestNeighbour.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 25-Feb-2015 11:22:41
//
#ifndef __NEARESTNEIGHBOUR_H__
#define __NEARESTNEIGHBOUR_H__

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
extern void nearestNeighbour(const double x[13], const emxArray_real_T *T,
  double kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
  kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r, double
  nodeIDCount, double xNear_data[], int xNear_size[2], emxArray_real_T
  *transitionArray, double *d);

#endif

//
// File trailer for nearestNeighbour.h
//
// [EOF]
//
