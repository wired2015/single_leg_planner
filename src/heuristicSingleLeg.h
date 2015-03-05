//
// File: heuristicSingleLeg.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//
#ifndef __HEURISTICSINGLELEG_H__
#define __HEURISTICSINGLELEG_H__

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
extern double b_heuristicSingleLeg(const double xA[13], const double xB[13],
  double kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
  kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r);
extern double c_heuristicSingleLeg(const double xA_data[], const double xB[93],
  double kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
  kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r);
extern double heuristicSingleLeg(const double xA[13], const double xB[93],
  double kC_l1, double kC_l2, double kC_l3, double kC_l4, double kC_l5, double
  kC_l6, double kC_l7, double kC_l8, double kC_zeta, double kC_r);

#endif

//
// File trailer for heuristicSingleLeg.h
//
// [EOF]
//
