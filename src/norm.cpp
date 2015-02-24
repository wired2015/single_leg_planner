//
// File: norm.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 24-Feb-2015 15:18:00
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "norm.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double x[3]
// Return Type  : double
//
double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

//
// File trailer for norm.cpp
//
// [EOF]
//
