//
// File: exp.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "exp.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : creal_T *x
// Return Type  : void
//
void b_exp(creal_T *x)
{
  double r;
  double x_im;
  if (rtIsInf(x->im) && rtIsInf(x->re) && (x->re < 0.0)) {
    x->re = 0.0;
    x->im = 0.0;
  } else {
    r = exp(x->re / 2.0);
    x_im = x->im;
    x->re = r * (r * cos(x->im));
    x->im = r * (r * sin(x_im));
  }
}

//
// File trailer for exp.cpp
//
// [EOF]
//
