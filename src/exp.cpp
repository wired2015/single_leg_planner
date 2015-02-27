//
// File: exp.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 27-Feb-2015 15:48:27
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
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
