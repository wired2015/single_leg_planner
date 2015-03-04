//
// File: sin.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:16:20
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sin.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : creal_T *x
// Return Type  : void
//
void b_sin(creal_T *x)
{
  double x_re;
  double x_im;
  if (x->im == 0.0) {
    x->re = std::sin(x->re);
    x->im = 0.0;
  } else {
    x_re = x->re;
    x_im = x->im;
    x->re = std::sin(x->re) * std::cosh(x->im);
    x->im = std::cos(x_re) * std::sinh(x_im);
  }
}

//
// File trailer for sin.cpp
//
// [EOF]
//
