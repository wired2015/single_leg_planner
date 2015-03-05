//
// File: asin.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "asin.h"
#include "getPhiAndOmega.h"
#include "sherpaTTPlanner_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : creal_T *x
// Return Type  : void
//
void b_asin(creal_T *x)
{
  creal_T v;
  creal_T u;
  double uvr;
  double yr;
  double yi;
  boolean_T xneg;
  if ((x->im == 0.0) && (!(std::abs(x->re) > 1.0))) {
    x->re = std::asin(x->re);
    x->im = 0.0;
  } else {
    v.re = 1.0 + x->re;
    v.im = x->im;
    eml_scalar_sqrt(&v);
    if (x->im != 0.0) {
      u.re = 1.0 - x->re;
      u.im = -x->im;
      eml_scalar_sqrt(&u);
    } else {
      u.re = 1.0 - x->re;
      u.im = x->im;
      eml_scalar_sqrt(&u);
    }

    uvr = u.re * v.re - u.im * v.im;
    yr = rt_atan2d_snf(std::abs(x->re), std::abs(uvr));
    if ((x->re < 0.0) != (uvr < 0.0)) {
      yr = -yr;
    }

    yi = u.re * v.im - u.im * v.re;
    xneg = (yi < 0.0);
    if (xneg) {
      yi = -yi;
    }

    if (yi >= 2.68435456E+8) {
      yi = std::log(yi) + 0.69314718055994529;
    } else if (yi > 2.0) {
      yi = std::log(2.0 * yi + 1.0 / (std::sqrt(yi * yi + 1.0) + yi));
    } else {
      uvr = yi * yi;
      yi += uvr / (1.0 + std::sqrt(1.0 + uvr));
      uvr = std::abs(yi);
      if ((uvr > 4.503599627370496E+15) || (!((!rtIsInf(yi)) && (!rtIsNaN(yi)))))
      {
        yi = std::log(1.0 + yi);
      } else if (uvr < 2.2204460492503131E-16) {
      } else {
        yi = std::log(1.0 + yi) * (yi / ((1.0 + yi) - 1.0));
      }
    }

    if (xneg) {
      yi = -yi;
    }

    x->re = yr;
    x->im = yi;
  }
}

//
// Arguments    : creal_T *x
// Return Type  : void
//
void eml_scalar_sqrt(creal_T *x)
{
  double absxi;
  double absxr;
  if (x->im == 0.0) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = std::sqrt(std::abs(x->re));
    } else {
      absxi = std::sqrt(x->re);
      absxr = 0.0;
    }
  } else if (x->re == 0.0) {
    if (x->im < 0.0) {
      absxi = std::sqrt(-x->im / 2.0);
      absxr = -absxi;
    } else {
      absxi = std::sqrt(x->im / 2.0);
      absxr = absxi;
    }
  } else if (rtIsNaN(x->re) || rtIsNaN(x->im)) {
    absxi = rtNaN;
    absxr = rtNaN;
  } else if (rtIsInf(x->im)) {
    absxi = rtInf;
    absxr = x->im;
  } else if (rtIsInf(x->re)) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = rtInf;
    } else {
      absxi = rtInf;
      absxr = 0.0;
    }
  } else {
    absxr = std::abs(x->re);
    absxi = std::abs(x->im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = hypot(absxr, absxi);
      if (absxi > absxr) {
        absxi = std::sqrt(absxi) * std::sqrt(1.0 + absxr / absxi);
      } else {
        absxi = std::sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxi = std::sqrt((hypot(absxr, absxi) + absxr) * 0.5);
    }

    if (x->re > 0.0) {
      absxr = 0.5 * (x->im / absxi);
    } else {
      if (x->im < 0.0) {
        absxr = -absxi;
      } else {
        absxr = absxi;
      }

      absxi = 0.5 * (x->im / absxr);
    }
  }

  x->re = absxi;
  x->im = absxr;
}

//
// File trailer for asin.cpp
//
// [EOF]
//
