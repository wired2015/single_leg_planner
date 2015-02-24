/*
 * asin.c
 *
 * Code generation for function 'asin'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "randomStateGenerator.h"
#include "asin.h"
#include <stdio.h>

/* Function Definitions */
void b_asin(creal_T *x)
{
  creal_T v;
  creal_T u;
  real_T uvr;
  real_T yr;
  real_T yi;
  boolean_T xneg;
  if ((x->im == 0.0) && (!(muDoubleScalarAbs(x->re) > 1.0))) {
    x->re = muDoubleScalarAsin(x->re);
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
    yr = muDoubleScalarAtan2(muDoubleScalarAbs(x->re), muDoubleScalarAbs(uvr));
    if ((x->re < 0.0) != (uvr < 0.0)) {
      yr = -yr;
    }

    yi = u.re * v.im - u.im * v.re;
    xneg = (yi < 0.0);
    if (xneg) {
      yi = -yi;
    }

    if (yi >= 2.68435456E+8) {
      yi = muDoubleScalarLog(yi) + 0.69314718055994529;
    } else if (yi > 2.0) {
      yi = muDoubleScalarLog(2.0 * yi + 1.0 / (muDoubleScalarSqrt(yi * yi + 1.0)
        + yi));
    } else {
      uvr = yi * yi;
      yi += uvr / (1.0 + muDoubleScalarSqrt(1.0 + uvr));
      uvr = muDoubleScalarAbs(yi);
      if ((uvr > 4.503599627370496E+15) || (!((!muDoubleScalarIsInf(yi)) &&
            (!muDoubleScalarIsNaN(yi))))) {
        yi = muDoubleScalarLog(1.0 + yi);
      } else if (uvr < 2.2204460492503131E-16) {
      } else {
        yi = muDoubleScalarLog(1.0 + yi) * (yi / ((1.0 + yi) - 1.0));
      }
    }

    if (xneg) {
      yi = -yi;
    }

    x->re = yr;
    x->im = yi;
  }
}

void eml_scalar_sqrt(creal_T *x)
{
  real_T absxi;
  real_T absxr;
  if (x->im == 0.0) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = muDoubleScalarSqrt(muDoubleScalarAbs(x->re));
    } else {
      absxi = muDoubleScalarSqrt(x->re);
      absxr = 0.0;
    }
  } else if (x->re == 0.0) {
    if (x->im < 0.0) {
      absxi = muDoubleScalarSqrt(-x->im / 2.0);
      absxr = -absxi;
    } else {
      absxi = muDoubleScalarSqrt(x->im / 2.0);
      absxr = absxi;
    }
  } else if (muDoubleScalarIsNaN(x->re) || muDoubleScalarIsNaN(x->im)) {
    absxi = rtNaN;
    absxr = rtNaN;
  } else if (muDoubleScalarIsInf(x->im)) {
    absxi = rtInf;
    absxr = x->im;
  } else if (muDoubleScalarIsInf(x->re)) {
    if (x->re < 0.0) {
      absxi = 0.0;
      absxr = rtInf;
    } else {
      absxi = rtInf;
      absxr = 0.0;
    }
  } else {
    absxr = muDoubleScalarAbs(x->re);
    absxi = muDoubleScalarAbs(x->im);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = muDoubleScalarHypot(absxr, absxi);
      if (absxi > absxr) {
        absxi = muDoubleScalarSqrt(absxi) * muDoubleScalarSqrt(1.0 + absxr /
          absxi);
      } else {
        absxi = muDoubleScalarSqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxi = muDoubleScalarSqrt((muDoubleScalarHypot(absxr, absxi) + absxr) *
        0.5);
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

/* End of code generation (asin.c) */
