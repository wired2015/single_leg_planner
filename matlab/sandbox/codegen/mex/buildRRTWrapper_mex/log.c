/*
 * log.c
 *
 * Code generation for function 'log'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "log.h"
#include <stdio.h>

/* Function Definitions */
void b_log(creal_T *x)
{
  real_T x_im;
  real_T x_re;
  if ((x->im == 0.0) && muDoubleScalarIsNaN(x->re)) {
  } else if ((muDoubleScalarAbs(x->re) > 8.9884656743115785E+307) ||
             (muDoubleScalarAbs(x->im) > 8.9884656743115785E+307)) {
    x_im = x->im;
    x_re = x->re;
    x->re = muDoubleScalarLog(muDoubleScalarHypot(x->re / 2.0, x->im / 2.0)) +
      0.69314718055994529;
    x->im = muDoubleScalarAtan2(x_im, x_re);
  } else {
    x_im = x->im;
    x_re = x->re;
    x->re = muDoubleScalarLog(muDoubleScalarHypot(x->re, x->im));
    x->im = muDoubleScalarAtan2(x_im, x_re);
  }
}

/* End of code generation (log.c) */
