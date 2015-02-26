/*
 * exp.c
 *
 * Code generation for function 'exp'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "exp.h"
#include <stdio.h>

/* Function Definitions */
void b_exp(creal_T *x)
{
  real_T r;
  real_T x_im;
  if (muDoubleScalarIsInf(x->im) && muDoubleScalarIsInf(x->re) && (x->re < 0.0))
  {
    x->re = 0.0;
    x->im = 0.0;
  } else {
    r = muDoubleScalarExp(x->re / 2.0);
    x_im = x->im;
    x->re = r * (r * muDoubleScalarCos(x->im));
    x->im = r * (r * muDoubleScalarSin(x_im));
  }
}

/* End of code generation (exp.c) */
