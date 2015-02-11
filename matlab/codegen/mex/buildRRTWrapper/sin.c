/*
 * sin.c
 *
 * Code generation for function 'sin'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "sin.h"
#include <stdio.h>

/* Function Definitions */
void b_sin(creal_T *x)
{
  real_T x_re;
  real_T x_im;
  if (x->im == 0.0) {
    x->re = muDoubleScalarSin(x->re);
    x->im = 0.0;
  } else {
    x_re = x->re;
    x_im = x->im;
    x->re = muDoubleScalarSin(x->re) * muDoubleScalarCosh(x->im);
    x->im = muDoubleScalarCos(x_re) * muDoubleScalarSinh(x_im);
  }
}

/* End of code generation (sin.c) */
