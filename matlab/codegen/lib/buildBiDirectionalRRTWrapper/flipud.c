/*
 * File: flipud.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "flipud.h"
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void flipud(emxArray_real_T *x)
{
  int m;
  int md2;
  int j;
  int i;
  int x_idx_0;
  double xtmp;
  int b_x_idx_0;
  m = x->size[0];
  md2 = x->size[0] >> 1;
  for (j = 0; j < 10; j++) {
    for (i = 1; i <= md2; i++) {
      x_idx_0 = x->size[0];
      xtmp = x->data[(i + x_idx_0 * j) - 1];
      x_idx_0 = x->size[0];
      b_x_idx_0 = x->size[0];
      x->data[(i + x_idx_0 * j) - 1] = x->data[(m - i) + b_x_idx_0 * j];
      x_idx_0 = x->size[0];
      x->data[(m - i) + x_idx_0 * j] = xtmp;
    }
  }
}

/*
 * File trailer for flipud.c
 *
 * [EOF]
 */
