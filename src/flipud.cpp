//
// File: flipud.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 15:29:21
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "flipud.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : emxArray_real_T *x
// Return Type  : void
//
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
  for (j = 0; j < 7; j++) {
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

//
// File trailer for flipud.cpp
//
// [EOF]
//
