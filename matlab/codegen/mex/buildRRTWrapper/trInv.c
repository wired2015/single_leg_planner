/*
 * trInv.c
 *
 * Code generation for function 'trInv'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "trInv.h"
#include <stdio.h>

/* Function Definitions */
void trInv(const real_T T[16], real_T TInv[16])
{
  real_T b_T[9];
  int32_T i3;
  int32_T i4;
  real_T c_T[3];
  static const int8_T iv1[4] = { 0, 0, 0, 1 };

  for (i3 = 0; i3 < 3; i3++) {
    for (i4 = 0; i4 < 3; i4++) {
      b_T[i4 + 3 * i3] = -T[i3 + (i4 << 2)];
    }
  }

  for (i3 = 0; i3 < 3; i3++) {
    c_T[i3] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      c_T[i3] += b_T[i3 + 3 * i4] * T[12 + i4];
    }

    for (i4 = 0; i4 < 3; i4++) {
      TInv[i4 + (i3 << 2)] = T[i3 + (i4 << 2)];
    }
  }

  for (i3 = 0; i3 < 3; i3++) {
    TInv[12 + i3] = c_T[i3];
  }

  for (i3 = 0; i3 < 4; i3++) {
    TInv[3 + (i3 << 2)] = iv1[i3];
  }
}

/* End of code generation (trInv.c) */
