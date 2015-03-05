/*
 * trInv.c
 *
 * Code generation for function 'trInv'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "trInv.h"
#include <stdio.h>

/* Function Definitions */
void trInv(const real_T T[16], real_T TInv[16])
{
  real_T b_T[9];
  int32_T i2;
  int32_T i3;
  real_T c_T[3];
  static const int8_T iv1[4] = { 0, 0, 0, 1 };

  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      b_T[i3 + 3 * i2] = -T[i2 + (i3 << 2)];
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    c_T[i2] = 0.0;
    for (i3 = 0; i3 < 3; i3++) {
      c_T[i2] += b_T[i2 + 3 * i3] * T[12 + i3];
    }

    for (i3 = 0; i3 < 3; i3++) {
      TInv[i3 + (i2 << 2)] = T[i2 + (i3 << 2)];
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    TInv[12 + i2] = c_T[i2];
  }

  for (i2 = 0; i2 < 4; i2++) {
    TInv[3 + (i2 << 2)] = iv1[i2];
  }
}

/* End of code generation (trInv.c) */
