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
  int32_T i1;
  int32_T i2;
  real_T c_T[3];
  static const int8_T iv1[4] = { 0, 0, 0, 1 };

  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      b_T[i2 + 3 * i1] = -T[i1 + (i2 << 2)];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    c_T[i1] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      c_T[i1] += b_T[i1 + 3 * i2] * T[12 + i2];
    }

    for (i2 = 0; i2 < 3; i2++) {
      TInv[i2 + (i1 << 2)] = T[i1 + (i2 << 2)];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    TInv[12 + i1] = c_T[i1];
  }

  for (i1 = 0; i1 < 4; i1++) {
    TInv[3 + (i1 << 2)] = iv1[i1];
  }
}

/* End of code generation (trInv.c) */
