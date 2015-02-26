/*
 * File: trInv.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "trInv.h"
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : const double T[16]
 *                double TInv[16]
 * Return Type  : void
 */
void trInv(const double T[16], double TInv[16])
{
  double b_T[9];
  int i1;
  int i2;
  double c_T[3];
  static const signed char iv1[4] = { 0, 0, 0, 1 };

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

/*
 * File trailer for trInv.c
 *
 * [EOF]
 */
