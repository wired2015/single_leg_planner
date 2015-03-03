//
// File: trInv.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 03-Mar-2015 11:19:40
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "trInv.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double T[16]
//                double TInv[16]
// Return Type  : void
//
void trInv(const double T[16], double TInv[16])
{
  double b_T[9];
  int i2;
  int i3;
  double c_T[3];
  static const signed char iv1[4] = { 0, 0, 0, 1 };

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

//
// File trailer for trInv.cpp
//
// [EOF]
//
