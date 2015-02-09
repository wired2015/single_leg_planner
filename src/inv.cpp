//
// File: inv.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 09-Feb-2015 13:36:11
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "inv.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double x[16]
//                double y[16]
// Return Type  : void
//
void inv(const double x[16], double y[16])
{
  double A[16];
  int i1;
  signed char ipiv[4];
  int j;
  int c;
  int jBcol;
  int ix;
  double smax;
  int k;
  double s;
  int i;
  int kAcol;
  signed char p[4];
  for (i1 = 0; i1 < 16; i1++) {
    y[i1] = 0.0;
    A[i1] = x[i1];
  }

  for (i1 = 0; i1 < 4; i1++) {
    ipiv[i1] = (signed char)(1 + i1);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    jBcol = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 1; k + 1 <= 4 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        jBcol = k;
        smax = s;
      }
    }

    if (A[c + jBcol] != 0.0) {
      if (jBcol != 0) {
        ipiv[j] = (signed char)((j + jBcol) + 1);
        ix = j;
        jBcol += j;
        for (k = 0; k < 4; k++) {
          smax = A[ix];
          A[ix] = A[jBcol];
          A[jBcol] = smax;
          ix += 4;
          jBcol += 4;
        }
      }

      i1 = (c - j) + 4;
      for (i = c + 1; i + 1 <= i1; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 4;
    for (i = 1; i <= 3 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0) {
        ix = c + 1;
        i1 = (jBcol - j) + 8;
        for (k = 5 + jBcol; k + 1 <= i1; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 4;
      jBcol += 4;
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    p[i1] = (signed char)(1 + i1);
  }

  for (k = 0; k < 3; k++) {
    if (ipiv[k] > 1 + k) {
      jBcol = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (signed char)jBcol;
    }
  }

  for (k = 0; k < 4; k++) {
    c = p[k] - 1;
    y[k + ((p[k] - 1) << 2)] = 1.0;
    for (j = k; j + 1 < 5; j++) {
      if (y[j + (c << 2)] != 0.0) {
        for (i = j + 1; i + 1 < 5; i++) {
          y[i + (c << 2)] -= y[j + (c << 2)] * A[i + (j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    jBcol = j << 2;
    for (k = 3; k > -1; k += -1) {
      kAcol = k << 2;
      if (y[k + jBcol] != 0.0) {
        y[k + jBcol] /= A[k + kAcol];
        for (i = 0; i + 1 <= k; i++) {
          y[i + jBcol] -= y[k + jBcol] * A[i + kAcol];
        }
      }
    }
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
