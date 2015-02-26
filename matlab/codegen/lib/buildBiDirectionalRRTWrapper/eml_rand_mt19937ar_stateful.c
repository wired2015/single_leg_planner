/*
 * File: eml_rand_mt19937ar_stateful.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void c_eml_rand_mt19937ar_stateful_i(void)
{
  unsigned int r;
  int mti;
  memset(&state[0], 0, 625U * sizeof(unsigned int));
  r = 5489U;
  state[0] = 5489U;
  for (mti = 0; mti < 623; mti++) {
    r = (r ^ r >> 30U) * 1812433253U + (1 + mti);
    state[mti + 1] = r;
  }

  state[624] = 624U;
}

/*
 * File trailer for eml_rand_mt19937ar_stateful.c
 *
 * [EOF]
 */
