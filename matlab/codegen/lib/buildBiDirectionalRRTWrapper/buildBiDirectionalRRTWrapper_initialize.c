/*
 * File: buildBiDirectionalRRTWrapper_initialize.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRTWrapper_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void buildBiDirectionalRRTWrapper_initialize(void)
{
  rt_InitInfAndNaN(8U);
  c_buildBiDirectionalRRTWrapper_();
  c_eml_rand_mt19937ar_stateful_i();
}

/*
 * File trailer for buildBiDirectionalRRTWrapper_initialize.c
 *
 * [EOF]
 */
