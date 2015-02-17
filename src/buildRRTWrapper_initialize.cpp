//
// File: buildRRTWrapper_initialize.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void buildRRTWrapper_initialize()
{
  rt_InitInfAndNaN(8U);
  cartesianLimits_not_empty_init();
  buildRRTWrapper_init();
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for buildRRTWrapper_initialize.cpp
//
// [EOF]
//
