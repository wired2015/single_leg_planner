//
// File: buildRRTWrapper_initialize.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 15:27:34
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
  buildRRTWrapper_init();
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for buildRRTWrapper_initialize.cpp
//
// [EOF]
//
