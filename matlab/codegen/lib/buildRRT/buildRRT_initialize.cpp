//
// File: buildRRT_initialize.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "buildRRT_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void buildRRT_initialize()
{
  rt_InitInfAndNaN(8U);
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for buildRRT_initialize.cpp
//
// [EOF]
//
