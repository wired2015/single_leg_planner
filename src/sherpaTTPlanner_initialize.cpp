//
// File: sherpaTTPlanner_initialize.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 15:01:21
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void sherpaTTPlanner_initialize()
{
  rt_InitInfAndNaN(8U);
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for sherpaTTPlanner_initialize.cpp
//
// [EOF]
//
