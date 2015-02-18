/*
 * rk4.c
 *
 * Code generation for function 'rk4'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "rk4.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo g_emlrtRTEI = { 5, 35, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtRTEInfo k_emlrtRTEI = { 16, 5, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 56, 9, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

static emlrtECInfo e_emlrtECI = { -1, 56, 9, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtDCInfo k_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  1 };

static emlrtDCInfo l_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  4 };

static emlrtBCInfo gb_emlrtBCI = { -1, -1, 14, 5, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

/* Function Definitions */
void rk4(const emlrtStack *sp, real_T u[3], real_T dt, real_T Dt, real_T
         xInit_data[], const real_T jointLimits[12], real_T xNew_data[], int32_T
         xNew_size[2], emxArray_real_T *transitionArray)
{
  real_T numIterations;
  int32_T i7;
  real_T b_xInit_data[11];
  int32_T xInit_size_idx_1;
  real_T y;
  int32_T loop_ub;
  int32_T unnamed_idx_1;
  int32_T i;
  real_T c_xInit_data[6];
  real_T k1[6];
  real_T k2[6];
  real_T k3[6];
  real_T b_y;
  real_T xInit[6];
  int32_T i8;
  real_T tmp_data[16];

  /* rk4.m */
  /* author: wreid */
  /* date: 20150107 */
  /* rk4 Summary of this function goes here */
  /*    Detailed explanation goes here */
  numIterations = muDoubleScalarRound(Dt / dt);
  xNew_size[0] = 1;
  xNew_size[1] = 11;
  for (i7 = 0; i7 < 11; i7++) {
    xNew_data[i7] = 0.0;
  }

  for (i7 = 0; i7 < 6; i7++) {
    b_xInit_data[i7] = xInit_data[3 + i7];
  }

  xInit_size_idx_1 = 6;
  for (i7 = 0; i7 < 6; i7++) {
    xInit_data[i7] = b_xInit_data[i7];
  }

  /* xInitOrig = xInit; */
  i7 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  y = (numIterations + 1.0) * 6.0;
  y = emlrtNonNegativeCheckFastR2012b(y, &l_emlrtDCI, sp);
  transitionArray->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(y, &k_emlrtDCI,
    sp);
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i7, (int32_T)sizeof
                    (real_T), &g_emlrtRTEI);
  y = (numIterations + 1.0) * 6.0;
  y = emlrtNonNegativeCheckFastR2012b(y, &l_emlrtDCI, sp);
  loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(y, &k_emlrtDCI, sp);
  for (i7 = 0; i7 < loop_ub; i7++) {
    transitionArray->data[i7] = 0.0;
  }

  unnamed_idx_1 = (int32_T)((numIterations + 1.0) * 6.0);
  for (i7 = 0; i7 < 6; i7++) {
    transitionArray->data[emlrtDynamicBoundsCheckFastR2012b(i7 + 1, 1,
      unnamed_idx_1, &gb_emlrtBCI, sp) - 1] = xInit_data[i7];
  }

  emlrtForLoopVectorCheckR2012b(1.0, 1.0, numIterations, mxDOUBLE_CLASS,
    (int32_T)numIterations, &k_emlrtRTEI, sp);
  i = 0;
  while (i <= (int32_T)numIterations - 1) {
    for (i7 = 0; i7 < xInit_size_idx_1; i7++) {
      c_xInit_data[i7] = xInit_data[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k1[i7] = c_xInit_data[3 + i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k1[i7 + 3] = u[i7];
    }

    y = dt / 2.0;
    for (i7 = 0; i7 < 6; i7++) {
      c_xInit_data[i7] = xInit_data[i7] + y * k1[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k2[i7] = c_xInit_data[3 + i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k2[i7 + 3] = u[i7];
    }

    y = dt / 2.0;
    for (i7 = 0; i7 < 6; i7++) {
      c_xInit_data[i7] = xInit_data[i7] + y * k2[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k3[i7] = c_xInit_data[3 + i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      k3[i7 + 3] = u[i7];
    }

    y = dt / 2.0;
    b_y = dt / 6.0;
    for (i7 = 0; i7 < 6; i7++) {
      c_xInit_data[i7] = xInit_data[i7] + y * k3[i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      xInit[i7] = c_xInit_data[3 + i7];
    }

    for (i7 = 0; i7 < 3; i7++) {
      xInit[i7 + 3] = u[i7];
    }

    xNew_size[0] = 1;
    xNew_size[1] = 6;
    for (i7 = 0; i7 < 6; i7++) {
      xNew_data[xNew_size[0] * i7] = xInit_data[i7] + b_y * (((k1[i7] + 2.0 *
        k2[i7]) + 2.0 * k3[i7]) + xInit[i7]);
    }

    /* Check pan angular position limits */
    if ((xNew_data[0] > jointLimits[1]) || (xNew_data[0] < jointLimits[0])) {
      xNew_data[0] = xInit_data[0];
      xNew_data[3] = 0.0;
      u[0] = 0.0;
    }

    /* Check inner and outer leg angular position limits */
    if ((xNew_data[1] > jointLimits[3]) || (xNew_data[1] < jointLimits[2]) ||
        (xNew_data[2] > jointLimits[5]) || (xNew_data[2] < jointLimits[4])) {
      xNew_data[1] = xInit_data[1];
      xNew_data[2] = xInit_data[2];
      xNew_data[4] = 0.0;
      xNew_data[5] = 0.0;
      u[1] = 0.0;
      u[2] = 0.0;
    }

    /* Check pan angular velocity limits */
    if ((xNew_data[3] > jointLimits[7]) || (xNew_data[3] < jointLimits[6])) {
      xNew_data[3] = xInit_data[3];
      u[0] = 0.0;
    }

    /* Check inner and outer leg angular velocity limits */
    if ((xNew_data[4] > jointLimits[9]) || (xNew_data[4] < jointLimits[8]) ||
        (xNew_data[5] > jointLimits[11]) || (xNew_data[5] < jointLimits[10])) {
      xNew_data[4] = xInit_data[4];
      xNew_data[5] = xInit_data[5];
      u[1] = 0.0;
      u[2] = 0.0;
    }

    xInit_size_idx_1 = xNew_size[1];
    loop_ub = xNew_size[0] * xNew_size[1];
    for (i7 = 0; i7 < loop_ub; i7++) {
      xInit_data[i7] = xNew_data[i7];
    }

    y = 6.0 * (1.0 + (real_T)i) + 1.0;
    b_y = 6.0 * ((1.0 + (real_T)i) + 1.0);
    if (y > b_y) {
      i7 = 1;
      i8 = 1;
    } else {
      i7 = transitionArray->size[1];
      i8 = (int32_T)y;
      i7 = emlrtDynamicBoundsCheckFastR2012b(i8, 1, i7, &fb_emlrtBCI, sp);
      i8 = transitionArray->size[1];
      unnamed_idx_1 = (int32_T)b_y;
      i8 = emlrtDynamicBoundsCheckFastR2012b(unnamed_idx_1, 1, i8, &fb_emlrtBCI,
        sp) + 1;
    }

    i8 -= i7;
    emlrtSizeEqCheck1DFastR2012b(i8, 6, &e_emlrtECI, sp);
    loop_ub = xNew_size[1];
    for (i8 = 0; i8 < loop_ub; i8++) {
      transitionArray->data[(i7 + i8) - 1] = xNew_data[xNew_size[0] * i8];
    }

    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* xInit = [zeros(1,3) xInitOrig 0 0]; */
  unnamed_idx_1 = 5 + xNew_size[1];
  for (i7 = 0; i7 < 3; i7++) {
    tmp_data[i7] = 0.0;
  }

  loop_ub = xNew_size[1];
  for (i7 = 0; i7 < loop_ub; i7++) {
    tmp_data[i7 + 3] = xNew_data[xNew_size[0] * i7];
  }

  tmp_data[3 + xNew_size[1]] = 0.0;
  tmp_data[4 + xNew_size[1]] = 0.0;
  xNew_size[0] = 1;
  xNew_size[1] = unnamed_idx_1;
  for (i7 = 0; i7 < unnamed_idx_1; i7++) {
    xNew_data[xNew_size[0] * i7] = tmp_data[i7];
  }
}

/* End of code generation (rk4.c) */
