/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "selectInput.h"
#include "heuristicSingleLeg.h"
#include "angDiff.h"
#include "rk4.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = { 30, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

/* Function Definitions */
void selectInput(const emlrtStack *sp, const real_T xNear[13], const real_T
                 xRand[13], real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T
                 kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8,
                 real_T kC_zeta, real_T kC_r, real_T kC_B2PXOffset, real_T
                 kC_B2PZOffset, const real_T kC_legAngleOffset[4], const real_T
                 jointLimits[20], const real_T uBDot[6], int32_T legNum, real_T
                 xNew[13], real_T transitionArray[80])
{
  real_T candStates[65];
  real_T distance[5];
  real_T candTransArrays[400];
  int32_T ixstart;
  int32_T i8;
  struct0_T expl_temp;
  real_T U[2];
  static const real_T b_U[10] = { 0.049999999999999996, -0.049999999999999996,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.049999999999999996, -0.049999999999999996, 0.0 };

  real_T dv16[80];
  real_T b_candStates[13];
  int32_T aDiff;
  real_T mtmp;
  int32_T itmp;
  boolean_T exitg1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* selectInput Selects the most appropriate control input. */
  /*    A control input is selected from a set of control inputs, U. An input */
  /*    is selected by applying each of the inputs to to state xNear, which */
  /*    results in p candidate states, where p is the size of the input set. */
  /*    The control input corresponding to candidate state that is closest to */
  /*    x1 is returned as u. */
  /* Initialize arrays to store the candidate new state data and the */
  /* distances between each candidate state and the xNear state. */
  memset(&candStates[0], 0, 65U * sizeof(real_T));

  /* UJoint = zeros(U_SIZE,3); */
  /* Transform the control inputs to joint space. */
  /* for i = 1:U_SIZE */
  /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
  /* gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6)); */
  /* UJoint(i,:) = [U(i,:) gammaDotDot]; */
  /* end     */
  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  for (ixstart = 0; ixstart < 5; ixstart++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    for (i8 = 0; i8 < 4; i8++) {
      expl_temp.legAngleOffset[i8] = kC_legAngleOffset[i8];
    }

    expl_temp.B2PZOffset = kC_B2PZOffset;
    expl_temp.B2PXOffset = kC_B2PXOffset;
    expl_temp.r = kC_r;
    expl_temp.zeta = kC_zeta;
    expl_temp.l8 = kC_l8;
    expl_temp.l7 = kC_l7;
    expl_temp.l6 = kC_l6;
    expl_temp.l5 = kC_l5;
    expl_temp.l4 = kC_l4;
    expl_temp.l3 = kC_l3;
    expl_temp.l2 = kC_l2;
    expl_temp.l1 = kC_l1;
    for (i8 = 0; i8 < 2; i8++) {
      U[i8] = b_U[ixstart + 5 * i8];
    }

    st.site = &mb_emlrtRSI;
    rk4(&st, U, uBDot, xNear, jointLimits, &expl_temp, legNum, b_candStates,
        dv16);
    for (i8 = 0; i8 < 80; i8++) {
      candTransArrays[ixstart + 5 * i8] = dv16[i8];
    }

    for (i8 = 0; i8 < 13; i8++) {
      candStates[ixstart + 5 * i8] = b_candStates[i8];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    if (angDiff(xNear[3], candStates[15 + ixstart]) > 0.39269908169872414) {
      aDiff = 1;
    } else {
      /* aDiff = abs(aDiff/ankleDiffMax); */
      aDiff = 0;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    for (i8 = 0; i8 < 13; i8++) {
      b_candStates[i8] = candStates[ixstart + 5 * i8];
    }

    distance[ixstart] = 0.5 * heuristicSingleLeg(b_candStates, xRand, kC_l1,
      kC_l2, kC_l3, kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r) + 0.5 *
      (real_T)aDiff;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  ixstart = 1;
  mtmp = distance[0];
  itmp = 0;
  if (muDoubleScalarIsNaN(distance[0])) {
    aDiff = 1;
    exitg1 = false;
    while ((!exitg1) && (aDiff + 1 < 6)) {
      ixstart = aDiff + 1;
      if (!muDoubleScalarIsNaN(distance[aDiff])) {
        mtmp = distance[aDiff];
        itmp = aDiff;
        exitg1 = true;
      } else {
        aDiff++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 < 6) {
      if (distance[ixstart] < mtmp) {
        mtmp = distance[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  for (i8 = 0; i8 < 13; i8++) {
    xNew[i8] = candStates[itmp + 5 * i8];
  }

  for (i8 = 0; i8 < 80; i8++) {
    transitionArray[i8] = candTransArrays[itmp + 5 * i8];
  }

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
}

/* End of code generation (selectInput.c) */
