/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "selectInput.h"
#include "heuristicSingleLeg.h"
#include "angDiff.h"
#include "rk4.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo ob_emlrtRSI = { 30, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

/* Function Definitions */
void b_selectInput(const emlrtStack *sp, const real_T xNear[13], const real_T
                   xRand[13], const struct0_T *kC, const real_T jointLimits[20],
                   const real_T uBDot[6], int32_T legNum, real_T xNew[13],
                   real_T transitionArray[80])
{
  real_T candStates[65];
  real_T distance[5];
  real_T candTransArrays[400];
  int32_T ixstart;
  real_T U[2];
  int32_T i18;
  static const real_T b_U[10] = { 0.049999999999999996, -0.049999999999999996,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.049999999999999996, -0.049999999999999996, 0.0 };

  real_T dv19[80];
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
    for (i18 = 0; i18 < 2; i18++) {
      U[i18] = b_U[ixstart + 5 * i18];
    }

    st.site = &ob_emlrtRSI;
    rk4(&st, U, uBDot, xNear, jointLimits, kC, legNum, b_candStates, dv19);
    for (i18 = 0; i18 < 80; i18++) {
      candTransArrays[ixstart + 5 * i18] = dv19[i18];
    }

    for (i18 = 0; i18 < 13; i18++) {
      candStates[ixstart + 5 * i18] = b_candStates[i18];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    if (angDiff(xNear[3], candStates[15 + ixstart]) > 0.087266462599716474) {
      aDiff = 1;
    } else {
      /* aDiff = abs(aDiff/ankleDiffMax); */
      aDiff = 0;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    for (i18 = 0; i18 < 13; i18++) {
      b_candStates[i18] = candStates[ixstart + 5 * i18];
    }

    distance[ixstart] = 0.5 * b_heuristicSingleLeg(b_candStates, xRand, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r) +
      0.5 * (real_T)aDiff;

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

  for (i18 = 0; i18 < 13; i18++) {
    xNew[i18] = candStates[itmp + 5 * i18];
  }

  for (i18 = 0; i18 < 80; i18++) {
    transitionArray[i18] = candTransArrays[itmp + 5 * i18];
  }

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
}

void selectInput(const emlrtStack *sp, const real_T xNear[13], const real_T
                 xRand[13], const struct0_T *kC, const real_T jointLimits[20],
                 const real_T uBDot[6], int32_T legNum, real_T xNew[13], real_T
                 transitionArray[80])
{
  real_T candStates[65];
  real_T distance[5];
  real_T candTransArrays[400];
  int32_T ixstart;
  real_T U[2];
  int32_T i9;
  static const real_T b_U[10] = { 0.049999999999999996, -0.049999999999999996,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.012499999999999999, -0.012499999999999999, 0.0 };

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
    for (i9 = 0; i9 < 2; i9++) {
      U[i9] = b_U[ixstart + 5 * i9];
    }

    st.site = &ob_emlrtRSI;
    rk4(&st, U, uBDot, xNear, jointLimits, kC, legNum, b_candStates, dv16);
    for (i9 = 0; i9 < 80; i9++) {
      candTransArrays[ixstart + 5 * i9] = dv16[i9];
    }

    for (i9 = 0; i9 < 13; i9++) {
      candStates[ixstart + 5 * i9] = b_candStates[i9];
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
    for (i9 = 0; i9 < 13; i9++) {
      b_candStates[i9] = candStates[ixstart + 5 * i9];
    }

    distance[ixstart] = 0.5 * b_heuristicSingleLeg(b_candStates, xRand, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r) +
      0.5 * (real_T)aDiff;

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

  for (i9 = 0; i9 < 13; i9++) {
    xNew[i9] = candStates[itmp + 5 * i9];
  }

  for (i9 = 0; i9 < 80; i9++) {
    transitionArray[i9] = candTransArrays[itmp + 5 * i9];
  }

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
}

/* End of code generation (selectInput.c) */
