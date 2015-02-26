/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "selectInput.h"
#include "buildRRTWrapper_mex_emxutil.h"
#include "norm.h"
#include "rk4.h"
#include "buildRRTWrapper_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = { 30, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo k_emlrtRTEI = { 1, 35, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo l_emlrtRTEI = { 12, 5, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo d_emlrtECI = { -1, 30, 26, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo e_emlrtECI = { -1, 30, 10, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtDCInfo e_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  1 };

static emlrtDCInfo f_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  4 };

/* Function Definitions */
void selectInput(const emlrtStack *sp, const real_T xNear_data[], const real_T
                 xRand[13], const real_T U[18], real_T dt, real_T Dt, const
                 struct0_T *kC, const real_T jointLimits[20], const real_T
                 uBDot[6], int32_T legNum, real_T xNew_data[], int32_T
                 xNew_size[2], emxArray_real_T *transitionArray)
{
  boolean_T b2;
  emxArray_real_T *candTransArrays;
  real_T x;
  int32_T i5;
  real_T d1;
  int32_T ixstart;
  emxArray_int32_T *r1;
  real_T candStates_data[117];
  real_T distance_data[9];
  emxArray_real_T *r2;
  int32_T i;
  real_T b_U[2];
  real_T b_xNear_data[13];
  int32_T tmp_size[2];
  real_T tmp_data[16];
  int32_T iv10[2];
  int32_T iv11[2];
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];
  int32_T itmp;
  boolean_T exitg1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  b2 = false;

  /* selectInput Selects the most appropriate control input. */
  /*    A control input is selected from a set of control inputs, U. An input */
  /*    is selected by applying each of the inputs to to state xNear, which */
  /*    results in p candidate states, where p is the size of the input set. */
  /*    The control input corresponding to candidate state that is closest to */
  /*    x1 is returned as u. */
  /* Initialize arrays to store the candidate new state data and the */
  /* distances between each candidate state and the xNear state. */
  emxInit_real_T(sp, &candTransArrays, 2, &l_emlrtRTEI, true);
  x = muDoubleScalarRound(Dt / dt);
  i5 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 9;
  d1 = (x + 1.0) * 10.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &f_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d1,
    &e_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i5, (int32_T)sizeof
                    (real_T), &k_emlrtRTEI);
  d1 = (x + 1.0) * 10.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &f_emlrtDCI, sp);
  ixstart = 9 * (int32_T)emlrtIntegerCheckFastR2012b(d1, &e_emlrtDCI, sp);
  for (i5 = 0; i5 < ixstart; i5++) {
    candTransArrays->data[i5] = 0.0;
  }

  emxInit_int32_T(sp, &r1, 1, &k_emlrtRTEI, true);

  /* UJoint = zeros(U_SIZE,3); */
  /* Transform the control inputs to joint space. */
  /* for i = 1:U_SIZE */
  /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
  /* gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6)); */
  /* UJoint(i,:) = [U(i,:) gammaDotDot]; */
  /* end     */
  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  emxInit_real_T(sp, &r2, 2, &k_emlrtRTEI, true);
  for (i = 0; i < 9; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    for (i5 = 0; i5 < 2; i5++) {
      b_U[i5] = U[i + 9 * i5];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 13U * sizeof(real_T));
    st.site = &mb_emlrtRSI;
    rk4(&st, b_U, uBDot, dt, Dt, b_xNear_data, jointLimits, kC, legNum, tmp_data,
        tmp_size, r2);
    if (!b2) {
      for (i5 = 0; i5 < 2; i5++) {
        iv10[i5] = 1 + 12 * i5;
      }

      b2 = true;
    }

    emlrtSubAssignSizeCheckR2012b(iv10, 2, tmp_size, 2, &e_emlrtECI, sp);
    ixstart = tmp_size[1];
    for (i5 = 0; i5 < ixstart; i5++) {
      candStates_data[i + 9 * i5] = tmp_data[tmp_size[0] * i5];
    }

    ixstart = candTransArrays->size[1];
    i5 = r1->size[0];
    r1->size[0] = ixstart;
    emxEnsureCapacity(sp, (emxArray__common *)r1, i5, (int32_T)sizeof(int32_T),
                      &k_emlrtRTEI);
    for (i5 = 0; i5 < ixstart; i5++) {
      r1->data[i5] = i5;
    }

    iv11[0] = 1;
    iv11[1] = r1->size[0];
    emlrtSubAssignSizeCheckR2012b(iv11, 2, *(int32_T (*)[2])r2->size, 2,
      &d_emlrtECI, sp);
    ixstart = r2->size[1];
    for (i5 = 0; i5 < ixstart; i5++) {
      candTransArrays->data[i + candTransArrays->size[0] * r1->data[i5]] =
        r2->data[r2->size[0] * i5];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    /* Calculate the distance between angular positions. */
    /*      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC); */
    /*      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC); */
    /*       */
    /*      dxStarMax = xStarMax-xStarMin; */
    /*      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2)); */
    /*       */
    /*      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax); */
    /*       */
    /*      xStarA = legRadius(betaA,gammaA,kC); */
    /*      xStarB = legRadius(betaB,gammaB,kC); */
    /*       */
    /*      dxStar = xStarB-xStarA; */
    /*      dAlpha = angDiff(alphaA,alphaB); */
    /*       */
    /*      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2); */
    /*       */
    /*      dPosNorm = dPos/dPosMax; */
    /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
    /* relative to the pan coordinate frame for the SherpaTT Leg. */
    /*  */
    /* Inputs: */
    /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
    /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
    /* Outputs: */
    /*  */
    /* sherpaTTFK.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFK Sherpa_TT Forward Kinematics */
    /*    Calculates the x,y,z position of the contact point given the alpha, */
    /*    beta and gamma joint values. */
    uA[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-candStates_data[i + 36])) +
               kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
              muDoubleScalarCos(candStates_data[i + 45] + kC->zeta)) - kC->l7) *
      muDoubleScalarCos(candStates_data[i + 27]);
    uA[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-candStates_data[i + 36])) +
               kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
              muDoubleScalarCos(candStates_data[i + 45] + kC->zeta)) - kC->l7) *
      muDoubleScalarSin(candStates_data[i + 27]);
    uA[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-candStates_data[i + 36])) -
               kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 *
              muDoubleScalarSin(candStates_data[i + 45] + kC->zeta)) - kC->l6) -
      (kC->l8 + kC->r);

    /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
    /* relative to the pan coordinate frame for the SherpaTT Leg. */
    /*  */
    /* Inputs: */
    /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
    /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
    /* Outputs: */
    /*  */
    /* sherpaTTFK.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFK Sherpa_TT Forward Kinematics */
    /*    Calculates the x,y,z position of the contact point given the alpha, */
    /*    beta and gamma joint values. */
    uB[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xRand[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(xRand[5]
               + kC->zeta)) - kC->l7) * muDoubleScalarCos(xRand[3]);
    uB[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xRand[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(xRand[5]
               + kC->zeta)) - kC->l7) * muDoubleScalarSin(xRand[3]);
    uB[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-xRand[4])) - kC->l4 *
               muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin(xRand[5]
               + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
    /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
    /*     uA = sherpaTTFK(xA(4:6),kC); */
    /*     uB = sherpaTTFK(xB(4:6),kC); */
    /* dPos = norm(uA-uB); */
    /* Calculate the total distance. */
    /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    /* angDiff Finds the angular difference between th1 and th2. */
    x = ((xNear_data[3] - candStates_data[i + 27]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(x - muDoubleScalarRound(x)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(x)) {
      x = 0.0;
    } else {
      x = (x - muDoubleScalarFloor(x)) * 6.2831853071795862;
    }

    if (muDoubleScalarAbs(x - 3.1415926535897931) > 0.087266462599716474) {
      ixstart = 1;
    } else {
      /* aDiff = abs(aDiff/ankleDiffMax); */
      ixstart = 0;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    qDot[0] = (-xRand[8] * muDoubleScalarSin(xRand[3]) * ((((kC->l2 - kC->l7) +
      kC->l5 * muDoubleScalarCos(xRand[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(xRand[4])) + kC->l4 * muDoubleScalarCos(kC->zeta)) -
               xRand[9] * kC->l3 * muDoubleScalarCos(xRand[3]) *
               muDoubleScalarSin(xRand[4])) - xRand[10] * kC->l5 *
      muDoubleScalarSin(xRand[5] + kC->zeta) * muDoubleScalarCos(xRand[3]);
    qDot[1] = (xRand[8] * muDoubleScalarCos(xRand[3]) * ((((kC->l2 - kC->l7) +
      kC->l5 * muDoubleScalarCos(xRand[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(xRand[4])) + kC->l4 * muDoubleScalarCos(kC->zeta)) -
               xRand[10] * kC->l5 * muDoubleScalarSin(xRand[5] + kC->zeta) *
               muDoubleScalarSin(xRand[3])) - xRand[9] * kC->l3 *
      muDoubleScalarSin(xRand[3]) * muDoubleScalarSin(xRand[4]);
    qDot[2] = -xRand[9] * kC->l3 * muDoubleScalarCos(xRand[4]) - kC->l5 * xRand
      [10] * muDoubleScalarCos(kC->zeta + xRand[5]);
    b_qDot[0] = (-candStates_data[i + 72] * muDoubleScalarSin(candStates_data[i
      + 27]) * ((((kC->l2 - kC->l7) + kC->l5 * muDoubleScalarCos
                  (candStates_data[i + 45] + kC->zeta)) + kC->l3 *
                 muDoubleScalarCos(candStates_data[i + 36])) + kC->l4 *
                muDoubleScalarCos(kC->zeta)) - candStates_data[i + 81] * kC->l3 *
                 muDoubleScalarCos(candStates_data[i + 27]) * muDoubleScalarSin
                 (candStates_data[i + 36])) - candStates_data[i + 90] * kC->l5 *
      muDoubleScalarSin(candStates_data[i + 45] + kC->zeta) * muDoubleScalarCos
      (candStates_data[i + 27]);
    b_qDot[1] = (candStates_data[i + 72] * muDoubleScalarCos(candStates_data[i +
      27]) * ((((kC->l2 - kC->l7) + kC->l5 * muDoubleScalarCos(candStates_data[i
      + 45] + kC->zeta)) + kC->l3 * muDoubleScalarCos(candStates_data[i + 36]))
              + kC->l4 * muDoubleScalarCos(kC->zeta)) - candStates_data[i + 90] *
                 kC->l5 * muDoubleScalarSin(candStates_data[i + 45] + kC->zeta) *
                 muDoubleScalarSin(candStates_data[i + 27])) - candStates_data[i
      + 81] * kC->l3 * muDoubleScalarSin(candStates_data[i + 27]) *
      muDoubleScalarSin(candStates_data[i + 36]);
    b_qDot[2] = -candStates_data[i + 81] * kC->l3 * muDoubleScalarCos
      (candStates_data[i + 36]) - kC->l5 * candStates_data[i + 90] *
      muDoubleScalarCos(kC->zeta + candStates_data[i + 45]);
    for (i5 = 0; i5 < 3; i5++) {
      b_uB[i5] = uB[i5] - uA[i5];
      c_qDot[i5] = qDot[i5] - b_qDot[i5];
    }

    distance_data[i] = 0.5 * (norm(b_uB) + 0.0 * b_norm(c_qDot)) + 0.5 * (real_T)
      ixstart;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r1);
  emxFree_real_T(&r2);
  ixstart = 1;
  x = distance_data[0];
  itmp = 0;
  if (muDoubleScalarIsNaN(distance_data[0])) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= 9)) {
      ixstart = i + 1;
      if (!muDoubleScalarIsNaN(distance_data[i])) {
        x = distance_data[i];
        itmp = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (ixstart < 9) {
    while (ixstart + 1 <= 9) {
      if (distance_data[ixstart] < x) {
        x = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i5 = 0; i5 < 13; i5++) {
    xNew_data[xNew_size[0] * i5] = candStates_data[itmp + 9 * i5];
  }

  ixstart = candTransArrays->size[1];
  i5 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ixstart;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i5, (int32_T)sizeof
                    (real_T), &k_emlrtRTEI);
  for (i5 = 0; i5 < ixstart; i5++) {
    transitionArray->data[transitionArray->size[0] * i5] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i5];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
