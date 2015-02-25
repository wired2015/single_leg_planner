/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "selectInput.h"
#include "buildRRTWrapper_emxutil.h"
#include "eml_error.h"
#include "norm.h"
#include "rk4.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo rb_emlrtRSI = { 30, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo sb_emlrtRSI = { 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo tb_emlrtRSI = { 60, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo m_emlrtRTEI = { 1, 35, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 12, 5, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo d_emlrtECI = { -1, 30, 10, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo e_emlrtECI = { -1, 30, 26, "selectInput",
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
                 xRand[13], const real_T U[10], real_T dt, real_T Dt, const
                 struct0_T *kC, const real_T jointLimits[20], const real_T
                 uBDot[6], int32_T legNum, real_T xNew_data[], int32_T
                 xNew_size[2], emxArray_real_T *transitionArray)
{
  boolean_T b2;
  emxArray_real_T *candTransArrays;
  real_T dAlphaMax;
  int32_T i6;
  real_T xStarMin;
  int32_T ixstart;
  emxArray_int32_T *r1;
  real_T candStates_data[65];
  real_T distance_data[5];
  emxArray_real_T *r2;
  int32_T i;
  real_T b_U[2];
  real_T b_xNear_data[13];
  int32_T tmp_size[2];
  real_T tmp_data[16];
  int32_T iv6[2];
  int32_T iv7[2];
  real_T dxStarMax;
  real_T r;
  real_T uA[3];
  real_T uB[3];
  real_T b_uB[3];
  int32_T itmp;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
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
  emxInit_real_T(sp, &candTransArrays, 2, &n_emlrtRTEI, true);
  dAlphaMax = muDoubleScalarRound(Dt / dt);
  i6 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  xStarMin = (dAlphaMax + 1.0) * 10.0;
  xStarMin = emlrtNonNegativeCheckFastR2012b(xStarMin, &f_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(xStarMin,
    &e_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i6, (int32_T)sizeof
                    (real_T), &m_emlrtRTEI);
  xStarMin = (dAlphaMax + 1.0) * 10.0;
  xStarMin = emlrtNonNegativeCheckFastR2012b(xStarMin, &f_emlrtDCI, sp);
  ixstart = 5 * (int32_T)emlrtIntegerCheckFastR2012b(xStarMin, &e_emlrtDCI, sp);
  for (i6 = 0; i6 < ixstart; i6++) {
    candTransArrays->data[i6] = 0.0;
  }

  emxInit_int32_T(sp, &r1, 1, &m_emlrtRTEI, true);

  /* UJoint = zeros(U_SIZE,3); */
  /* Transform the control inputs to joint space. */
  /* for i = 1:U_SIZE */
  /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
  /* gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6)); */
  /* UJoint(i,:) = [U(i,:) gammaDotDot]; */
  /* end     */
  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  emxInit_real_T(sp, &r2, 2, &m_emlrtRTEI, true);
  for (i = 0; i < 5; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    for (i6 = 0; i6 < 2; i6++) {
      b_U[i6] = U[i + 5 * i6];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 13U * sizeof(real_T));
    st.site = &rb_emlrtRSI;
    rk4(&st, b_U, uBDot, dt, Dt, b_xNear_data, jointLimits, kC, legNum, tmp_data,
        tmp_size, r2);
    if (!b2) {
      for (i6 = 0; i6 < 2; i6++) {
        iv6[i6] = 1 + 12 * i6;
      }

      b2 = true;
    }

    emlrtSubAssignSizeCheckR2012b(iv6, 2, tmp_size, 2, &d_emlrtECI, sp);
    ixstart = tmp_size[1];
    for (i6 = 0; i6 < ixstart; i6++) {
      candStates_data[i + 5 * i6] = tmp_data[tmp_size[0] * i6];
    }

    ixstart = candTransArrays->size[1];
    i6 = r1->size[0];
    r1->size[0] = ixstart;
    emxEnsureCapacity(sp, (emxArray__common *)r1, i6, (int32_T)sizeof(int32_T),
                      &m_emlrtRTEI);
    for (i6 = 0; i6 < ixstart; i6++) {
      r1->data[i6] = i6;
    }

    iv7[0] = 1;
    iv7[1] = r1->size[0];
    emlrtSubAssignSizeCheckR2012b(iv7, 2, *(int32_T (*)[2])r2->size, 2,
      &e_emlrtECI, sp);
    ixstart = r2->size[1];
    for (i6 = 0; i6 < ixstart; i6++) {
      candTransArrays->data[i + candTransArrays->size[0] * r1->data[i6]] =
        r2->data[r2->size[0] * i6];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    st.site = &sb_emlrtRSI;

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    /* Calculate the distance between angular positions. */
    xStarMin = (((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[2])) + kC->l4 *
                 muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                (kC->zeta + jointLimits[4])) - kC->l7;
    dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[3])) +
                   kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
                  muDoubleScalarCos(kC->zeta + jointLimits[5])) - kC->l7) -
      xStarMin;

    /* angDiff Finds the angular difference between th1 and th2. */
    r = ((jointLimits[0] - jointLimits[2]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(r)) {
      r = 0.0;
    } else {
      r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
    }

    dAlphaMax = muDoubleScalarAbs(r - 3.1415926535897931);
    b_st.site = &ib_emlrtRSI;
    c_st.site = &kb_emlrtRSI;
    if (dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax * dAlphaMax) <
        0.0) {
      d_st.site = &i_emlrtRSI;
      eml_error(&d_st);
    }

    dAlphaMax = (((kC->l2 + kC->l3 * muDoubleScalarCos(candStates_data[i + 20]))
                  + kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
                 muDoubleScalarCos(kC->zeta + candStates_data[i + 25])) - kC->l7;
    dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(xRand[4])) + kC->l4 *
                   muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                  (kC->zeta + xRand[5])) - kC->l7) - dAlphaMax;

    /* angDiff Finds the angular difference between th1 and th2. */
    r = ((candStates_data[i + 15] - xRand[3]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(r)) {
      r = 0.0;
    } else {
      r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
    }

    xStarMin = muDoubleScalarAbs(r - 3.1415926535897931);
    b_st.site = &jb_emlrtRSI;
    if (dxStarMax * dxStarMax + dAlphaMax * dAlphaMax * (xStarMin * xStarMin) <
        0.0) {
      c_st.site = &i_emlrtRSI;
      eml_error(&c_st);
    }

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
    uA[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-candStates_data[i + 20])) +
               kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
              muDoubleScalarCos(candStates_data[i + 25] + kC->zeta)) - kC->l7) *
      muDoubleScalarCos(candStates_data[i + 15]);
    uA[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-candStates_data[i + 20])) +
               kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
              muDoubleScalarCos(candStates_data[i + 25] + kC->zeta)) - kC->l7) *
      muDoubleScalarSin(candStates_data[i + 15]);
    uA[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-candStates_data[i + 20])) -
               kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 *
              muDoubleScalarSin(candStates_data[i + 25] + kC->zeta)) - kC->l6) -
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
    r = ((xNear_data[3] - candStates_data[i + 15]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(r)) {
      r = 0.0;
    } else {
      r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
    }

    if (muDoubleScalarAbs(r - 3.1415926535897931) > 0.39269908169872414) {
      ixstart = 1;
    } else {
      /* aDiff = abs(aDiff/ankleDiffMax); */
      ixstart = 0;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    for (i6 = 0; i6 < 3; i6++) {
      b_uB[i6] = uB[i6] - uA[i6];
    }

    distance_data[i] = 0.5 * norm(b_uB) + 0.5 * (real_T)ixstart;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r1);
  emxFree_real_T(&r2);
  st.site = &tb_emlrtRSI;
  b_st.site = &lb_emlrtRSI;
  c_st.site = &mb_emlrtRSI;
  ixstart = 1;
  dAlphaMax = distance_data[0];
  itmp = 0;
  if (muDoubleScalarIsNaN(distance_data[0])) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= 5)) {
      ixstart = i + 1;
      if (!muDoubleScalarIsNaN(distance_data[i])) {
        dAlphaMax = distance_data[i];
        itmp = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < dAlphaMax) {
        dAlphaMax = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i6 = 0; i6 < 13; i6++) {
    xNew_data[xNew_size[0] * i6] = candStates_data[itmp + 5 * i6];
  }

  ixstart = candTransArrays->size[1];
  i6 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ixstart;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i6, (int32_T)sizeof
                    (real_T), &m_emlrtRTEI);
  for (i6 = 0; i6 < ixstart; i6++) {
    transitionArray->data[transitionArray->size[0] * i6] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i6];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
