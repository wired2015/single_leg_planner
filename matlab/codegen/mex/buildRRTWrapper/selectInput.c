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
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo db_emlrtRSI = { 32, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo eb_emlrtRSI = { 38, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo fb_emlrtRSI = { 70, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 1, 35, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 12, 5, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo d_emlrtECI = { -1, 32, 10, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo e_emlrtECI = { -1, 32, 26, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo f_emlrtECI = { -1, 56, 9, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo db_emlrtBCI = { -1, -1, 56, 9, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

static emlrtRTEInfo k_emlrtRTEI = { 16, 5, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtDCInfo i_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  1 };

static emlrtDCInfo j_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  4 };

static emlrtDCInfo k_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  1 };

static emlrtDCInfo l_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  4 };

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 14, 5, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

/* Function Definitions */
void selectInput(const emlrtStack *sp, const real_T xNear_data[], const int32_T
                 xNear_size[2], const real_T xRand_data[], const int32_T
                 xRand_size[2], const real_T U[10], real_T dt, real_T Dt, real_T
                 kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l7,
                 real_T kC_zeta, const real_T jointLimits[12], real_T xNew_data[],
                 int32_T xNew_size[2], emxArray_real_T *transitionArray)
{
  boolean_T b2;
  real_T candStates_data[55];
  emxArray_real_T *candTransArrays;
  real_T scale;
  int32_T i5;
  real_T d1;
  int32_T loop_ub;
  real_T UJoint_data[15];
  int32_T i;
  emxArray_real_T *r1;
  emxArray_int32_T *r2;
  real_T distance_data[5];
  real_T u[3];
  real_T numIterations;
  int32_T tmp_size[2];
  real_T tmp_data[16];
  real_T b_xNear_data[11];
  real_T xInit_data[11];
  int32_T ixstart;
  real_T b_xInit_data[6];
  real_T k1[6];
  real_T k2[6];
  real_T k3[6];
  real_T absxk;
  real_T xInit[6];
  int32_T itmp;
  real_T b_tmp_data[16];
  int32_T iv6[2];
  int32_T iv7[2];
  emxArray_real_T b_xRand_data;
  real_T hDiff;
  real_T aGain;
  real_T v_LS[2];
  int32_T i6;
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
  memset(&candStates_data[0], 0, 55U * sizeof(real_T));
  emxInit_real_T(sp, &candTransArrays, 2, &f_emlrtRTEI, true);
  scale = muDoubleScalarRound(Dt / dt);
  i5 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  d1 = (scale + 1.0) * 6.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &j_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d1,
    &i_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i5, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  d1 = (scale + 1.0) * 6.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &j_emlrtDCI, sp);
  loop_ub = 5 * (int32_T)emlrtIntegerCheckFastR2012b(d1, &i_emlrtDCI, sp);
  for (i5 = 0; i5 < loop_ub; i5++) {
    candTransArrays->data[i5] = 0.0;
  }

  /* Transform the control inputs to joint space. */
  for (i = 0; i < 5; i++) {
    /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
    /* getConstrainedGammaDotDot Summary of this function goes here */
    /*    Detailed explanation goes here */
    /* [~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); */
    for (i5 = 0; i5 < 2; i5++) {
      UJoint_data[i + 5 * i5] = U[i + 5 * i5];
    }

    UJoint_data[i + 10] = ((-U[5 + i] * kC_l3 * muDoubleScalarCos(xNear_data[4])
      + xNear_data[7] * xNear_data[7] * kC_l3 * muDoubleScalarSin(xNear_data[4]))
      + xNear_data[8] * xNear_data[8] * kC_l5 * muDoubleScalarSin(kC_zeta +
      xNear_data[5])) / (kC_l5 * muDoubleScalarCos(kC_zeta + xNear_data[5]));
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  emxInit_real_T(sp, &r1, 2, &e_emlrtRTEI, true);
  emxInit_int32_T(sp, &r2, 1, &e_emlrtRTEI, true);
  for (i = 0; i < 5; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    st.site = &db_emlrtRSI;
    for (i5 = 0; i5 < 3; i5++) {
      u[i5] = UJoint_data[i + 5 * i5];
    }

    /* rk4.m */
    /* author: wreid */
    /* date: 20150107 */
    /* rk4 Summary of this function goes here */
    /*    Detailed explanation goes here */
    numIterations = muDoubleScalarRound(Dt / dt);
    tmp_size[1] = 11;
    memset(&tmp_data[0], 0, 11U * sizeof(real_T));
    for (i5 = 0; i5 < 11; i5++) {
      b_xNear_data[i5] = xNear_data[xNear_size[0] * i5];
    }

    for (i5 = 0; i5 < 6; i5++) {
      xInit_data[i5] = b_xNear_data[3 + i5];
    }

    /* xInitOrig = xInit; */
    i5 = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    d1 = (numIterations + 1.0) * 6.0;
    d1 = emlrtNonNegativeCheckFastR2012b(d1, &l_emlrtDCI, &st);
    r1->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d1, &k_emlrtDCI, &st);
    emxEnsureCapacity(&st, (emxArray__common *)r1, i5, (int32_T)sizeof(real_T),
                      &e_emlrtRTEI);
    d1 = (numIterations + 1.0) * 6.0;
    d1 = emlrtNonNegativeCheckFastR2012b(d1, &l_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d1, &k_emlrtDCI, &st);
    for (i5 = 0; i5 < loop_ub; i5++) {
      r1->data[i5] = 0.0;
    }

    ixstart = (int32_T)((numIterations + 1.0) * 6.0);
    for (i5 = 0; i5 < 6; i5++) {
      r1->data[emlrtDynamicBoundsCheckFastR2012b(i5 + 1, 1, ixstart,
        &eb_emlrtBCI, &st) - 1] = xInit_data[i5];
    }

    emlrtForLoopVectorCheckR2012b(1.0, 1.0, numIterations, mxDOUBLE_CLASS,
      (int32_T)numIterations, &k_emlrtRTEI, &st);
    ixstart = 0;
    while (ixstart <= (int32_T)numIterations - 1) {
      for (i5 = 0; i5 < 6; i5++) {
        b_xInit_data[i5] = xInit_data[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k1[i5] = b_xInit_data[3 + i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k1[i5 + 3] = u[i5];
      }

      scale = dt / 2.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_xInit_data[i5] = xInit_data[i5] + scale * k1[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k2[i5] = b_xInit_data[3 + i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k2[i5 + 3] = u[i5];
      }

      scale = dt / 2.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_xInit_data[i5] = xInit_data[i5] + scale * k2[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k3[i5] = b_xInit_data[3 + i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        k3[i5 + 3] = u[i5];
      }

      scale = dt / 2.0;
      absxk = dt / 6.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_xInit_data[i5] = xInit_data[i5] + scale * k3[i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        xInit[i5] = b_xInit_data[3 + i5];
      }

      for (i5 = 0; i5 < 3; i5++) {
        xInit[i5 + 3] = u[i5];
      }

      tmp_size[1] = 6;
      for (i5 = 0; i5 < 6; i5++) {
        tmp_data[i5] = xInit_data[i5] + absxk * (((k1[i5] + 2.0 * k2[i5]) + 2.0 *
          k3[i5]) + xInit[i5]);
      }

      /* Check pan angular position limits */
      if ((tmp_data[0] > jointLimits[1]) || (tmp_data[0] < jointLimits[0])) {
        tmp_data[0] = xInit_data[0];
        tmp_data[3] = 0.0;
        u[0] = 0.0;
      }

      /* Check inner and outer leg angular position limits */
      if ((tmp_data[1] > jointLimits[3]) || (tmp_data[1] < jointLimits[2]) ||
          (tmp_data[2] > jointLimits[5]) || (tmp_data[2] < jointLimits[4])) {
        tmp_data[1] = xInit_data[1];
        tmp_data[2] = xInit_data[2];
        tmp_data[4] = 0.0;
        tmp_data[5] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
      }

      /* Check pan angular velocity limits */
      if ((tmp_data[3] > jointLimits[7]) || (tmp_data[3] < jointLimits[6])) {
        tmp_data[3] = xInit_data[3];
        u[0] = 0.0;
      }

      /* Check inner and outer leg angular velocity limits */
      if ((tmp_data[4] > jointLimits[9]) || (tmp_data[4] < jointLimits[8]) ||
          (tmp_data[5] > jointLimits[11]) || (tmp_data[5] < jointLimits[10])) {
        tmp_data[4] = xInit_data[4];
        tmp_data[5] = xInit_data[5];
        u[1] = 0.0;
        u[2] = 0.0;
      }

      for (i5 = 0; i5 < 6; i5++) {
        xInit_data[i5] = tmp_data[i5];
      }

      d1 = 6.0 * (1.0 + (real_T)ixstart) + 1.0;
      scale = 6.0 * ((1.0 + (real_T)ixstart) + 1.0);
      if (d1 > scale) {
        i5 = 0;
        loop_ub = 0;
      } else {
        i5 = r1->size[1];
        loop_ub = (int32_T)d1;
        i5 = emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i5, &db_emlrtBCI, &st)
          - 1;
        loop_ub = r1->size[1];
        itmp = (int32_T)scale;
        loop_ub = emlrtDynamicBoundsCheckFastR2012b(itmp, 1, loop_ub,
          &db_emlrtBCI, &st);
      }

      loop_ub -= i5;
      emlrtSizeEqCheck1DFastR2012b(loop_ub, 6, &f_emlrtECI, &st);
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        r1->data[i5 + loop_ub] = tmp_data[loop_ub];
      }

      ixstart++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    /* xInit = [zeros(1,3) xInitOrig 0 0]; */
    ixstart = 5 + tmp_size[1];
    for (i5 = 0; i5 < 3; i5++) {
      b_tmp_data[i5] = 0.0;
    }

    loop_ub = tmp_size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_tmp_data[i5 + 3] = tmp_data[i5];
    }

    b_tmp_data[3 + tmp_size[1]] = 0.0;
    b_tmp_data[4 + tmp_size[1]] = 0.0;
    tmp_size[0] = 1;
    tmp_size[1] = ixstart;
    for (i5 = 0; i5 < ixstart; i5++) {
      tmp_data[i5] = b_tmp_data[i5];
    }

    if (!b2) {
      for (i5 = 0; i5 < 2; i5++) {
        iv6[i5] = 1 + 10 * i5;
      }

      b2 = true;
    }

    emlrtSubAssignSizeCheckR2012b(iv6, 2, tmp_size, 2, &d_emlrtECI, sp);
    for (i5 = 0; i5 < ixstart; i5++) {
      candStates_data[i + 5 * i5] = tmp_data[i5];
    }

    loop_ub = candTransArrays->size[1];
    i5 = r2->size[0];
    r2->size[0] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)r2, i5, (int32_T)sizeof(int32_T),
                      &e_emlrtRTEI);
    for (i5 = 0; i5 < loop_ub; i5++) {
      r2->data[i5] = i5;
    }

    iv7[0] = 1;
    iv7[1] = r2->size[0];
    emlrtSubAssignSizeCheckR2012b(iv7, 2, *(int32_T (*)[2])r1->size, 2,
      &e_emlrtECI, sp);
    loop_ub = r1->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      candTransArrays->data[i + candTransArrays->size[0] * r2->data[i5]] =
        r1->data[r1->size[0] * i5];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    for (i5 = 0; i5 < 11; i5++) {
      b_xNear_data[i5] = candStates_data[i + 5 * i5];
    }

    b_xRand_data.data = (real_T *)xRand_data;
    b_xRand_data.size = (int32_T *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    st.site = &eb_emlrtRSI;
    hDiff = heuristicSingleLeg(&st, b_xNear_data, &b_xRand_data, jointLimits,
      kC_l2, kC_l3, kC_l4, kC_l5, kC_l7, kC_zeta);

    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    aGain = 0.5;
    if (xNear_data[0] == 1.0) {
      aGain = 0.0;
    }

    /* Ss = [1000 1]; */
    /* qWDot = 1; */
    /* r = 0.378/2; */
    /* calcPhi.m */
    /* function [qS,qWDot] = calcPhi(qDot,q,kinematicConst,sS,qWDot,r) */
    /* v_XS = sin(conj(alpha))*conj(L6)*conj(alphaDot) - sin(conj(alpha))*conj(L2)*conj(alphaDot) - cos(conj(alpha))*conj(L7)*conj(gammaDot) - cos(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(beta))*sin(conj(alpha))*conj(L3)*conj(alphaDot) - cos(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - sin(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) - cos(conj(gamma))*sin(conj(alpha))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot) + sin(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot); */
    /* v_YS = cos(conj(alpha))*conj(L2)*conj(alphaDot) - sin(conj(alpha))*conj(L7)*conj(gammaDot) - sin(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(alpha))*conj(L6)*conj(alphaDot) + cos(conj(alpha))*cos(conj(beta))*conj(L3)*conj(alphaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - 1.0*sin(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot); */
    /* qA1 = atan(sS(2)/sS(1)); */
    /* v_WS = [qWDot*r*sign(sS(1))*cos(qA1); qWDot*r*sign(sS(1))*sin(qA1)]; */
    v_LS[0] = -kC_l3 * candStates_data[i + 35] * muDoubleScalarSin
      (candStates_data[i + 20]) - candStates_data[i + 40] * kC_l5 *
      muDoubleScalarSin(kC_zeta + candStates_data[i + 25]);
    v_LS[1] = candStates_data[i + 30] * (((kC_l2 + kC_l3 * muDoubleScalarCos
      (candStates_data[i + 20])) + kC_l4 * muDoubleScalarCos(kC_zeta)) + kC_l5 *
      muDoubleScalarCos(kC_zeta + candStates_data[i + 25]));

    /*  + v_WS;     */
    d1 = 0.0;
    scale = 2.2250738585072014E-308;
    for (ixstart = 0; ixstart < 2; ixstart++) {
      absxk = muDoubleScalarAbs(v_LS[ixstart]);
      if (absxk > scale) {
        numIterations = scale / absxk;
        d1 = 1.0 + d1 * numIterations * numIterations;
        scale = absxk;
      } else {
        numIterations = absxk / scale;
        d1 += numIterations * numIterations;
      }
    }

    d1 = scale * muDoubleScalarSqrt(d1);
    candStates_data[i + 45] = muDoubleScalarAtan(v_LS[0] / v_LS[1]);
    candStates_data[i + 50] = d1;

    /* ,Ss,qWDot,r); */
    /* angDiff Finds the angular difference between th1 and th2. */
    scale = ((xNear_data[10] - candStates_data[i + 45]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(scale - muDoubleScalarRound(scale)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(scale)) {
      scale = 0.0;
    } else {
      scale = (scale - muDoubleScalarFloor(scale)) * 6.2831853071795862;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    if (muDoubleScalarAbs(scale - 3.1415926535897931) > 0.39269908169872414) {
      i6 = 1;
    } else {
      i6 = 0;
    }

    distance_data[i] = (1.0 - aGain) * hDiff + aGain * (real_T)i6;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r2);
  emxFree_real_T(&r1);
  st.site = &fb_emlrtRSI;
  ixstart = 1;
  scale = distance_data[0];
  itmp = 0;
  if (muDoubleScalarIsNaN(distance_data[0])) {
    loop_ub = 1;
    exitg1 = false;
    while ((!exitg1) && (loop_ub + 1 <= 5)) {
      ixstart = loop_ub + 1;
      if (!muDoubleScalarIsNaN(distance_data[loop_ub])) {
        scale = distance_data[loop_ub];
        itmp = loop_ub;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < scale) {
        scale = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 11;
  for (i5 = 0; i5 < 11; i5++) {
    xNew_data[xNew_size[0] * i5] = candStates_data[itmp + 5 * i5];
  }

  loop_ub = candTransArrays->size[1];
  i5 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i5, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  for (i5 = 0; i5 < loop_ub; i5++) {
    transitionArray->data[transitionArray->size[0] * i5] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i5];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
