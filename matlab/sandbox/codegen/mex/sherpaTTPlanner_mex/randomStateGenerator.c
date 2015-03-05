/*
 * randomStateGenerator.c
 *
 * Code generation for function 'randomStateGenerator'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_mex_emxutil.h"
#include "randomState.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "eml_int_forloop_overflow_check.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo hc_emlrtRSI = { 46, "randomStateGenerator",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtRSInfo ic_emlrtRSI = { 47, "randomStateGenerator",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtRSInfo jc_emlrtRSI = { 3, "randomPoint",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomPoint.m"
};

static emlrtRSInfo kc_emlrtRSI = { 8, "randomPoint",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomPoint.m"
};

static emlrtRTEInfo p_emlrtRTEI = { 21, 19, "randomStateGenerator",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 47, 16, "states",
  "randomStateGenerator",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m",
  0 };

static emlrtDCInfo d_emlrtDCI = { 30, 20, "randomStateGenerator",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m",
  4 };

/* Function Definitions */
void randomStateGenerator(const emlrtStack *sp, int32_T N, const real_T
  jointLimits[20], const struct0_T *kC, real_T panHeight, int32_T legNum,
  emxArray_real_T *states)
{
  int64_T i18;
  int32_T i19;
  int32_T i;
  real_T u2_idx_2;
  real_T cartesianLimits_idx_0;
  real_T cartesianLimits_idx_2;
  real_T cartesianLimits_idx_3;
  boolean_T b6;
  int32_T b_i;
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv16[2] = { 1, 17 };

  const mxArray *m5;
  char_T cv14[17];
  static const char_T cv15[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T u1[3];
  real_T randomInitJoint_idx_3;
  real_T dv19[13];
  real_T uP[3];
  real_T TP2B[16];
  static const int8_T iv17[4] = { 0, 0, 0, 1 };

  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;

  /* RANDOMSTATEGENERATOR Generates N number of random states that are valid in */
  /* the Sherpa_TT leg's configuration space. */
  /*  */
  /* Inputs: */
  /* -N: The number of random states to be generated. */
  /* -jointLimits: A 2x10 array of joint angular limits and angular positions. */
  /* -kC: A structure containing the kinematic constants of the Sherpa TT leg. */
  /* -panHeight: The height of the pan coordinate frame relative to the the */
  /* wheel contact point. */
  /* -legNum: The leg identifier. */
  /*  */
  /* Outputs: */
  /* -states: A Nx6 array. Each row of the array describes a single state. A */
  /* state has the structure [x y z xDot yDot zDot]. The state coordinates are */
  /* realtive to the body frame. */
  /*  */
  /* randomStateGenerator.m */
  /* author: wreid */
  /* date: 20150305 */
  i18 = legNum + 1L;
  if (i18 > 2147483647L) {
    i18 = 2147483647L;
  } else {
    if (i18 < -2147483648L) {
      i18 = -2147483648L;
    }
  }

  legNum = (int32_T)i18;
  if (legNum > 4) {
    legNum = 4;
  } else {
    if (legNum < 1) {
      legNum = 1;
    }
  }

  i19 = states->size[0] * states->size[1];
  states->size[0] = (int32_T)emlrtNonNegativeCheckFastR2012b(N, &d_emlrtDCI, sp);
  states->size[1] = 6;
  emxEnsureCapacity(sp, (emxArray__common *)states, i19, (int32_T)sizeof(real_T),
                    &p_emlrtRTEI);
  i = (int32_T)emlrtNonNegativeCheckFastR2012b(N, &d_emlrtDCI, sp) * 6;
  for (i19 = 0; i19 < i; i19++) {
    states->data[i19] = 0.0;
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
  u2_idx_2 = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[3])) - kC->l4 *
                muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
               (jointLimits[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
  cartesianLimits_idx_0 = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[2]))
    - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
    (jointLimits[4] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
  cartesianLimits_idx_2 = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[2]))
    - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
    (jointLimits[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
  cartesianLimits_idx_3 = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[3]))
    - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
    (jointLimits[4] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
  st.site = &hc_emlrtRSI;
  if (1 > N) {
    b6 = false;
  } else {
    b6 = (N > 2147483646);
  }

  if (b6) {
    b_st.site = &mb_emlrtRSI;
    b_check_forloop_overflow_error(&b_st);
  }

  b_i = 1;
  while (b_i <= N) {
    st.site = &ic_emlrtRSI;
    b_st.site = &jc_emlrtRSI;

    /* randomState.m */
    /* author: wreid */
    /* date: 20150107 */
    /* randomState Picks a random state from the state space. */
    /*    A random state is selected from the state space within the boundaries of */
    /*    the state space as defined by the MIN and MAX vectors. The state space has */
    /*    a dimension n. */
    /*    Inputs: */
    /*        MIN:    The 1xn vector containing the minimum boundaries for the state */
    /*                space. */
    /*        MAX:    The 1xn vector containing the maximum boundaries for the state */
    /*                space. */
    /*    Outputs: */
    /*        xRand:  The 1xn vector describing the selected random state. */
    /* [~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); */
    c_st.site = &i_emlrtRSI;
    emlrtRandu(&r, 1);
    if ((panHeight <= cartesianLimits_idx_0) && (panHeight >=
         cartesianLimits_idx_2)) {
      c_st.site = &j_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &k_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < cartesianLimits_idx_2) && (panHeight >=
                cartesianLimits_idx_3)) {
      c_st.site = &l_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &m_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < cartesianLimits_idx_3) && (panHeight >= u2_idx_2)) {
      c_st.site = &n_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[3], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &o_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else {
      xMax = 0.0;
      xMin = 0.0;
      y = NULL;
      m5 = emlrtCreateCharArray(2, iv16);
      for (i = 0; i < 17; i++) {
        cv14[i] = cv15[i];
      }

      emlrtInitCharArrayR2013a(&b_st, 17, m5, cv14);
      emlrtAssign(&y, m5);
      c_st.site = &mc_emlrtRSI;
      disp(&c_st, y, &b_emlrtMCI);
    }

    c_st.site = &p_emlrtRSI;
    emlrtRandu(&b_r, 1);
    b_xMin[0] = xMin + (xMax - xMin) * b_r;
    b_xMin[1] = 0.0;
    b_xMin[2] = panHeight;
    c_st.site = &q_emlrtRSI;
    b_sherpaTTIK(&c_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, u1);
    c_st.site = &r_emlrtRSI;
    emlrtRandu(&b_r, 1);
    c_st.site = &s_emlrtRSI;
    emlrtRandu(&xMax, 1);
    c_st.site = &t_emlrtRSI;
    emlrtRandu(&xMin, 1);

    /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
    randomInitJoint_idx_3 = jointLimits[0] + (jointLimits[1] - jointLimits[0]) *
      r;

    /* if mod(nodeIDCount,goalSeedFreq) == 0 */
    /*     xRand = nGoal; */
    /* end */
    for (i19 = 0; i19 < 3; i19++) {
      dv19[i19] = 0.0;
    }

    dv19[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
    dv19[4] = u1[1];
    dv19[5] = u1[2];
    dv19[6] = 0.0;
    dv19[7] = 0.0;
    dv19[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
    dv19[9] = (jointLimits[11] - jointLimits[10]) * xMax + jointLimits[10];
    dv19[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
    dv19[11] = 0.0;
    dv19[12] = 0.0;
    for (i19 = 0; i19 < 3; i19++) {
      u1[i19] = dv19[3 + i19];
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
    uP[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-u1[1])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(u1[2] +
               kC->zeta)) - kC->l7) * muDoubleScalarCos(randomInitJoint_idx_3);
    uP[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-u1[1])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(u1[2] +
               kC->zeta)) - kC->l7) * muDoubleScalarSin(randomInitJoint_idx_3);
    uP[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-u1[1])) - kC->l4 *
               muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin(u1[2] +
               kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
    b_st.site = &kc_emlrtRSI;

    /* TRP2B Generates the homogeneous transformation matrix between the body */
    /* and pan coordinate frames. kC is a struct containing the kinematic */
    /* constants of the leg. legNum indicates the number of the leg that is being */
    /* considered. */
    /*  */
    /* Inputs: */
    /* -kC: Struct of kinematic constants of the Sherpa_TT leg */
    /* -legNum: The leg number identification. */
    /* Outputs: */
    /* -TP2B: The homogeneous transformation matrix that is used to transform */
    /* coordinates from the pan frame to the body frame. */
    /*  */
    /* trP2B.m */
    /* author:    wreid */
    /* date:      20150214 */
    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TP2B[0] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2B[4] = -muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2B[8] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]) * 0.0;
    TP2B[12] = kC->B2PXOffset * muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2B[1] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2B[5] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2B[9] = -muDoubleScalarCos(kC->legAngleOffset[legNum - 1]) * 0.0;
    TP2B[13] = kC->B2PXOffset * muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2B[2] = 0.0;
    TP2B[6] = 0.0;
    TP2B[10] = 1.0;
    TP2B[14] = kC->B2PZOffset;
    for (i19 = 0; i19 < 4; i19++) {
      TP2B[3 + (i19 << 2)] = iv17[i19];
    }

    i = states->size[0];
    emlrtDynamicBoundsCheckFastR2012b(b_i, 1, i, &eb_emlrtBCI, sp);
    for (i19 = 0; i19 < 3; i19++) {
      xMax = 0.0;
      for (i = 0; i < 3; i++) {
        xMax += TP2B[i19 + (i << 2)] * uP[i];
      }

      u1[i19] = xMax + TP2B[12 + i19];
    }

    for (i19 = 0; i19 < 3; i19++) {
      states->data[(b_i + states->size[0] * i19) - 1] = u1[i19];
    }

    states->data[(b_i + states->size[0] * 3) - 1] = 0.0;
    states->data[(b_i + (states->size[0] << 2)) - 1] = 0.0;
    states->data[(b_i + states->size[0] * 5) - 1] = 0.0;
    b_i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* save('experimentStates','states') */
}

/* End of code generation (randomStateGenerator.c) */
