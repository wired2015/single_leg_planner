/*
 * randomStateGenerator.c
 *
 * Code generation for function 'randomStateGenerator'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "randomStateGenerator.h"
#include "randomStateGenerator_emxutil.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "eml_int_forloop_overflow_check.h"
#include "randomStateGenerator_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 19, "randomStateGenerator",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtRSInfo b_emlrtRSI = { 20, "randomStateGenerator",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtRSInfo c_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtRSInfo d_emlrtRSI = { 3, "randomPoint",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomPoint.m"
};

static emlrtRSInfo e_emlrtRSI = { 8, "randomPoint",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomPoint.m"
};

static emlrtRSInfo f_emlrtRSI = { 70, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo g_emlrtRSI = { 69, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo h_emlrtRSI = { 68, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo i_emlrtRSI = { 62, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo j_emlrtRSI = { 59, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo k_emlrtRSI = { 52, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo l_emlrtRSI = { 51, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo m_emlrtRSI = { 49, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo n_emlrtRSI = { 48, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 46, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 45, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 37, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtMCInfo c_emlrtMCI = { 56, 9, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRTEInfo emlrtRTEI = { 1, 19, "randomStateGenerator",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m"
};

static emlrtBCInfo emlrtBCI = { -1, -1, 20, 16, "states", "randomStateGenerator",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { 1, 4, 18, 17, "kC.legAngleOffset", "trP2B",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/trP2B.m",
  0 };

static emlrtDCInfo emlrtDCI = { 3, 20, "randomStateGenerator",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation/randomStateGenerator.m",
  4 };

static emlrtRSInfo w_emlrtRSI = { 56, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

/* Function Declarations */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

void randomStateGenerator(const emlrtStack *sp, int32_T NUM_POINTS, const real_T
  jointLimits[20], const struct0_T *kC, real_T panHeight, int32_T legNum,
  emxArray_real_T *states)
{
  int32_T i0;
  int32_T i;
  real_T u2_idx_2;
  real_T cartesianLimits_idx_0;
  real_T cartesianLimits_idx_2;
  real_T cartesianLimits_idx_3;
  boolean_T b0;
  int32_T b_i;
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv0[2] = { 1, 17 };

  const mxArray *m0;
  char_T cv0[17];
  static const char_T cv1[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T u1[3];
  real_T randomInitJoint_idx_3;
  real_T dv0[13];
  real_T uP[3];
  real_T TP2B[16];
  static const int8_T iv1[4] = { 0, 0, 0, 1 };

  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  i0 = states->size[0] * states->size[1];
  states->size[0] = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_POINTS,
    &emlrtDCI, sp);
  states->size[1] = 6;
  emxEnsureCapacity(sp, (emxArray__common *)states, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  i = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_POINTS, &emlrtDCI, sp) * 6;
  for (i0 = 0; i0 < i; i0++) {
    states->data[i0] = 0.0;
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
  st.site = &emlrtRSI;
  if (1 > NUM_POINTS) {
    b0 = false;
  } else {
    b0 = (NUM_POINTS > 2147483646);
  }

  if (b0) {
    b_st.site = &c_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  b_i = 1;
  while (b_i <= NUM_POINTS) {
    st.site = &b_emlrtRSI;
    b_st.site = &d_emlrtRSI;

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
    c_st.site = &q_emlrtRSI;
    emlrtRandu(&r, 1);
    if ((panHeight <= cartesianLimits_idx_0) && (panHeight >=
         cartesianLimits_idx_2)) {
      c_st.site = &p_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &o_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < cartesianLimits_idx_2) && (panHeight >=
                cartesianLimits_idx_3)) {
      c_st.site = &n_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &m_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < cartesianLimits_idx_3) && (panHeight >= u2_idx_2)) {
      c_st.site = &l_emlrtRSI;
      xMax = getXStar(&c_st, panHeight, jointLimits[3], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
      c_st.site = &k_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else {
      xMax = 0.0;
      xMin = 0.0;
      y = NULL;
      m0 = emlrtCreateCharArray(2, iv0);
      for (i = 0; i < 17; i++) {
        cv0[i] = cv1[i];
      }

      emlrtInitCharArrayR2013a(&b_st, 17, m0, cv0);
      emlrtAssign(&y, m0);
      c_st.site = &w_emlrtRSI;
      disp(&c_st, y, &c_emlrtMCI);
    }

    c_st.site = &j_emlrtRSI;
    emlrtRandu(&b_r, 1);
    b_xMin[0] = xMin + (xMax - xMin) * b_r;
    b_xMin[1] = 0.0;
    b_xMin[2] = panHeight;
    c_st.site = &i_emlrtRSI;
    sherpaTTIK(&c_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
               kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, u1);
    c_st.site = &h_emlrtRSI;
    emlrtRandu(&b_r, 1);
    c_st.site = &g_emlrtRSI;
    emlrtRandu(&xMax, 1);
    c_st.site = &f_emlrtRSI;
    emlrtRandu(&xMin, 1);

    /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
    randomInitJoint_idx_3 = jointLimits[0] + (jointLimits[1] - jointLimits[0]) *
      r;
    for (i0 = 0; i0 < 3; i0++) {
      dv0[i0] = 0.0;
    }

    dv0[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
    dv0[4] = u1[1];
    dv0[5] = u1[2];
    dv0[6] = 0.0;
    dv0[7] = 0.0;
    dv0[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
    dv0[9] = (jointLimits[11] - jointLimits[10]) * xMax + jointLimits[10];
    dv0[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
    dv0[11] = 0.0;
    dv0[12] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      u1[i0] = dv0[3 + i0];
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
    b_st.site = &e_emlrtRSI;

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
    emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &b_emlrtBCI, &b_st);

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
    for (i0 = 0; i0 < 4; i0++) {
      TP2B[3 + (i0 << 2)] = iv1[i0];
    }

    i = states->size[0];
    emlrtDynamicBoundsCheckFastR2012b(b_i, 1, i, &emlrtBCI, sp);
    for (i0 = 0; i0 < 3; i0++) {
      xMax = 0.0;
      for (i = 0; i < 3; i++) {
        xMax += TP2B[i0 + (i << 2)] * uP[i];
      }

      u1[i0] = xMax + TP2B[12 + i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      states->data[(b_i + states->size[0] * i0) - 1] = u1[i0];
    }

    states->data[(b_i + states->size[0] * 3) - 1] = 0.0;
    states->data[(b_i + (states->size[0] << 2)) - 1] = 0.0;
    states->data[(b_i + states->size[0] * 5) - 1] = 0.0;
    b_i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }
}

/* End of code generation (randomStateGenerator.c) */
