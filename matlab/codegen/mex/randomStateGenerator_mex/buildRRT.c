/*
 * buildRRT.c
 *
 * Code generation for function 'buildRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "buildRRT.h"
#include "randomStateGenerator_mex_emxutil.h"
#include "eml_error.h"
#include "norm.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "eml_int_forloop_overflow_check.h"
#include "randomStateGenerator_mex_mexutil.h"
#include "randomStateGenerator_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = { 66, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo i_emlrtRSI = { 43, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 26, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 73, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo n_emlrtRSI = { 74, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo o_emlrtRSI = { 75, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo p_emlrtRSI = { 80, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo wb_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRSInfo xb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo b_emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo d_emlrtRTEI = { 46, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 48, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 53, 9, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 66, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 5, 13, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 71, 28, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtECInfo emlrtECI = { 1, 67, 12, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo h_emlrtBCI = { -1, -1, 62, 27, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 60, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 59, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo emlrtDCI = { 59, 18, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 59, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtRTEInfo r_emlrtRTEI = { 54, 9, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo l_emlrtBCI = { -1, -1, 51, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { -1, 19, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo m_emlrtBCI = { -1, -1, 19, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo b_emlrtDCI = { 17, 15, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo c_emlrtDCI = { 17, 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo d_emlrtDCI = { 19, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtDCInfo e_emlrtDCI = { 19, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtBCInfo n_emlrtBCI = { -1, -1, 55, 47, "transitionArray", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo f_emlrtECI = { -1, 82, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo kb_emlrtBCI = { -1, -1, 82, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtRSInfo jc_emlrtRSI = { 104, "mod",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/mod.m" };

/* Function Declarations */
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator);
static void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointLimits[20], const struct0_T *kC, real_T panHeight,
                    const real_T U[10], real_T Dt, real_T dt, const real_T
                    HGAINS[3], real_T *nodeIDCount, const real_T nGoal[13],
                    const real_T uBDot[6], int32_T legNum);

/* Function Definitions */
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator)
{
  int32_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  boolean_T quotientNeedsNegation;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }

    emlrtDivisionByZeroErrorR2012b(NULL, sp);
  } else {
    if (numerator >= 0) {
      absNumerator = (uint32_T)numerator;
    } else {
      absNumerator = (uint32_T)-numerator;
    }

    if (denominator >= 0) {
      absDenominator = (uint32_T)denominator;
    } else {
      absDenominator = (uint32_T)-denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
    }

    if (quotientNeedsNegation) {
      quotient = -(int32_T)tempAbsQuotient;
    } else {
      quotient = (int32_T)tempAbsQuotient;
    }
  }

  return quotient;
}

static void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointLimits[20], const struct0_T *kC, real_T panHeight,
                    const real_T U[10], real_T Dt, real_T dt, const real_T
                    HGAINS[3], real_T *nodeIDCount, const real_T nGoal[13],
                    const real_T uBDot[6], int32_T legNum)
{
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv26[2] = { 1, 17 };

  const mxArray *m11;
  char_T cv12[17];
  int32_T i;
  static const char_T cv13[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  real_T dxStarMax;
  int32_T i16;
  real_T xRand[13];
  int32_T b_i;
  emxArray_real_T *unusedU4;
  real_T unusedU5;
  int32_T xNear_size[2];
  real_T xNear_data[13];
  int32_T xNew_size[2];
  real_T xNew_data[13];
  real_T uA[3];
  real_T uB[3];
  emxArray_int32_T *r3;
  emxArray_real_T *r4;
  int32_T iv27[2];
  int32_T b_nodeIDCount;
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
  st.site = &m_emlrtRSI;

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
  b_st.site = &q_emlrtRSI;
  emlrtRandu(&r, 1);
  if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
    b_st.site = &r_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &s_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
    b_st.site = &t_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &u_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
    b_st.site = &v_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[3], true, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &w_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else {
    xMax = 0.0;
    xMin = 0.0;
    y = NULL;
    m11 = emlrtCreateCharArray(2, iv26);
    for (i = 0; i < 17; i++) {
      cv12[i] = cv13[i];
    }

    emlrtInitCharArrayR2013a(&st, 17, m11, cv12);
    emlrtAssign(&y, m11);
    b_st.site = &ec_emlrtRSI;
    disp(&b_st, y, &c_emlrtMCI);
  }

  b_st.site = &x_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_xMin[0] = xMin + (xMax - xMin) * b_r;
  b_xMin[1] = 0.0;
  b_xMin[2] = panHeight;
  b_st.site = &y_emlrtRSI;
  b_sherpaTTIK(&b_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
               kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, q);
  b_st.site = &ab_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_st.site = &bb_emlrtRSI;
  emlrtRandu(&dxStarMax, 1);
  b_st.site = &cb_emlrtRSI;
  emlrtRandu(&xMax, 1);

  /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
  for (i16 = 0; i16 < 3; i16++) {
    xRand[i16] = 0.0;
  }

  xRand[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
  xRand[4] = q[1];
  xRand[5] = q[2];
  xRand[6] = 0.0;
  xRand[7] = 0.0;
  xRand[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
  xRand[9] = (jointLimits[11] - jointLimits[10]) * dxStarMax + jointLimits[10];
  xRand[10] = (jointLimits[13] - jointLimits[12]) * xMax + jointLimits[12];
  xRand[11] = 0.0;
  xRand[12] = 0.0;
  xMax = muDoubleScalarRound(*nodeIDCount);
  if (xMax < 2.147483648E+9) {
    if (xMax >= -2.147483648E+9) {
      i = (int32_T)xMax;
    } else {
      i = MIN_int32_T;
    }
  } else if (xMax >= 2.147483648E+9) {
    i = MAX_int32_T;
  } else {
    i = 0;
  }

  xMax = muDoubleScalarRound(*nodeIDCount - muDoubleScalarFloor(*nodeIDCount /
    20.0) * 20.0);
  if (xMax < 2.147483648E+9) {
    if (xMax >= -2.147483648E+9) {
      i16 = (int32_T)xMax;
    } else {
      i16 = MIN_int32_T;
    }
  } else if (xMax >= 2.147483648E+9) {
    i16 = MAX_int32_T;
  } else {
    i16 = 0;
  }

  if (i == *nodeIDCount) {
    b_st.site = &jc_emlrtRSI;
    b_i = i - div_s32_floor(&b_st, i, 20) * 20;
  } else {
    b_i = i16;
  }

  if (b_i == 0) {
    memcpy(&xRand[0], &nGoal[0], 13U * sizeof(real_T));
  }

  emxInit_real_T(&st, &unusedU4, 2, &i_emlrtRTEI, true);
  st.site = &n_emlrtRSI;
  nearestNeighbour(&st, xRand, T, jointLimits, kC->l1, kC->l2, kC->l3, kC->l4,
                   kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, *nodeIDCount,
                   13, xNear_data, xNear_size, unusedU4, &unusedU5);
  st.site = &o_emlrtRSI;
  selectInput(&st, xNear_data, xRand, U, dt, Dt, HGAINS, kC, jointLimits, uBDot,
              legNum, xNew_data, xNew_size, unusedU4);
  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  /* Node ID */
  xNew_data[1] = xNear_data[0];

  /* Parent ID */
  st.site = &p_emlrtRSI;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  /* Calculate the distance between angular positions. */
  xMin = (((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[2])) + kC->l4 *
           muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(kC->zeta +
           jointLimits[4])) - kC->l7;
  dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[3])) + kC->l4 *
                 muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                (kC->zeta + jointLimits[5])) - kC->l7) - xMin;

  /* angDiff Finds the angular difference between th1 and th2. */
  r = ((jointLimits[0] - jointLimits[2]) + 3.1415926535897931) /
    6.2831853071795862;
  if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
      muDoubleScalarAbs(r)) {
    r = 0.0;
  } else {
    r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
  }

  xMax = muDoubleScalarAbs(r - 3.1415926535897931);
  b_st.site = &ib_emlrtRSI;
  c_st.site = &kb_emlrtRSI;
  if (dxStarMax * dxStarMax + xMin * xMin * (xMax * xMax) < 0.0) {
    d_st.site = &g_emlrtRSI;
    eml_error(&d_st);
  }

  xMin = (((kC->l2 + kC->l3 * muDoubleScalarCos(xNew_data[4])) + kC->l4 *
           muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos(kC->zeta +
           xNew_data[5])) - kC->l7;
  dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(xNear_data[4])) + kC->l4 *
                 muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                (kC->zeta + xNear_data[5])) - kC->l7) - xMin;

  /* angDiff Finds the angular difference between th1 and th2. */
  r = ((xNew_data[3] - xNear_data[3]) + 3.1415926535897931) / 6.2831853071795862;
  if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
      muDoubleScalarAbs(r)) {
    r = 0.0;
  } else {
    r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
  }

  xMax = muDoubleScalarAbs(r - 3.1415926535897931);
  b_st.site = &jb_emlrtRSI;
  if (dxStarMax * dxStarMax + xMin * xMin * (xMax * xMax) < 0.0) {
    c_st.site = &g_emlrtRSI;
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
  uA[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNew_data[4])) + kC->l4 *
             muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
            (xNew_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos(xNew_data[3]);
  uA[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNew_data[4])) + kC->l4 *
             muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
            (xNew_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin(xNew_data[3]);
  uA[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-xNew_data[4])) - kC->l4 *
             muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
            (xNew_data[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
  uB[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNear_data[4])) + kC->l4 *
             muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
            (xNear_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos
    (xNear_data[3]);
  uB[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNear_data[4])) + kC->l4 *
             muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
            (xNear_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin
    (xNear_data[3]);
  uB[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-xNear_data[4])) - kC->l4 *
             muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
            (xNear_data[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

  /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
  /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
  /*     uA = sherpaTTFK(xA(4:6),kC); */
  /*     uB = sherpaTTFK(xB(4:6),kC); */
  /* dPos = norm(uA-uB); */
  /* Calculate the total distance. */
  /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
  for (i16 = 0; i16 < 3; i16++) {
    b_xMin[i16] = uB[i16] - uA[i16];
  }

  emxInit_int32_T(sp, &r3, 1, &i_emlrtRTEI, true);
  xNew_data[2] = xNear_data[2] + norm(b_xMin);

  /* Cost */
  i16 = T->size[0];
  i = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(i, 1, i16, &kb_emlrtBCI, sp);
  i = T->size[1];
  i16 = r3->size[0];
  r3->size[0] = i;
  emxEnsureCapacity(sp, (emxArray__common *)r3, i16, (int32_T)sizeof(int32_T),
                    &i_emlrtRTEI);
  for (i16 = 0; i16 < i; i16++) {
    r3->data[i16] = i16;
  }

  emxInit_real_T(sp, &r4, 2, &i_emlrtRTEI, true);
  i16 = r4->size[0] * r4->size[1];
  r4->size[0] = 1;
  r4->size[1] = 13 + unusedU4->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)r4, i16, (int32_T)sizeof(real_T),
                    &i_emlrtRTEI);
  for (i16 = 0; i16 < 13; i16++) {
    r4->data[r4->size[0] * i16] = xNew_data[xNew_size[0] * i16];
  }

  i = unusedU4->size[1];
  for (i16 = 0; i16 < i; i16++) {
    r4->data[r4->size[0] * (i16 + 13)] = unusedU4->data[unusedU4->size[0] * i16];
  }

  emxFree_real_T(&unusedU4);
  iv27[0] = 1;
  iv27[1] = r3->size[0];
  emlrtSubAssignSizeCheckR2012b(iv27, 2, *(int32_T (*)[2])r4->size, 2,
    &f_emlrtECI, sp);
  b_nodeIDCount = (int32_T)*nodeIDCount;
  i = r4->size[1];
  for (i16 = 0; i16 < i; i16++) {
    T->data[(b_nodeIDCount + T->size[0] * r3->data[i16]) - 1] = r4->data
      [r4->size[0] * i16];
  }

  emxFree_real_T(&r4);
  emxFree_int32_T(&r3);

  /* Append the new node to the tree.     */
  /* if mod(nodeIDCount,100) == 0 */
  /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
  /* end */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRT(const emlrtStack *sp, const real_T nInit[13], const real_T nGoal
              [13], int32_T NUM_NODES, const real_T jointLimits[20], real_T
              panHeight, const real_T HGAINS[3], const real_T U[10], real_T dt,
              real_T Dt, const struct0_T *kC, const real_T uBDot[6], int32_T
              legNum, emxArray_real_T *T, emxArray_real_T *path)
{
  real_T transitionArrayLength;
  int32_T i3;
  real_T d1;
  int32_T cdiff;
  int32_T absb;
  emxArray_int32_T *r0;
  emxArray_real_T *next;
  int32_T iv2[2];
  boolean_T b0;
  int32_T apnd;
  emxArray_real_T *transitionArray;
  real_T unusedU2;
  int32_T xNearest_size[2];
  real_T xNearest_data[13];
  emxArray_real_T *b_path;
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  boolean_T exitg1;
  uint32_T i;
  int32_T i4;
  int32_T ndbl;
  emxArray_real_T *t;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* buildRRT.m */
  /* author: wreid */
  /* date: 20150107 */
  /* buildRRT Icrementally builds a rapidly exploring random tree. */
  /*    An RRT is build by incrementally selecting a random state from the */
  /*    available state space as defined by the MIN and MAX vectors. The tree is */
  /*    started at xInit and is extended until the number of maximum nodes, K has */
  /*    been reached. A path is selected if the goal region as defined by xGoal */
  /*    has been reached by the RRT. */
  /* Constant Declaration                                                       */
  transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 10.0;

  /* Variable Initialization */
  i3 = T->size[0] * T->size[1];
  T->size[0] = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_NODES, &b_emlrtDCI,
    sp);
  d1 = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (d1 < 2.147483648E+9) {
    if (d1 >= -2.147483648E+9) {
      cdiff = (int32_T)d1;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (d1 >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(cdiff, &c_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  d1 = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (d1 < 2.147483648E+9) {
    if (d1 >= -2.147483648E+9) {
      i3 = (int32_T)d1;
    } else {
      i3 = MIN_int32_T;
    }
  } else if (d1 >= 2.147483648E+9) {
    i3 = MAX_int32_T;
  } else {
    i3 = 0;
  }

  absb = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_NODES, &b_emlrtDCI, sp) *
    (int32_T)emlrtNonNegativeCheckFastR2012b(i3, &c_emlrtDCI, sp);
  for (i3 = 0; i3 < absb; i3++) {
    T->data[i3] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &b_emlrtRTEI, true);

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  emlrtDynamicBoundsCheckFastR2012b(1, 1, NUM_NODES, &m_emlrtBCI, sp);
  d1 = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (d1 < 2.147483648E+9) {
    if (d1 >= -2.147483648E+9) {
      absb = (int32_T)d1;
    } else {
      absb = MIN_int32_T;
    }
  } else if (d1 >= 2.147483648E+9) {
    absb = MAX_int32_T;
  } else {
    absb = 0;
  }

  i3 = r0->size[0];
  d1 = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (d1 < 2.147483648E+9) {
    if (d1 >= -2.147483648E+9) {
      cdiff = (int32_T)d1;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (d1 >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  r0->size[0] = cdiff;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < absb; i3++) {
    r0->data[i3] = i3;
  }

  emxInit_real_T(sp, &next, 2, &d_emlrtRTEI, true);
  d1 = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &e_emlrtDCI, sp);
  emlrtIntegerCheckFastR2012b(d1, &d_emlrtDCI, sp);
  i3 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 13 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 13; i3++) {
    next->data[next->size[0] * i3] = nInit[i3];
  }

  absb = (int32_T)transitionArrayLength;
  for (i3 = 0; i3 < absb; i3++) {
    next->data[next->size[0] * (i3 + 13)] = 0.0;
  }

  iv2[0] = 1;
  iv2[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv2, 2, *(int32_T (*)[2])next->size, 2,
    &b_emlrtECI, sp);
  absb = next->size[1];
  for (i3 = 0; i3 < absb; i3++) {
    T->data[T->size[0] * r0->data[i3]] = next->data[next->size[0] * i3];
  }

  emxFree_int32_T(&r0);

  /* Initialize the tree with initial state. */
  transitionArrayLength = 1.0;
  st.site = &k_emlrtRSI;
  if (2 > NUM_NODES) {
    b0 = false;
  } else {
    b0 = (NUM_NODES > 2147483646);
  }

  if (b0) {
    b_st.site = &l_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  apnd = 2;
  while (apnd <= NUM_NODES) {
    st.site = &j_emlrtRSI;
    rrtLoop(&st, T, jointLimits, kC, panHeight, U, Dt, dt, HGAINS,
            &transitionArrayLength, nGoal, uBDot, legNum);
    apnd++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxInit_real_T(sp, &transitionArray, 2, &e_emlrtRTEI, true);

  /* Find the closest node in the tree to the goal node. */
  st.site = &i_emlrtRSI;
  nearestNeighbour(&st, nGoal, T, jointLimits, kC->l1, kC->l2, kC->l3, kC->l4,
                   kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r,
                   transitionArrayLength, 13, xNearest_data, xNearest_size,
                   transitionArray, &unusedU2);
  transitionArrayLength = xNearest_data[0];
  i3 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 13;
  emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 13; i3++) {
    next->data[i3] = xNearest_data[i3];
  }

  emxInit_real_T(sp, &b_path, 2, &h_emlrtRTEI, true);
  i3 = b_path->size[0] * b_path->size[1];
  b_path->size[0] = 0;
  b_path->size[1] = 10;
  emxEnsureCapacity(sp, (emxArray__common *)b_path, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(sp, &transitionPath, 2, &f_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &b_emlrtRTEI, true);
  exitg1 = false;
  while ((!exitg1) && (transitionArrayLength != 0.0)) {
    i3 = next->size[1];
    emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &l_emlrtBCI, sp);
    if (next->data[1] != 0.0) {
      i3 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = 0;
      transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i3, (int32_T)
                        sizeof(real_T), &b_emlrtRTEI);
      i3 = (int32_T)(((real_T)transitionArray->size[1] + 9.0) / 10.0);
      emlrtForLoopVectorCheckR2012b(1.0, 10.0, transitionArray->size[1],
        mxDOUBLE_CLASS, i3, &r_emlrtRTEI, sp);
      apnd = 0;
      while (apnd <= i3 - 1) {
        i = apnd * 10U + 1U;
        cdiff = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, cdiff,
                          (int32_T)sizeof(real_T), &b_emlrtRTEI);
        for (cdiff = 0; cdiff < 10; cdiff++) {
          absb = transitionPath->size[0];
          for (i4 = 0; i4 < absb; i4++) {
            c_transitionPath->data[i4 + c_transitionPath->size[0] * cdiff] =
              transitionPath->data[i4 + transitionPath->size[0] * cdiff];
          }
        }

        for (cdiff = 0; cdiff < 10; cdiff++) {
          i4 = transitionArray->size[1];
          ndbl = (int32_T)(cdiff + i);
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * cdiff] = transitionArray->
            data[emlrtDynamicBoundsCheckFastR2012b(ndbl, 1, i4, &n_emlrtBCI, sp)
            - 1];
        }

        cdiff = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)transitionPath, cdiff,
                          (int32_T)sizeof(real_T), &b_emlrtRTEI);
        for (cdiff = 0; cdiff < 10; cdiff++) {
          absb = c_transitionPath->size[0];
          for (i4 = 0; i4 < absb; i4++) {
            transitionPath->data[i4 + transitionPath->size[0] * cdiff] =
              c_transitionPath->data[i4 + c_transitionPath->size[0] * cdiff];
          }
        }

        apnd++;
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
      }

      i3 = b_transitionPath->size[0] * b_transitionPath->size[1];
      b_transitionPath->size[0] = transitionPath->size[0] + b_path->size[0];
      b_transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)b_transitionPath, i3, (int32_T)
                        sizeof(real_T), &b_emlrtRTEI);
      for (i3 = 0; i3 < 10; i3++) {
        absb = transitionPath->size[0];
        for (cdiff = 0; cdiff < absb; cdiff++) {
          b_transitionPath->data[cdiff + b_transitionPath->size[0] * i3] =
            transitionPath->data[cdiff + transitionPath->size[0] * i3];
        }
      }

      for (i3 = 0; i3 < 10; i3++) {
        absb = b_path->size[0];
        for (cdiff = 0; cdiff < absb; cdiff++) {
          b_transitionPath->data[(cdiff + transitionPath->size[0]) +
            b_transitionPath->size[0] * i3] = b_path->data[cdiff + b_path->size
            [0] * i3];
        }
      }

      i3 = b_path->size[0] * b_path->size[1];
      b_path->size[0] = b_transitionPath->size[0];
      b_path->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)b_path, i3, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      for (i3 = 0; i3 < 10; i3++) {
        absb = b_transitionPath->size[0];
        for (cdiff = 0; cdiff < absb; cdiff++) {
          b_path->data[cdiff + b_path->size[0] * i3] = b_transitionPath->
            data[cdiff + b_transitionPath->size[0] * i3];
        }
      }

      i3 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &k_emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      absb = T->size[1];
      i3 = T->size[0];
      cdiff = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength,
        &emlrtDCI, sp);
      ndbl = emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, i3, &j_emlrtBCI, sp);
      i3 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = absb;
      emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i3 = 0; i3 < absb; i3++) {
        next->data[next->size[0] * i3] = T->data[(ndbl + T->size[0] * i3) - 1];
      }

      i3 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &i_emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      if (14 > next->size[1]) {
        i3 = 0;
        cdiff = 0;
      } else {
        i3 = 13;
        cdiff = next->size[1];
        i4 = next->size[1];
        cdiff = emlrtDynamicBoundsCheckFastR2012b(i4, 1, cdiff, &h_emlrtBCI, sp);
      }

      i4 = transitionArray->size[0] * transitionArray->size[1];
      transitionArray->size[0] = 1;
      transitionArray->size[1] = cdiff - i3;
      emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i4, (int32_T)
                        sizeof(real_T), &b_emlrtRTEI);
      absb = cdiff - i3;
      for (cdiff = 0; cdiff < absb; cdiff++) {
        transitionArray->data[transitionArray->size[0] * cdiff] = next->data[i3
          + cdiff];
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    } else {
      exitg1 = true;
    }
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
  emxFree_real_T(&transitionArray);
  st.site = &h_emlrtRSI;
  b_st.site = &wb_emlrtRSI;
  c_st.site = &xb_emlrtRSI;
  if (b_path->size[0] < 1) {
    absb = -1;
    apnd = 0;
  } else {
    ndbl = (int32_T)muDoubleScalarFloor(((real_T)b_path->size[0] - 1.0) + 0.5);
    apnd = ndbl + 1;
    cdiff = (ndbl - b_path->size[0]) + 1;
    absb = b_path->size[0];
    if (muDoubleScalarAbs(cdiff) < 4.4408920985006262E-16 * (real_T)absb) {
      ndbl++;
      apnd = b_path->size[0];
    } else if (cdiff > 0) {
      apnd = ndbl;
    } else {
      ndbl++;
    }

    absb = ndbl - 1;
  }

  i3 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = absb + 1;
  emxEnsureCapacity(&c_st, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  if (absb + 1 > 0) {
    next->data[0] = 1.0;
    if (absb + 1 > 1) {
      next->data[absb] = apnd;
      i3 = absb + (absb < 0);
      if (i3 >= 0) {
        ndbl = (int32_T)((uint32_T)i3 >> 1);
      } else {
        ndbl = (int32_T)~(~(uint32_T)i3 >> 1);
      }

      for (cdiff = 1; cdiff < ndbl; cdiff++) {
        next->data[cdiff] = 1.0 + (real_T)cdiff;
        next->data[absb - cdiff] = apnd - cdiff;
      }

      if (ndbl << 1 == absb) {
        next->data[ndbl] = (1.0 + (real_T)apnd) / 2.0;
      } else {
        next->data[ndbl] = 1.0 + (real_T)ndbl;
        next->data[ndbl + 1] = apnd - ndbl;
      }
    }
  }

  b_emxInit_real_T(&c_st, &t, 1, &g_emlrtRTEI, true);
  i3 = t->size[0];
  t->size[0] = next->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)t, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  absb = next->size[1];
  for (i3 = 0; i3 < absb; i3++) {
    t->data[i3] = dt * next->data[next->size[0] * i3];
  }

  emxFree_real_T(&next);
  ndbl = t->size[0];
  i3 = b_path->size[0];
  emlrtDimSizeEqCheckFastR2012b(ndbl, i3, &emlrtECI, sp);
  ndbl = t->size[0];
  i3 = path->size[0] * path->size[1];
  path->size[0] = ndbl;
  path->size[1] = 11;
  emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < ndbl; i3++) {
    path->data[i3] = t->data[i3];
  }

  emxFree_real_T(&t);
  for (i3 = 0; i3 < 10; i3++) {
    absb = b_path->size[0];
    for (cdiff = 0; cdiff < absb; cdiff++) {
      path->data[cdiff + path->size[0] * (i3 + 1)] = b_path->data[cdiff +
        b_path->size[0] * i3];
    }
  }

  emxFree_real_T(&b_path);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
