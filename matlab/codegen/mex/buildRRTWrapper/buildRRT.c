/*
 * buildRRT.c
 *
 * Code generation for function 'buildRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "buildRRTWrapper_emxutil.h"
#include "eml_error.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "buildRRTWrapper_mexutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = { 47, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo i_emlrtRSI = { 30, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 62, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 63, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo l_emlrtRSI = { 64, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 69, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo n_emlrtRSI = { 31, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 39, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 40, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 42, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo r_emlrtRSI = { 43, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo s_emlrtRSI = { 45, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo t_emlrtRSI = { 46, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo u_emlrtRSI = { 53, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo v_emlrtRSI = { 56, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo ab_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo bb_emlrtRSI = { 26, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo gb_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo hb_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ib_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo jb_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo kb_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtMCInfo c_emlrtMCI = { 50, 9, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtMCInfo d_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo e_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo d_emlrtRTEI = { 60, 28, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo r_emlrtBCI = { -1, -1, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo s_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo u_emlrtBCI = { -1, -1, 31, 24, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo v_emlrtBCI = { -1, -1, 32, 34, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtECInfo emlrtECI = { -1, 23, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo w_emlrtBCI = { -1, -1, 52, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtBCInfo x_emlrtBCI = { -1, -1, 53, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo e_emlrtDCI = { 53, 18, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtBCInfo y_emlrtBCI = { -1, -1, 53, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { 2, 54, 16, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo ab_emlrtBCI = { -1, -1, 55, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo f_emlrtDCI = { 21, 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo g_emlrtDCI = { 23, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtDCInfo h_emlrtDCI = { 23, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtBCInfo bb_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtECInfo f_emlrtECI = { -1, 71, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 71, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtRSInfo qb_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo tb_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ub_emlrtRSI = { 104, "mod",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/mod.m" };

static emlrtRSInfo vb_emlrtRSI = { 50, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

/* Function Declarations */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator);
static void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointLimits[12], const real_T b_cartesianLimits[4], const
                    struct0_T *kC, real_T panHeight, const real_T U[10], real_T
                    Dt, real_T dt, real_T *nodeIDCount, const real_T nGoal[11],
                    const real_T uBDot[6], int32_T legNum);

/* Function Definitions */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m7;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m7, 1, &pArray, "message", true, location);
}

static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

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
                    jointLimits[12], const real_T b_cartesianLimits[4], const
                    struct0_T *kC, real_T panHeight, const real_T U[10], real_T
                    Dt, real_T dt, real_T *nodeIDCount, const real_T nGoal[11],
                    const real_T uBDot[6], int32_T legNum)
{
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv20[2] = { 1, 17 };

  const mxArray *m8;
  char_T cv6[17];
  int32_T i;
  static const char_T cv7[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  real_T dv14[9];
  int32_T ix;
  int32_T xRand_size[2];
  real_T xRand_data[11];
  int32_T b_i;
  emxArray_real_T *d;
  int32_T n;
  emxArray_real_T *b_T;
  boolean_T b3;
  const mxArray *b_y;
  static const int32_T iv21[2] = { 1, 36 };

  char_T cv8[36];
  static const char_T cv9[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b4;
  boolean_T exitg1;
  boolean_T c_i;
  int32_T xNear_size[2];
  real_T xNear_data[11];
  emxArray_int32_T *r3;
  int32_T xNew_size[2];
  real_T xNew_data[11];
  emxArray_real_T b_xNear_data;
  emxArray_real_T *r4;
  int32_T iv22[2];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  g_st.prev = &e_st;
  g_st.tls = e_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  st.site = &j_emlrtRSI;

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
  b_st.site = &n_emlrtRSI;
  emlrtRandu(&r, 1);
  if ((panHeight <= b_cartesianLimits[0]) && (panHeight >= b_cartesianLimits[2]))
  {
    b_st.site = &o_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &p_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else if ((panHeight < b_cartesianLimits[2]) && (panHeight >=
              b_cartesianLimits[3])) {
    b_st.site = &q_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &r_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else if ((panHeight < b_cartesianLimits[3]) && (panHeight >=
              b_cartesianLimits[1])) {
    b_st.site = &s_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[3], true, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
    b_st.site = &t_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                    kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                    kC->r);
  } else {
    xMax = 0.0;
    xMin = 0.0;
    y = NULL;
    m8 = emlrtCreateCharArray(2, iv20);
    for (i = 0; i < 17; i++) {
      cv6[i] = cv7[i];
    }

    emlrtInitCharArrayR2013a(&st, 17, m8, cv6);
    emlrtAssign(&y, m8);
    b_st.site = &vb_emlrtRSI;
    disp(&b_st, y, &c_emlrtMCI);
  }

  b_st.site = &u_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_xMin[0] = xMin + (xMax - xMin) * b_r;
  b_xMin[1] = 0.0;
  b_xMin[2] = panHeight;
  b_st.site = &v_emlrtRSI;
  b_sherpaTTIK(&b_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
               kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, q);

  /* alphaDotRand = range(4)*rand+MIN(4); */
  /* gammaDotRand = range(5)*rand+MIN(5); */
  /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
  for (ix = 0; ix < 3; ix++) {
    dv14[ix] = 0.0;
  }

  dv14[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
  dv14[4] = q[1];
  dv14[5] = q[2];
  dv14[6] = 0.0;
  dv14[7] = 0.0;
  dv14[8] = 0.0;
  xRand_size[0] = 1;
  xRand_size[1] = 9;
  for (ix = 0; ix < 9; ix++) {
    xRand_data[ix] = dv14[ix];
  }

  r = muDoubleScalarRound(*nodeIDCount);
  if (r < 2.147483648E+9) {
    if (r >= -2.147483648E+9) {
      i = (int32_T)r;
    } else {
      i = MIN_int32_T;
    }
  } else if (r >= 2.147483648E+9) {
    i = MAX_int32_T;
  } else {
    i = 0;
  }

  r = muDoubleScalarRound(*nodeIDCount - muDoubleScalarFloor(*nodeIDCount / 20.0)
    * 20.0);
  if (r < 2.147483648E+9) {
    if (r >= -2.147483648E+9) {
      ix = (int32_T)r;
    } else {
      ix = MIN_int32_T;
    }
  } else if (r >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  if (i == *nodeIDCount) {
    b_st.site = &ub_emlrtRSI;
    b_i = i - div_s32_floor(&b_st, i, 20) * 20;
  } else {
    b_i = ix;
  }

  if (b_i == 0) {
    xRand_size[0] = 1;
    xRand_size[1] = 11;
    memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(real_T));
  }

  emxInit_real_T(&st, &d, 2, &c_emlrtRTEI, true);
  st.site = &k_emlrtRSI;

  /* nearestNeigbour.m */
  /* author: wreid */
  /* date: 20150107 */
  /* nearestNeigbour Finds the node in the tree closest to x. */
  /*    This function scans each node within the tree and finds the node that */
  /*    is closest to the xRand node. The nearest node is returned by the */
  /*    function. A distance heuristic is  used */
  /*    Inputs: */
  /*        x:  The 1xn state that each node in the tree will be compared to, */
  /*            to find the node with the minimum distance to it. n refers to */
  /*            the number of dimensions within the state space. */
  /*        T:  The nxm tree being searched, m is the number of possible nodes */
  /*            within the tree. */
  /*        HGAINS: The gains applied to the heuristic function. */
  /*    Outputs: */
  /*        xNear:  The node in the tree that is closet to x. */
  /* Iterate over the entire tree and apply the distance heuristic function */
  /* to each node. */
  ix = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int32_T)*nodeIDCount;
  emxEnsureCapacity(&st, (emxArray__common *)d, ix, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  n = (int32_T)*nodeIDCount;
  for (ix = 0; ix < n; ix++) {
    d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, *nodeIDCount, mxDOUBLE_CLASS, (int32_T)*
    nodeIDCount, &i_emlrtRTEI, &st);
  i = 1;
  emxInit_real_T(&st, &b_T, 2, &d_emlrtRTEI, true);
  while (i - 1 <= (int32_T)*nodeIDCount - 1) {
    n = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, 1000, &r_emlrtBCI, &st);
    ix = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = n;
    emxEnsureCapacity(&st, (emxArray__common *)b_T, ix, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    for (ix = 0; ix < n; ix++) {
      b_T->data[b_T->size[0] * ix] = T->data[(i + T->size[0] * ix) - 1];
    }

    ix = d->size[1];
    b_st.site = &bb_emlrtRSI;
    d->data[emlrtDynamicBoundsCheckFastR2012b(i, 1, ix, &bb_emlrtBCI, &st) - 1] =
      heuristicSingleLeg(&b_st, xRand_data, b_T, jointLimits, kC->l2, kC->l3,
                         kC->l4, kC->l5, kC->l7, kC->zeta);
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  emxFree_real_T(&b_T);
  ix = d->size[1];
  i = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(i, 1, ix, &s_emlrtBCI, &st);
  b_st.site = &ab_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (((int32_T)*nodeIDCount == 1) || ((int32_T)*nodeIDCount != 1)) {
    b3 = true;
  } else {
    b3 = false;
  }

  if (b3) {
  } else {
    b_y = NULL;
    m8 = emlrtCreateCharArray(2, iv21);
    for (i = 0; i < 36; i++) {
      cv8[i] = cv9[i];
    }

    emlrtInitCharArrayR2013a(&d_st, 36, m8, cv8);
    emlrtAssign(&b_y, m8);
    e_st.site = &qb_emlrtRSI;
    f_st.site = &tb_emlrtRSI;
    error(&e_st, b_message(&f_st, b_y, &d_emlrtMCI), &e_emlrtMCI);
  }

  e_st.site = &hb_emlrtRSI;
  i = 1;
  n = (int32_T)*nodeIDCount;
  r = d->data[0];
  itmp = 0;
  if ((int32_T)*nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(r)) {
      g_st.site = &jb_emlrtRSI;
      if (2 > (int32_T)*nodeIDCount) {
        b4 = false;
      } else {
        b4 = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b4) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        i = ix;
        if (!muDoubleScalarIsNaN(d->data[ix - 1])) {
          r = d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (i < (int32_T)*nodeIDCount) {
      g_st.site = &ib_emlrtRSI;
      if (i + 1 > (int32_T)*nodeIDCount) {
        c_i = false;
      } else {
        c_i = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (c_i) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      while (i + 1 <= n) {
        if (d->data[i] < r) {
          r = d->data[i];
          itmp = i;
        }

        i++;
      }
    }
  }

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  ix = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, ix, &u_emlrtBCI, &st);
  ix = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(11, 1, ix, &u_emlrtBCI, &st);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1000, &t_emlrtBCI, &st);
  xNear_size[0] = 1;
  xNear_size[1] = 11;
  for (ix = 0; ix < 11; ix++) {
    xNear_data[xNear_size[0] * ix] = T->data[itmp + T->size[0] * ix];
  }

  if (12 > T->size[1]) {
  } else {
    ix = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(12, 1, ix, &v_emlrtBCI, &st);
    ix = T->size[1];
    i = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, ix, &v_emlrtBCI, &st);
  }

  emxInit_int32_T(&st, &r3, 1, &d_emlrtRTEI, true);
  st.site = &l_emlrtRSI;
  selectInput(&st, xNear_data, xNear_size, xRand_data, xRand_size, U, dt, Dt, kC,
              jointLimits, uBDot, legNum, xNew_data, xNew_size, d);
  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  /* Node ID */
  xNew_data[1] = T->data[itmp];

  /* Parent ID */
  b_xNear_data.data = (real_T *)&xNear_data;
  b_xNear_data.size = (int32_T *)&xNear_size;
  b_xNear_data.allocatedSize = 11;
  b_xNear_data.numDimensions = 2;
  b_xNear_data.canFreeData = false;
  st.site = &m_emlrtRSI;
  xNew_data[2] = heuristicSingleLeg(&st, xNew_data, &b_xNear_data, jointLimits,
    kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);

  /* Cost */
  ix = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ix, 1, 1000, &fb_emlrtBCI, sp);
  n = T->size[1];
  ix = r3->size[0];
  r3->size[0] = n;
  emxEnsureCapacity(sp, (emxArray__common *)r3, ix, (int32_T)sizeof(int32_T),
                    &d_emlrtRTEI);
  for (ix = 0; ix < n; ix++) {
    r3->data[ix] = ix;
  }

  emxInit_real_T(sp, &r4, 2, &d_emlrtRTEI, true);
  ix = r4->size[0] * r4->size[1];
  r4->size[0] = 1;
  r4->size[1] = 11 + d->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)r4, ix, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  for (ix = 0; ix < 11; ix++) {
    r4->data[r4->size[0] * ix] = xNew_data[xNew_size[0] * ix];
  }

  n = d->size[1];
  for (ix = 0; ix < n; ix++) {
    r4->data[r4->size[0] * (ix + 11)] = d->data[d->size[0] * ix];
  }

  emxFree_real_T(&d);
  iv22[0] = 1;
  iv22[1] = r3->size[0];
  emlrtSubAssignSizeCheckR2012b(iv22, 2, *(int32_T (*)[2])r4->size, 2,
    &f_emlrtECI, sp);
  i = (int32_T)*nodeIDCount;
  n = r4->size[1];
  for (ix = 0; ix < n; ix++) {
    T->data[(i + T->size[0] * r3->data[ix]) - 1] = r4->data[r4->size[0] * ix];
  }

  emxFree_real_T(&r4);
  emxFree_int32_T(&r3);

  /* Append the new node to the tree.     */
  /* if mod(nodeIDCount,100) == 0 */
  /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
  /* end */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRT(const emlrtStack *sp, const real_T nInit[11], const real_T nGoal
              [11], const real_T jointLimits[12], const real_T
              b_cartesianLimits[4], real_T panHeight, const real_T U[10], real_T
              dt, real_T Dt, const struct0_T *kC, const real_T uBDot[6], int32_T
              legNum, emxArray_real_T *T, emxArray_real_T *path)
{
  real_T transitionArrayLength;
  int32_T i5;
  real_T xStarMin;
  int32_T i6;
  int32_T loop_ub;
  emxArray_int32_T *r0;
  emxArray_real_T *d;
  int32_T iv2[2];
  real_T nodeIDCount;
  int32_T ixstart;
  real_T dxStarMax;
  boolean_T b0;
  const mxArray *y;
  static const int32_T iv3[2] = { 1, 36 };

  const mxArray *m0;
  char_T cv0[36];
  static const char_T cv1[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b1;
  boolean_T exitg2;
  boolean_T b_ixstart;
  real_T xNearest_data[11];
  emxArray_real_T *b_path;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  g_st.prev = &e_st;
  g_st.tls = e_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
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
  /* %TODO: Make a structure input for nInit and nGoal */
  /* %TEMPORARY definition of init and goal nodes. */
  /* nInit = makeNode(1,0,0,nInit(4:6),nInit(7:9),) */
  /* Constant Declaration                                                       */
  transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 6.0;

  /* Variable Initialization */
  i5 = T->size[0] * T->size[1];
  T->size[0] = 1000;
  xStarMin = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (xStarMin < 2.147483648E+9) {
    if (xStarMin >= -2.147483648E+9) {
      i6 = (int32_T)xStarMin;
    } else {
      i6 = MIN_int32_T;
    }
  } else if (xStarMin >= 2.147483648E+9) {
    i6 = MAX_int32_T;
  } else {
    i6 = 0;
  }

  T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(i6, &f_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T, i5, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  xStarMin = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (xStarMin < 2.147483648E+9) {
    if (xStarMin >= -2.147483648E+9) {
      i5 = (int32_T)xStarMin;
    } else {
      i5 = MIN_int32_T;
    }
  } else if (xStarMin >= 2.147483648E+9) {
    i5 = MAX_int32_T;
  } else {
    i5 = 0;
  }

  loop_ub = 1000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i5, &f_emlrtDCI, sp);
  for (i5 = 0; i5 < loop_ub; i5++) {
    T->data[i5] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &b_emlrtRTEI, true);

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  xStarMin = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (xStarMin < 2.147483648E+9) {
    if (xStarMin >= -2.147483648E+9) {
      loop_ub = (int32_T)xStarMin;
    } else {
      loop_ub = MIN_int32_T;
    }
  } else if (xStarMin >= 2.147483648E+9) {
    loop_ub = MAX_int32_T;
  } else {
    loop_ub = 0;
  }

  i5 = r0->size[0];
  xStarMin = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (xStarMin < 2.147483648E+9) {
    if (xStarMin >= -2.147483648E+9) {
      i6 = (int32_T)xStarMin;
    } else {
      i6 = MIN_int32_T;
    }
  } else if (xStarMin >= 2.147483648E+9) {
    i6 = MAX_int32_T;
  } else {
    i6 = 0;
  }

  r0->size[0] = i6;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i5, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i5 = 0; i5 < loop_ub; i5++) {
    r0->data[i5] = i5;
  }

  emxInit_real_T(sp, &d, 2, &c_emlrtRTEI, true);
  xStarMin = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &h_emlrtDCI,
    sp);
  emlrtIntegerCheckFastR2012b(xStarMin, &g_emlrtDCI, sp);
  i5 = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = 11 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)d, i5, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i5 = 0; i5 < 11; i5++) {
    d->data[d->size[0] * i5] = nInit[i5];
  }

  loop_ub = (int32_T)transitionArrayLength;
  for (i5 = 0; i5 < loop_ub; i5++) {
    d->data[d->size[0] * (i5 + 11)] = 0.0;
  }

  iv2[0] = 1;
  iv2[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv2, 2, *(int32_T (*)[2])d->size, 2, &emlrtECI,
    sp);
  loop_ub = d->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    T->data[T->size[0] * r0->data[i5]] = d->data[d->size[0] * i5];
  }

  emxFree_int32_T(&r0);

  /* Initialize the tree with initial state. */
  nodeIDCount = 1.0;
  for (ixstart = 0; ixstart < 999; ixstart++) {
    st.site = &i_emlrtRSI;
    rrtLoop(&st, T, jointLimits, b_cartesianLimits, kC, panHeight, U, Dt, dt,
            &nodeIDCount, nGoal, uBDot, legNum);
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Find the closest node in the tree to the goal node. */
  st.site = &h_emlrtRSI;

  /* nearestNeigbour.m */
  /* author: wreid */
  /* date: 20150107 */
  /* nearestNeigbour Finds the node in the tree closest to x. */
  /*    This function scans each node within the tree and finds the node that */
  /*    is closest to the xRand node. The nearest node is returned by the */
  /*    function. A distance heuristic is  used */
  /*    Inputs: */
  /*        x:  The 1xn state that each node in the tree will be compared to, */
  /*            to find the node with the minimum distance to it. n refers to */
  /*            the number of dimensions within the state space. */
  /*        T:  The nxm tree being searched, m is the number of possible nodes */
  /*            within the tree. */
  /*        HGAINS: The gains applied to the heuristic function. */
  /*    Outputs: */
  /*        xNear:  The node in the tree that is closet to x. */
  /* Iterate over the entire tree and apply the distance heuristic function */
  /* to each node. */
  i5 = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int32_T)nodeIDCount;
  emxEnsureCapacity(&st, (emxArray__common *)d, i5, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  loop_ub = (int32_T)nodeIDCount;
  for (i5 = 0; i5 < loop_ub; i5++) {
    d->data[i5] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &i_emlrtRTEI, &st);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    b_st.site = &bb_emlrtRSI;
    i5 = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(i5, 1, 1000, &r_emlrtBCI, &b_st);

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(4, 1, i5, &q_emlrtBCI, &b_st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i5, &p_emlrtBCI, &b_st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i5, &o_emlrtBCI, &b_st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(7, 1, i5, &n_emlrtBCI, &b_st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(8, 1, i5, &m_emlrtBCI, &b_st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(9, 1, i5, &l_emlrtBCI, &b_st);

    /* Calculate the distance between angular positions. */
    xStarMin = (((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[2])) + kC->l4 *
                 muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                (kC->zeta + jointLimits[4])) - kC->l7;
    dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(jointLimits[3])) +
                   kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
                  muDoubleScalarCos(kC->zeta + jointLimits[5])) - kC->l7) -
      xStarMin;

    /* angDiff Finds the angular difference between th1 and th2. */
    transitionArrayLength = ((jointLimits[0] - jointLimits[2]) +
      3.1415926535897931) / 6.2831853071795862;
    if (muDoubleScalarAbs(transitionArrayLength - muDoubleScalarRound
                          (transitionArrayLength)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(transitionArrayLength)) {
      transitionArrayLength = 0.0;
    } else {
      transitionArrayLength = (transitionArrayLength - muDoubleScalarFloor
        (transitionArrayLength)) * 6.2831853071795862;
    }

    transitionArrayLength = muDoubleScalarAbs(transitionArrayLength -
      3.1415926535897931);
    c_st.site = &cb_emlrtRSI;
    d_st.site = &eb_emlrtRSI;
    if (dxStarMax * dxStarMax + xStarMin * xStarMin * (transitionArrayLength *
         transitionArrayLength) < 0.0) {
      e_st.site = &g_emlrtRSI;
      eml_error(&e_st);
    }

    xStarMin = (((kC->l2 + kC->l3 * muDoubleScalarCos(nGoal[4])) + kC->l4 *
                 muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
                (kC->zeta + nGoal[5])) - kC->l7;
    dxStarMax = ((((kC->l2 + kC->l3 * muDoubleScalarCos(T->data[ixstart +
      (T->size[0] << 2)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5 *
                  muDoubleScalarCos(kC->zeta + T->data[ixstart + T->size[0] * 5]))
                 - kC->l7) - xStarMin;

    /* angDiff Finds the angular difference between th1 and th2. */
    transitionArrayLength = ((nGoal[3] - T->data[ixstart + T->size[0] * 3]) +
      3.1415926535897931) / 6.2831853071795862;
    if (muDoubleScalarAbs(transitionArrayLength - muDoubleScalarRound
                          (transitionArrayLength)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(transitionArrayLength)) {
      transitionArrayLength = 0.0;
    } else {
      transitionArrayLength = (transitionArrayLength - muDoubleScalarFloor
        (transitionArrayLength)) * 6.2831853071795862;
    }

    transitionArrayLength = muDoubleScalarAbs(transitionArrayLength -
      3.1415926535897931);
    c_st.site = &db_emlrtRSI;
    transitionArrayLength = dxStarMax * dxStarMax + xStarMin * xStarMin *
      (transitionArrayLength * transitionArrayLength);
    if (transitionArrayLength < 0.0) {
      d_st.site = &g_emlrtRSI;
      eml_error(&d_st);
    }

    /* Calculate the total distance. */
    /* dPosNorm+dVelNorm  */
    i5 = d->size[1];
    d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, i5, &bb_emlrtBCI,
      &st) - 1] = muDoubleScalarSqrt(transitionArrayLength);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  i5 = d->size[1];
  i6 = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(i6, 1, i5, &s_emlrtBCI, &st);
  b_st.site = &ab_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b0 = true;
  } else {
    b0 = false;
  }

  if (b0) {
  } else {
    y = NULL;
    m0 = emlrtCreateCharArray(2, iv3);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv0[ixstart] = cv1[ixstart];
    }

    emlrtInitCharArrayR2013a(&d_st, 36, m0, cv0);
    emlrtAssign(&y, m0);
    e_st.site = &qb_emlrtRSI;
    f_st.site = &tb_emlrtRSI;
    error(&e_st, b_message(&f_st, y, &d_emlrtMCI), &e_emlrtMCI);
  }

  e_st.site = &hb_emlrtRSI;
  ixstart = 1;
  transitionArrayLength = d->data[0];
  itmp = 0;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(transitionArrayLength)) {
      g_st.site = &jb_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b1 = false;
      } else {
        b1 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b1) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      loop_ub = 2;
      exitg2 = false;
      while ((!exitg2) && (loop_ub <= (int32_T)nodeIDCount)) {
        ixstart = loop_ub;
        if (!muDoubleScalarIsNaN(d->data[loop_ub - 1])) {
          transitionArrayLength = d->data[loop_ub - 1];
          itmp = loop_ub - 1;
          exitg2 = true;
        } else {
          loop_ub++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      g_st.site = &ib_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (d->data[ixstart] < transitionArrayLength) {
          transitionArrayLength = d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  i5 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i5, &u_emlrtBCI, &st);
  i5 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(11, 1, i5, &u_emlrtBCI, &st);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1000, &t_emlrtBCI, &st);
  for (i5 = 0; i5 < 11; i5++) {
    xNearest_data[i5] = T->data[itmp + T->size[0] * i5];
  }

  if (12 > T->size[1]) {
    i5 = -11;
    i6 = 0;
  } else {
    i5 = 0;
    i6 = T->size[1];
    ixstart = T->size[1];
    i6 = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, i6, &v_emlrtBCI, &st);
  }

  transitionArrayLength = T->data[itmp];
  ixstart = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = 11;
  emxEnsureCapacity(sp, (emxArray__common *)d, ixstart, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (ixstart = 0; ixstart < 11; ixstart++) {
    d->data[ixstart] = xNearest_data[ixstart];
  }

  ixstart = path->size[0] * path->size[1];
  path->size[0] = 1;
  path->size[1] = i6 - i5;
  emxEnsureCapacity(sp, (emxArray__common *)path, ixstart, (int32_T)sizeof
                    (real_T), &b_emlrtRTEI);
  for (ixstart = 0; ixstart < 11; ixstart++) {
    path->data[path->size[0] * ixstart] = xNearest_data[ixstart];
  }

  loop_ub = i6 - i5;
  for (i6 = 0; i6 <= loop_ub - 12; i6++) {
    path->data[path->size[0] * (i6 + 11)] = T->data[itmp + T->size[0] * ((i5 +
      i6) + 11)];
  }

  emxInit_real_T(sp, &b_path, 2, &b_emlrtRTEI, true);
  exitg1 = false;
  while ((!exitg1) && (transitionArrayLength != 0.0)) {
    i5 = d->size[1];
    emlrtDynamicBoundsCheckFastR2012b(2, 1, i5, &w_emlrtBCI, sp);
    if (d->data[1] != 0.0) {
      i5 = d->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i5, &x_emlrtBCI, sp);
      transitionArrayLength = d->data[1];
      loop_ub = T->size[1];
      i5 = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength,
        &e_emlrtDCI, sp);
      emlrtDynamicBoundsCheckFastR2012b(i5, 1, 1000, &y_emlrtBCI, sp);
      i5 = d->size[0] * d->size[1];
      d->size[0] = 1;
      d->size[1] = loop_ub;
      emxEnsureCapacity(sp, (emxArray__common *)d, i5, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i5 = 0; i5 < loop_ub; i5++) {
        d->data[d->size[0] * i5] = T->data[((int32_T)transitionArrayLength +
          T->size[0] * i5) - 1];
      }

      i5 = path->size[1];
      i6 = d->size[1];
      emlrtDimSizeEqCheckFastR2012b(i5, i6, &b_emlrtECI, sp);
      i5 = b_path->size[0] * b_path->size[1];
      b_path->size[0] = path->size[0] + 1;
      b_path->size[1] = path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)b_path, i5, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      loop_ub = path->size[1];
      for (i5 = 0; i5 < loop_ub; i5++) {
        ixstart = path->size[0];
        for (i6 = 0; i6 < ixstart; i6++) {
          b_path->data[i6 + b_path->size[0] * i5] = path->data[i6 + path->size[0]
            * i5];
        }
      }

      loop_ub = d->size[1];
      for (i5 = 0; i5 < loop_ub; i5++) {
        b_path->data[path->size[0] + b_path->size[0] * i5] = d->data[d->size[0] *
          i5];
      }

      i5 = path->size[0] * path->size[1];
      path->size[0] = b_path->size[0];
      path->size[1] = b_path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)path, i5, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      loop_ub = b_path->size[1];
      for (i5 = 0; i5 < loop_ub; i5++) {
        ixstart = b_path->size[0];
        for (i6 = 0; i6 < ixstart; i6++) {
          path->data[i6 + path->size[0] * i5] = b_path->data[i6 + b_path->size[0]
            * i5];
        }
      }

      i5 = d->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i5, &ab_emlrtBCI, sp);
      transitionArrayLength = d->data[1];
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    } else {
      exitg1 = true;
    }
  }

  emxFree_real_T(&b_path);
  emxFree_real_T(&d);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
