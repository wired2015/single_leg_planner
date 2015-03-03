/*
 * buildBiDirectionalRRT.c
 *
 * Code generation for function 'buildBiDirectionalRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRT.h"
#include "buildBiDirectionalRRTWrapper_emxutil.h"
#include "norm.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "flipud.h"
#include "buildBiDirectionalRRTWrapper_mexutil.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo g_emlrtRSI = { 37, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo h_emlrtRSI = { 45, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo i_emlrtRSI = { 46, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo j_emlrtRSI = { 48, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo k_emlrtRSI = { 49, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo l_emlrtRSI = { 51, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo m_emlrtRSI = { 52, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo n_emlrtRSI = { 59, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 62, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 68, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 69, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo r_emlrtRSI = { 70, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo v_emlrtRSI = { 39, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo w_emlrtRSI = { 63, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo x_emlrtRSI = { 64, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo y_emlrtRSI = { 72, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo ab_emlrtRSI = { 75, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo bb_emlrtRSI = { 115, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo cb_emlrtRSI = { 116, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo db_emlrtRSI = { 117, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo eb_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo fb_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

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

static emlrtRSInfo ob_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRSInfo pb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtMCInfo emlrtMCI = { 56, 9, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtMCInfo b_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo c_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 5, 32, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo d_emlrtRTEI = { 63, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 64, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 72, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 29, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 88, 17, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 100, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo k_emlrtRTEI = { 113, 33, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo l_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo b_emlrtBCI = { -1, -1, 142, 15, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 140, 48, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 136, 50, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 136, 37, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 135, 31, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { -1, -1, 76, 32, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo emlrtECI = { 1, 73, 20, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo h_emlrtBCI = { -1, -1, 142, 29, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo c_emlrtECI = { -1, 109, 27, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo j_emlrtBCI = { 1, 750, 106, 18, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtDCInfo emlrtDCI = { 106, 18, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtECInfo d_emlrtECI = { -1, 102, 47, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 101, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo k_emlrtBCI = { 1, 750, 95, 25, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtDCInfo b_emlrtDCI = { 95, 25, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtBCInfo l_emlrtBCI = { 1, 750, 124, 7, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo m_emlrtBCI = { 1, 750, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo n_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo o_emlrtBCI = { 1, 750, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRTEInfo o_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo p_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRSInfo rb_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ub_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo vb_emlrtRSI = { 56, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

/* Function Declarations */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static const mxArray *message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);
static void rrtLoop(const emlrtStack *sp, real_T T[69750], const real_T
                    jointLimits[20], real_T kC_l1, real_T kC_l2, real_T kC_l3,
                    real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7,
                    real_T kC_l8, real_T kC_zeta, real_T kC_r, real_T
                    kC_B2PXOffset, real_T kC_B2PZOffset, const real_T
                    kC_legAngleOffset[4], real_T panHeight, real_T *nodeIDCount,
                    const real_T uBDot[6], int32_T legNum, real_T xNew[13]);
static void traceBranch(const emlrtStack *sp, const real_T T[69750], const
  real_T midPoint_data[], emxArray_real_T *path);

/* Function Definitions */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

static const mxArray *message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m5;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m5, 1, &pArray, "message", true, location);
}

static void rrtLoop(const emlrtStack *sp, real_T T[69750], const real_T
                    jointLimits[20], real_T kC_l1, real_T kC_l2, real_T kC_l3,
                    real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7,
                    real_T kC_l8, real_T kC_zeta, real_T kC_r, real_T
                    kC_B2PXOffset, real_T kC_B2PZOffset, const real_T
                    kC_legAngleOffset[4], real_T panHeight, real_T *nodeIDCount,
                    const real_T uBDot[6], int32_T legNum, real_T xNew[13])
{
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv16[2] = { 1, 17 };

  const mxArray *m7;
  char_T cv4[17];
  int32_T i;
  static const char_T cv5[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  int32_T n;
  real_T xRand[13];
  emxArray_real_T *d;
  real_T uA[3];
  real_T uB[3];
  real_T q_idx_1;
  real_T q_idx_2;
  real_T qDot[3];
  real_T b_qDot[3];
  real_T c_qDot[3];
  boolean_T b1;
  const mxArray *b_y;
  static const int32_T iv17[2] = { 1, 36 };

  char_T cv6[36];
  static const char_T cv7[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b2;
  int32_T ix;
  boolean_T exitg1;
  boolean_T b_i;
  real_T b_T[13];
  real_T transitionArray[80];
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
  st.site = &bb_emlrtRSI;

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
  b_st.site = &g_emlrtRSI;
  emlrtRandu(&r, 1);
  if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
    b_st.site = &h_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    b_st.site = &i_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[2], true, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
    b_st.site = &j_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[4], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    b_st.site = &k_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
    b_st.site = &l_emlrtRSI;
    xMax = getXStar(&b_st, panHeight, jointLimits[3], true, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    b_st.site = &m_emlrtRSI;
    xMin = getXStar(&b_st, panHeight, jointLimits[5], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else {
    xMax = 0.0;
    xMin = 0.0;
    y = NULL;
    m7 = emlrtCreateCharArray(2, iv16);
    for (i = 0; i < 17; i++) {
      cv4[i] = cv5[i];
    }

    emlrtInitCharArrayR2013a(&st, 17, m7, cv4);
    emlrtAssign(&y, m7);
    b_st.site = &vb_emlrtRSI;
    disp(&b_st, y, &emlrtMCI);
  }

  b_st.site = &n_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_xMin[0] = xMin + (xMax - xMin) * b_r;
  b_xMin[1] = 0.0;
  b_xMin[2] = panHeight;
  b_st.site = &o_emlrtRSI;
  b_sherpaTTIK(&b_st, b_xMin, kC_l1, kC_l2, kC_l3, kC_l4, kC_l5, kC_l6, kC_l7,
               kC_l8, kC_zeta, kC_r, jointLimits, q);
  b_st.site = &p_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_st.site = &q_emlrtRSI;
  emlrtRandu(&xMax, 1);
  b_st.site = &r_emlrtRSI;
  emlrtRandu(&xMin, 1);

  /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
  for (n = 0; n < 3; n++) {
    xRand[n] = 0.0;
  }

  emxInit_real_T(&st, &d, 2, &l_emlrtRTEI, true);
  xRand[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
  xRand[4] = q[1];
  xRand[5] = q[2];
  xRand[6] = 0.0;
  xRand[7] = 0.0;
  xRand[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
  xRand[9] = (jointLimits[11] - jointLimits[10]) * xMax + jointLimits[10];
  xRand[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
  xRand[11] = 0.0;
  xRand[12] = 0.0;

  /* if mod(nodeIDCount,goalSeedFreq) == 0 */
  /*     xRand = nGoal; */
  /* end */
  st.site = &cb_emlrtRSI;

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
  n = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int32_T)*nodeIDCount;
  emxEnsureCapacity(&st, (emxArray__common *)d, n, (int32_T)sizeof(real_T),
                    &k_emlrtRTEI);
  i = (int32_T)*nodeIDCount;
  for (n = 0; n < i; n++) {
    d->data[n] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, *nodeIDCount, mxDOUBLE_CLASS, (int32_T)*
    nodeIDCount, &o_emlrtRTEI, &st);
  i = 0;
  while (i <= (int32_T)*nodeIDCount - 1) {
    n = i + 1;
    emlrtDynamicBoundsCheckFastR2012b(n, 1, 750, &o_emlrtBCI, &st);

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
    uA[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xRand[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xRand[5]
               + kC_zeta)) - kC_l7) * muDoubleScalarCos(xRand[3]);
    uA[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-xRand[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(xRand[5]
               + kC_zeta)) - kC_l7) * muDoubleScalarSin(xRand[3]);
    uA[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-xRand[4])) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(xRand[5]
               + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);
    q[0] = T[2250 + i];
    q[1] = T[3000 + i];
    q[2] = T[3750 + i];

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
    uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q[1])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q[2] +
               kC_zeta)) - kC_l7) * muDoubleScalarCos(q[0]);
    uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q[1])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q[2] +
               kC_zeta)) - kC_l7) * muDoubleScalarSin(q[0]);
    uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-q[1])) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(q[2] +
               kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    xMax = T[6000 + i];
    xMin = T[6750 + i];
    r = T[7500 + i];
    b_r = T[2250 + i];
    q_idx_1 = T[3000 + i];
    q_idx_2 = T[3750 + i];

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
    qDot[0] = (-xMax * muDoubleScalarSin(b_r) * ((((kC_l2 - kC_l7) + kC_l5 *
      muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos(q_idx_1))
                + kC_l4 * muDoubleScalarCos(kC_zeta)) - xMin * kC_l3 *
               muDoubleScalarCos(b_r) * muDoubleScalarSin(q_idx_1)) - r * kC_l5 *
      muDoubleScalarSin(q_idx_2 + kC_zeta) * muDoubleScalarCos(b_r);
    qDot[1] = (xMax * muDoubleScalarCos(b_r) * ((((kC_l2 - kC_l7) + kC_l5 *
      muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos(q_idx_1))
                + kC_l4 * muDoubleScalarCos(kC_zeta)) - r * kC_l5 *
               muDoubleScalarSin(q_idx_2 + kC_zeta) * muDoubleScalarSin(b_r)) -
      xMin * kC_l3 * muDoubleScalarSin(b_r) * muDoubleScalarSin(q_idx_1);
    qDot[2] = -xMin * kC_l3 * muDoubleScalarCos(q_idx_1) - kC_l5 * r *
      muDoubleScalarCos(kC_zeta + q_idx_2);
    b_qDot[0] = (-xRand[8] * muDoubleScalarSin(xRand[3]) * ((((kC_l2 - kC_l7) +
      kC_l5 * muDoubleScalarCos(xRand[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos
                   (xRand[4])) + kC_l4 * muDoubleScalarCos(kC_zeta)) - xRand[9] *
                 kC_l3 * muDoubleScalarCos(xRand[3]) * muDoubleScalarSin(xRand[4]))
      - xRand[10] * kC_l5 * muDoubleScalarSin(xRand[5] + kC_zeta) *
      muDoubleScalarCos(xRand[3]);
    b_qDot[1] = (xRand[8] * muDoubleScalarCos(xRand[3]) * ((((kC_l2 - kC_l7) +
      kC_l5 * muDoubleScalarCos(xRand[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos
                   (xRand[4])) + kC_l4 * muDoubleScalarCos(kC_zeta)) - xRand[10]
                 * kC_l5 * muDoubleScalarSin(xRand[5] + kC_zeta) *
                 muDoubleScalarSin(xRand[3])) - xRand[9] * kC_l3 *
      muDoubleScalarSin(xRand[3]) * muDoubleScalarSin(xRand[4]);
    b_qDot[2] = -xRand[9] * kC_l3 * muDoubleScalarCos(xRand[4]) - kC_l5 * xRand
      [10] * muDoubleScalarCos(kC_zeta + xRand[5]);
    for (n = 0; n < 3; n++) {
      b_xMin[n] = uB[n] - uA[n];
      c_qDot[n] = qDot[n] - b_qDot[n];
    }

    n = d->size[1];
    d->data[emlrtDynamicBoundsCheckFastR2012b(i + 1, 1, n, &p_emlrtBCI, &st) - 1]
      = norm(b_xMin) + 0.0 * b_norm(c_qDot);
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  n = d->size[1];
  i = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(i, 1, n, &n_emlrtBCI, &st);
  b_st.site = &eb_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  if (((int32_T)*nodeIDCount == 1) || ((int32_T)*nodeIDCount != 1)) {
    b1 = true;
  } else {
    b1 = false;
  }

  if (b1) {
  } else {
    b_y = NULL;
    m7 = emlrtCreateCharArray(2, iv17);
    for (i = 0; i < 36; i++) {
      cv6[i] = cv7[i];
    }

    emlrtInitCharArrayR2013a(&d_st, 36, m7, cv6);
    emlrtAssign(&b_y, m7);
    e_st.site = &rb_emlrtRSI;
    f_st.site = &ub_emlrtRSI;
    error(&e_st, message(&f_st, b_y, &b_emlrtMCI), &c_emlrtMCI);
  }

  e_st.site = &hb_emlrtRSI;
  i = 1;
  n = (int32_T)*nodeIDCount;
  xMax = d->data[0];
  itmp = 0;
  if ((int32_T)*nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(xMax)) {
      g_st.site = &jb_emlrtRSI;
      if (2 > (int32_T)*nodeIDCount) {
        b2 = false;
      } else {
        b2 = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b2) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        i = ix;
        if (!muDoubleScalarIsNaN(d->data[ix - 1])) {
          xMax = d->data[ix - 1];
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
        b_i = false;
      } else {
        b_i = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b_i) {
        h_st.site = &kb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      while (i + 1 <= n) {
        if (d->data[i] < xMax) {
          xMax = d->data[i];
          itmp = i;
        }

        i++;
      }
    }
  }

  emxFree_real_T(&d);

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  n = itmp + 1;
  emlrtDynamicBoundsCheckFastR2012b(n, 1, 750, &m_emlrtBCI, &st);
  for (n = 0; n < 13; n++) {
    b_T[n] = T[itmp + 750 * n];
  }

  st.site = &db_emlrtRSI;
  selectInput(&st, b_T, xRand, kC_l1, kC_l2, kC_l3, kC_l4, kC_l5, kC_l6, kC_l7,
              kC_l8, kC_zeta, kC_r, kC_B2PXOffset, kC_B2PZOffset,
              kC_legAngleOffset, jointLimits, uBDot, legNum, xNew,
              transitionArray);
  (*nodeIDCount)++;
  xNew[0] = *nodeIDCount;

  /* Node ID */
  xNew[1] = T[itmp];

  /* Parent ID */
  for (n = 0; n < 13; n++) {
    b_T[n] = T[itmp + 750 * n];
  }

  xNew[2] = T[1500 + itmp] + heuristicSingleLeg(xNew, b_T, kC_l1, kC_l2, kC_l3,
    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);

  /* Cost */
  n = (int32_T)*nodeIDCount;
  i = emlrtDynamicBoundsCheckFastR2012b(n, 1, 750, &l_emlrtBCI, sp) - 1;
  for (n = 0; n < 13; n++) {
    T[i + 750 * n] = xNew[n];
  }

  for (n = 0; n < 80; n++) {
    T[i + 750 * (n + 13)] = transitionArray[n];
  }

  /* Append the new node to the tree.     */
  /* if mod(nodeIDCount,100) == 0 */
  /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
  /* end */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void traceBranch(const emlrtStack *sp, const real_T T[69750], const
  real_T midPoint_data[], emxArray_real_T *path)
{
  boolean_T b0;
  real_T check;
  real_T next_data[93];
  int32_T i14;
  int32_T transitionArray_size[2];
  real_T transitionArray_data[80];
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int32_T i;
  int32_T b_i;
  int32_T iv8[2];
  int32_T loop_ub;
  int32_T i15;
  int32_T next_size[2];
  int32_T tmp_size[2];
  int8_T tmp_data[80];
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  b0 = false;

  /* Assignn the  */
  check = midPoint_data[0];
  memcpy(&next_data[0], &midPoint_data[0], 13U * sizeof(real_T));
  i14 = path->size[0] * path->size[1];
  path->size[0] = 0;
  path->size[1] = 10;
  emxEnsureCapacity(sp, (emxArray__common *)path, i14, (int32_T)sizeof(real_T),
                    &h_emlrtRTEI);
  i14 = (int32_T)emlrtIntegerCheckFastR2012b(midPoint_data[0], &b_emlrtDCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(i14, 1, 750, &k_emlrtBCI, sp);
  transitionArray_size[0] = 1;
  transitionArray_size[1] = 80;
  for (i14 = 0; i14 < 80; i14++) {
    transitionArray_data[i14] = T[((int32_T)midPoint_data[0] + 750 * (13 + i14))
      - 1];
  }

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(sp, &transitionPath, 2, &i_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &h_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &h_emlrtRTEI, true);
  while ((check != 0.0) && (next_data[1] != 0.0)) {
    i14 = transitionPath->size[0] * transitionPath->size[1];
    transitionPath->size[0] = 0;
    transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i14, (int32_T)
                      sizeof(real_T), &h_emlrtRTEI);
    emlrtForLoopVectorCheckR2012b(1.0, 10.0, 80.0, mxDOUBLE_CLASS, 8,
      &n_emlrtRTEI, sp);
    for (i = 0; i < 8; i++) {
      b_i = i * 10;
      if (!b0) {
        for (i14 = 0; i14 < 2; i14++) {
          iv8[i14] = 1 + 9 * i14;
        }

        b0 = true;
      }

      emlrtMatrixMatrixIndexCheckR2012b(transitionArray_size, 2, iv8, 2,
        &d_emlrtECI, sp);
      i14 = c_transitionPath->size[0] * c_transitionPath->size[1];
      c_transitionPath->size[0] = transitionPath->size[0] + 1;
      c_transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, i14, (int32_T)
                        sizeof(real_T), &h_emlrtRTEI);
      for (i14 = 0; i14 < 10; i14++) {
        loop_ub = transitionPath->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          c_transitionPath->data[i15 + c_transitionPath->size[0] * i14] =
            transitionPath->data[i15 + transitionPath->size[0] * i14];
        }
      }

      for (i14 = 0; i14 < 10; i14++) {
        c_transitionPath->data[transitionPath->size[0] + c_transitionPath->size
          [0] * i14] = transitionArray_data[i14 + b_i];
      }

      i14 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = c_transitionPath->size[0];
      transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i14, (int32_T)
                        sizeof(real_T), &h_emlrtRTEI);
      for (i14 = 0; i14 < 10; i14++) {
        loop_ub = c_transitionPath->size[0];
        for (i15 = 0; i15 < loop_ub; i15++) {
          transitionPath->data[i15 + transitionPath->size[0] * i14] =
            c_transitionPath->data[i15 + c_transitionPath->size[0] * i14];
        }
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }

    i14 = b_transitionPath->size[0] * b_transitionPath->size[1];
    b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
    b_transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)b_transitionPath, i14, (int32_T)
                      sizeof(real_T), &h_emlrtRTEI);
    for (i14 = 0; i14 < 10; i14++) {
      loop_ub = transitionPath->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        b_transitionPath->data[i15 + b_transitionPath->size[0] * i14] =
          transitionPath->data[i15 + transitionPath->size[0] * i14];
      }
    }

    for (i14 = 0; i14 < 10; i14++) {
      loop_ub = path->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        b_transitionPath->data[(i15 + transitionPath->size[0]) +
          b_transitionPath->size[0] * i14] = path->data[i15 + path->size[0] *
          i14];
      }
    }

    i14 = path->size[0] * path->size[1];
    path->size[0] = b_transitionPath->size[0];
    path->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)path, i14, (int32_T)sizeof(real_T),
                      &h_emlrtRTEI);
    for (i14 = 0; i14 < 10; i14++) {
      loop_ub = b_transitionPath->size[0];
      for (i15 = 0; i15 < loop_ub; i15++) {
        path->data[i15 + path->size[0] * i14] = b_transitionPath->data[i15 +
          b_transitionPath->size[0] * i14];
      }
    }

    check = next_data[1];
    i14 = (int32_T)emlrtIntegerCheckFastR2012b(next_data[1], &emlrtDCI, sp);
    emlrtDynamicBoundsCheckFastR2012b(i14, 1, 750, &j_emlrtBCI, sp);
    next_size[0] = 1;
    next_size[1] = 93;
    for (i14 = 0; i14 < 93; i14++) {
      next_data[i14] = T[((int32_T)check + 750 * i14) - 1];
    }

    check = next_data[1];
    tmp_size[0] = 1;
    tmp_size[1] = 80;
    for (i14 = 0; i14 < 80; i14++) {
      tmp_data[i14] = (int8_T)(14 + i14);
    }

    emlrtMatrixMatrixIndexCheckR2012b(next_size, 2, tmp_size, 2, &c_emlrtECI, sp);
    transitionArray_size[0] = 1;
    transitionArray_size[1] = 80;
    for (i14 = 0; i14 < 80; i14++) {
      transitionArray_data[i14] = next_data[tmp_data[i14] - 1];
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildBiDirectionalRRT(c_buildBiDirectionalRRTWrapperS *SD, const emlrtStack
  *sp, const real_T nInit[13], const real_T nGoal[13], const real_T jointLimits
  [20], real_T panHeight, const struct0_T *kC, const real_T uBDot[6], int32_T
  legNum, const real_T TP2B[16], real_T T1[69750], real_T T2[69750],
  emxArray_real_T *pathJ, emxArray_real_T *pathC)
{
  int32_T i4;
  real_T nodeIDCount1;
  real_T nodeIDCount2;
  real_T pathLengthMin;
  int32_T i;
  real_T dist2Go;
  real_T unusedExpr[13];
  emxArray_real_T *pathT1;
  emxArray_real_T *pathT2;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *y;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_t;
  real_T d[750];
  int32_T cdiff;
  real_T T1_data[13];
  real_T b_T2[93];
  int32_T ixstart;
  int32_T absb;
  boolean_T exitg1;
  int32_T apnd;
  real_T uP[3];
  real_T uB[3];
  real_T b_uB[3];
  real_T b_path[3];
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

  /* buildBiDirectionalRRT.m */
  /* author: wreid */
  /* date: 20150107 */
  /* buildRRT Icrementally builds a rapidly exploring random tree. */
  /*    An RRT is build by incrementally selecting a random state from the */
  /*    available state space as defined by the MIN and MAX vectors. The tree is */
  /*    started at xInit and is extended until the number of maximum nodes, K has */
  /*    been reached. A path is selected if the goal region as defined by xGoal */
  /*    has been reached by the RRT. */
  /* Constant Declaration                                                       */
  for (i4 = 0; i4 < 69750; i4++) {
    T1[i4] = 0.0;
  }

  for (i4 = 0; i4 < 69750; i4++) {
    T2[i4] = 0.0;
  }

  for (i4 = 0; i4 < 13; i4++) {
    T1[750 * i4] = nInit[i4];
  }

  for (i4 = 0; i4 < 80; i4++) {
    T1[750 * (i4 + 13)] = 0.0;
  }

  for (i4 = 0; i4 < 13; i4++) {
    T2[750 * i4] = nGoal[i4];
  }

  for (i4 = 0; i4 < 80; i4++) {
    T2[750 * (i4 + 13)] = 0.0;
  }

  nodeIDCount1 = 1.0;
  nodeIDCount2 = 1.0;
  pathLengthMin = 100.0;
  i4 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = 0;
  pathC->size[1] = 0;
  emxEnsureCapacity(sp, (emxArray__common *)pathC, i4, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  i4 = pathJ->size[0] * pathJ->size[1];
  pathJ->size[0] = 0;
  pathJ->size[1] = 0;
  emxEnsureCapacity(sp, (emxArray__common *)pathJ, i4, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i = 0; i < 1498; i++) {
    for (i4 = 0; i4 < 69750; i4++) {
      SD->f0.T1[i4] = T1[i4];
    }

    dist2Go = nodeIDCount1;
    st.site = &v_emlrtRSI;
    rrtLoop(&st, SD->f0.T1, jointLimits, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
            kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, kC->B2PXOffset,
            kC->B2PZOffset, kC->legAngleOffset, panHeight, &dist2Go, uBDot,
            legNum, unusedExpr);
    nodeIDCount1++;

    /* Swap the trees. */
    for (i4 = 0; i4 < 69750; i4++) {
      T1[i4] = T2[i4];
    }

    memcpy(&T2[0], &SD->f0.T1[0], 69750U * sizeof(real_T));

    /* Swap the trees. */
    dist2Go = nodeIDCount1;
    nodeIDCount1 = nodeIDCount2;
    nodeIDCount2 = dist2Go;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxInit_real_T(sp, &pathT1, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &pathT2, 2, &e_emlrtRTEI, true);
  b_emxInit_real_T(sp, &t, 1, &f_emlrtRTEI, true);
  emxInit_real_T(sp, &path, 2, &g_emlrtRTEI, true);
  emxInit_real_T(sp, &y, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathC, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &b_t, 2, &b_emlrtRTEI, true);
  for (i = 0; i < 750; i++) {
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
    /* parfor i = 1:nodeIDCount */
    for (cdiff = 0; cdiff < 750; cdiff++) {
      for (i4 = 0; i4 < 13; i4++) {
        T1_data[i4] = T1[i + 750 * i4];
      }

      for (i4 = 0; i4 < 93; i4++) {
        b_T2[i4] = T2[cdiff + 750 * i4];
      }

      d[cdiff] = b_heuristicSingleLeg(T1_data, b_T2, kC->l1, kC->l2, kC->l3,
        kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }

    ixstart = 1;
    dist2Go = d[0];
    absb = 0;
    if (muDoubleScalarIsNaN(d[0])) {
      cdiff = 2;
      exitg1 = false;
      while ((!exitg1) && (cdiff < 751)) {
        ixstart = cdiff;
        if (!muDoubleScalarIsNaN(d[cdiff - 1])) {
          dist2Go = d[cdiff - 1];
          absb = cdiff - 1;
          exitg1 = true;
        } else {
          cdiff++;
        }
      }
    }

    if (ixstart < 750) {
      while (ixstart + 1 < 751) {
        if (d[ixstart] < dist2Go) {
          dist2Go = d[ixstart];
          absb = ixstart;
        }

        ixstart++;
      }
    }

    /* [d,minIndex] = min(d(1:nodeIDCount)); */
    if (dist2Go < 0.04) {
      for (i4 = 0; i4 < 13; i4++) {
        T1_data[i4] = T1[i + 750 * i4];
      }

      st.site = &w_emlrtRSI;
      traceBranch(&st, T1, T1_data, pathT1);
      for (i4 = 0; i4 < 13; i4++) {
        T1_data[i4] = T2[absb + 750 * i4];
      }

      st.site = &x_emlrtRSI;
      traceBranch(&st, T2, T1_data, pathT2);
      if ((T1[2250] == nInit[3]) && (T1[3000] == nInit[4]) && (T1[3750] ==
           nInit[5])) {
        flipud(pathT2);
        i4 = path->size[0] * path->size[1];
        path->size[0] = pathT1->size[0] + pathT2->size[0];
        path->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)path, i4, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        for (i4 = 0; i4 < 10; i4++) {
          ixstart = pathT1->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[absb + path->size[0] * i4] = pathT1->data[absb +
              pathT1->size[0] * i4];
          }
        }

        for (i4 = 0; i4 < 10; i4++) {
          ixstart = pathT2->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[(absb + pathT1->size[0]) + path->size[0] * i4] =
              pathT2->data[absb + pathT2->size[0] * i4];
          }
        }
      } else {
        flipud(pathT1);
        i4 = path->size[0] * path->size[1];
        path->size[0] = pathT2->size[0] + pathT1->size[0];
        path->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)path, i4, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        for (i4 = 0; i4 < 10; i4++) {
          ixstart = pathT2->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[absb + path->size[0] * i4] = pathT2->data[absb +
              pathT2->size[0] * i4];
          }
        }

        for (i4 = 0; i4 < 10; i4++) {
          ixstart = pathT1->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[(absb + pathT2->size[0]) + path->size[0] * i4] =
              pathT1->data[absb + pathT1->size[0] * i4];
          }
        }
      }

      st.site = &y_emlrtRSI;
      b_st.site = &ob_emlrtRSI;
      c_st.site = &pb_emlrtRSI;
      if (path->size[0] < 1) {
        absb = -1;
        apnd = 0;
      } else {
        ixstart = (int32_T)muDoubleScalarFloor(((real_T)path->size[0] - 1.0) +
          0.5);
        apnd = ixstart + 1;
        cdiff = (ixstart - path->size[0]) + 1;
        absb = path->size[0];
        if (muDoubleScalarAbs(cdiff) < 4.4408920985006262E-16 * (real_T)absb) {
          ixstart++;
          apnd = path->size[0];
        } else if (cdiff > 0) {
          apnd = ixstart;
        } else {
          ixstart++;
        }

        absb = ixstart - 1;
      }

      i4 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = absb + 1;
      emxEnsureCapacity(&c_st, (emxArray__common *)y, i4, (int32_T)sizeof(real_T),
                        &c_emlrtRTEI);
      if (absb + 1 > 0) {
        y->data[0] = 1.0;
        if (absb + 1 > 1) {
          y->data[absb] = apnd;
          i4 = absb + (absb < 0);
          if (i4 >= 0) {
            ixstart = (int32_T)((uint32_T)i4 >> 1);
          } else {
            ixstart = (int32_T)~(~(uint32_T)i4 >> 1);
          }

          for (cdiff = 1; cdiff < ixstart; cdiff++) {
            y->data[cdiff] = 1.0 + (real_T)cdiff;
            y->data[absb - cdiff] = apnd - cdiff;
          }

          if (ixstart << 1 == absb) {
            y->data[ixstart] = (1.0 + (real_T)apnd) / 2.0;
          } else {
            y->data[ixstart] = 1.0 + (real_T)ixstart;
            y->data[ixstart + 1] = apnd - ixstart;
          }
        }
      }

      i4 = t->size[0];
      t->size[0] = y->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)t, i4, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      ixstart = y->size[1];
      for (i4 = 0; i4 < ixstart; i4++) {
        t->data[i4] = 0.1 * y->data[y->size[0] * i4];
      }

      ixstart = t->size[0];
      i4 = path->size[0];
      emlrtDimSizeEqCheckFastR2012b(ixstart, i4, &emlrtECI, sp);
      ixstart = t->size[0];
      i4 = b_t->size[0] * b_t->size[1];
      b_t->size[0] = ixstart;
      b_t->size[1] = 1 + path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)b_t, i4, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i4 = 0; i4 < ixstart; i4++) {
        b_t->data[i4] = t->data[i4];
      }

      ixstart = path->size[1];
      for (i4 = 0; i4 < ixstart; i4++) {
        cdiff = path->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          b_t->data[absb + b_t->size[0] * (i4 + 1)] = path->data[absb +
            path->size[0] * i4];
        }
      }

      i4 = path->size[0] * path->size[1];
      path->size[0] = b_t->size[0];
      path->size[1] = b_t->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)path, i4, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      ixstart = b_t->size[1];
      for (i4 = 0; i4 < ixstart; i4++) {
        cdiff = b_t->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          path->data[absb + path->size[0] * i4] = b_t->data[absb + b_t->size[0] *
            i4];
        }
      }

      st.site = &ab_emlrtRSI;
      ixstart = path->size[0];
      i4 = b_pathC->size[0] * b_pathC->size[1];
      b_pathC->size[0] = ixstart;
      b_pathC->size[1] = 9;
      emxEnsureCapacity(&st, (emxArray__common *)b_pathC, i4, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      ixstart = path->size[0] * 9;
      for (i4 = 0; i4 < ixstart; i4++) {
        b_pathC->data[i4] = 0.0;
      }

      dist2Go = 0.0;
      cdiff = 0;
      while (cdiff <= path->size[0] - 1) {
        i4 = path->size[0];
        absb = cdiff + 1;
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &f_emlrtBCI, &st);

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
        uP[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-path->data[cdiff +
          (path->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                  kC->l5 * muDoubleScalarCos(path->data[cdiff + path->size[0] *
                   3] + kC->zeta)) - kC->l7) * muDoubleScalarCos(path->
          data[cdiff + path->size[0]]);
        uP[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-path->data[cdiff +
          (path->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                  kC->l5 * muDoubleScalarCos(path->data[cdiff + path->size[0] *
                   3] + kC->zeta)) - kC->l7) * muDoubleScalarSin(path->
          data[cdiff + path->size[0]]);
        uP[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-path->data[cdiff +
          (path->size[0] << 1)])) - kC->l4 * muDoubleScalarSin(kC->zeta)) -
                  kC->l5 * muDoubleScalarSin(path->data[cdiff + path->size[0] *
                   3] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
        i4 = path->size[0];
        absb = cdiff + 1;
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &e_emlrtBCI, &st);
        i4 = path->size[0];
        absb = cdiff + 1;
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &d_emlrtBCI, &st);

        /* sherpaTTFKVel.m */
        /* author: wreid */
        /* date: 20150122 */
        /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
        for (i4 = 0; i4 < 3; i4++) {
          nodeIDCount1 = 0.0;
          for (absb = 0; absb < 3; absb++) {
            nodeIDCount1 += TP2B[i4 + (absb << 2)] * uP[absb];
          }

          uB[i4] = nodeIDCount1 + TP2B[12 + i4];
        }

        if (1 + cdiff != 1) {
          i4 = b_pathC->size[0];
          ixstart = emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, i4, &c_emlrtBCI,
            &st);
          for (i4 = 0; i4 < 3; i4++) {
            b_uB[i4] = uB[i4] - b_pathC->data[(ixstart + b_pathC->size[0] * (2 +
              i4)) - 1];
          }

          dist2Go += norm(b_uB);
        }

        ixstart = b_pathC->size[0];
        i4 = 1 + cdiff;
        emlrtDynamicBoundsCheckFastR2012b(i4, 1, ixstart, &b_emlrtBCI, &st);
        b_path[0] = (-path->data[cdiff + path->size[0] * 6] * muDoubleScalarSin
                     (path->data[cdiff + path->size[0]]) * ((((kC->l2 - kC->l7)
          + kC->l5 * muDoubleScalarCos(path->data[cdiff + path->size[0] * 3] +
          kC->zeta)) + kC->l3 * muDoubleScalarCos(path->data[cdiff + (path->
          size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - path->
                     data[cdiff + path->size[0] * 7] * kC->l3 *
                     muDoubleScalarCos(path->data[cdiff + path->size[0]]) *
                     muDoubleScalarSin(path->data[cdiff + (path->size[0] << 1)]))
          - path->data[cdiff + (path->size[0] << 3)] * kC->l5 *
          muDoubleScalarSin(path->data[cdiff + path->size[0] * 3] + kC->zeta) *
          muDoubleScalarCos(path->data[cdiff + path->size[0]]);
        b_path[1] = (path->data[cdiff + path->size[0] * 6] * muDoubleScalarCos
                     (path->data[cdiff + path->size[0]]) * ((((kC->l2 - kC->l7)
          + kC->l5 * muDoubleScalarCos(path->data[cdiff + path->size[0] * 3] +
          kC->zeta)) + kC->l3 * muDoubleScalarCos(path->data[cdiff + (path->
          size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - path->
                     data[cdiff + (path->size[0] << 3)] * kC->l5 *
                     muDoubleScalarSin(path->data[cdiff + path->size[0] * 3] +
          kC->zeta) * muDoubleScalarSin(path->data[cdiff + path->size[0]])) -
          path->data[cdiff + path->size[0] * 7] * kC->l3 * muDoubleScalarSin
          (path->data[cdiff + path->size[0]]) * muDoubleScalarSin(path->
          data[cdiff + (path->size[0] << 1)]);
        b_path[2] = -path->data[cdiff + path->size[0] * 7] * kC->l3 *
          muDoubleScalarCos(path->data[cdiff + (path->size[0] << 1)]) - kC->l5 *
          path->data[cdiff + (path->size[0] << 3)] * muDoubleScalarCos(kC->zeta
          + path->data[cdiff + path->size[0] * 3]);
        for (i4 = 0; i4 < 3; i4++) {
          b_uB[i4] = 0.0;
          for (absb = 0; absb < 3; absb++) {
            b_uB[i4] += TP2B[i4 + (absb << 2)] * b_path[absb];
          }
        }

        i4 = path->size[0];
        absb = 1 + cdiff;
        b_pathC->data[cdiff] = path->data[emlrtDynamicBoundsCheckFastR2012b(absb,
          1, i4, &h_emlrtBCI, &st) - 1];
        b_pathC->data[cdiff + b_pathC->size[0]] = dist2Go;
        for (i4 = 0; i4 < 3; i4++) {
          b_pathC->data[cdiff + b_pathC->size[0] * (i4 + 2)] = uB[i4];
        }

        for (i4 = 0; i4 < 3; i4++) {
          b_pathC->data[cdiff + b_pathC->size[0] * (i4 + 5)] = b_uB[i4];
        }

        b_pathC->data[cdiff + (b_pathC->size[0] << 3)] = 0.0;
        cdiff++;
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
      }

      i4 = b_pathC->size[0];
      absb = b_pathC->size[0];
      emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &g_emlrtBCI, sp);
      if (b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1] <
          pathLengthMin) {
        pathLengthMin = b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1];
        i4 = pathC->size[0] * pathC->size[1];
        pathC->size[0] = b_pathC->size[0];
        pathC->size[1] = 9;
        emxEnsureCapacity(sp, (emxArray__common *)pathC, i4, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        ixstart = b_pathC->size[0] * b_pathC->size[1];
        for (i4 = 0; i4 < ixstart; i4++) {
          pathC->data[i4] = b_pathC->data[i4];
        }

        i4 = pathJ->size[0] * pathJ->size[1];
        pathJ->size[0] = path->size[0];
        pathJ->size[1] = path->size[1];
        emxEnsureCapacity(sp, (emxArray__common *)pathJ, i4, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        ixstart = path->size[0] * path->size[1];
        for (i4 = 0; i4 < ixstart; i4++) {
          pathJ->data[i4] = path->data[i4];
        }
      }
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&b_t);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&y);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&pathT2);
  emxFree_real_T(&pathT1);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildBiDirectionalRRT.c) */
