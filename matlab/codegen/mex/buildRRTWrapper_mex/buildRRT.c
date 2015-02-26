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
#include "buildRRTWrapper_mex_emxutil.h"
#include "norm.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "buildRRTWrapper_mex_mexutil.h"
#include "buildRRTWrapper_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = { 66, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo i_emlrtRSI = { 43, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 26, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 73, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo l_emlrtRSI = { 74, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 75, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo qb_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRSInfo rb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo b_emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo d_emlrtRTEI = { 46, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 48, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 53, 9, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 66, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 5, 13, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtECInfo emlrtECI = { 1, 67, 12, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo h_emlrtBCI = { -1, -1, 62, 27, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 60, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 59, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtDCInfo emlrtDCI = { 59, 18, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  1 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 59, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtRTEInfo q_emlrtRTEI = { 54, 9, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo l_emlrtBCI = { -1, -1, 51, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { -1, 19, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo m_emlrtBCI = { -1, -1, 82, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtECInfo c_emlrtECI = { -1, 82, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtDCInfo b_emlrtDCI = { 17, 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  4 };

static emlrtDCInfo c_emlrtDCI = { 19, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  1 };

static emlrtDCInfo d_emlrtDCI = { 19, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  4 };

static emlrtBCInfo n_emlrtBCI = { -1, -1, 55, 47, "transitionArray", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

/* Function Definitions */
void buildRRT(const emlrtStack *sp, const real_T nInit[13], const real_T nGoal
              [13], const real_T jointLimits[20], real_T panHeight, const real_T
              U[18], real_T dt, real_T Dt, const struct0_T *kC, const real_T
              uBDot[6], int32_T legNum, emxArray_real_T *T, emxArray_real_T
              *path)
{
  real_T transitionArrayLength;
  int32_T i3;
  real_T xMin;
  int32_T cdiff;
  int32_T ndbl;
  emxArray_int32_T *r0;
  emxArray_real_T *next;
  int32_T iv2[2];
  real_T nodeIDCount;
  real_T xRand[13];
  emxArray_real_T *transitionArray;
  int32_T apnd;
  real_T r;
  const mxArray *y;
  static const int32_T iv3[2] = { 1, 17 };

  const mxArray *m0;
  char_T cv0[17];
  static const char_T cv1[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  real_T unusedU5;
  int32_T xNearest_size[2];
  real_T xNearest_data[13];
  int32_T xNew_size[2];
  real_T xNew_data[13];
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T c_qDot[3];
  int32_T iv4[2];
  real_T unusedU2;
  emxArray_real_T *b_path;
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  boolean_T exitg1;
  uint32_T i;
  int32_T absb;
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
  T->size[0] = 2000;
  xMin = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (xMin < 2.147483648E+9) {
    if (xMin >= -2.147483648E+9) {
      cdiff = (int32_T)xMin;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (xMin >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(cdiff, &b_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  xMin = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (xMin < 2.147483648E+9) {
    if (xMin >= -2.147483648E+9) {
      i3 = (int32_T)xMin;
    } else {
      i3 = MIN_int32_T;
    }
  } else if (xMin >= 2.147483648E+9) {
    i3 = MAX_int32_T;
  } else {
    i3 = 0;
  }

  ndbl = 2000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i3, &b_emlrtDCI, sp);
  for (i3 = 0; i3 < ndbl; i3++) {
    T->data[i3] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &b_emlrtRTEI, true);

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  xMin = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (xMin < 2.147483648E+9) {
    if (xMin >= -2.147483648E+9) {
      ndbl = (int32_T)xMin;
    } else {
      ndbl = MIN_int32_T;
    }
  } else if (xMin >= 2.147483648E+9) {
    ndbl = MAX_int32_T;
  } else {
    ndbl = 0;
  }

  i3 = r0->size[0];
  xMin = muDoubleScalarRound(13.0 + transitionArrayLength);
  if (xMin < 2.147483648E+9) {
    if (xMin >= -2.147483648E+9) {
      cdiff = (int32_T)xMin;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (xMin >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  r0->size[0] = cdiff;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < ndbl; i3++) {
    r0->data[i3] = i3;
  }

  emxInit_real_T(sp, &next, 2, &d_emlrtRTEI, true);
  xMin = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &d_emlrtDCI, sp);
  emlrtIntegerCheckFastR2012b(xMin, &c_emlrtDCI, sp);
  i3 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 13 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 13; i3++) {
    next->data[next->size[0] * i3] = nInit[i3];
  }

  ndbl = (int32_T)transitionArrayLength;
  for (i3 = 0; i3 < ndbl; i3++) {
    next->data[next->size[0] * (i3 + 13)] = 0.0;
  }

  iv2[0] = 1;
  iv2[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv2, 2, *(int32_T (*)[2])next->size, 2,
    &b_emlrtECI, sp);
  ndbl = next->size[1];
  for (i3 = 0; i3 < ndbl; i3++) {
    T->data[T->size[0] * r0->data[i3]] = next->data[next->size[0] * i3];
  }

  /* Initialize the tree with initial state. */
  nodeIDCount = 1.0;
  emxInit_real_T(sp, &transitionArray, 2, &e_emlrtRTEI, true);
  for (apnd = 0; apnd < 1999; apnd++) {
    st.site = &j_emlrtRSI;
    b_st.site = &k_emlrtRSI;

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
    c_st.site = &n_emlrtRSI;
    emlrtRandu(&r, 1);
    if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
      c_st.site = &o_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[4], false,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &p_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
      c_st.site = &q_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[4], false,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &r_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
      c_st.site = &s_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[3], true,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &t_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else {
      transitionArrayLength = 0.0;
      xMin = 0.0;
      y = NULL;
      m0 = emlrtCreateCharArray(2, iv3);
      for (ndbl = 0; ndbl < 17; ndbl++) {
        cv0[ndbl] = cv1[ndbl];
      }

      emlrtInitCharArrayR2013a(&b_st, 17, m0, cv0);
      emlrtAssign(&y, m0);
      c_st.site = &xb_emlrtRSI;
      disp(&c_st, y, &c_emlrtMCI);
    }

    c_st.site = &u_emlrtRSI;
    emlrtRandu(&b_r, 1);
    b_xMin[0] = xMin + (transitionArrayLength - xMin) * b_r;
    b_xMin[1] = 0.0;
    b_xMin[2] = panHeight;
    c_st.site = &v_emlrtRSI;
    b_sherpaTTIK(&c_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, q);
    c_st.site = &w_emlrtRSI;
    emlrtRandu(&b_r, 1);
    c_st.site = &x_emlrtRSI;
    emlrtRandu(&transitionArrayLength, 1);
    c_st.site = &y_emlrtRSI;
    emlrtRandu(&xMin, 1);

    /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
    for (i3 = 0; i3 < 3; i3++) {
      xRand[i3] = 0.0;
    }

    xRand[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
    xRand[4] = q[1];
    xRand[5] = q[2];
    xRand[6] = 0.0;
    xRand[7] = 0.0;
    xRand[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
    xRand[9] = (jointLimits[11] - jointLimits[10]) * transitionArrayLength +
      jointLimits[10];
    xRand[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
    xRand[11] = 0.0;
    xRand[12] = 0.0;

    /* if mod(nodeIDCount,goalSeedFreq) == 0 */
    /*     xRand = nGoal; */
    /* end */
    b_st.site = &l_emlrtRSI;
    nearestNeighbour(&b_st, xRand, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                     kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount, 13,
                     xNearest_data, xNearest_size, next, &unusedU5);
    b_st.site = &m_emlrtRSI;
    selectInput(&b_st, xNearest_data, xRand, U, dt, Dt, kC, jointLimits, uBDot,
                legNum, xNew_data, xNew_size, transitionArray);
    xNew_data[0] = nodeIDCount + 1.0;

    /* Node ID */
    xNew_data[1] = xNearest_data[0];

    /* Parent ID */
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
    uA[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNew_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (xNew_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos
      (xNew_data[3]);
    uA[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNew_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (xNew_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin
      (xNew_data[3]);
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
    uB[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNearest_data[4])) + kC->l4
               * muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (xNearest_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos
      (xNearest_data[3]);
    uB[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-xNearest_data[4])) + kC->l4
               * muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (xNearest_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin
      (xNearest_data[3]);
    uB[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-xNearest_data[4])) - kC->l4
               * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
              (xNearest_data[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    qDot[0] = (-xNearest_data[8] * muDoubleScalarSin(xNearest_data[3]) *
               ((((kC->l2 - kC->l7) + kC->l5 * muDoubleScalarCos(xNearest_data[5]
      + kC->zeta)) + kC->l3 * muDoubleScalarCos(xNearest_data[4])) + kC->l4 *
                muDoubleScalarCos(kC->zeta)) - xNearest_data[9] * kC->l3 *
               muDoubleScalarCos(xNearest_data[3]) * muDoubleScalarSin
               (xNearest_data[4])) - xNearest_data[10] * kC->l5 *
      muDoubleScalarSin(xNearest_data[5] + kC->zeta) * muDoubleScalarCos
      (xNearest_data[3]);
    qDot[1] = (xNearest_data[8] * muDoubleScalarCos(xNearest_data[3]) *
               ((((kC->l2 - kC->l7) + kC->l5 * muDoubleScalarCos(xNearest_data[5]
      + kC->zeta)) + kC->l3 * muDoubleScalarCos(xNearest_data[4])) + kC->l4 *
                muDoubleScalarCos(kC->zeta)) - xNearest_data[10] * kC->l5 *
               muDoubleScalarSin(xNearest_data[5] + kC->zeta) *
               muDoubleScalarSin(xNearest_data[3])) - xNearest_data[9] * kC->l3 *
      muDoubleScalarSin(xNearest_data[3]) * muDoubleScalarSin(xNearest_data[4]);
    qDot[2] = -xNearest_data[9] * kC->l3 * muDoubleScalarCos(xNearest_data[4]) -
      kC->l5 * xNearest_data[10] * muDoubleScalarCos(kC->zeta + xNearest_data[5]);
    b_qDot[0] = (-xNew_data[8] * muDoubleScalarSin(xNew_data[3]) * ((((kC->l2 -
      kC->l7) + kC->l5 * muDoubleScalarCos(xNew_data[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(xNew_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta)) -
                 xNew_data[9] * kC->l3 * muDoubleScalarCos(xNew_data[3]) *
                 muDoubleScalarSin(xNew_data[4])) - xNew_data[10] * kC->l5 *
      muDoubleScalarSin(xNew_data[5] + kC->zeta) * muDoubleScalarCos(xNew_data[3]);
    b_qDot[1] = (xNew_data[8] * muDoubleScalarCos(xNew_data[3]) * ((((kC->l2 -
      kC->l7) + kC->l5 * muDoubleScalarCos(xNew_data[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(xNew_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta)) -
                 xNew_data[10] * kC->l5 * muDoubleScalarSin(xNew_data[5] +
      kC->zeta) * muDoubleScalarSin(xNew_data[3])) - xNew_data[9] * kC->l3 *
      muDoubleScalarSin(xNew_data[3]) * muDoubleScalarSin(xNew_data[4]);
    b_qDot[2] = -xNew_data[9] * kC->l3 * muDoubleScalarCos(xNew_data[4]) -
      kC->l5 * xNew_data[10] * muDoubleScalarCos(kC->zeta + xNew_data[5]);
    for (i3 = 0; i3 < 3; i3++) {
      q[i3] = uB[i3] - uA[i3];
      c_qDot[i3] = qDot[i3] - b_qDot[i3];
    }

    xNew_data[2] = xNearest_data[2] + (norm(q) + 0.0 * b_norm(c_qDot));

    /* Cost */
    i3 = (int32_T)(nodeIDCount + 1.0);
    emlrtDynamicBoundsCheckFastR2012b(i3, 1, 2000, &m_emlrtBCI, &st);
    ndbl = T->size[1];
    i3 = r0->size[0];
    r0->size[0] = ndbl;
    emxEnsureCapacity(&st, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                      &b_emlrtRTEI);
    for (i3 = 0; i3 < ndbl; i3++) {
      r0->data[i3] = i3;
    }

    i3 = next->size[0] * next->size[1];
    next->size[0] = 1;
    next->size[1] = 13 + transitionArray->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    for (i3 = 0; i3 < 13; i3++) {
      next->data[next->size[0] * i3] = xNew_data[xNew_size[0] * i3];
    }

    ndbl = transitionArray->size[1];
    for (i3 = 0; i3 < ndbl; i3++) {
      next->data[next->size[0] * (i3 + 13)] = transitionArray->
        data[transitionArray->size[0] * i3];
    }

    iv4[0] = 1;
    iv4[1] = r0->size[0];
    emlrtSubAssignSizeCheckR2012b(iv4, 2, *(int32_T (*)[2])next->size, 2,
      &c_emlrtECI, &st);
    ndbl = next->size[1];
    for (i3 = 0; i3 < ndbl; i3++) {
      T->data[((int32_T)(nodeIDCount + 1.0) + T->size[0] * r0->data[i3]) - 1] =
        next->data[next->size[0] * i3];
    }

    /* Append the new node to the tree.     */
    /* if mod(nodeIDCount,100) == 0 */
    /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
    /* end */
    nodeIDCount++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r0);

  /* Find the closest node in the tree to the goal node. */
  st.site = &i_emlrtRSI;
  nearestNeighbour(&st, nGoal, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                   kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount, 13,
                   xNearest_data, xNearest_size, transitionArray, &unusedU2);
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
        mxDOUBLE_CLASS, i3, &q_emlrtRTEI, sp);
      apnd = 0;
      while (apnd <= i3 - 1) {
        i = apnd * 10U + 1U;
        cdiff = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, cdiff,
                          (int32_T)sizeof(real_T), &b_emlrtRTEI);
        for (cdiff = 0; cdiff < 10; cdiff++) {
          ndbl = transitionPath->size[0];
          for (absb = 0; absb < ndbl; absb++) {
            c_transitionPath->data[absb + c_transitionPath->size[0] * cdiff] =
              transitionPath->data[absb + transitionPath->size[0] * cdiff];
          }
        }

        for (cdiff = 0; cdiff < 10; cdiff++) {
          absb = transitionArray->size[1];
          ndbl = (int32_T)(cdiff + i);
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * cdiff] = transitionArray->
            data[emlrtDynamicBoundsCheckFastR2012b(ndbl, 1, absb, &n_emlrtBCI,
            sp) - 1];
        }

        cdiff = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)transitionPath, cdiff,
                          (int32_T)sizeof(real_T), &b_emlrtRTEI);
        for (cdiff = 0; cdiff < 10; cdiff++) {
          ndbl = c_transitionPath->size[0];
          for (absb = 0; absb < ndbl; absb++) {
            transitionPath->data[absb + transitionPath->size[0] * cdiff] =
              c_transitionPath->data[absb + c_transitionPath->size[0] * cdiff];
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
        ndbl = transitionPath->size[0];
        for (cdiff = 0; cdiff < ndbl; cdiff++) {
          b_transitionPath->data[cdiff + b_transitionPath->size[0] * i3] =
            transitionPath->data[cdiff + transitionPath->size[0] * i3];
        }
      }

      for (i3 = 0; i3 < 10; i3++) {
        ndbl = b_path->size[0];
        for (cdiff = 0; cdiff < ndbl; cdiff++) {
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
        ndbl = b_transitionPath->size[0];
        for (cdiff = 0; cdiff < ndbl; cdiff++) {
          b_path->data[cdiff + b_path->size[0] * i3] = b_transitionPath->
            data[cdiff + b_transitionPath->size[0] * i3];
        }
      }

      i3 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &k_emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      ndbl = T->size[1];
      i3 = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength, &emlrtDCI,
        sp);
      emlrtDynamicBoundsCheckFastR2012b(i3, 1, 2000, &j_emlrtBCI, sp);
      i3 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = ndbl;
      emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i3 = 0; i3 < ndbl; i3++) {
        next->data[next->size[0] * i3] = T->data[((int32_T)transitionArrayLength
          + T->size[0] * i3) - 1];
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
        absb = next->size[1];
        cdiff = emlrtDynamicBoundsCheckFastR2012b(absb, 1, cdiff, &h_emlrtBCI,
          sp);
      }

      absb = transitionArray->size[0] * transitionArray->size[1];
      transitionArray->size[0] = 1;
      transitionArray->size[1] = cdiff - i3;
      emxEnsureCapacity(sp, (emxArray__common *)transitionArray, absb, (int32_T)
                        sizeof(real_T), &b_emlrtRTEI);
      ndbl = cdiff - i3;
      for (cdiff = 0; cdiff < ndbl; cdiff++) {
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
  b_st.site = &qb_emlrtRSI;
  c_st.site = &rb_emlrtRSI;
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
  ndbl = next->size[1];
  for (i3 = 0; i3 < ndbl; i3++) {
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
    ndbl = b_path->size[0];
    for (cdiff = 0; cdiff < ndbl; cdiff++) {
      path->data[cdiff + path->size[0] * (i3 + 1)] = b_path->data[cdiff +
        b_path->size[0] * i3];
    }
  }

  emxFree_real_T(&b_path);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
