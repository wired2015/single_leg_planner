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
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "buildRRTWrapper_mex_mexutil.h"
#include "buildRRTWrapper_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo y_emlrtRSI = { 68, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo ab_emlrtRSI = { 43, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo bb_emlrtRSI = { 26, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo cb_emlrtRSI = { 75, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo db_emlrtRSI = { 76, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo eb_emlrtRSI = { 77, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo pb_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRSInfo qb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo c_emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo d_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo e_emlrtRTEI = { 55, 9, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 68, 5, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 5, 13, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo k_emlrtBCI = { 1, 1500, 84, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtRTEInfo m_emlrtRTEI = { 56, 9, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtECInfo b_emlrtECI = { -1, 57, 47, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtDCInfo emlrtDCI = { 61, 18, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  1 };

static emlrtBCInfo l_emlrtBCI = { 1, 1500, 61, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtECInfo c_emlrtECI = { -1, 64, 27, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtECInfo d_emlrtECI = { 1, 69, 12, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

/* Function Definitions */
void buildRRT(const emlrtStack *sp, const real_T nInit[13], const real_T nGoal
              [13], const real_T jointLimits[20], real_T panHeight, const
              struct0_T *kC, const real_T uBDot[6], int32_T legNum, real_T T
              [139500], emxArray_real_T *path)
{
  boolean_T b0;
  int32_T i4;
  real_T nodeIDCount;
  real_T xRand[13];
  int32_T ndbl;
  real_T r;
  real_T check;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv2[2] = { 1, 17 };

  const mxArray *m0;
  char_T cv0[17];
  int32_T apnd;
  static const char_T cv1[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  real_T transitionArrayNearest[80];
  real_T xNear[13];
  real_T xNew[13];
  real_T next_data[93];
  emxArray_real_T *b_path;
  int32_T transitionArray_size[2];
  real_T transitionArray_data[80];
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int32_T iv3[2];
  int32_T cdiff;
  int32_T absb;
  int32_T next_size[2];
  int32_T tmp_size[2];
  int8_T tmp_data[80];
  emxArray_real_T *b_y;
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
  b0 = false;

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
  /* Variable Initialization */
  for (i4 = 0; i4 < 139500; i4++) {
    T[i4] = 0.0;
  }

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  for (i4 = 0; i4 < 13; i4++) {
    T[1500 * i4] = nInit[i4];
  }

  for (i4 = 0; i4 < 80; i4++) {
    T[1500 * (i4 + 13)] = 0.0;
  }

  /* Initialize the tree with initial state. */
  nodeIDCount = 1.0;
  for (ndbl = 0; ndbl < 1499; ndbl++) {
    st.site = &bb_emlrtRSI;
    b_st.site = &cb_emlrtRSI;

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
    c_st.site = &j_emlrtRSI;
    emlrtRandu(&r, 1);
    if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
      c_st.site = &k_emlrtRSI;
      check = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                       kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                       kC->r);
      c_st.site = &l_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
      c_st.site = &m_emlrtRSI;
      check = getXStar(&c_st, panHeight, jointLimits[4], false, kC->l1, kC->l2,
                       kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                       kC->r);
      c_st.site = &n_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
      c_st.site = &o_emlrtRSI;
      check = getXStar(&c_st, panHeight, jointLimits[3], true, kC->l1, kC->l2,
                       kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                       kC->r);
      c_st.site = &p_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else {
      check = 0.0;
      xMin = 0.0;
      y = NULL;
      m0 = emlrtCreateCharArray(2, iv2);
      for (apnd = 0; apnd < 17; apnd++) {
        cv0[apnd] = cv1[apnd];
      }

      emlrtInitCharArrayR2013a(&b_st, 17, m0, cv0);
      emlrtAssign(&y, m0);
      c_st.site = &wb_emlrtRSI;
      disp(&c_st, y, &emlrtMCI);
    }

    c_st.site = &q_emlrtRSI;
    emlrtRandu(&b_r, 1);
    b_xMin[0] = xMin + (check - xMin) * b_r;
    b_xMin[1] = 0.0;
    b_xMin[2] = panHeight;
    c_st.site = &r_emlrtRSI;
    b_sherpaTTIK(&c_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, q);
    c_st.site = &s_emlrtRSI;
    emlrtRandu(&b_r, 1);
    c_st.site = &t_emlrtRSI;
    emlrtRandu(&check, 1);
    c_st.site = &u_emlrtRSI;
    emlrtRandu(&xMin, 1);

    /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
    for (i4 = 0; i4 < 3; i4++) {
      xRand[i4] = 0.0;
    }

    xRand[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
    xRand[4] = q[1];
    xRand[5] = q[2];
    xRand[6] = 0.0;
    xRand[7] = 0.0;
    xRand[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
    xRand[9] = (jointLimits[11] - jointLimits[10]) * check + jointLimits[10];
    xRand[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
    xRand[11] = 0.0;
    xRand[12] = 0.0;

    /* if mod(nodeIDCount,goalSeedFreq) == 0 */
    /*     xRand = nGoal; */
    /* end */
    b_st.site = &db_emlrtRSI;
    nearestNeighbour(&b_st, xRand, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                     kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount, xNear,
                     transitionArrayNearest, &check);
    b_st.site = &eb_emlrtRSI;
    selectInput(&b_st, xNear, xRand, kC, jointLimits, uBDot, legNum, xNew,
                transitionArrayNearest);
    xNew[0] = nodeIDCount + 1.0;

    /* Node ID */
    xNew[1] = xNear[0];

    /* Parent ID */
    xNew[2] = xNear[2] + heuristicSingleLeg(xNew, xNear, kC->l1, kC->l2, kC->l3,
      kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);

    /* Cost */
    i4 = (int32_T)(nodeIDCount + 1.0);
    emlrtDynamicBoundsCheckFastR2012b(i4, 1, 1500, &k_emlrtBCI, &st);
    for (i4 = 0; i4 < 13; i4++) {
      T[((int32_T)(nodeIDCount + 1.0) + 1500 * i4) - 1] = xNew[i4];
    }

    for (i4 = 0; i4 < 80; i4++) {
      T[((int32_T)(nodeIDCount + 1.0) + 1500 * (i4 + 13)) - 1] =
        transitionArrayNearest[i4];
    }

    /* Append the new node to the tree.     */
    /* if mod(nodeIDCount,100) == 0 */
    /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
    /* end */
    nodeIDCount++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Find the closest node in the tree to the goal node. */
  st.site = &ab_emlrtRSI;
  nearestNeighbour(&st, nGoal, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                   kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount, xRand,
                   transitionArrayNearest, &check);
  check = xRand[0];
  memcpy(&next_data[0], &xRand[0], 13U * sizeof(real_T));
  emxInit_real_T(sp, &b_path, 2, &g_emlrtRTEI, true);
  i4 = b_path->size[0] * b_path->size[1];
  b_path->size[0] = 0;
  b_path->size[1] = 10;
  emxEnsureCapacity(sp, (emxArray__common *)b_path, i4, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  transitionArray_size[0] = 1;
  transitionArray_size[1] = 80;
  memcpy(&transitionArray_data[0], &transitionArrayNearest[0], 80U * sizeof
         (real_T));

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(sp, &transitionPath, 2, &e_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &c_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &c_emlrtRTEI, true);
  while ((check != 0.0) && (next_data[1] != 0.0)) {
    i4 = transitionPath->size[0] * transitionPath->size[1];
    transitionPath->size[0] = 0;
    transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i4, (int32_T)
                      sizeof(real_T), &c_emlrtRTEI);
    emlrtForLoopVectorCheckR2012b(1.0, 10.0, 80.0, mxDOUBLE_CLASS, 8,
      &m_emlrtRTEI, sp);
    for (ndbl = 0; ndbl < 8; ndbl++) {
      apnd = ndbl * 10;
      if (!b0) {
        for (i4 = 0; i4 < 2; i4++) {
          iv3[i4] = 1 + 9 * i4;
        }

        b0 = true;
      }

      emlrtMatrixMatrixIndexCheckR2012b(transitionArray_size, 2, iv3, 2,
        &b_emlrtECI, sp);
      i4 = c_transitionPath->size[0] * c_transitionPath->size[1];
      c_transitionPath->size[0] = transitionPath->size[0] + 1;
      c_transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, i4, (int32_T)
                        sizeof(real_T), &c_emlrtRTEI);
      for (i4 = 0; i4 < 10; i4++) {
        cdiff = transitionPath->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          c_transitionPath->data[absb + c_transitionPath->size[0] * i4] =
            transitionPath->data[absb + transitionPath->size[0] * i4];
        }
      }

      for (i4 = 0; i4 < 10; i4++) {
        c_transitionPath->data[transitionPath->size[0] + c_transitionPath->size
          [0] * i4] = transitionArray_data[i4 + apnd];
      }

      i4 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = c_transitionPath->size[0];
      transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i4, (int32_T)
                        sizeof(real_T), &c_emlrtRTEI);
      for (i4 = 0; i4 < 10; i4++) {
        cdiff = c_transitionPath->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          transitionPath->data[absb + transitionPath->size[0] * i4] =
            c_transitionPath->data[absb + c_transitionPath->size[0] * i4];
        }
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }

    i4 = b_transitionPath->size[0] * b_transitionPath->size[1];
    b_transitionPath->size[0] = transitionPath->size[0] + b_path->size[0];
    b_transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)b_transitionPath, i4, (int32_T)
                      sizeof(real_T), &c_emlrtRTEI);
    for (i4 = 0; i4 < 10; i4++) {
      cdiff = transitionPath->size[0];
      for (absb = 0; absb < cdiff; absb++) {
        b_transitionPath->data[absb + b_transitionPath->size[0] * i4] =
          transitionPath->data[absb + transitionPath->size[0] * i4];
      }
    }

    for (i4 = 0; i4 < 10; i4++) {
      cdiff = b_path->size[0];
      for (absb = 0; absb < cdiff; absb++) {
        b_transitionPath->data[(absb + transitionPath->size[0]) +
          b_transitionPath->size[0] * i4] = b_path->data[absb + b_path->size[0] *
          i4];
      }
    }

    i4 = b_path->size[0] * b_path->size[1];
    b_path->size[0] = b_transitionPath->size[0];
    b_path->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)b_path, i4, (int32_T)sizeof(real_T),
                      &c_emlrtRTEI);
    for (i4 = 0; i4 < 10; i4++) {
      cdiff = b_transitionPath->size[0];
      for (absb = 0; absb < cdiff; absb++) {
        b_path->data[absb + b_path->size[0] * i4] = b_transitionPath->data[absb
          + b_transitionPath->size[0] * i4];
      }
    }

    check = next_data[1];
    i4 = (int32_T)emlrtIntegerCheckFastR2012b(next_data[1], &emlrtDCI, sp);
    emlrtDynamicBoundsCheckFastR2012b(i4, 1, 1500, &l_emlrtBCI, sp);
    next_size[0] = 1;
    next_size[1] = 93;
    for (i4 = 0; i4 < 93; i4++) {
      next_data[i4] = T[((int32_T)check + 1500 * i4) - 1];
    }

    check = next_data[1];
    tmp_size[0] = 1;
    tmp_size[1] = 80;
    for (i4 = 0; i4 < 80; i4++) {
      tmp_data[i4] = (int8_T)(14 + i4);
    }

    emlrtMatrixMatrixIndexCheckR2012b(next_size, 2, tmp_size, 2, &c_emlrtECI, sp);
    transitionArray_size[0] = 1;
    transitionArray_size[1] = 80;
    for (i4 = 0; i4 < 80; i4++) {
      transitionArray_data[i4] = next_data[tmp_data[i4] - 1];
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
  st.site = &y_emlrtRSI;
  b_st.site = &pb_emlrtRSI;
  c_st.site = &qb_emlrtRSI;
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

  emxInit_real_T(&c_st, &b_y, 2, &c_emlrtRTEI, true);
  i4 = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = absb + 1;
  emxEnsureCapacity(&c_st, (emxArray__common *)b_y, i4, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  if (absb + 1 > 0) {
    b_y->data[0] = 1.0;
    if (absb + 1 > 1) {
      b_y->data[absb] = apnd;
      i4 = absb + (absb < 0);
      if (i4 >= 0) {
        ndbl = (int32_T)((uint32_T)i4 >> 1);
      } else {
        ndbl = (int32_T)~(~(uint32_T)i4 >> 1);
      }

      for (cdiff = 1; cdiff < ndbl; cdiff++) {
        b_y->data[cdiff] = 1.0 + (real_T)cdiff;
        b_y->data[absb - cdiff] = apnd - cdiff;
      }

      if (ndbl << 1 == absb) {
        b_y->data[ndbl] = (1.0 + (real_T)apnd) / 2.0;
      } else {
        b_y->data[ndbl] = 1.0 + (real_T)ndbl;
        b_y->data[ndbl + 1] = apnd - ndbl;
      }
    }
  }

  b_emxInit_real_T(&c_st, &t, 1, &f_emlrtRTEI, true);
  i4 = t->size[0];
  t->size[0] = b_y->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)t, i4, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  cdiff = b_y->size[1];
  for (i4 = 0; i4 < cdiff; i4++) {
    t->data[i4] = 0.1 * b_y->data[b_y->size[0] * i4];
  }

  emxFree_real_T(&b_y);
  ndbl = t->size[0];
  i4 = b_path->size[0];
  emlrtDimSizeEqCheckFastR2012b(ndbl, i4, &d_emlrtECI, sp);
  ndbl = t->size[0];
  i4 = path->size[0] * path->size[1];
  path->size[0] = ndbl;
  path->size[1] = 11;
  emxEnsureCapacity(sp, (emxArray__common *)path, i4, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  for (i4 = 0; i4 < ndbl; i4++) {
    path->data[i4] = t->data[i4];
  }

  emxFree_real_T(&t);
  for (i4 = 0; i4 < 10; i4++) {
    cdiff = b_path->size[0];
    for (absb = 0; absb < cdiff; absb++) {
      path->data[absb + path->size[0] * (i4 + 1)] = b_path->data[absb +
        b_path->size[0] * i4];
    }
  }

  emxFree_real_T(&b_path);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
