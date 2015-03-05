/*
 * buildBiDirectionalRRT.c
 *
 * Code generation for function 'buildBiDirectionalRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "buildBiDirectionalRRT.h"
#include "sherpaTTPlanner_mex_emxutil.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "randomState.h"
#include "norm.h"
#include "flipud.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo x_emlrtRSI = { 39, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo y_emlrtRSI = { 63, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo ab_emlrtRSI = { 64, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo bb_emlrtRSI = { 72, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo cb_emlrtRSI = { 75, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo db_emlrtRSI = { 115, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo eb_emlrtRSI = { 116, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo fb_emlrtRSI = { 117, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo sb_emlrtRSI = { 283, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo b_emlrtRTEI = { 5, 32, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo d_emlrtRTEI = { 63, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 64, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 72, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 29, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 88, 17, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo j_emlrtRTEI = { 100, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo b_emlrtBCI = { 1, 500, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { 1, 500, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { 1, 500, 124, 7, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 142, 15, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { -1, -1, 140, 48, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo h_emlrtBCI = { -1, -1, 136, 50, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 136, 37, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 135, 31, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 76, 32, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo emlrtECI = { 1, 73, 20, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo l_emlrtBCI = { -1, -1, 142, 29, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo c_emlrtECI = { -1, 109, 27, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo o_emlrtBCI = { 1, 500, 106, 18, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtDCInfo emlrtDCI = { 106, 18, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtECInfo d_emlrtECI = { -1, 102, 47, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo t_emlrtRTEI = { 101, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo p_emlrtBCI = { 1, 500, 95, 25, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtDCInfo b_emlrtDCI = { 95, 25, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

/* Function Declarations */
static void traceBranch(const emlrtStack *sp, const real_T T[46500], const
  real_T midPoint_data[], emxArray_real_T *path);

/* Function Definitions */
static void traceBranch(const emlrtStack *sp, const real_T T[46500], const
  real_T midPoint_data[], emxArray_real_T *path)
{
  boolean_T b2;
  real_T check;
  real_T next_data[93];
  int32_T i15;
  int32_T transitionArray_size[2];
  real_T transitionArray_data[80];
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int32_T i;
  int32_T b_i;
  int32_T iv12[2];
  int32_T loop_ub;
  int32_T i16;
  int32_T next_size[2];
  int32_T tmp_size[2];
  int8_T tmp_data[80];
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  b2 = false;

  /* Assignn the  */
  check = midPoint_data[0];
  memcpy(&next_data[0], &midPoint_data[0], 13U * sizeof(real_T));
  i15 = path->size[0] * path->size[1];
  path->size[0] = 0;
  path->size[1] = 10;
  emxEnsureCapacity(sp, (emxArray__common *)path, i15, (int32_T)sizeof(real_T),
                    &i_emlrtRTEI);
  i15 = (int32_T)emlrtIntegerCheckFastR2012b(midPoint_data[0], &b_emlrtDCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(i15, 1, 500, &p_emlrtBCI, sp);
  transitionArray_size[0] = 1;
  transitionArray_size[1] = 80;
  for (i15 = 0; i15 < 80; i15++) {
    transitionArray_data[i15] = T[((int32_T)midPoint_data[0] + 500 * (13 + i15))
      - 1];
  }

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(sp, &transitionPath, 2, &j_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &i_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &i_emlrtRTEI, true);
  while ((check != 0.0) && (next_data[1] != 0.0)) {
    i15 = transitionPath->size[0] * transitionPath->size[1];
    transitionPath->size[0] = 0;
    transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i15, (int32_T)
                      sizeof(real_T), &i_emlrtRTEI);
    emlrtForLoopVectorCheckR2012b(1.0, 10.0, 80.0, mxDOUBLE_CLASS, 8,
      &t_emlrtRTEI, sp);
    for (i = 0; i < 8; i++) {
      b_i = i * 10;
      if (!b2) {
        for (i15 = 0; i15 < 2; i15++) {
          iv12[i15] = 1 + 9 * i15;
        }

        b2 = true;
      }

      emlrtMatrixMatrixIndexCheckR2012b(transitionArray_size, 2, iv12, 2,
        &d_emlrtECI, sp);
      i15 = c_transitionPath->size[0] * c_transitionPath->size[1];
      c_transitionPath->size[0] = transitionPath->size[0] + 1;
      c_transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, i15, (int32_T)
                        sizeof(real_T), &i_emlrtRTEI);
      for (i15 = 0; i15 < 10; i15++) {
        loop_ub = transitionPath->size[0];
        for (i16 = 0; i16 < loop_ub; i16++) {
          c_transitionPath->data[i16 + c_transitionPath->size[0] * i15] =
            transitionPath->data[i16 + transitionPath->size[0] * i15];
        }
      }

      for (i15 = 0; i15 < 10; i15++) {
        c_transitionPath->data[transitionPath->size[0] + c_transitionPath->size
          [0] * i15] = transitionArray_data[i15 + b_i];
      }

      i15 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = c_transitionPath->size[0];
      transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i15, (int32_T)
                        sizeof(real_T), &i_emlrtRTEI);
      for (i15 = 0; i15 < 10; i15++) {
        loop_ub = c_transitionPath->size[0];
        for (i16 = 0; i16 < loop_ub; i16++) {
          transitionPath->data[i16 + transitionPath->size[0] * i15] =
            c_transitionPath->data[i16 + c_transitionPath->size[0] * i15];
        }
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }

    i15 = b_transitionPath->size[0] * b_transitionPath->size[1];
    b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
    b_transitionPath->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)b_transitionPath, i15, (int32_T)
                      sizeof(real_T), &i_emlrtRTEI);
    for (i15 = 0; i15 < 10; i15++) {
      loop_ub = transitionPath->size[0];
      for (i16 = 0; i16 < loop_ub; i16++) {
        b_transitionPath->data[i16 + b_transitionPath->size[0] * i15] =
          transitionPath->data[i16 + transitionPath->size[0] * i15];
      }
    }

    for (i15 = 0; i15 < 10; i15++) {
      loop_ub = path->size[0];
      for (i16 = 0; i16 < loop_ub; i16++) {
        b_transitionPath->data[(i16 + transitionPath->size[0]) +
          b_transitionPath->size[0] * i15] = path->data[i16 + path->size[0] *
          i15];
      }
    }

    i15 = path->size[0] * path->size[1];
    path->size[0] = b_transitionPath->size[0];
    path->size[1] = 10;
    emxEnsureCapacity(sp, (emxArray__common *)path, i15, (int32_T)sizeof(real_T),
                      &i_emlrtRTEI);
    for (i15 = 0; i15 < 10; i15++) {
      loop_ub = b_transitionPath->size[0];
      for (i16 = 0; i16 < loop_ub; i16++) {
        path->data[i16 + path->size[0] * i15] = b_transitionPath->data[i16 +
          b_transitionPath->size[0] * i15];
      }
    }

    check = next_data[1];
    i15 = (int32_T)emlrtIntegerCheckFastR2012b(next_data[1], &emlrtDCI, sp);
    emlrtDynamicBoundsCheckFastR2012b(i15, 1, 500, &o_emlrtBCI, sp);
    next_size[0] = 1;
    next_size[1] = 93;
    for (i15 = 0; i15 < 93; i15++) {
      next_data[i15] = T[((int32_T)check + 500 * i15) - 1];
    }

    check = next_data[1];
    tmp_size[0] = 1;
    tmp_size[1] = 80;
    for (i15 = 0; i15 < 80; i15++) {
      tmp_data[i15] = (int8_T)(14 + i15);
    }

    emlrtMatrixMatrixIndexCheckR2012b(next_size, 2, tmp_size, 2, &c_emlrtECI, sp);
    transitionArray_size[0] = 1;
    transitionArray_size[1] = 80;
    for (i15 = 0; i15 < 80; i15++) {
      transitionArray_data[i15] = next_data[tmp_data[i15] - 1];
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildBiDirectionalRRT(sherpaTTPlanner_mexStackData *SD, const emlrtStack
  *sp, const real_T nInit[13], const real_T nGoal[13], const real_T jointLimits
  [20], real_T panHeight, const struct0_T *kC, const real_T uBDot[6], int32_T
  legNum, const real_T TP2B[16], real_T T1[46500], real_T T2[46500],
  emxArray_real_T *pathJ, emxArray_real_T *pathC)
{
  int32_T i4;
  uint32_T nodeIDCount1;
  uint32_T nodeIDCount2;
  real_T pathLengthMin;
  emxArray_real_T *d;
  int32_T i;
  real_T xRand[13];
  int32_T ixstart;
  int32_T cdiff;
  real_T b_T2[93];
  int32_T absb;
  boolean_T b0;
  const mxArray *y;
  static const int32_T iv5[2] = { 1, 36 };

  const mxArray *m2;
  char_T cv6[36];
  static const char_T cv7[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  real_T dist2Go;
  boolean_T b1;
  boolean_T exitg2;
  boolean_T b_ixstart;
  real_T b_T1[13];
  real_T transitionArray[80];
  real_T unusedU1[13];
  uint32_T nodeIDCountTemp;
  emxArray_real_T *pathT1;
  emxArray_real_T *pathT2;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_t;
  real_T b_d[500];
  boolean_T exitg1;
  int32_T apnd;
  real_T uP[3];
  real_T uB[3];
  real_T d1;
  real_T b_uB[3];
  real_T b_path[3];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &e_st;
  g_st.tls = e_st.tls;
  h_st.prev = &f_st;
  h_st.tls = f_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
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
  for (i4 = 0; i4 < 46500; i4++) {
    T1[i4] = 0.0;
  }

  for (i4 = 0; i4 < 46500; i4++) {
    T2[i4] = 0.0;
  }

  for (i4 = 0; i4 < 13; i4++) {
    T1[500 * i4] = nInit[i4];
  }

  for (i4 = 0; i4 < 80; i4++) {
    T1[500 * (i4 + 13)] = 0.0;
  }

  for (i4 = 0; i4 < 13; i4++) {
    T2[500 * i4] = nGoal[i4];
  }

  for (i4 = 0; i4 < 80; i4++) {
    T2[500 * (i4 + 13)] = 0.0;
  }

  nodeIDCount1 = 1U;
  nodeIDCount2 = 1U;
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
  emxInit_real_T(sp, &d, 2, &h_emlrtRTEI, true);
  for (i = 0; i < 998; i++) {
    st.site = &x_emlrtRSI;
    for (i4 = 0; i4 < 46500; i4++) {
      SD->u1.f0.T1[i4] = T1[i4];
    }

    b_st.site = &db_emlrtRSI;
    randomState(&b_st, jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4,
                kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);
    b_st.site = &eb_emlrtRSI;

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
    i4 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = (int32_T)nodeIDCount1;
    emxEnsureCapacity(&b_st, (emxArray__common *)d, i4, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    ixstart = (int32_T)nodeIDCount1;
    for (i4 = 0; i4 < ixstart; i4++) {
      d->data[i4] = 0.0;
    }

    /* parfor i = 1:nodeIDCount */
    emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount1, mxDOUBLE_CLASS,
      (int32_T)nodeIDCount1, &s_emlrtRTEI, &b_st);
    cdiff = 1;
    while (cdiff - 1 <= (int32_T)nodeIDCount1 - 1) {
      emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, 500, &b_emlrtBCI, &b_st);
      for (i4 = 0; i4 < 93; i4++) {
        b_T2[i4] = T1[(cdiff + 500 * i4) - 1];
      }

      i4 = d->size[1];
      d->data[emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, i4, &m_emlrtBCI, &b_st)
        - 1] = heuristicSingleLeg(xRand, b_T2, kC->l1, kC->l2, kC->l3, kC->l4,
        kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
      cdiff++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &b_st);
    }

    i4 = d->size[1];
    absb = (int32_T)nodeIDCount1;
    emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &c_emlrtBCI, &b_st);
    c_st.site = &gb_emlrtRSI;
    d_st.site = &hb_emlrtRSI;
    e_st.site = &ib_emlrtRSI;
    if (((int32_T)nodeIDCount1 == 1) || ((int32_T)nodeIDCount1 != 1)) {
      b0 = true;
    } else {
      b0 = false;
    }

    if (b0) {
    } else {
      y = NULL;
      m2 = emlrtCreateCharArray(2, iv5);
      for (cdiff = 0; cdiff < 36; cdiff++) {
        cv6[cdiff] = cv7[cdiff];
      }

      emlrtInitCharArrayR2013a(&e_st, 36, m2, cv6);
      emlrtAssign(&y, m2);
      f_st.site = &oc_emlrtRSI;
      g_st.site = &qc_emlrtRSI;
      error(&f_st, message(&g_st, y, &c_emlrtMCI), &d_emlrtMCI);
    }

    f_st.site = &jb_emlrtRSI;
    ixstart = 1;
    dist2Go = d->data[0];
    absb = 0;
    if ((int32_T)nodeIDCount1 > 1) {
      if (muDoubleScalarIsNaN(dist2Go)) {
        h_st.site = &lb_emlrtRSI;
        if (2 > (int32_T)nodeIDCount1) {
          b1 = false;
        } else {
          b1 = ((int32_T)nodeIDCount1 > 2147483646);
        }

        if (b1) {
          i_st.site = &mb_emlrtRSI;
          check_forloop_overflow_error(&i_st);
        }

        cdiff = 2;
        exitg2 = false;
        while ((!exitg2) && (cdiff <= (int32_T)nodeIDCount1)) {
          ixstart = cdiff;
          if (!muDoubleScalarIsNaN(d->data[cdiff - 1])) {
            dist2Go = d->data[cdiff - 1];
            absb = cdiff - 1;
            exitg2 = true;
          } else {
            cdiff++;
          }
        }
      }

      if (ixstart < (int32_T)nodeIDCount1) {
        h_st.site = &kb_emlrtRSI;
        if (ixstart + 1 > (int32_T)nodeIDCount1) {
          b_ixstart = false;
        } else {
          b_ixstart = ((int32_T)nodeIDCount1 > 2147483646);
        }

        if (b_ixstart) {
          i_st.site = &mb_emlrtRSI;
          check_forloop_overflow_error(&i_st);
        }

        for (cdiff = ixstart + 1; cdiff <= (int32_T)nodeIDCount1; cdiff++) {
          if (d->data[cdiff - 1] < dist2Go) {
            dist2Go = d->data[cdiff - 1];
            absb = cdiff - 1;
          }
        }
      }
    }

    /* [d,minIndex] = min(d(1:nodeIDCount)); */
    i4 = absb + 1;
    emlrtDynamicBoundsCheckFastR2012b(i4, 1, 500, &d_emlrtBCI, &b_st);
    for (i4 = 0; i4 < 13; i4++) {
      b_T1[i4] = T1[absb + 500 * i4];
    }

    b_st.site = &fb_emlrtRSI;
    selectInput(&b_st, b_T1, xRand, kC, jointLimits, uBDot, legNum, unusedU1,
                transitionArray);
    unusedU1[0] = (real_T)nodeIDCount1 + 1.0;

    /* Node ID */
    unusedU1[1] = T1[absb];

    /* Parent ID */
    for (i4 = 0; i4 < 13; i4++) {
      b_T1[i4] = T1[absb + 500 * i4];
    }

    unusedU1[2] = T1[1000 + absb] + b_heuristicSingleLeg(unusedU1, b_T1, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);

    /* Cost */
    i4 = (int32_T)(nodeIDCount1 + 1U);
    ixstart = emlrtDynamicBoundsCheckFastR2012b(i4, 1, 500, &e_emlrtBCI, &st) -
      1;
    for (i4 = 0; i4 < 13; i4++) {
      SD->u1.f0.T1[ixstart + 500 * i4] = unusedU1[i4];
    }

    for (i4 = 0; i4 < 80; i4++) {
      SD->u1.f0.T1[ixstart + 500 * (i4 + 13)] = transitionArray[i4];
    }

    /* Append the new node to the tree.     */
    /* if mod(nodeIDCount,100) == 0 */
    /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
    /* end */
    /* Swap the trees. */
    for (i4 = 0; i4 < 46500; i4++) {
      T1[i4] = T2[i4];
    }

    memcpy(&T2[0], &SD->u1.f0.T1[0], 46500U * sizeof(real_T));

    /* Swap the trees. */
    nodeIDCountTemp = nodeIDCount1;
    nodeIDCount1 = nodeIDCount2;
    nodeIDCount2 = nodeIDCountTemp + 1U;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxInit_real_T(sp, &pathT1, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &pathT2, 2, &e_emlrtRTEI, true);
  b_emxInit_real_T(sp, &t, 1, &f_emlrtRTEI, true);
  emxInit_real_T(sp, &path, 2, &g_emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathC, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &b_t, 2, &b_emlrtRTEI, true);
  for (i = 0; i < 500; i++) {
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
    for (cdiff = 0; cdiff < 500; cdiff++) {
      for (i4 = 0; i4 < 13; i4++) {
        xRand[i4] = T1[i + 500 * i4];
      }

      for (i4 = 0; i4 < 93; i4++) {
        b_T2[i4] = T2[cdiff + 500 * i4];
      }

      b_d[cdiff] = c_heuristicSingleLeg(xRand, b_T2, kC->l1, kC->l2, kC->l3,
        kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }

    ixstart = 1;
    dist2Go = b_d[0];
    absb = 0;
    if (muDoubleScalarIsNaN(b_d[0])) {
      cdiff = 2;
      exitg1 = false;
      while ((!exitg1) && (cdiff < 501)) {
        ixstart = cdiff;
        if (!muDoubleScalarIsNaN(b_d[cdiff - 1])) {
          dist2Go = b_d[cdiff - 1];
          absb = cdiff - 1;
          exitg1 = true;
        } else {
          cdiff++;
        }
      }
    }

    if (ixstart < 500) {
      while (ixstart + 1 < 501) {
        if (b_d[ixstart] < dist2Go) {
          dist2Go = b_d[ixstart];
          absb = ixstart;
        }

        ixstart++;
      }
    }

    /* [d,minIndex] = min(d(1:nodeIDCount)); */
    if (dist2Go < 0.04) {
      for (i4 = 0; i4 < 13; i4++) {
        xRand[i4] = T1[i + 500 * i4];
      }

      st.site = &y_emlrtRSI;
      traceBranch(&st, T1, xRand, pathT1);
      for (i4 = 0; i4 < 13; i4++) {
        xRand[i4] = T2[absb + 500 * i4];
      }

      st.site = &ab_emlrtRSI;
      traceBranch(&st, T2, xRand, pathT2);
      if ((T1[1500] == nInit[3]) && (T1[2000] == nInit[4]) && (T1[2500] ==
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

      st.site = &bb_emlrtRSI;
      b_st.site = &qb_emlrtRSI;
      c_st.site = &rb_emlrtRSI;
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

      d_st.site = &sb_emlrtRSI;
      i4 = d->size[0] * d->size[1];
      d->size[0] = 1;
      d->size[1] = absb + 1;
      emxEnsureCapacity(&c_st, (emxArray__common *)d, i4, (int32_T)sizeof(real_T),
                        &c_emlrtRTEI);
      if (absb + 1 > 0) {
        d->data[0] = 1.0;
        if (absb + 1 > 1) {
          d->data[absb] = apnd;
          i4 = absb + (absb < 0);
          if (i4 >= 0) {
            ixstart = (int32_T)((uint32_T)i4 >> 1);
          } else {
            ixstart = (int32_T)~(~(uint32_T)i4 >> 1);
          }

          for (cdiff = 1; cdiff < ixstart; cdiff++) {
            d->data[cdiff] = 1.0 + (real_T)cdiff;
            d->data[absb - cdiff] = apnd - cdiff;
          }

          if (ixstart << 1 == absb) {
            d->data[ixstart] = (1.0 + (real_T)apnd) / 2.0;
          } else {
            d->data[ixstart] = 1.0 + (real_T)ixstart;
            d->data[ixstart + 1] = apnd - ixstart;
          }
        }
      }

      i4 = t->size[0];
      t->size[0] = d->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)t, i4, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      ixstart = d->size[1];
      for (i4 = 0; i4 < ixstart; i4++) {
        t->data[i4] = 0.1 * d->data[d->size[0] * i4];
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

      st.site = &cb_emlrtRSI;
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
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &j_emlrtBCI, &st);

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
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &i_emlrtBCI, &st);
        i4 = path->size[0];
        absb = cdiff + 1;
        emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &h_emlrtBCI, &st);

        /* sherpaTTFKVel.m */
        /* author: wreid */
        /* date: 20150122 */
        /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
        for (i4 = 0; i4 < 3; i4++) {
          d1 = 0.0;
          for (absb = 0; absb < 3; absb++) {
            d1 += TP2B[i4 + (absb << 2)] * uP[absb];
          }

          uB[i4] = d1 + TP2B[12 + i4];
        }

        if (1 + cdiff != 1) {
          i4 = b_pathC->size[0];
          ixstart = emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, i4, &g_emlrtBCI,
            &st);
          for (i4 = 0; i4 < 3; i4++) {
            b_uB[i4] = uB[i4] - b_pathC->data[(ixstart + b_pathC->size[0] * (2 +
              i4)) - 1];
          }

          dist2Go += norm(b_uB);
        }

        ixstart = b_pathC->size[0];
        i4 = 1 + cdiff;
        emlrtDynamicBoundsCheckFastR2012b(i4, 1, ixstart, &f_emlrtBCI, &st);
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
          1, i4, &l_emlrtBCI, &st) - 1];
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
      emlrtDynamicBoundsCheckFastR2012b(absb, 1, i4, &k_emlrtBCI, sp);
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
  emxFree_real_T(&d);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&pathT2);
  emxFree_real_T(&pathT1);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildBiDirectionalRRT.c) */
