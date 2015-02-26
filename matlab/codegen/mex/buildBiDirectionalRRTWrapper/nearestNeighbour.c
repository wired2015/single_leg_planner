/*
 * nearestNeighbour.c
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "nearestNeighbour.h"
#include "buildBiDirectionalRRTWrapper_emxutil.h"
#include "norm.h"
#include "eml_int_forloop_overflow_check.h"
#include "heuristicSingleLeg.h"
#include "buildBiDirectionalRRTWrapper_mexutil.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo fb_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo gb_emlrtRSI = { 26, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo hb_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

static emlrtRSInfo ib_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo jb_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo kb_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo lb_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo mb_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtMCInfo d_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo e_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo j_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo h_emlrtBCI = { -1, -1, 32, 34, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 31, 24, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo l_emlrtBCI = { -1, -1, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRTEInfo u_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo w_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRSInfo vb_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo xb_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

/* Function Declarations */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *b_message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m8;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m8, 1, &pArray, "message", true, location);
}

void b_nearestNeighbour(const emlrtStack *sp, const real_T x_data[], const
  emxArray_real_T *T, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4,
  real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T
  kC_r, int32_T NODE_SIZE, real_T xNear_data[], int32_T xNear_size[2],
  emxArray_real_T *transitionArray, real_T *d)
{
  real_T d_data[1000];
  emxArray_real_T *b_T;
  int32_T ixstart;
  int32_T loop_ub;
  int32_T i8;
  emxArray_real_T *b_d;
  real_T mtmp;
  int32_T itmp;
  int32_T ix;
  emxArray_real_T *c_d;
  emxArray_real_T *d_d;
  boolean_T exitg1;
  emxArray_real_T *e_d;
  emxArray_real_T *f_d;
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
  emxInit_real_T(sp, &b_T, 2, &i_emlrtRTEI, true);
  for (ixstart = 0; ixstart < 1000; ixstart++) {
    loop_ub = T->size[1];
    i8 = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)b_T, i8, (int32_T)sizeof(real_T),
                      &i_emlrtRTEI);
    for (i8 = 0; i8 < loop_ub; i8++) {
      b_T->data[b_T->size[0] * i8] = T->data[ixstart + T->size[0] * i8];
    }

    st.site = &gb_emlrtRSI;
    d_data[ixstart] = heuristicSingleLeg(&st, x_data, b_T, kC_l1, kC_l2, kC_l3,
      kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&b_T);
  st.site = &fb_emlrtRSI;
  b_st.site = &hb_emlrtRSI;
  c_st.site = &ib_emlrtRSI;
  emxInit_real_T(&c_st, &b_d, 2, &i_emlrtRTEI, true);
  d_st.site = &jb_emlrtRSI;
  ixstart = 1;
  i8 = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = 1000;
  emxEnsureCapacity(&d_st, (emxArray__common *)b_d, i8, (int32_T)sizeof(real_T),
                    &i_emlrtRTEI);
  for (i8 = 0; i8 < 1000; i8++) {
    b_d->data[b_d->size[0] * i8] = d_data[i8];
  }

  mtmp = b_d->data[0];
  itmp = 0;
  emxFree_real_T(&b_d);
  if (muDoubleScalarIsNaN(mtmp)) {
    ix = 1;
    emxInit_real_T(&d_st, &c_d, 2, &i_emlrtRTEI, true);
    emxInit_real_T(&d_st, &d_d, 2, &i_emlrtRTEI, true);
    exitg1 = false;
    while ((!exitg1) && (ix + 1 <= 1000)) {
      ixstart = ix + 1;
      i8 = c_d->size[0] * c_d->size[1];
      c_d->size[0] = 1;
      c_d->size[1] = 1000;
      emxEnsureCapacity(&d_st, (emxArray__common *)c_d, i8, (int32_T)sizeof
                        (real_T), &i_emlrtRTEI);
      for (i8 = 0; i8 < 1000; i8++) {
        c_d->data[c_d->size[0] * i8] = d_data[i8];
      }

      if (!muDoubleScalarIsNaN(c_d->data[ix])) {
        i8 = d_d->size[0] * d_d->size[1];
        d_d->size[0] = 1;
        d_d->size[1] = 1000;
        emxEnsureCapacity(&d_st, (emxArray__common *)d_d, i8, (int32_T)sizeof
                          (real_T), &i_emlrtRTEI);
        for (i8 = 0; i8 < 1000; i8++) {
          d_d->data[d_d->size[0] * i8] = d_data[i8];
        }

        mtmp = d_d->data[ix];
        itmp = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }

    emxFree_real_T(&d_d);
    emxFree_real_T(&c_d);
  }

  if (ixstart < 1000) {
    emxInit_real_T(&d_st, &e_d, 2, &i_emlrtRTEI, true);
    emxInit_real_T(&d_st, &f_d, 2, &i_emlrtRTEI, true);
    while (ixstart + 1 <= 1000) {
      i8 = e_d->size[0] * e_d->size[1];
      e_d->size[0] = 1;
      e_d->size[1] = 1000;
      emxEnsureCapacity(&d_st, (emxArray__common *)e_d, i8, (int32_T)sizeof
                        (real_T), &i_emlrtRTEI);
      for (i8 = 0; i8 < 1000; i8++) {
        e_d->data[e_d->size[0] * i8] = d_data[i8];
      }

      if (e_d->data[ixstart] < mtmp) {
        i8 = f_d->size[0] * f_d->size[1];
        f_d->size[0] = 1;
        f_d->size[1] = 1000;
        emxEnsureCapacity(&d_st, (emxArray__common *)f_d, i8, (int32_T)sizeof
                          (real_T), &i_emlrtRTEI);
        for (i8 = 0; i8 < 1000; i8++) {
          f_d->data[f_d->size[0] * i8] = d_data[i8];
        }

        mtmp = f_d->data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }

    emxFree_real_T(&f_d);
    emxFree_real_T(&e_d);
  }

  *d = mtmp;

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  i8 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i8, &i_emlrtBCI, sp);
  i8 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i8, &i_emlrtBCI, sp);
  xNear_size[0] = 1;
  xNear_size[1] = 13;
  for (i8 = 0; i8 < 13; i8++) {
    xNear_data[xNear_size[0] * i8] = T->data[itmp + T->size[0] * i8];
  }

  if (14 > T->size[1]) {
    i8 = 0;
    ix = 0;
  } else {
    i8 = 13;
    ix = T->size[1];
    ixstart = T->size[1];
    ix = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &h_emlrtBCI, sp);
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ix - i8;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, ixstart, (int32_T)
                    sizeof(real_T), &i_emlrtRTEI);
  loop_ub = ix - i8;
  for (ix = 0; ix < loop_ub; ix++) {
    transitionArray->data[transitionArray->size[0] * ix] = T->data[itmp +
      T->size[0] * (i8 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void nearestNeighbour(const emlrtStack *sp, const real_T x[13], const
                      emxArray_real_T *T, real_T kC_l1, real_T kC_l2, real_T
                      kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T
                      kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, real_T
                      nodeIDCount, int32_T NODE_SIZE, real_T xNear_data[],
                      int32_T xNear_size[2], emxArray_real_T *transitionArray,
                      real_T *d)
{
  emxArray_real_T *b_d;
  int32_T i4;
  int32_T ixstart;
  real_T uA[3];
  real_T mtmp;
  real_T q_idx_1;
  real_T q_idx_2;
  real_T uB[3];
  real_T qDot_idx_0;
  real_T qDot_idx_1;
  real_T qDot_idx_2;
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];
  int32_T ix;
  boolean_T b0;
  const mxArray *y;
  static const int32_T iv6[2] = { 1, 36 };

  const mxArray *m1;
  char_T cv2[36];
  static const char_T cv3[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b1;
  boolean_T exitg1;
  boolean_T b_ixstart;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &c_st;
  e_st.tls = c_st.tls;
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &b_d, 2, &j_emlrtRTEI, true);

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
  i4 = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = (int32_T)nodeIDCount;
  emxEnsureCapacity(sp, (emxArray__common *)b_d, i4, (int32_T)sizeof(real_T),
                    &i_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (i4 = 0; i4 < ixstart; i4++) {
    b_d->data[i4] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &u_emlrtRTEI, sp);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    st.site = &gb_emlrtRSI;
    i4 = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(i4, 1, 1000, &l_emlrtBCI, &st);

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(4, 1, i4, &m_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i4, &n_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i4, &o_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(7, 1, i4, &p_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(8, 1, i4, &q_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(9, 1, i4, &r_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(10, 1, i4, &s_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(11, 1, i4, &t_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(12, 1, i4, &u_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(13, 1, i4, &v_emlrtBCI, &st);

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
    uA[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-x[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(x[5] +
               kC_zeta)) - kC_l7) * muDoubleScalarCos(x[3]);
    uA[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-x[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(x[5] +
               kC_zeta)) - kC_l7) * muDoubleScalarSin(x[3]);
    uA[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-x[4])) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(x[5] +
               kC_zeta)) - kC_l6) - (kC_l8 + kC_r);
    mtmp = T->data[ixstart + T->size[0] * 3];
    q_idx_1 = T->data[ixstart + (T->size[0] << 2)];
    q_idx_2 = T->data[ixstart + T->size[0] * 5];

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
    uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q_idx_1)) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q_idx_2 +
               kC_zeta)) - kC_l7) * muDoubleScalarCos(mtmp);
    uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-q_idx_1)) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(q_idx_2 +
               kC_zeta)) - kC_l7) * muDoubleScalarSin(mtmp);
    uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-q_idx_1)) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(q_idx_2 +
               kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    qDot_idx_0 = T->data[ixstart + (T->size[0] << 3)];
    qDot_idx_1 = T->data[ixstart + T->size[0] * 9];
    qDot_idx_2 = T->data[ixstart + T->size[0] * 10];
    mtmp = T->data[ixstart + T->size[0] * 3];
    q_idx_1 = T->data[ixstart + (T->size[0] << 2)];
    q_idx_2 = T->data[ixstart + T->size[0] * 5];

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
    qDot[0] = (-qDot_idx_0 * muDoubleScalarSin(mtmp) * ((((kC_l2 - kC_l7) +
      kC_l5 * muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos
      (q_idx_1)) + kC_l4 * muDoubleScalarCos(kC_zeta)) - qDot_idx_1 * kC_l3 *
               muDoubleScalarCos(mtmp) * muDoubleScalarSin(q_idx_1)) -
      qDot_idx_2 * kC_l5 * muDoubleScalarSin(q_idx_2 + kC_zeta) *
      muDoubleScalarCos(mtmp);
    qDot[1] = (qDot_idx_0 * muDoubleScalarCos(mtmp) * ((((kC_l2 - kC_l7) + kC_l5
      * muDoubleScalarCos(q_idx_2 + kC_zeta)) + kC_l3 * muDoubleScalarCos
      (q_idx_1)) + kC_l4 * muDoubleScalarCos(kC_zeta)) - qDot_idx_2 * kC_l5 *
               muDoubleScalarSin(q_idx_2 + kC_zeta) * muDoubleScalarSin(mtmp)) -
      qDot_idx_1 * kC_l3 * muDoubleScalarSin(mtmp) * muDoubleScalarSin(q_idx_1);
    qDot[2] = -qDot_idx_1 * kC_l3 * muDoubleScalarCos(q_idx_1) - kC_l5 *
      qDot_idx_2 * muDoubleScalarCos(kC_zeta + q_idx_2);
    b_qDot[0] = (-x[8] * muDoubleScalarSin(x[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
      muDoubleScalarCos(x[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(x[4])) +
      kC_l4 * muDoubleScalarCos(kC_zeta)) - x[9] * kC_l3 * muDoubleScalarCos(x[3])
                 * muDoubleScalarSin(x[4])) - x[10] * kC_l5 * muDoubleScalarSin
      (x[5] + kC_zeta) * muDoubleScalarCos(x[3]);
    b_qDot[1] = (x[8] * muDoubleScalarCos(x[3]) * ((((kC_l2 - kC_l7) + kC_l5 *
      muDoubleScalarCos(x[5] + kC_zeta)) + kC_l3 * muDoubleScalarCos(x[4])) +
      kC_l4 * muDoubleScalarCos(kC_zeta)) - x[10] * kC_l5 * muDoubleScalarSin(x
      [5] + kC_zeta) * muDoubleScalarSin(x[3])) - x[9] * kC_l3 *
      muDoubleScalarSin(x[3]) * muDoubleScalarSin(x[4]);
    b_qDot[2] = -x[9] * kC_l3 * muDoubleScalarCos(x[4]) - kC_l5 * x[10] *
      muDoubleScalarCos(kC_zeta + x[5]);
    for (i4 = 0; i4 < 3; i4++) {
      b_uB[i4] = uB[i4] - uA[i4];
      c_qDot[i4] = qDot[i4] - b_qDot[i4];
    }

    i4 = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, i4, &w_emlrtBCI,
      sp) - 1] = norm(b_uB) + 0.0 * b_norm(c_qDot);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  i4 = b_d->size[1];
  ix = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ix, 1, i4, &k_emlrtBCI, sp);
  st.site = &fb_emlrtRSI;
  b_st.site = &hb_emlrtRSI;
  c_st.site = &ib_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b0 = true;
  } else {
    b0 = false;
  }

  if (b0) {
  } else {
    y = NULL;
    m1 = emlrtCreateCharArray(2, iv6);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv2[ixstart] = cv3[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m1, cv2);
    emlrtAssign(&y, m1);
    d_st.site = &vb_emlrtRSI;
    e_st.site = &xb_emlrtRSI;
    error(&d_st, b_message(&e_st, y, &d_emlrtMCI), &e_emlrtMCI);
  }

  d_st.site = &jb_emlrtRSI;
  ixstart = 1;
  mtmp = b_d->data[0];
  itmp = 0;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(mtmp)) {
      f_st.site = &lb_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b1 = false;
      } else {
        b1 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b1) {
        g_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int32_T)nodeIDCount)) {
        ixstart = ix;
        if (!muDoubleScalarIsNaN(b_d->data[ix - 1])) {
          mtmp = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      f_st.site = &kb_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &mb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (b_d->data[ixstart] < mtmp) {
          mtmp = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = mtmp;

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  i4 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i4, &i_emlrtBCI, sp);
  i4 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i4, &i_emlrtBCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1000, &j_emlrtBCI, sp);
  xNear_size[0] = 1;
  xNear_size[1] = 13;
  for (i4 = 0; i4 < 13; i4++) {
    xNear_data[xNear_size[0] * i4] = T->data[itmp + T->size[0] * i4];
  }

  if (14 > T->size[1]) {
    i4 = 0;
    ix = 0;
  } else {
    i4 = 13;
    ix = T->size[1];
    ixstart = T->size[1];
    ix = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &h_emlrtBCI, sp);
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ix - i4;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, ixstart, (int32_T)
                    sizeof(real_T), &i_emlrtRTEI);
  ixstart = ix - i4;
  for (ix = 0; ix < ixstart; ix++) {
    transitionArray->data[transitionArray->size[0] * ix] = T->data[itmp +
      T->size[0] * (i4 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
