/*
 * nearestNeighbour.c
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "nearestNeighbour.h"
#include "buildRRTWrapper_mex_emxutil.h"
#include "norm.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildRRTWrapper_mex_mexutil.h"
#include "buildRRTWrapper_mex_data.h"
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

static emlrtMCInfo d_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo e_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo j_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo k_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo s_emlrtBCI = { -1, -1, 32, 34, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 31, 24, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo u_emlrtBCI = { -1, -1, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo v_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo w_emlrtBCI = { -1, -1, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRTEInfo s_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo x_emlrtBCI = { -1, -1, 14, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo y_emlrtBCI = { -1, -1, 15, 13, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo ab_emlrtBCI = { -1, -1, 16, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo bb_emlrtBCI = { -1, -1, 17, 12, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo cb_emlrtBCI = { -1, -1, 18, 16, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo db_emlrtBCI = { -1, -1, 26, 17, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 27, 16, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 28, 17, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo gb_emlrtBCI = { -1, -1, 29, 15, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo hb_emlrtBCI = { -1, -1, 30, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo ib_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRSInfo cc_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ec_emlrtRSI = { 41, "eml_min_or_max",
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
  const mxArray *m11;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m11, 1, &pArray, "message", true,
    location);
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
  int32_T i5;
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
  static const int32_T iv7[2] = { 1, 36 };

  const mxArray *m2;
  char_T cv6[36];
  static const char_T cv7[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
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
  emxInit_real_T(sp, &b_d, 2, &k_emlrtRTEI, true);

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
  i5 = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = (int32_T)nodeIDCount;
  emxEnsureCapacity(sp, (emxArray__common *)b_d, i5, (int32_T)sizeof(real_T),
                    &j_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (i5 = 0; i5 < ixstart; i5++) {
    b_d->data[i5] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &s_emlrtRTEI, sp);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    st.site = &gb_emlrtRSI;
    i5 = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(i5, 1, 2000, &w_emlrtBCI, &st);

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(4, 1, i5, &x_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i5, &y_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i5, &ab_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(7, 1, i5, &bb_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(8, 1, i5, &cb_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(9, 1, i5, &db_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(10, 1, i5, &eb_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(11, 1, i5, &fb_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(12, 1, i5, &gb_emlrtBCI, &st);
    i5 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(13, 1, i5, &hb_emlrtBCI, &st);

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
    for (i5 = 0; i5 < 3; i5++) {
      b_uB[i5] = uB[i5] - uA[i5];
      c_qDot[i5] = qDot[i5] - b_qDot[i5];
    }

    i5 = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, i5, &ib_emlrtBCI,
      sp) - 1] = norm(b_uB) + 0.0 * b_norm(c_qDot);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  i5 = b_d->size[1];
  ix = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ix, 1, i5, &v_emlrtBCI, sp);
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
    m2 = emlrtCreateCharArray(2, iv7);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv6[ixstart] = cv7[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m2, cv6);
    emlrtAssign(&y, m2);
    d_st.site = &cc_emlrtRSI;
    e_st.site = &ec_emlrtRSI;
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
        b_check_forloop_overflow_error(&g_st);
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
        b_check_forloop_overflow_error(&g_st);
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
  i5 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i5, &t_emlrtBCI, sp);
  i5 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i5, &t_emlrtBCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 2000, &u_emlrtBCI, sp);
  xNear_size[0] = 1;
  xNear_size[1] = 13;
  for (i5 = 0; i5 < 13; i5++) {
    xNear_data[xNear_size[0] * i5] = T->data[itmp + T->size[0] * i5];
  }

  if (14 > T->size[1]) {
    i5 = 0;
    ix = 0;
  } else {
    i5 = 13;
    ix = T->size[1];
    ixstart = T->size[1];
    ix = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &s_emlrtBCI, sp);
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ix - i5;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, ixstart, (int32_T)
                    sizeof(real_T), &j_emlrtRTEI);
  ixstart = ix - i5;
  for (ix = 0; ix < ixstart; ix++) {
    transitionArray->data[transitionArray->size[0] * ix] = T->data[itmp +
      T->size[0] * (i5 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
