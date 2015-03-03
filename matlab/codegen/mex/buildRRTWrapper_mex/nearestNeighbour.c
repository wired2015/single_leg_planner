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
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo gb_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

static emlrtRSInfo hb_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ib_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo jb_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo kb_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo b_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo c_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo i_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo m_emlrtBCI = { 1, 1500, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo n_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo o_emlrtBCI = { 1, 1500, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRTEInfo n_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo p_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRSInfo yb_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo bc_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

/* Function Declarations */
static const mxArray *message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *message(const emlrtStack *sp, const mxArray *b,
  emlrtMCInfo *location)
{
  const mxArray *pArray;
  const mxArray *m10;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m10, 1, &pArray, "message", true,
    location);
}

void nearestNeighbour(const emlrtStack *sp, const real_T x[13], const real_T T
                      [139500], real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T
                      kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T
                      kC_l8, real_T kC_zeta, real_T kC_r, real_T nodeIDCount,
                      real_T xNear[13], real_T transitionArray[80], real_T *d)
{
  emxArray_real_T *b_d;
  int32_T ix;
  int32_T ixstart;
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T b_uB[3];
  real_T c_qDot[3];
  boolean_T b1;
  const mxArray *y;
  static const int32_T iv4[2] = { 1, 36 };

  const mxArray *m1;
  char_T cv2[36];
  static const char_T cv3[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b2;
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
  emxInit_real_T(sp, &b_d, 2, &i_emlrtRTEI, true);

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
  ix = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = (int32_T)nodeIDCount;
  emxEnsureCapacity(sp, (emxArray__common *)b_d, ix, (int32_T)sizeof(real_T),
                    &h_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (ix = 0; ix < ixstart; ix++) {
    b_d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &n_emlrtRTEI, sp);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    ix = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(ix, 1, 1500, &o_emlrtBCI, sp);

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
    uA[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-x[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(x[5] +
               kC_zeta)) - kC_l7) * muDoubleScalarCos(x[3]);
    uA[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-x[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(x[5] +
               kC_zeta)) - kC_l7) * muDoubleScalarSin(x[3]);
    uA[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-x[4])) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(x[5] +
               kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

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
    uB[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-T[6000 + ixstart])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(T[7500 +
               ixstart] + kC_zeta)) - kC_l7) * muDoubleScalarCos(T[4500 +
      ixstart]);
    uB[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-T[6000 + ixstart])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(T[7500 +
               ixstart] + kC_zeta)) - kC_l7) * muDoubleScalarSin(T[4500 +
      ixstart]);
    uB[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-T[6000 + ixstart])) - kC_l4 *
               muDoubleScalarSin(kC_zeta)) - kC_l5 * muDoubleScalarSin(T[7500 +
               ixstart] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

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
    qDot[0] = (-T[12000 + ixstart] * muDoubleScalarSin(T[4500 + ixstart]) *
               ((((kC_l2 - kC_l7) + kC_l5 * muDoubleScalarCos(T[7500 + ixstart]
      + kC_zeta)) + kC_l3 * muDoubleScalarCos(T[6000 + ixstart])) + kC_l4 *
                muDoubleScalarCos(kC_zeta)) - T[13500 + ixstart] * kC_l3 *
               muDoubleScalarCos(T[4500 + ixstart]) * muDoubleScalarSin(T[6000 +
                ixstart])) - T[15000 + ixstart] * kC_l5 * muDoubleScalarSin(T
      [7500 + ixstart] + kC_zeta) * muDoubleScalarCos(T[4500 + ixstart]);
    qDot[1] = (T[12000 + ixstart] * muDoubleScalarCos(T[4500 + ixstart]) *
               ((((kC_l2 - kC_l7) + kC_l5 * muDoubleScalarCos(T[7500 + ixstart]
      + kC_zeta)) + kC_l3 * muDoubleScalarCos(T[6000 + ixstart])) + kC_l4 *
                muDoubleScalarCos(kC_zeta)) - T[15000 + ixstart] * kC_l5 *
               muDoubleScalarSin(T[7500 + ixstart] + kC_zeta) *
               muDoubleScalarSin(T[4500 + ixstart])) - T[13500 + ixstart] *
      kC_l3 * muDoubleScalarSin(T[4500 + ixstart]) * muDoubleScalarSin(T[6000 +
      ixstart]);
    qDot[2] = -T[13500 + ixstart] * kC_l3 * muDoubleScalarCos(T[6000 + ixstart])
      - kC_l5 * T[15000 + ixstart] * muDoubleScalarCos(kC_zeta + T[7500 +
      ixstart]);
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
    for (ix = 0; ix < 3; ix++) {
      b_uB[ix] = uB[ix] - uA[ix];
      c_qDot[ix] = qDot[ix] - b_qDot[ix];
    }

    ix = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, ix, &p_emlrtBCI,
      sp) - 1] = norm(b_uB) + 0.0 * b_norm(c_qDot);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  ix = b_d->size[1];
  ixstart = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &n_emlrtBCI, sp);
  st.site = &fb_emlrtRSI;
  b_st.site = &gb_emlrtRSI;
  c_st.site = &hb_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b1 = true;
  } else {
    b1 = false;
  }

  if (b1) {
  } else {
    y = NULL;
    m1 = emlrtCreateCharArray(2, iv4);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv2[ixstart] = cv3[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m1, cv2);
    emlrtAssign(&y, m1);
    d_st.site = &yb_emlrtRSI;
    e_st.site = &bc_emlrtRSI;
    error(&d_st, message(&e_st, y, &b_emlrtMCI), &c_emlrtMCI);
  }

  d_st.site = &ib_emlrtRSI;
  ixstart = 1;
  *d = b_d->data[0];
  itmp = 0;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(*d)) {
      f_st.site = &kb_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b2 = false;
      } else {
        b2 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b2) {
        g_st.site = &lb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int32_T)nodeIDCount)) {
        ixstart = ix;
        if (!muDoubleScalarIsNaN(b_d->data[ix - 1])) {
          *d = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      f_st.site = &jb_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &lb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (b_d->data[ixstart] < *d) {
          *d = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1500, &m_emlrtBCI, sp);
  for (ix = 0; ix < 13; ix++) {
    xNear[ix] = T[itmp + 1500 * ix];
  }

  for (ix = 0; ix < 80; ix++) {
    transitionArray[ix] = T[itmp + 1500 * (13 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
