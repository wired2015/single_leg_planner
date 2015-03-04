/*
 * nearestNeighbour.c
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "nearestNeighbour.h"
#include "buildBiDirectionalRRT.h"
#include "sherpaTTPlanner_mex_emxutil.h"
#include "heuristicSingleLeg.h"
#include "eml_int_forloop_overflow_check.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo o_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo cb_emlrtBCI = { 1, 1500, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo db_emlrtBCI = { 1, 1500, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

/* Function Definitions */
void nearestNeighbour(const emlrtStack *sp, const real_T x[13], const real_T T
                      [139500], real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T
                      kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T
                      kC_l8, real_T kC_zeta, real_T kC_r, real_T nodeIDCount,
                      real_T xNear[13], real_T transitionArray[80], real_T *d)
{
  emxArray_real_T *b_d;
  int32_T ix;
  int32_T ixstart;
  real_T b_T[93];
  boolean_T b4;
  const mxArray *y;
  static const int32_T iv13[2] = { 1, 36 };

  const mxArray *m3;
  char_T cv8[36];
  static const char_T cv9[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b5;
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
  emxInit_real_T(sp, &b_d, 2, &h_emlrtRTEI, true);

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
                    &o_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (ix = 0; ix < ixstart; ix++) {
    b_d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &s_emlrtRTEI, sp);
  ixstart = 1;
  while (ixstart - 1 <= (int32_T)nodeIDCount - 1) {
    emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, 1500, &db_emlrtBCI, sp);
    for (ix = 0; ix < 93; ix++) {
      b_T[ix] = T[(ixstart + 1500 * ix) - 1];
    }

    ix = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &m_emlrtBCI, sp)
      - 1] = heuristicSingleLeg(x, b_T, kC_l1, kC_l2, kC_l3, kC_l4, kC_l5, kC_l6,
      kC_l7, kC_l8, kC_zeta, kC_r);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  ix = b_d->size[1];
  ixstart = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &c_emlrtBCI, sp);
  st.site = &eb_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b4 = true;
  } else {
    b4 = false;
  }

  if (b4) {
  } else {
    y = NULL;
    m3 = emlrtCreateCharArray(2, iv13);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv8[ixstart] = cv9[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m3, cv8);
    emlrtAssign(&y, m3);
    d_st.site = &lc_emlrtRSI;
    e_st.site = &nc_emlrtRSI;
    error(&d_st, message(&e_st, y, &b_emlrtMCI), &c_emlrtMCI);
  }

  d_st.site = &hb_emlrtRSI;
  ixstart = 1;
  *d = b_d->data[0];
  itmp = 0;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(*d)) {
      f_st.site = &jb_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b5 = false;
      } else {
        b5 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b5) {
        g_st.site = &kb_emlrtRSI;
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
      f_st.site = &ib_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &kb_emlrtRSI;
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
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1500, &cb_emlrtBCI, sp);
  for (ix = 0; ix < 13; ix++) {
    xNear[ix] = T[itmp + 1500 * ix];
  }

  for (ix = 0; ix < 80; ix++) {
    transitionArray[ix] = T[itmp + 1500 * (13 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
