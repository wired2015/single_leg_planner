/*
 * nearestNeighbour.c
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "nearestNeighbour.h"
#include "buildRRT.h"
#include "buildRRTWrapper_emxutil.h"
#include "eml_error.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildRRTWrapper_mexutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo f_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

/* Function Definitions */
void nearestNeighbour(const emlrtStack *sp, const real_T x[11], const
                      emxArray_real_T *T, const real_T jointLimits[12], const
                      real_T kinematicConst[16], real_T nodeIDCount, int32_T
                      NODE_SIZE, real_T xNear_data[], int32_T xNear_size[2],
                      emxArray_real_T *transitionArray, real_T *d)
{
  emxArray_real_T *b_d;
  int32_T i4;
  int32_T ixstart;
  real_T xStarMin;
  real_T dxStarMax;
  real_T dAlphaMax;
  int32_T ix;
  boolean_T b1;
  const mxArray *y;
  static const int32_T iv6[2] = { 1, 36 };

  const mxArray *m1;
  char_T cv4[36];
  static const char_T cv5[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
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
  emxInit_real_T(sp, &b_d, 2, &g_emlrtRTEI, true);

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
                    &f_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (i4 = 0; i4 < ixstart; i4++) {
    b_d->data[i4] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &k_emlrtRTEI, sp);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    st.site = &u_emlrtRSI;
    i4 = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(i4, 1, 1000, &gb_emlrtBCI, &st);

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(4, 1, i4, &db_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i4, &cb_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i4, &bb_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(7, 1, i4, &ab_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(8, 1, i4, &y_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(9, 1, i4, &x_emlrtBCI, &st);

    /* Calculate the distance between angular positions. */
    xStarMin = (((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                  (jointLimits[2])) + kinematicConst[3] * muDoubleScalarCos
                 (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarCos
                (kinematicConst[8] + jointLimits[4])) - kinematicConst[6];
    dxStarMax = ((((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                    (jointLimits[3])) + kinematicConst[3] * muDoubleScalarCos
                   (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarCos
                  (kinematicConst[8] + jointLimits[5])) - kinematicConst[6]) -
      xStarMin;

    /* angDiff Finds the angular difference between th1 and th2. */
    dAlphaMax = ((jointLimits[0] - jointLimits[2]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(dAlphaMax - muDoubleScalarRound(dAlphaMax)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(dAlphaMax)) {
      dAlphaMax = 0.0;
    } else {
      dAlphaMax = (dAlphaMax - muDoubleScalarFloor(dAlphaMax)) *
        6.2831853071795862;
    }

    dAlphaMax = muDoubleScalarAbs(dAlphaMax - 3.1415926535897931);
    b_st.site = &v_emlrtRSI;
    c_st.site = &x_emlrtRSI;
    if (dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax * dAlphaMax) <
        0.0) {
      d_st.site = &f_emlrtRSI;
      eml_error(&d_st);
    }

    xStarMin = (((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos(x[4]))
                 + kinematicConst[3] * muDoubleScalarCos(kinematicConst[8])) +
                kinematicConst[4] * muDoubleScalarCos(kinematicConst[8] + x[5]))
      - kinematicConst[6];
    dxStarMax = ((((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                    (T->data[ixstart + (T->size[0] << 2)])) + kinematicConst[3] *
                   muDoubleScalarCos(kinematicConst[8])) + kinematicConst[4] *
                  muDoubleScalarCos(kinematicConst[8] + T->data[ixstart +
      T->size[0] * 5])) - kinematicConst[6]) - xStarMin;

    /* angDiff Finds the angular difference between th1 and th2. */
    dAlphaMax = ((x[3] - T->data[ixstart + T->size[0] * 3]) + 3.1415926535897931)
      / 6.2831853071795862;
    if (muDoubleScalarAbs(dAlphaMax - muDoubleScalarRound(dAlphaMax)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(dAlphaMax)) {
      dAlphaMax = 0.0;
    } else {
      dAlphaMax = (dAlphaMax - muDoubleScalarFloor(dAlphaMax)) *
        6.2831853071795862;
    }

    dAlphaMax = muDoubleScalarAbs(dAlphaMax - 3.1415926535897931);
    b_st.site = &w_emlrtRSI;
    dAlphaMax = dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax *
      dAlphaMax);
    if (dAlphaMax < 0.0) {
      c_st.site = &f_emlrtRSI;
      eml_error(&c_st);
    }

    /* Calculate the total distance. */
    /* dPosNorm+dVelNorm  */
    i4 = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, i4, &lb_emlrtBCI,
      sp) - 1] = muDoubleScalarSqrt(dAlphaMax);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  i4 = b_d->size[1];
  ix = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ix, 1, i4, &hb_emlrtBCI, sp);
  st.site = &t_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b1 = true;
  } else {
    b1 = false;
  }

  if (b1) {
  } else {
    y = NULL;
    m1 = emlrtCreateCharArray(2, iv6);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv4[ixstart] = cv5[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m1, cv4);
    emlrtAssign(&y, m1);
    d_st.site = &ib_emlrtRSI;
    e_st.site = &lb_emlrtRSI;
    error(&d_st, b_message(&e_st, y, &c_emlrtMCI), &d_emlrtMCI);
  }

  d_st.site = &bb_emlrtRSI;
  ixstart = 1;
  dAlphaMax = b_d->data[0];
  itmp = 0;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(dAlphaMax)) {
      f_st.site = &db_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b2 = false;
      } else {
        b2 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b2) {
        g_st.site = &eb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int32_T)nodeIDCount)) {
        ixstart = ix;
        if (!muDoubleScalarIsNaN(b_d->data[ix - 1])) {
          dAlphaMax = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      f_st.site = &cb_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &eb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (b_d->data[ixstart] < dAlphaMax) {
          dAlphaMax = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = dAlphaMax;

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  i4 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i4, &jb_emlrtBCI, sp);
  i4 = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i4, &jb_emlrtBCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1000, &ib_emlrtBCI, sp);
  xNear_size[0] = 1;
  xNear_size[1] = 11;
  for (i4 = 0; i4 < 11; i4++) {
    xNear_data[xNear_size[0] * i4] = T->data[itmp + T->size[0] * i4];
  }

  if (12 > T->size[1]) {
    i4 = 0;
    ix = 0;
  } else {
    i4 = 11;
    ix = T->size[1];
    ixstart = T->size[1];
    ix = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &kb_emlrtBCI, sp);
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ix - i4;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, ixstart, (int32_T)
                    sizeof(real_T), &f_emlrtRTEI);
  ixstart = ix - i4;
  for (ix = 0; ix < ixstart; ix++) {
    transitionArray->data[transitionArray->size[0] * ix] = T->data[itmp +
      T->size[0] * (i4 + ix)];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
