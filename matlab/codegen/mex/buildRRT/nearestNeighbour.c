/*
 * nearestNeighbour.c
 *
 * Code generation for function 'nearestNeighbour'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "nearestNeighbour.h"
#include "buildRRT_emxutil.h"
#include "eml_error.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildRRT_mexutil.h"
#include "buildRRT_data.h"

/* Variable Definitions */
static emlrtRTEInfo i_emlrtRTEI = { 5, 38, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m"
};

/* Function Definitions */
void nearestNeighbour(const emlrtStack *sp, const real_T x[11], const
                      emxArray_real_T *T, const real_T kinematicConst[12],
                      real_T nodeIDCount, int32_T NODE_SIZE, emxArray_real_T
                      *xNear, emxArray_real_T *transitionArray, real_T *d)
{
  emxArray_real_T *b_d;
  int32_T i4;
  int32_T loop_ub;
  int32_T ixstart;
  int32_T i5;
  real_T xStarA;
  real_T dxStar;
  real_T dAlpha;
  boolean_T b2;
  const mxArray *y;
  static const int32_T iv10[2] = { 1, 36 };

  const mxArray *m3;
  char_T cv12[36];
  static const char_T cv13[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  const mxArray *b_y;
  static const int32_T iv11[2] = { 1, 39 };

  char_T cv14[39];
  static const char_T cv15[39] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'e', 'm', 'l', '_', 'm', 'i', 'n', '_', 'o', 'r',
    '_', 'm', 'a', 'x', '_', 'v', 'a', 'r', 'D', 'i', 'm', 'Z', 'e', 'r', 'o' };

  int32_T itmp;
  boolean_T b3;
  boolean_T exitg1;
  boolean_T b_ixstart;
  int64_T i6;
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
  loop_ub = (int32_T)nodeIDCount;
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_d->data[i4] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &o_emlrtRTEI, sp);
  ixstart = 0;
  while (ixstart <= (int32_T)nodeIDCount - 1) {
    st.site = &r_emlrtRSI;
    i4 = T->size[0];
    i5 = ixstart + 1;
    emlrtDynamicBoundsCheckFastR2012b(i5, 1, i4, &bc_emlrtBCI, &st);

    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* heuristic Calculates the distance between states x1 and x2. */
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(4, 1, i4, &i_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i4, &h_emlrtBCI, &st);
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i4, &g_emlrtBCI, &st);

    /* [x1,y1,z1] = fk(alpha1,beta1,gamma1,kinematicConst); */
    /* [x2,y2,z2] = fk(alpha2,beta2,gamma2,kinematicConst); */
    /* distMAX = [sqrt(range(1)^2+range(2)+range(3)^2) sqrt(range(4)^2+range(5)^2+range(6)^2)]; */
    /* distMAX = [10 1]; */
    /* d = HGAINS(1)*cartDist(xA(4:6),xB(4:6))/distMAX(1) +... */
    /*     HGAINS(2)*abs(z1-z2); */
    /* d = HGAINS(1)*cartDist([x1 y1 z1],[x2 y2 z2])/distMAX(1); */
    /*     HGAINS(2)*cartDist(x1(7:9),x2(7:9))/distMAX(2); */
    xStarA = (((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos(x[4]))
               + kinematicConst[3] * muDoubleScalarCos(kinematicConst[8])) +
              kinematicConst[4] * muDoubleScalarCos(kinematicConst[8] + x[5])) -
      kinematicConst[6];
    dxStar = ((((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                 (T->data[ixstart + (T->size[0] << 2)])) + kinematicConst[3] *
                muDoubleScalarCos(kinematicConst[8])) + kinematicConst[4] *
               muDoubleScalarCos(kinematicConst[8] + T->data[ixstart + T->size[0]
                * 5])) - kinematicConst[6]) - xStarA;

    /* angDiff Finds the angular difference between th1 and th2. */
    dAlpha = ((x[3] - T->data[ixstart + T->size[0] * 3]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(dAlpha - muDoubleScalarRound(dAlpha)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(dAlpha)) {
      dAlpha = 0.0;
    } else {
      dAlpha = (dAlpha - muDoubleScalarFloor(dAlpha)) * 6.2831853071795862;
    }

    dAlpha = muDoubleScalarAbs(dAlpha - 3.1415926535897931);
    b_st.site = &s_emlrtRSI;
    dAlpha = dxStar * dxStar + xStarA * xStarA * (dAlpha * dAlpha);
    if (dAlpha < 0.0) {
      c_st.site = &t_emlrtRSI;
      b_eml_error(&c_st);
    }

    i4 = b_d->size[1];
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart + 1, 1, i4, &hc_emlrtBCI,
      sp) - 1] = muDoubleScalarSqrt(dAlpha);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  i4 = b_d->size[1];
  i5 = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(i5, 1, i4, &cc_emlrtBCI, sp);
  st.site = &q_emlrtRSI;
  b_st.site = &u_emlrtRSI;
  c_st.site = &v_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b2 = true;
  } else {
    b2 = false;
  }

  if (b2) {
  } else {
    y = NULL;
    m3 = emlrtCreateCharArray(2, iv10);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv12[ixstart] = cv13[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m3, cv12);
    emlrtAssign(&y, m3);
    d_st.site = &hb_emlrtRSI;
    e_st.site = &lb_emlrtRSI;
    error(&d_st, b_message(&e_st, y, &c_emlrtMCI), &d_emlrtMCI);
  }

  if ((int32_T)nodeIDCount > 0) {
  } else {
    b_y = NULL;
    m3 = emlrtCreateCharArray(2, iv11);
    for (ixstart = 0; ixstart < 39; ixstart++) {
      cv14[ixstart] = cv15[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 39, m3, cv14);
    emlrtAssign(&b_y, m3);
    d_st.site = &gb_emlrtRSI;
    e_st.site = &kb_emlrtRSI;
    error(&d_st, b_message(&e_st, b_y, &e_emlrtMCI), &f_emlrtMCI);
  }

  d_st.site = &w_emlrtRSI;
  ixstart = 1;
  dAlpha = b_d->data[0];
  itmp = 1;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(dAlpha)) {
      f_st.site = &y_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b3 = false;
      } else {
        b3 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b3) {
        g_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&g_st);
      }

      loop_ub = 2;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= (int32_T)nodeIDCount)) {
        ixstart = loop_ub;
        if (!muDoubleScalarIsNaN(b_d->data[loop_ub - 1])) {
          dAlpha = b_d->data[loop_ub - 1];
          itmp = loop_ub;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      f_st.site = &x_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (b_d->data[ixstart] < dAlpha) {
          dAlpha = b_d->data[ixstart];
          itmp = ixstart + 1;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = dAlpha;

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  if (1 > NODE_SIZE) {
    loop_ub = 0;
  } else {
    i4 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(1, 1, i4, &ec_emlrtBCI, sp);
    i4 = T->size[1];
    loop_ub = emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i4, &ec_emlrtBCI,
      sp);
  }

  i4 = T->size[0];
  ixstart = emlrtDynamicBoundsCheckFastR2012b(itmp, 1, i4, &dc_emlrtBCI, sp);
  i4 = xNear->size[0] * xNear->size[1];
  xNear->size[0] = 1;
  xNear->size[1] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)xNear, i4, (int32_T)sizeof(real_T),
                    &i_emlrtRTEI);
  for (i4 = 0; i4 < loop_ub; i4++) {
    xNear->data[xNear->size[0] * i4] = T->data[(ixstart + T->size[0] * i4) - 1];
  }

  i6 = NODE_SIZE + 1L;
  if (i6 > 2147483647L) {
    i6 = 2147483647L;
  } else {
    if (i6 < -2147483648L) {
      i6 = -2147483648L;
    }
  }

  i4 = (int32_T)i6;
  if (i4 > T->size[1]) {
    i4 = 0;
    i5 = 0;
  } else {
    i5 = T->size[1];
    i4 = emlrtDynamicBoundsCheckFastR2012b(i4, 1, i5, &gc_emlrtBCI, sp) - 1;
    i5 = T->size[1];
    ixstart = T->size[1];
    i5 = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, i5, &gc_emlrtBCI, sp);
  }

  ixstart = T->size[0];
  itmp = emlrtDynamicBoundsCheckFastR2012b(itmp, 1, ixstart, &fc_emlrtBCI, sp);
  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = i5 - i4;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, ixstart, (int32_T)
                    sizeof(real_T), &i_emlrtRTEI);
  loop_ub = i5 - i4;
  for (i5 = 0; i5 < loop_ub; i5++) {
    transitionArray->data[transitionArray->size[0] * i5] = T->data[(itmp +
      T->size[0] * (i4 + i5)) - 1];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
