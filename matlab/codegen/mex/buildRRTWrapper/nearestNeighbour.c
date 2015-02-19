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
#include "buildRRTWrapper_emxutil.h"
#include "heuristicSingleLeg.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildRRTWrapper_mexutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo ib_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo jb_emlrtRSI = { 26, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRSInfo nb_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

static emlrtRSInfo ob_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo pb_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo qb_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo rb_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo sb_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

static emlrtMCInfo d_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtMCInfo e_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 5, 22, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo o_emlrtRTEI = { 26, 37, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo p_emlrtRTEI = { 31, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtRTEInfo q_emlrtRTEI = { 5, 17, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo m_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRTEInfo ab_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

static emlrtBCInfo n_emlrtBCI = { -1, -1, 31, 13, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo o_emlrtBCI = { -1, -1, 26, 37, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtBCInfo p_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

static emlrtRSInfo gc_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

static emlrtRSInfo ic_emlrtRSI = { 41, "eml_min_or_max",
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
  const mxArray *m7;
  pArray = b;
  return emlrtCallMATLABR2012b(sp, 1, &m7, 1, &pArray, "message", true, location);
}

void nearestNeighbour(const emlrtStack *sp, real_T s_alpha, real_T s_beta,
                      real_T s_gamma, const emxArray_struct1_T *T, const real_T
                      jointLimits[14], real_T kC_l2, real_T kC_l3, real_T kC_l4,
                      real_T kC_l5, real_T kC_l7, real_T kC_zeta, real_T
                      nodeIDCount, struct1_T *xNear, real_T *d)
{
  emxArray_real_T *b_d;
  int32_T ix;
  int32_T ixstart;
  struct1_T expl_temp;
  boolean_T b0;
  const mxArray *y;
  static const int32_T iv3[2] = { 1, 36 };

  const mxArray *m1;
  char_T cv2[36];
  static const char_T cv3[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  real_T mtmp;
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
  emxInit_real_T(sp, &b_d, 2, &q_emlrtRTEI, true);

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
                    &n_emlrtRTEI);
  ixstart = (int32_T)nodeIDCount;
  for (ix = 0; ix < ixstart; ix++) {
    b_d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS, (int32_T)
    nodeIDCount, &ab_emlrtRTEI, sp);
  ixstart = 1;
  emxInitStruct_struct1_T(sp, &expl_temp, &o_emlrtRTEI, true);
  while (ixstart - 1 <= (int32_T)nodeIDCount - 1) {
    ix = T->size[1];
    emxCopyStruct_struct1_T(sp, &expl_temp, &T->
      data[emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &o_emlrtBCI, sp) -
      1], &o_emlrtRTEI);
    ix = b_d->size[1];
    st.site = &jb_emlrtRSI;
    b_d->data[emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &p_emlrtBCI, sp)
      - 1] = heuristicSingleLeg(&st, s_alpha, s_beta, s_gamma, expl_temp.alpha,
      expl_temp.beta, expl_temp.gamma, jointLimits, kC_l2, kC_l3, kC_l4, kC_l5,
      kC_l7, kC_zeta);
    ixstart++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFreeStruct_struct1_T(&expl_temp);
  ix = b_d->size[1];
  ixstart = (int32_T)nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, ix, &m_emlrtBCI, sp);
  st.site = &ib_emlrtRSI;
  b_st.site = &nb_emlrtRSI;
  c_st.site = &ob_emlrtRSI;
  if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
    b0 = true;
  } else {
    b0 = false;
  }

  if (b0) {
  } else {
    y = NULL;
    m1 = emlrtCreateCharArray(2, iv3);
    for (ixstart = 0; ixstart < 36; ixstart++) {
      cv2[ixstart] = cv3[ixstart];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m1, cv2);
    emlrtAssign(&y, m1);
    d_st.site = &gc_emlrtRSI;
    e_st.site = &ic_emlrtRSI;
    error(&d_st, b_message(&e_st, y, &d_emlrtMCI), &e_emlrtMCI);
  }

  d_st.site = &pb_emlrtRSI;
  ixstart = 1;
  mtmp = b_d->data[0];
  itmp = 1;
  if ((int32_T)nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(mtmp)) {
      f_st.site = &rb_emlrtRSI;
      if (2 > (int32_T)nodeIDCount) {
        b1 = false;
      } else {
        b1 = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b1) {
        g_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int32_T)nodeIDCount)) {
        ixstart = ix;
        if (!muDoubleScalarIsNaN(b_d->data[ix - 1])) {
          mtmp = b_d->data[ix - 1];
          itmp = ix;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int32_T)nodeIDCount) {
      f_st.site = &qb_emlrtRSI;
      if (ixstart + 1 > (int32_T)nodeIDCount) {
        b_ixstart = false;
      } else {
        b_ixstart = ((int32_T)nodeIDCount > 2147483646);
      }

      if (b_ixstart) {
        g_st.site = &sb_emlrtRSI;
        check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= (int32_T)nodeIDCount) {
        if (b_d->data[ixstart] < mtmp) {
          mtmp = b_d->data[ixstart];
          itmp = ixstart + 1;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = mtmp;

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  ix = T->size[1];
  emxCopyStruct_struct1_T(sp, xNear, &T->data[emlrtDynamicBoundsCheckFastR2012b
    (itmp, 1, ix, &n_emlrtBCI, sp) - 1], &p_emlrtRTEI);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (nearestNeighbour.c) */
