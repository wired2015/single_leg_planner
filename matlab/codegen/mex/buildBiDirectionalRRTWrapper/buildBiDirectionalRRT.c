/*
 * buildBiDirectionalRRT.c
 *
 * Code generation for function 'buildBiDirectionalRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRT.h"
#include "buildBiDirectionalRRTWrapper_emxutil.h"
#include "norm.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "flipud.h"
#include "buildBiDirectionalRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo g_emlrtRSI = { 75, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo h_emlrtRSI = { 72, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo i_emlrtRSI = { 64, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 63, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 60, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo l_emlrtRSI = { 39, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 114, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo n_emlrtRSI = { 115, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo o_emlrtRSI = { 116, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRSInfo p_emlrtRSI = { 37, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 45, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo r_emlrtRSI = { 46, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo s_emlrtRSI = { 48, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo t_emlrtRSI = { 49, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo u_emlrtRSI = { 51, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo v_emlrtRSI = { 52, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo w_emlrtRSI = { 59, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo x_emlrtRSI = { 62, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo y_emlrtRSI = { 68, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo ab_emlrtRSI = { 69, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo bb_emlrtRSI = { 70, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo rb_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRSInfo sb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtMCInfo c_emlrtMCI = { 56, 9, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 5, 32, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

static emlrtRTEInfo d_emlrtRTEI = { 33, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 63, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 64, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 72, 13, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 29, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 88, 17, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo o_emlrtRTEI = { 92, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo p_emlrtRTEI = { 94, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo q_emlrtRTEI = { 99, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtRTEInfo r_emlrtRTEI = { 129, 18, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo b_emlrtBCI = { -1, -1, 76, 32, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo emlrtECI = { 1, 73, 20, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo c_emlrtBCI = { -1, -1, 65, 71, "T1", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 65, 46, "T1", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 65, 21, "T1", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 59, 22, "T1", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { -1, 20, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtECInfo c_emlrtECI = { -1, 19, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo g_emlrtBCI = { -1, -1, 123, 7, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo d_emlrtECI = { -1, 123, 5, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtDCInfo emlrtDCI = { 16, 35, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  4 };

static emlrtDCInfo b_emlrtDCI = { 17, 35, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  4 };

static emlrtDCInfo c_emlrtDCI = { 19, 30, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtDCInfo d_emlrtDCI = { 19, 30, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  4 };

static emlrtDCInfo i_emlrtDCI = { 94, 25, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtBCInfo bb_emlrtBCI = { -1, -1, 94, 25, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo cb_emlrtBCI = { -1, -1, 94, 37, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo db_emlrtBCI = { -1, -1, 97, 25, "next",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtRTEInfo w_emlrtRTEI = { 100, 9, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 105, 18, "next",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtDCInfo j_emlrtDCI = { 105, 18, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  1 };

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 105, 18, "T", "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo gb_emlrtBCI = { -1, -1, 106, 17, "next",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo hb_emlrtBCI = { -1, -1, 108, 27, "next",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtECInfo h_emlrtECI = { -1, 108, 27, "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m"
};

static emlrtBCInfo ib_emlrtBCI = { -1, -1, 101, 47, "transitionArray",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo jb_emlrtBCI = { -1, -1, 134, 31, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo kb_emlrtBCI = { -1, -1, 135, 37, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo lb_emlrtBCI = { -1, -1, 135, 50, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo mb_emlrtBCI = { -1, -1, 139, 48, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo nb_emlrtBCI = { -1, -1, 141, 15, "pathC",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtBCInfo ob_emlrtBCI = { -1, -1, 141, 29, "pathJ",
  "buildBiDirectionalRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT/buildBiDirectionalRRT.m",
  0 };

static emlrtRSInfo yb_emlrtRSI = { 56, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

/* Function Declarations */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static void traceBranch(const emlrtStack *sp, const emxArray_real_T *T, const
  real_T midPoint_data[], emxArray_real_T *path);
static void transformPath(const emlrtStack *sp, const emxArray_real_T *pathJ,
  real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T
  kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const real_T
  TP2B[16], emxArray_real_T *pathC);

/* Function Definitions */
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

static void traceBranch(const emlrtStack *sp, const emxArray_real_T *T, const
  real_T midPoint_data[], emxArray_real_T *path)
{
  emxArray_real_T *next;
  real_T check;
  int32_T i10;
  int32_T i11;
  int32_T i12;
  emxArray_real_T *transitionArray;
  int32_T loop_ub;
  emxArray_real_T *transitionPath;
  emxArray_int32_T *r4;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  boolean_T exitg1;
  int32_T i;
  uint32_T b_i;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &next, 2, &o_emlrtRTEI, true);

  /* Assignn the  */
  check = midPoint_data[0];
  i10 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 13;
  emxEnsureCapacity(sp, (emxArray__common *)next, i10, (int32_T)sizeof(real_T),
                    &n_emlrtRTEI);
  for (i10 = 0; i10 < 13; i10++) {
    next->data[i10] = midPoint_data[i10];
  }

  i10 = path->size[0] * path->size[1];
  path->size[0] = 0;
  path->size[1] = 10;
  emxEnsureCapacity(sp, (emxArray__common *)path, i10, (int32_T)sizeof(real_T),
                    &n_emlrtRTEI);
  if (14 > T->size[1]) {
    i10 = 0;
    i11 = 0;
  } else {
    i10 = 13;
    i11 = T->size[1];
    i12 = T->size[1];
    i11 = emlrtDynamicBoundsCheckFastR2012b(i12, 1, i11, &cb_emlrtBCI, sp);
  }

  emxInit_real_T(sp, &transitionArray, 2, &p_emlrtRTEI, true);
  i12 = (int32_T)emlrtIntegerCheckFastR2012b(midPoint_data[0], &i_emlrtDCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(i12, 1, 1000, &bb_emlrtBCI, sp);
  i12 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = i11 - i10;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i12, (int32_T)
                    sizeof(real_T), &n_emlrtRTEI);
  loop_ub = i11 - i10;
  for (i11 = 0; i11 < loop_ub; i11++) {
    transitionArray->data[transitionArray->size[0] * i11] = T->data[((int32_T)
      midPoint_data[0] + T->size[0] * (i10 + i11)) - 1];
  }

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(sp, &transitionPath, 2, &q_emlrtRTEI, true);
  b_emxInit_int32_T(sp, &r4, 2, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &n_emlrtRTEI, true);
  exitg1 = false;
  while ((!exitg1) && (check != 0.0)) {
    i10 = next->size[1];
    emlrtDynamicBoundsCheckFastR2012b(2, 1, i10, &db_emlrtBCI, sp);
    if (next->data[1] != 0.0) {
      i10 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = 0;
      transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i10, (int32_T)
                        sizeof(real_T), &n_emlrtRTEI);
      i10 = (int32_T)(((real_T)transitionArray->size[1] + 9.0) / 10.0);
      emlrtForLoopVectorCheckR2012b(1.0, 10.0, transitionArray->size[1],
        mxDOUBLE_CLASS, i10, &w_emlrtRTEI, sp);
      i = 0;
      while (i <= i10 - 1) {
        b_i = i * 10U + 1U;
        i11 = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)c_transitionPath, i11,
                          (int32_T)sizeof(real_T), &n_emlrtRTEI);
        for (i11 = 0; i11 < 10; i11++) {
          loop_ub = transitionPath->size[0];
          for (i12 = 0; i12 < loop_ub; i12++) {
            c_transitionPath->data[i12 + c_transitionPath->size[0] * i11] =
              transitionPath->data[i12 + transitionPath->size[0] * i11];
          }
        }

        for (i11 = 0; i11 < 10; i11++) {
          i12 = transitionArray->size[1];
          loop_ub = (int32_T)(i11 + b_i);
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * i11] = transitionArray->
            data[emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i12, &ib_emlrtBCI,
            sp) - 1];
        }

        i11 = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)transitionPath, i11, (int32_T)
                          sizeof(real_T), &n_emlrtRTEI);
        for (i11 = 0; i11 < 10; i11++) {
          loop_ub = c_transitionPath->size[0];
          for (i12 = 0; i12 < loop_ub; i12++) {
            transitionPath->data[i12 + transitionPath->size[0] * i11] =
              c_transitionPath->data[i12 + c_transitionPath->size[0] * i11];
          }
        }

        i++;
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
      }

      i10 = b_transitionPath->size[0] * b_transitionPath->size[1];
      b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
      b_transitionPath->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)b_transitionPath, i10, (int32_T)
                        sizeof(real_T), &n_emlrtRTEI);
      for (i10 = 0; i10 < 10; i10++) {
        loop_ub = transitionPath->size[0];
        for (i11 = 0; i11 < loop_ub; i11++) {
          b_transitionPath->data[i11 + b_transitionPath->size[0] * i10] =
            transitionPath->data[i11 + transitionPath->size[0] * i10];
        }
      }

      for (i10 = 0; i10 < 10; i10++) {
        loop_ub = path->size[0];
        for (i11 = 0; i11 < loop_ub; i11++) {
          b_transitionPath->data[(i11 + transitionPath->size[0]) +
            b_transitionPath->size[0] * i10] = path->data[i11 + path->size[0] *
            i10];
        }
      }

      i10 = path->size[0] * path->size[1];
      path->size[0] = b_transitionPath->size[0];
      path->size[1] = 10;
      emxEnsureCapacity(sp, (emxArray__common *)path, i10, (int32_T)sizeof
                        (real_T), &n_emlrtRTEI);
      for (i10 = 0; i10 < 10; i10++) {
        loop_ub = b_transitionPath->size[0];
        for (i11 = 0; i11 < loop_ub; i11++) {
          path->data[i11 + path->size[0] * i10] = b_transitionPath->data[i11 +
            b_transitionPath->size[0] * i10];
        }
      }

      i10 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i10, &eb_emlrtBCI, sp);
      check = next->data[1];
      loop_ub = T->size[1];
      i10 = (int32_T)emlrtIntegerCheckFastR2012b(check, &j_emlrtDCI, sp);
      emlrtDynamicBoundsCheckFastR2012b(i10, 1, 1000, &fb_emlrtBCI, sp);
      i10 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = loop_ub;
      emxEnsureCapacity(sp, (emxArray__common *)next, i10, (int32_T)sizeof
                        (real_T), &n_emlrtRTEI);
      for (i10 = 0; i10 < loop_ub; i10++) {
        next->data[next->size[0] * i10] = T->data[((int32_T)check + T->size[0] *
          i10) - 1];
      }

      i10 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i10, &gb_emlrtBCI, sp);
      check = next->data[1];
      if (14 > next->size[1]) {
        i10 = 1;
        i11 = 0;
      } else {
        i10 = 14;
        i11 = next->size[1];
        i12 = next->size[1];
        i11 = emlrtDynamicBoundsCheckFastR2012b(i12, 1, i11, &hb_emlrtBCI, sp);
      }

      i12 = r4->size[0] * r4->size[1];
      r4->size[0] = 1;
      r4->size[1] = (i11 - i10) + 1;
      emxEnsureCapacity(sp, (emxArray__common *)r4, i12, (int32_T)sizeof(int32_T),
                        &n_emlrtRTEI);
      loop_ub = i11 - i10;
      for (i11 = 0; i11 <= loop_ub; i11++) {
        r4->data[r4->size[0] * i11] = i10 + i11;
      }

      emlrtMatrixMatrixIndexCheckR2012b(*(int32_T (*)[2])next->size, 2,
        *(int32_T (*)[2])r4->size, 2, &h_emlrtECI, sp);
      i10 = transitionArray->size[0] * transitionArray->size[1];
      transitionArray->size[0] = 1;
      transitionArray->size[1] = r4->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i10, (int32_T)
                        sizeof(real_T), &n_emlrtRTEI);
      loop_ub = r4->size[0] * r4->size[1];
      for (i10 = 0; i10 < loop_ub; i10++) {
        transitionArray->data[i10] = next->data[r4->data[i10] - 1];
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    } else {
      exitg1 = true;
    }
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_int32_T(&r4);
  emxFree_real_T(&transitionPath);
  emxFree_real_T(&transitionArray);
  emxFree_real_T(&next);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

static void transformPath(const emlrtStack *sp, const emxArray_real_T *pathJ,
  real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T
  kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const real_T
  TP2B[16], emxArray_real_T *pathC)
{
  int32_T pathJ_idx_0;
  int32_T i13;
  real_T dist2Go;
  int32_T i;
  real_T uP[3];
  real_T uB[3];
  real_T d2;
  real_T b_uB[3];
  real_T b_pathJ[3];
  pathJ_idx_0 = pathJ->size[0];
  i13 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = pathJ_idx_0;
  pathC->size[1] = 9;
  emxEnsureCapacity(sp, (emxArray__common *)pathC, i13, (int32_T)sizeof(real_T),
                    &r_emlrtRTEI);
  pathJ_idx_0 = pathJ->size[0] * 9;
  for (i13 = 0; i13 < pathJ_idx_0; i13++) {
    pathC->data[i13] = 0.0;
  }

  dist2Go = 0.0;
  i = 0;
  while (i <= pathJ->size[0] - 1) {
    i13 = pathJ->size[0];
    pathJ_idx_0 = i + 1;
    emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0, 1, i13, &jb_emlrtBCI, sp);

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
    uP[0] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-pathJ->data[i + (pathJ->size
      [0] << 1)])) + kC_l4 * muDoubleScalarCos(kC_zeta)) + kC_l5 *
              muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] + kC_zeta))
             - kC_l7) * muDoubleScalarCos(pathJ->data[i + pathJ->size[0]]);
    uP[1] = ((((kC_l2 + kC_l3 * muDoubleScalarCos(-pathJ->data[i + (pathJ->size
      [0] << 1)])) + kC_l4 * muDoubleScalarCos(kC_zeta)) + kC_l5 *
              muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] + kC_zeta))
             - kC_l7) * muDoubleScalarSin(pathJ->data[i + pathJ->size[0]]);
    uP[2] = ((((kC_l1 + kC_l3 * muDoubleScalarSin(-pathJ->data[i + (pathJ->size
      [0] << 1)])) - kC_l4 * muDoubleScalarSin(kC_zeta)) - kC_l5 *
              muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * 3] + kC_zeta))
             - kC_l6) - (kC_l8 + kC_r);
    i13 = pathJ->size[0];
    pathJ_idx_0 = i + 1;
    emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0, 1, i13, &kb_emlrtBCI, sp);
    i13 = pathJ->size[0];
    pathJ_idx_0 = i + 1;
    emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0, 1, i13, &lb_emlrtBCI, sp);

    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    for (i13 = 0; i13 < 3; i13++) {
      d2 = 0.0;
      for (pathJ_idx_0 = 0; pathJ_idx_0 < 3; pathJ_idx_0++) {
        d2 += TP2B[i13 + (pathJ_idx_0 << 2)] * uP[pathJ_idx_0];
      }

      uB[i13] = d2 + TP2B[12 + i13];
    }

    if (1 + i != 1) {
      i13 = pathC->size[0];
      pathJ_idx_0 = emlrtDynamicBoundsCheckFastR2012b(i, 1, i13, &mb_emlrtBCI,
        sp);
      for (i13 = 0; i13 < 3; i13++) {
        b_uB[i13] = uB[i13] - pathC->data[(pathJ_idx_0 + pathC->size[0] * (2 +
          i13)) - 1];
      }

      dist2Go += norm(b_uB);
    }

    pathJ_idx_0 = pathC->size[0];
    i13 = 1 + i;
    emlrtDynamicBoundsCheckFastR2012b(i13, 1, pathJ_idx_0, &nb_emlrtBCI, sp);
    b_pathJ[0] = (-pathJ->data[i + pathJ->size[0] * 6] * muDoubleScalarSin
                  (pathJ->data[i + pathJ->size[0]]) * ((((kC_l2 - kC_l7) + kC_l5
      * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] + kC_zeta)) +
      kC_l3 * muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] << 1)])) + kC_l4
      * muDoubleScalarCos(kC_zeta)) - pathJ->data[i + pathJ->size[0] * 7] *
                  kC_l3 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0]]) *
                  muDoubleScalarSin(pathJ->data[i + (pathJ->size[0] << 1)])) -
      pathJ->data[i + (pathJ->size[0] << 3)] * kC_l5 * muDoubleScalarSin
      (pathJ->data[i + pathJ->size[0] * 3] + kC_zeta) * muDoubleScalarCos
      (pathJ->data[i + pathJ->size[0]]);
    b_pathJ[1] = (pathJ->data[i + pathJ->size[0] * 6] * muDoubleScalarCos
                  (pathJ->data[i + pathJ->size[0]]) * ((((kC_l2 - kC_l7) + kC_l5
      * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] + kC_zeta)) +
      kC_l3 * muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] << 1)])) + kC_l4
      * muDoubleScalarCos(kC_zeta)) - pathJ->data[i + (pathJ->size[0] << 3)] *
                  kC_l5 * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * 3]
      + kC_zeta) * muDoubleScalarSin(pathJ->data[i + pathJ->size[0]])) -
      pathJ->data[i + pathJ->size[0] * 7] * kC_l3 * muDoubleScalarSin
      (pathJ->data[i + pathJ->size[0]]) * muDoubleScalarSin(pathJ->data[i +
      (pathJ->size[0] << 1)]);
    b_pathJ[2] = -pathJ->data[i + pathJ->size[0] * 7] * kC_l3 *
      muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] << 1)]) - kC_l5 *
      pathJ->data[i + (pathJ->size[0] << 3)] * muDoubleScalarCos(kC_zeta +
      pathJ->data[i + pathJ->size[0] * 3]);
    for (i13 = 0; i13 < 3; i13++) {
      b_uB[i13] = 0.0;
      for (pathJ_idx_0 = 0; pathJ_idx_0 < 3; pathJ_idx_0++) {
        b_uB[i13] += TP2B[i13 + (pathJ_idx_0 << 2)] * b_pathJ[pathJ_idx_0];
      }
    }

    i13 = pathJ->size[0];
    pathJ_idx_0 = 1 + i;
    pathC->data[i] = pathJ->data[emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0,
      1, i13, &ob_emlrtBCI, sp) - 1];
    pathC->data[i + pathC->size[0]] = dist2Go;
    for (i13 = 0; i13 < 3; i13++) {
      pathC->data[i + pathC->size[0] * (i13 + 2)] = uB[i13];
    }

    for (i13 = 0; i13 < 3; i13++) {
      pathC->data[i + pathC->size[0] * (i13 + 5)] = b_uB[i13];
    }

    pathC->data[i + (pathC->size[0] << 3)] = 0.0;
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }
}

void buildBiDirectionalRRT(const emlrtStack *sp, const real_T nInit[13], const
  real_T nGoal[13], const real_T jointLimits[20], real_T panHeight, int32_T
  NODE_SIZE, const real_T U[18], real_T dt, real_T Dt, const struct0_T *kC,
  const real_T uBDot[6], int32_T legNum, const real_T TP2B[16], emxArray_real_T *
  T1, emxArray_real_T *T2, emxArray_real_T *pathJ, emxArray_real_T *pathC)
{
  real_T transitionArrayLength;
  int32_T i3;
  real_T xMin;
  int32_T ndbl;
  int32_T cdiff;
  emxArray_int32_T *r0;
  emxArray_real_T *unusedU4;
  int32_T iv2[2];
  int32_T iv3[2];
  emxArray_real_T *pathCMin;
  emxArray_real_T *b_T1;
  real_T nodeIDCount1;
  real_T nodeIDCount2;
  real_T pathLengthMin;
  real_T xRand[13];
  emxArray_real_T *transitionArray;
  int32_T i;
  real_T r;
  const mxArray *y;
  static const int32_T iv4[2] = { 1, 17 };

  const mxArray *m0;
  char_T cv0[17];
  static const char_T cv1[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  real_T unusedU7;
  int32_T nMid1_size[2];
  real_T nMid1_data[13];
  int32_T nMid2_size[2];
  real_T nMid2_data[13];
  real_T uA[3];
  real_T uB[3];
  real_T qDot[3];
  real_T b_qDot[3];
  real_T c_qDot[3];
  int32_T iv5[2];
  emxArray_real_T *pathT1;
  emxArray_real_T *pathT2;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *r1;
  emxArray_real_T *b_t;
  real_T d;
  boolean_T guard1 = false;
  int32_T absb;
  int32_T apnd;
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
  transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 10.0;
  i3 = T1->size[0] * T1->size[1];
  T1->size[0] = 1000;
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

  T1->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(ndbl, &emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T1, i3, (int32_T)sizeof(real_T),
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

  cdiff = 1000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i3, &emlrtDCI, sp);
  for (i3 = 0; i3 < cdiff; i3++) {
    T1->data[i3] = 0.0;
  }

  i3 = T2->size[0] * T2->size[1];
  T2->size[0] = 1000;
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

  T2->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(ndbl, &b_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T2, i3, (int32_T)sizeof(real_T),
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

  cdiff = 1000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i3, &b_emlrtDCI, sp);
  for (i3 = 0; i3 < cdiff; i3++) {
    T2->data[i3] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &b_emlrtRTEI, true);
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

  i3 = r0->size[0];
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

  r0->size[0] = ndbl;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < cdiff; i3++) {
    r0->data[i3] = i3;
  }

  emxInit_real_T(sp, &unusedU4, 2, &b_emlrtRTEI, true);
  xMin = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &d_emlrtDCI, sp);
  emlrtIntegerCheckFastR2012b(xMin, &c_emlrtDCI, sp);
  i3 = unusedU4->size[0] * unusedU4->size[1];
  unusedU4->size[0] = 1;
  unusedU4->size[1] = 13 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)unusedU4, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 13; i3++) {
    unusedU4->data[unusedU4->size[0] * i3] = nInit[i3];
  }

  cdiff = (int32_T)transitionArrayLength;
  for (i3 = 0; i3 < cdiff; i3++) {
    unusedU4->data[unusedU4->size[0] * (i3 + 13)] = 0.0;
  }

  iv2[0] = 1;
  iv2[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv2, 2, *(int32_T (*)[2])unusedU4->size, 2,
    &c_emlrtECI, sp);
  cdiff = unusedU4->size[1];
  for (i3 = 0; i3 < cdiff; i3++) {
    T1->data[T1->size[0] * r0->data[i3]] = unusedU4->data[unusedU4->size[0] * i3];
  }

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

  i3 = r0->size[0];
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

  r0->size[0] = ndbl;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < cdiff; i3++) {
    r0->data[i3] = i3;
  }

  i3 = unusedU4->size[0] * unusedU4->size[1];
  unusedU4->size[0] = 1;
  unusedU4->size[1] = 13 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)unusedU4, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 13; i3++) {
    unusedU4->data[unusedU4->size[0] * i3] = nGoal[i3];
  }

  cdiff = (int32_T)transitionArrayLength;
  for (i3 = 0; i3 < cdiff; i3++) {
    unusedU4->data[unusedU4->size[0] * (i3 + 13)] = 0.0;
  }

  iv3[0] = 1;
  iv3[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv3, 2, *(int32_T (*)[2])unusedU4->size, 2,
    &b_emlrtECI, sp);
  cdiff = unusedU4->size[1];
  for (i3 = 0; i3 < cdiff; i3++) {
    T2->data[T2->size[0] * r0->data[i3]] = unusedU4->data[unusedU4->size[0] * i3];
  }

  emxInit_real_T(sp, &pathCMin, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &b_T1, 2, &b_emlrtRTEI, true);
  nodeIDCount1 = 1.0;
  nodeIDCount2 = 1.0;
  pathLengthMin = 100.0;
  i3 = pathCMin->size[0] * pathCMin->size[1];
  pathCMin->size[0] = 0;
  pathCMin->size[1] = 0;
  emxEnsureCapacity(sp, (emxArray__common *)pathCMin, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  i3 = pathJ->size[0] * pathJ->size[1];
  pathJ->size[0] = 0;
  pathJ->size[1] = 0;
  emxEnsureCapacity(sp, (emxArray__common *)pathJ, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  emxInit_real_T(sp, &transitionArray, 2, &b_emlrtRTEI, true);
  for (i = 0; i < 1998; i++) {
    st.site = &l_emlrtRSI;
    i3 = b_T1->size[0] * b_T1->size[1];
    b_T1->size[0] = 1000;
    b_T1->size[1] = T1->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)b_T1, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    cdiff = T1->size[0] * T1->size[1];
    for (i3 = 0; i3 < cdiff; i3++) {
      b_T1->data[i3] = T1->data[i3];
    }

    b_st.site = &m_emlrtRSI;

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
    c_st.site = &p_emlrtRSI;
    emlrtRandu(&r, 1);
    if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
      c_st.site = &q_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[4], false,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &r_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[2], true, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
      c_st.site = &s_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[4], false,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &t_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
      c_st.site = &u_emlrtRSI;
      transitionArrayLength = getXStar(&c_st, panHeight, jointLimits[3], true,
        kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
        kC->r);
      c_st.site = &v_emlrtRSI;
      xMin = getXStar(&c_st, panHeight, jointLimits[5], false, kC->l1, kC->l2,
                      kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta,
                      kC->r);
    } else {
      transitionArrayLength = 0.0;
      xMin = 0.0;
      y = NULL;
      m0 = emlrtCreateCharArray(2, iv4);
      for (ndbl = 0; ndbl < 17; ndbl++) {
        cv0[ndbl] = cv1[ndbl];
      }

      emlrtInitCharArrayR2013a(&b_st, 17, m0, cv0);
      emlrtAssign(&y, m0);
      c_st.site = &yb_emlrtRSI;
      disp(&c_st, y, &c_emlrtMCI);
    }

    c_st.site = &w_emlrtRSI;
    emlrtRandu(&b_r, 1);
    b_xMin[0] = xMin + (transitionArrayLength - xMin) * b_r;
    b_xMin[1] = 0.0;
    b_xMin[2] = panHeight;
    c_st.site = &x_emlrtRSI;
    b_sherpaTTIK(&c_st, b_xMin, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, q);
    c_st.site = &y_emlrtRSI;
    emlrtRandu(&b_r, 1);
    c_st.site = &ab_emlrtRSI;
    emlrtRandu(&transitionArrayLength, 1);
    c_st.site = &bb_emlrtRSI;
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
    b_st.site = &n_emlrtRSI;
    nearestNeighbour(&b_st, xRand, T1, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                     kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount1, 13,
                     nMid1_data, nMid1_size, unusedU4, &unusedU7);
    b_st.site = &o_emlrtRSI;
    selectInput(&b_st, nMid1_data, xRand, U, dt, Dt, kC, jointLimits, uBDot,
                legNum, nMid2_data, nMid2_size, transitionArray);
    nMid2_data[0] = nodeIDCount1 + 1.0;

    /* Node ID */
    nMid2_data[1] = nMid1_data[0];

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
    uA[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-nMid2_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (nMid2_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos
      (nMid2_data[3]);
    uA[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-nMid2_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (nMid2_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin
      (nMid2_data[3]);
    uA[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-nMid2_data[4])) - kC->l4 *
               muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
              (nMid2_data[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    uB[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-nMid1_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (nMid1_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarCos
      (nMid1_data[3]);
    uB[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-nMid1_data[4])) + kC->l4 *
               muDoubleScalarCos(kC->zeta)) + kC->l5 * muDoubleScalarCos
              (nMid1_data[5] + kC->zeta)) - kC->l7) * muDoubleScalarSin
      (nMid1_data[3]);
    uB[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-nMid1_data[4])) - kC->l4 *
               muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
              (nMid1_data[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    qDot[0] = (-nMid1_data[8] * muDoubleScalarSin(nMid1_data[3]) * ((((kC->l2 -
      kC->l7) + kC->l5 * muDoubleScalarCos(nMid1_data[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(nMid1_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta))
               - nMid1_data[9] * kC->l3 * muDoubleScalarCos(nMid1_data[3]) *
               muDoubleScalarSin(nMid1_data[4])) - nMid1_data[10] * kC->l5 *
      muDoubleScalarSin(nMid1_data[5] + kC->zeta) * muDoubleScalarCos
      (nMid1_data[3]);
    qDot[1] = (nMid1_data[8] * muDoubleScalarCos(nMid1_data[3]) * ((((kC->l2 -
      kC->l7) + kC->l5 * muDoubleScalarCos(nMid1_data[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(nMid1_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta))
               - nMid1_data[10] * kC->l5 * muDoubleScalarSin(nMid1_data[5] +
                kC->zeta) * muDoubleScalarSin(nMid1_data[3])) - nMid1_data[9] *
      kC->l3 * muDoubleScalarSin(nMid1_data[3]) * muDoubleScalarSin(nMid1_data[4]);
    qDot[2] = -nMid1_data[9] * kC->l3 * muDoubleScalarCos(nMid1_data[4]) -
      kC->l5 * nMid1_data[10] * muDoubleScalarCos(kC->zeta + nMid1_data[5]);
    b_qDot[0] = (-nMid2_data[8] * muDoubleScalarSin(nMid2_data[3]) * ((((kC->l2
      - kC->l7) + kC->l5 * muDoubleScalarCos(nMid2_data[5] + kC->zeta)) + kC->l3
      * muDoubleScalarCos(nMid2_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta))
                 - nMid2_data[9] * kC->l3 * muDoubleScalarCos(nMid2_data[3]) *
                 muDoubleScalarSin(nMid2_data[4])) - nMid2_data[10] * kC->l5 *
      muDoubleScalarSin(nMid2_data[5] + kC->zeta) * muDoubleScalarCos
      (nMid2_data[3]);
    b_qDot[1] = (nMid2_data[8] * muDoubleScalarCos(nMid2_data[3]) * ((((kC->l2 -
      kC->l7) + kC->l5 * muDoubleScalarCos(nMid2_data[5] + kC->zeta)) + kC->l3 *
      muDoubleScalarCos(nMid2_data[4])) + kC->l4 * muDoubleScalarCos(kC->zeta))
                 - nMid2_data[10] * kC->l5 * muDoubleScalarSin(nMid2_data[5] +
      kC->zeta) * muDoubleScalarSin(nMid2_data[3])) - nMid2_data[9] * kC->l3 *
      muDoubleScalarSin(nMid2_data[3]) * muDoubleScalarSin(nMid2_data[4]);
    b_qDot[2] = -nMid2_data[9] * kC->l3 * muDoubleScalarCos(nMid2_data[4]) -
      kC->l5 * nMid2_data[10] * muDoubleScalarCos(kC->zeta + nMid2_data[5]);
    for (i3 = 0; i3 < 3; i3++) {
      q[i3] = uB[i3] - uA[i3];
      c_qDot[i3] = qDot[i3] - b_qDot[i3];
    }

    nMid2_data[2] = nMid1_data[2] + (norm(q) + 0.0 * b_norm(c_qDot));

    /* Cost */
    i3 = (int32_T)(nodeIDCount1 + 1.0);
    emlrtDynamicBoundsCheckFastR2012b(i3, 1, 1000, &g_emlrtBCI, &st);
    cdiff = T1->size[1];
    i3 = r0->size[0];
    r0->size[0] = cdiff;
    emxEnsureCapacity(&st, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                      &b_emlrtRTEI);
    for (i3 = 0; i3 < cdiff; i3++) {
      r0->data[i3] = i3;
    }

    i3 = unusedU4->size[0] * unusedU4->size[1];
    unusedU4->size[0] = 1;
    unusedU4->size[1] = 13 + transitionArray->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)unusedU4, i3, (int32_T)sizeof
                      (real_T), &b_emlrtRTEI);
    for (i3 = 0; i3 < 13; i3++) {
      unusedU4->data[unusedU4->size[0] * i3] = nMid2_data[nMid2_size[0] * i3];
    }

    cdiff = transitionArray->size[1];
    for (i3 = 0; i3 < cdiff; i3++) {
      unusedU4->data[unusedU4->size[0] * (i3 + 13)] = transitionArray->
        data[transitionArray->size[0] * i3];
    }

    iv5[0] = 1;
    iv5[1] = r0->size[0];
    emlrtSubAssignSizeCheckR2012b(iv5, 2, *(int32_T (*)[2])unusedU4->size, 2,
      &d_emlrtECI, &st);
    cdiff = unusedU4->size[1];
    for (i3 = 0; i3 < cdiff; i3++) {
      b_T1->data[((int32_T)(nodeIDCount1 + 1.0) + b_T1->size[0] * r0->data[i3])
        - 1] = unusedU4->data[unusedU4->size[0] * i3];
    }

    /* Append the new node to the tree.     */
    /* if mod(nodeIDCount,100) == 0 */
    /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
    /* end */
    nodeIDCount1++;

    /* Swap the trees. */
    i3 = T1->size[0] * T1->size[1];
    T1->size[0] = 1000;
    T1->size[1] = T2->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)T1, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    cdiff = T2->size[0] * T2->size[1];
    for (i3 = 0; i3 < cdiff; i3++) {
      T1->data[i3] = T2->data[i3];
    }

    i3 = T2->size[0] * T2->size[1];
    T2->size[0] = 1000;
    T2->size[1] = b_T1->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)T2, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    cdiff = b_T1->size[0] * b_T1->size[1];
    for (i3 = 0; i3 < cdiff; i3++) {
      T2->data[i3] = b_T1->data[i3];
    }

    /* Swap the trees. */
    transitionArrayLength = nodeIDCount1;
    nodeIDCount1 = nodeIDCount2;
    nodeIDCount2 = transitionArrayLength;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&transitionArray);
  emxFree_int32_T(&r0);
  emxFree_real_T(&b_T1);
  emxInit_real_T(sp, &pathT1, 2, &e_emlrtRTEI, true);
  emxInit_real_T(sp, &pathT2, 2, &f_emlrtRTEI, true);
  b_emxInit_real_T(sp, &t, 1, &g_emlrtRTEI, true);
  emxInit_real_T(sp, &path, 2, &h_emlrtRTEI, true);
  emxInit_real_T(sp, &r1, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &b_t, 2, &b_emlrtRTEI, true);
  for (i = 0; i < 1000; i++) {
    i3 = T1->size[1];
    emlrtDynamicBoundsCheckFastR2012b(1, 1, i3, &f_emlrtBCI, sp);
    i3 = T1->size[1];
    emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, i3, &f_emlrtBCI, sp);
    for (i3 = 0; i3 < 13; i3++) {
      nMid1_data[i3] = T1->data[i + T1->size[0] * i3];
    }

    st.site = &k_emlrtRSI;
    b_nearestNeighbour(&st, nMid1_data, T2, kC->l1, kC->l2, kC->l3, kC->l4,
                       kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, 13,
                       nMid2_data, nMid2_size, unusedU4, &d);
    if (d < 0.04) {
      st.site = &j_emlrtRSI;
      traceBranch(&st, T1, nMid1_data, pathT1);
      st.site = &i_emlrtRSI;
      traceBranch(&st, T2, nMid2_data, pathT2);
      i3 = T1->size[1];
      emlrtDynamicBoundsCheckFastR2012b(4, 1, i3, &e_emlrtBCI, sp);
      guard1 = false;
      if (T1->data[T1->size[0] * 3] == nInit[3]) {
        i3 = T1->size[1];
        emlrtDynamicBoundsCheckFastR2012b(5, 1, i3, &d_emlrtBCI, sp);
        if (T1->data[T1->size[0] << 2] == nInit[4]) {
          i3 = T1->size[1];
          emlrtDynamicBoundsCheckFastR2012b(6, 1, i3, &c_emlrtBCI, sp);
          if (T1->data[T1->size[0] * 5] == nInit[5]) {
            flipud(pathT2);
            i3 = path->size[0] * path->size[1];
            path->size[0] = pathT1->size[0] + pathT2->size[0];
            path->size[1] = 10;
            emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof
                              (real_T), &b_emlrtRTEI);
            for (i3 = 0; i3 < 10; i3++) {
              cdiff = pathT1->size[0];
              for (ndbl = 0; ndbl < cdiff; ndbl++) {
                path->data[ndbl + path->size[0] * i3] = pathT1->data[ndbl +
                  pathT1->size[0] * i3];
              }
            }

            for (i3 = 0; i3 < 10; i3++) {
              cdiff = pathT2->size[0];
              for (ndbl = 0; ndbl < cdiff; ndbl++) {
                path->data[(ndbl + pathT1->size[0]) + path->size[0] * i3] =
                  pathT2->data[ndbl + pathT2->size[0] * i3];
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        flipud(pathT1);
        i3 = path->size[0] * path->size[1];
        path->size[0] = pathT2->size[0] + pathT1->size[0];
        path->size[1] = 10;
        emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        for (i3 = 0; i3 < 10; i3++) {
          cdiff = pathT2->size[0];
          for (ndbl = 0; ndbl < cdiff; ndbl++) {
            path->data[ndbl + path->size[0] * i3] = pathT2->data[ndbl +
              pathT2->size[0] * i3];
          }
        }

        for (i3 = 0; i3 < 10; i3++) {
          cdiff = pathT1->size[0];
          for (ndbl = 0; ndbl < cdiff; ndbl++) {
            path->data[(ndbl + pathT2->size[0]) + path->size[0] * i3] =
              pathT1->data[ndbl + pathT1->size[0] * i3];
          }
        }
      }

      st.site = &h_emlrtRSI;
      b_st.site = &rb_emlrtRSI;
      c_st.site = &sb_emlrtRSI;
      if (path->size[0] < 1) {
        absb = -1;
        apnd = 0;
      } else {
        ndbl = (int32_T)muDoubleScalarFloor(((real_T)path->size[0] - 1.0) + 0.5);
        apnd = ndbl + 1;
        cdiff = (ndbl - path->size[0]) + 1;
        absb = path->size[0];
        if (muDoubleScalarAbs(cdiff) < 4.4408920985006262E-16 * (real_T)absb) {
          ndbl++;
          apnd = path->size[0];
        } else if (cdiff > 0) {
          apnd = ndbl;
        } else {
          ndbl++;
        }

        absb = ndbl - 1;
      }

      i3 = unusedU4->size[0] * unusedU4->size[1];
      unusedU4->size[0] = 1;
      unusedU4->size[1] = absb + 1;
      emxEnsureCapacity(&c_st, (emxArray__common *)unusedU4, i3, (int32_T)sizeof
                        (real_T), &c_emlrtRTEI);
      if (absb + 1 > 0) {
        unusedU4->data[0] = 1.0;
        if (absb + 1 > 1) {
          unusedU4->data[absb] = apnd;
          i3 = absb + (absb < 0);
          if (i3 >= 0) {
            ndbl = (int32_T)((uint32_T)i3 >> 1);
          } else {
            ndbl = (int32_T)~(~(uint32_T)i3 >> 1);
          }

          for (cdiff = 1; cdiff < ndbl; cdiff++) {
            unusedU4->data[cdiff] = 1.0 + (real_T)cdiff;
            unusedU4->data[absb - cdiff] = apnd - cdiff;
          }

          if (ndbl << 1 == absb) {
            unusedU4->data[ndbl] = (1.0 + (real_T)apnd) / 2.0;
          } else {
            unusedU4->data[ndbl] = 1.0 + (real_T)ndbl;
            unusedU4->data[ndbl + 1] = apnd - ndbl;
          }
        }
      }

      i3 = t->size[0];
      t->size[0] = unusedU4->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)t, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      cdiff = unusedU4->size[1];
      for (i3 = 0; i3 < cdiff; i3++) {
        t->data[i3] = dt * unusedU4->data[unusedU4->size[0] * i3];
      }

      ndbl = t->size[0];
      i3 = path->size[0];
      emlrtDimSizeEqCheckFastR2012b(ndbl, i3, &emlrtECI, sp);
      ndbl = t->size[0];
      i3 = b_t->size[0] * b_t->size[1];
      b_t->size[0] = ndbl;
      b_t->size[1] = 1 + path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)b_t, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i3 = 0; i3 < ndbl; i3++) {
        b_t->data[i3] = t->data[i3];
      }

      cdiff = path->size[1];
      for (i3 = 0; i3 < cdiff; i3++) {
        absb = path->size[0];
        for (ndbl = 0; ndbl < absb; ndbl++) {
          b_t->data[ndbl + b_t->size[0] * (i3 + 1)] = path->data[ndbl +
            path->size[0] * i3];
        }
      }

      i3 = path->size[0] * path->size[1];
      path->size[0] = b_t->size[0];
      path->size[1] = b_t->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      cdiff = b_t->size[1];
      for (i3 = 0; i3 < cdiff; i3++) {
        absb = b_t->size[0];
        for (ndbl = 0; ndbl < absb; ndbl++) {
          path->data[ndbl + path->size[0] * i3] = b_t->data[ndbl + b_t->size[0] *
            i3];
        }
      }

      st.site = &g_emlrtRSI;
      transformPath(&st, path, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                    kC->l7, kC->l8, kC->zeta, kC->r, TP2B, r1);
      i3 = pathC->size[0] * pathC->size[1];
      pathC->size[0] = r1->size[0];
      pathC->size[1] = 9;
      emxEnsureCapacity(sp, (emxArray__common *)pathC, i3, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      cdiff = r1->size[0] * r1->size[1];
      for (i3 = 0; i3 < cdiff; i3++) {
        pathC->data[i3] = r1->data[i3];
      }

      i3 = pathC->size[0];
      ndbl = pathC->size[0];
      emlrtDynamicBoundsCheckFastR2012b(ndbl, 1, i3, &b_emlrtBCI, sp);
      if (pathC->data[(pathC->size[0] + pathC->size[0]) - 1] < pathLengthMin) {
        pathLengthMin = pathC->data[(pathC->size[0] + pathC->size[0]) - 1];
        i3 = pathCMin->size[0] * pathCMin->size[1];
        pathCMin->size[0] = pathC->size[0];
        pathCMin->size[1] = pathC->size[1];
        emxEnsureCapacity(sp, (emxArray__common *)pathCMin, i3, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        cdiff = pathC->size[0] * pathC->size[1];
        for (i3 = 0; i3 < cdiff; i3++) {
          pathCMin->data[i3] = pathC->data[i3];
        }

        i3 = pathJ->size[0] * pathJ->size[1];
        pathJ->size[0] = path->size[0];
        pathJ->size[1] = path->size[1];
        emxEnsureCapacity(sp, (emxArray__common *)pathJ, i3, (int32_T)sizeof
                          (real_T), &b_emlrtRTEI);
        cdiff = path->size[0] * path->size[1];
        for (i3 = 0; i3 < cdiff; i3++) {
          pathJ->data[i3] = path->data[i3];
        }
      }
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&b_t);
  emxFree_real_T(&r1);
  emxFree_real_T(&path);
  emxFree_real_T(&unusedU4);
  emxFree_real_T(&t);
  emxFree_real_T(&pathT2);
  emxFree_real_T(&pathT1);
  i3 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = pathCMin->size[0];
  pathC->size[1] = pathCMin->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)pathC, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  cdiff = pathCMin->size[0] * pathCMin->size[1];
  for (i3 = 0; i3 < cdiff; i3++) {
    pathC->data[i3] = pathCMin->data[i3];
  }

  emxFree_real_T(&pathCMin);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildBiDirectionalRRT.c) */
