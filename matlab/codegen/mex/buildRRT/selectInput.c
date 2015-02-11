/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "selectInput.h"
#include "buildRRT_emxutil.h"
#include "calcPhi.h"
#include "heuristicSingleLeg.h"
#include "eml_int_forloop_overflow_check.h"
#include "buildRRT_mexutil.h"
#include "buildRRT_data.h"

/* Variable Definitions */
static emlrtRSInfo ab_emlrtRSI = { 20, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRSInfo bb_emlrtRSI = { 37, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRSInfo cb_emlrtRSI = { 41, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRSInfo db_emlrtRSI = { 47, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRSInfo eb_emlrtRSI = { 79, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRSInfo fb_emlrtRSI = { 18, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtRTEInfo d_emlrtRTEI = { 1, 35, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 11, 5, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 12, 5, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRTEInfo g_emlrtRTEI = { 13, 5, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtRTEInfo h_emlrtRTEI = { 17, 5, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtBCInfo m_emlrtBCI = { -1, -1, 15, 14, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo n_emlrtBCI = { 1, 5, 21, 24, "U", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo o_emlrtBCI = { -1, -1, 22, 19, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo p_emlrtBCI = { -1, -1, 23, 20, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo q_emlrtBCI = { -1, -1, 24, 16, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo r_emlrtBCI = { -1, -1, 25, 17, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo s_emlrtBCI = { -1, -1, 30, 17, "U_joint", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 41, 62, "U_joint", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo u_emlrtBCI = { -1, -1, 41, 21, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtECInfo c_emlrtECI = { -1, 41, 10, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtBCInfo v_emlrtBCI = { -1, -1, 43, 45, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo w_emlrtBCI = { -1, -1, 43, 64, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo x_emlrtBCI = { -1, -1, 47, 47, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo y_emlrtBCI = { -1, -1, 53, 12, "xNear", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo ab_emlrtBCI = { -1, -1, 61, 66, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo bb_emlrtBCI = { -1, -1, 61, 84, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo cb_emlrtBCI = { -1, -1, 61, 23, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo db_emlrtBCI = { -1, -1, 61, 40, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 62, 43, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 62, 45, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo gb_emlrtBCI = { -1, -1, 80, 23, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo hb_emlrtBCI = { -1, -1, 41, 42, "candTransArrays",
  "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtECInfo d_emlrtECI = { -1, 41, 26, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m"
};

static emlrtBCInfo ib_emlrtBCI = { -1, -1, 81, 39, "candTransArrays",
  "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo jb_emlrtBCI = { -1, -1, 11, 13, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtRTEInfo n_emlrtRTEI = { 16, 5, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtECInfo e_emlrtECI = { 2, 19, 16, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtECInfo f_emlrtECI = { 2, 20, 16, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtECInfo g_emlrtECI = { 2, 21, 16, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtECInfo h_emlrtECI = { 2, 22, 16, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtBCInfo kb_emlrtBCI = { -1, -1, 26, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo lb_emlrtBCI = { -1, -1, 33, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo mb_emlrtBCI = { -1, -1, 34, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo nb_emlrtBCI = { -1, -1, 43, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo ob_emlrtBCI = { -1, -1, 49, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo pb_emlrtBCI = { -1, -1, 50, 23, "xInit", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtECInfo i_emlrtECI = { -1, 14, 5, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtBCInfo qb_emlrtBCI = { -1, -1, 56, 9, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtECInfo j_emlrtECI = { -1, 56, 9, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m" };

static emlrtDCInfo f_emlrtDCI = { 11, 24, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  4 };

static emlrtDCInfo g_emlrtDCI = { 11, 31, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  4 };

static emlrtDCInfo h_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  1 };

static emlrtDCInfo i_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  4 };

static emlrtDCInfo j_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 1 };

static emlrtDCInfo k_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 4 };

static emlrtBCInfo rb_emlrtBCI = { -1, -1, 14, 5, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

static emlrtBCInfo sb_emlrtBCI = { -1, -1, 43, 47, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo tb_emlrtBCI = { -1, -1, 43, 66, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo ub_emlrtBCI = { -1, -1, 61, 68, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo vb_emlrtBCI = { -1, -1, 61, 86, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo wb_emlrtBCI = { -1, -1, 61, 21, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo xb_emlrtBCI = { -1, -1, 61, 38, "candStates", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo yb_emlrtBCI = { -1, -1, 74, 9, "distance", "selectInput",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/selectInput.m",
  0 };

static emlrtBCInfo ac_emlrtBCI = { -1, -1, 65, 13, "x", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/rk4.m", 0 };

/* Function Definitions */
void selectInput(const emlrtStack *sp, const emxArray_real_T *xNear, const
                 real_T xRand_data[], const int32_T xRand_size[2], const real_T
                 U[10], real_T dt, real_T Dt, int32_T NODE_SIZE, int32_T U_SIZE,
                 const real_T HGAINS[3], const real_T kinematicConst[12], real_T
                 ankleThreshold, const real_T jointLimits[12], emxArray_real_T
                 *xNew, emxArray_real_T *transitionArray)
{
  emxArray_real_T *candStates;
  int32_T i2;
  int32_T loop_ub;
  emxArray_real_T *candTransArrays;
  real_T x;
  real_T d0;
  emxArray_real_T *distance;
  emxArray_real_T *U_joint;
  int32_T ixstart;
  boolean_T b1;
  int32_T i;
  emxArray_real_T *r1;
  emxArray_real_T *r2;
  emxArray_int32_T *r3;
  emxArray_real_T *xInit;
  emxArray_real_T *b_candStates;
  emxArray_real_T *r4;
  int32_T n;
  real_T u[3];
  real_T numIterations;
  int32_T ix;
  real_T k1[6];
  int32_T b_xInit[2];
  int32_T iv5[2];
  real_T c_xInit[6];
  real_T k2[6];
  real_T k3[6];
  real_T hDiff;
  real_T d_xInit[6];
  int32_T iv6[2];
  int32_T iv7[2];
  emxArray_real_T b_xRand_data;
  real_T c_candStates[3];
  int32_T i3;
  boolean_T overflow;
  const mxArray *y;
  static const int32_T iv8[2] = { 1, 36 };

  const mxArray *m2;
  char_T cv8[36];
  static const char_T cv9[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  const mxArray *b_y;
  static const int32_T iv9[2] = { 1, 39 };

  char_T cv10[39];
  static const char_T cv11[39] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'e', 'm', 'l', '_', 'm', 'i', 'n', '_', 'o', 'r',
    '_', 'm', 'a', 'x', '_', 'v', 'a', 'r', 'D', 'i', 'm', 'Z', 'e', 'r', 'o' };

  boolean_T exitg1;
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
  emxInit_real_T(sp, &candStates, 2, &e_emlrtRTEI, true);

  /* selectInput Selects the most appropriate control input. */
  /*    A control input is selected from a set of control inputs, U. An input */
  /*    is selected by applying each of the inputs to to state xNear, which */
  /*    results in p candidate states, where p is the size of the input set. */
  /*    The control input corresponding to candidate state that is closest to */
  /*    x1 is returned as u. */
  /* Initialize arrays to store the candidate new state data and the */
  /* distances between each candidate state and the xNear state. */
  i2 = candStates->size[0] * candStates->size[1];
  candStates->size[0] = (int32_T)emlrtNonNegativeCheckFastR2012b(U_SIZE,
    &f_emlrtDCI, sp);
  candStates->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(NODE_SIZE,
    &g_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candStates, i2, (int32_T)sizeof
                    (real_T), &d_emlrtRTEI);
  loop_ub = (int32_T)emlrtNonNegativeCheckFastR2012b(U_SIZE, &f_emlrtDCI, sp) *
    (int32_T)emlrtNonNegativeCheckFastR2012b(NODE_SIZE, &g_emlrtDCI, sp);
  for (i2 = 0; i2 < loop_ub; i2++) {
    candStates->data[i2] = 0.0;
  }

  emxInit_real_T(sp, &candTransArrays, 2, &f_emlrtRTEI, true);
  x = muDoubleScalarRound(Dt / dt);
  i2 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = U_SIZE;
  d0 = (x + 1.0) * 6.0;
  d0 = emlrtNonNegativeCheckFastR2012b(d0, &i_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d0,
    &h_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i2, (int32_T)sizeof
                    (real_T), &d_emlrtRTEI);
  d0 = (x + 1.0) * 6.0;
  d0 = emlrtNonNegativeCheckFastR2012b(d0, &i_emlrtDCI, sp);
  loop_ub = U_SIZE * (int32_T)emlrtIntegerCheckFastR2012b(d0, &h_emlrtDCI, sp);
  for (i2 = 0; i2 < loop_ub; i2++) {
    candTransArrays->data[i2] = 0.0;
  }

  emxInit_real_T(sp, &distance, 2, &g_emlrtRTEI, true);
  i2 = distance->size[0] * distance->size[1];
  distance->size[0] = 1;
  distance->size[1] = U_SIZE;
  emxEnsureCapacity(sp, (emxArray__common *)distance, i2, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  for (i2 = 0; i2 < U_SIZE; i2++) {
    distance->data[i2] = 0.0;
  }

  emxInit_real_T(sp, &U_joint, 2, &h_emlrtRTEI, true);
  i2 = xNear->size[1];
  ixstart = xNear->size[1];
  emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, i2, &m_emlrtBCI, sp);
  i2 = U_joint->size[0] * U_joint->size[1];
  U_joint->size[0] = U_SIZE;
  U_joint->size[1] = 3;
  emxEnsureCapacity(sp, (emxArray__common *)U_joint, i2, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  loop_ub = U_SIZE * 3;
  for (i2 = 0; i2 < loop_ub; i2++) {
    U_joint->data[i2] = 0.0;
  }

  /* Transform the control inputs to joint space. */
  st.site = &ab_emlrtRSI;
  if (1 > U_SIZE) {
    b1 = false;
  } else {
    b1 = (U_SIZE > 2147483646);
  }

  if (b1) {
    b_st.site = &f_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  i = 1;
  while (i <= U_SIZE) {
    emlrtDynamicBoundsCheckFastR2012b(i, 1, 5, &n_emlrtBCI, sp);
    i2 = xNear->size[1];
    emlrtDynamicBoundsCheckFastR2012b(8, 1, i2, &o_emlrtBCI, sp);
    i2 = xNear->size[1];
    emlrtDynamicBoundsCheckFastR2012b(9, 1, i2, &p_emlrtBCI, sp);
    i2 = xNear->size[1];
    emlrtDynamicBoundsCheckFastR2012b(5, 1, i2, &q_emlrtBCI, sp);
    i2 = xNear->size[1];
    emlrtDynamicBoundsCheckFastR2012b(6, 1, i2, &r_emlrtBCI, sp);
    ixstart = U_joint->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, ixstart, &s_emlrtBCI, sp);
    for (i2 = 0; i2 < 2; i2++) {
      U_joint->data[(i + U_joint->size[0] * i2) - 1] = U[(i + 5 * i2) - 1];
    }

    U_joint->data[(i + (U_joint->size[0] << 1)) - 1] = ((-U[i + 4] *
      kinematicConst[2] * muDoubleScalarCos(xNear->data[4]) + xNear->data[7] *
      xNear->data[7] * kinematicConst[2] * muDoubleScalarSin(xNear->data[4])) +
      xNear->data[8] * xNear->data[8] * kinematicConst[4] * muDoubleScalarSin
      (kinematicConst[8] + xNear->data[5])) / (kinematicConst[4] *
      muDoubleScalarCos(kinematicConst[8] + xNear->data[5]));

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  st.site = &bb_emlrtRSI;
  i = 1;
  emxInit_real_T(sp, &r1, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &r2, 2, &d_emlrtRTEI, true);
  emxInit_int32_T(sp, &r3, 1, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &xInit, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &b_candStates, 2, &d_emlrtRTEI, true);
  emxInit_real_T(sp, &r4, 2, &d_emlrtRTEI, true);
  while (i <= U_SIZE) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    st.site = &cb_emlrtRSI;
    i2 = U_joint->size[0];
    n = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &t_emlrtBCI, &st);
    for (i2 = 0; i2 < 3; i2++) {
      u[i2] = U_joint->data[(n + U_joint->size[0] * i2) - 1];
    }

    /* rk4.m */
    /* author: wreid */
    /* date: 20150107 */
    /* rk4 Summary of this function goes here */
    /*    Detailed explanation goes here */
    numIterations = muDoubleScalarRound(Dt / dt);
    i2 = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    emxEnsureCapacity(&st, (emxArray__common *)r1, i2, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    ixstart = xNear->size[1];
    i2 = r1->size[0] * r1->size[1];
    r1->size[1] = ixstart;
    emxEnsureCapacity(&st, (emxArray__common *)r1, i2, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    loop_ub = xNear->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      r1->data[i2] = 0.0;
    }

    if (4 > xNear->size[1] - 2) {
      i2 = 1;
      ixstart = 1;
    } else {
      i2 = xNear->size[1];
      emlrtDynamicBoundsCheckFastR2012b(4, 1, i2, &jb_emlrtBCI, &st);
      i2 = 4;
      ixstart = xNear->size[1];
      ix = xNear->size[1] - 2;
      ixstart = emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &jb_emlrtBCI,
        &st) + 1;
    }

    ix = xInit->size[0] * xInit->size[1];
    xInit->size[0] = 1;
    xInit->size[1] = ixstart - i2;
    emxEnsureCapacity(&st, (emxArray__common *)xInit, ix, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    loop_ub = ixstart - i2;
    for (ix = 0; ix < loop_ub; ix++) {
      xInit->data[xInit->size[0] * ix] = xNear->data[(i2 + ix) - 1];
    }

    /* xInitOrig = xInit; */
    ix = r2->size[0] * r2->size[1];
    r2->size[0] = 1;
    d0 = (numIterations + 1.0) * 6.0;
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &k_emlrtDCI, &st);
    r2->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d0, &j_emlrtDCI, &st);
    emxEnsureCapacity(&st, (emxArray__common *)r2, ix, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    d0 = (numIterations + 1.0) * 6.0;
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &k_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d0, &j_emlrtDCI, &st);
    for (ix = 0; ix < loop_ub; ix++) {
      r2->data[ix] = 0.0;
    }

    i2 = ixstart - i2;
    emlrtSizeEqCheck1DFastR2012b(6, i2, &i_emlrtECI, &st);
    ixstart = (int32_T)((numIterations + 1.0) * 6.0);
    for (i2 = 0; i2 < 6; i2++) {
      r2->data[emlrtDynamicBoundsCheckFastR2012b(i2 + 1, 1, ixstart,
        &rb_emlrtBCI, &st) - 1] = xInit->data[i2];
    }

    emlrtForLoopVectorCheckR2012b(1.0, 1.0, numIterations, mxDOUBLE_CLASS,
      (int32_T)numIterations, &n_emlrtRTEI, &st);
    n = 0;
    while (n <= (int32_T)numIterations - 1) {
      b_st.site = &fb_emlrtRSI;
      for (i2 = 0; i2 < 3; i2++) {
        ixstart = xInit->size[1];
        ix = 4 + i2;
        k1[i2] = xInit->data[emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart,
          &ac_emlrtBCI, &b_st) - 1];
      }

      for (i2 = 0; i2 < 3; i2++) {
        k1[i2 + 3] = u[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        b_xInit[i2] = xInit->size[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        iv5[i2] = 1 + 5 * i2;
      }

      emlrtSizeEqCheck2DFastR2012b(b_xInit, iv5, &e_emlrtECI, &st);
      x = dt / 2.0;
      for (i2 = 0; i2 < 6; i2++) {
        c_xInit[i2] = xInit->data[i2] + x * k1[i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        k2[i2] = c_xInit[3 + i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        k2[i2 + 3] = u[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        b_xInit[i2] = xInit->size[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        iv5[i2] = 1 + 5 * i2;
      }

      emlrtSizeEqCheck2DFastR2012b(b_xInit, iv5, &f_emlrtECI, &st);
      x = dt / 2.0;
      for (i2 = 0; i2 < 6; i2++) {
        c_xInit[i2] = xInit->data[i2] + x * k2[i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        k3[i2] = c_xInit[3 + i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        k3[i2 + 3] = u[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        b_xInit[i2] = xInit->size[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        iv5[i2] = 1 + 5 * i2;
      }

      emlrtSizeEqCheck2DFastR2012b(b_xInit, iv5, &g_emlrtECI, &st);
      x = dt / 2.0;
      for (i2 = 0; i2 < 2; i2++) {
        b_xInit[i2] = xInit->size[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        iv5[i2] = 1 + 5 * i2;
      }

      emlrtSizeEqCheck2DFastR2012b(b_xInit, iv5, &h_emlrtECI, &st);
      hDiff = dt / 6.0;
      for (i2 = 0; i2 < 6; i2++) {
        c_xInit[i2] = xInit->data[i2] + x * k3[i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        d_xInit[i2] = c_xInit[3 + i2];
      }

      for (i2 = 0; i2 < 3; i2++) {
        d_xInit[i2 + 3] = u[i2];
      }

      i2 = r1->size[0] * r1->size[1];
      r1->size[0] = 1;
      r1->size[1] = 6;
      emxEnsureCapacity(&st, (emxArray__common *)r1, i2, (int32_T)sizeof(real_T),
                        &d_emlrtRTEI);
      for (i2 = 0; i2 < 6; i2++) {
        r1->data[r1->size[0] * i2] = xInit->data[i2] + hDiff * (((k1[i2] + 2.0 *
          k2[i2]) + 2.0 * k3[i2]) + d_xInit[i2]);
      }

      /* Check pan angular position limits */
      if ((r1->data[0] > jointLimits[1]) || (r1->data[0] < jointLimits[0])) {
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(1, 1, i2, &kb_emlrtBCI, &st);
        r1->data[0] = xInit->data[0];
        r1->data[3] = 0.0;
        u[0] = 0.0;
      }

      /* Check inner and outer leg angular position limits */
      if ((r1->data[1] > jointLimits[3]) || (r1->data[1] < jointLimits[2]) ||
          (r1->data[2] > jointLimits[5]) || (r1->data[2] < jointLimits[4])) {
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(2, 1, i2, &lb_emlrtBCI, &st);
        r1->data[1] = xInit->data[1];
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(3, 1, i2, &mb_emlrtBCI, &st);
        r1->data[2] = xInit->data[2];
        r1->data[4] = 0.0;
        r1->data[5] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
      }

      /* Check pan angular velocity limits */
      if ((r1->data[3] > jointLimits[7]) || (r1->data[3] < jointLimits[6])) {
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(4, 1, i2, &nb_emlrtBCI, &st);
        r1->data[3] = xInit->data[3];
        u[0] = 0.0;
      }

      /* Check inner and outer leg angular velocity limits */
      if ((r1->data[4] > jointLimits[9]) || (r1->data[4] < jointLimits[8]) ||
          (r1->data[5] > jointLimits[11]) || (r1->data[5] < jointLimits[10])) {
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(5, 1, i2, &ob_emlrtBCI, &st);
        r1->data[4] = xInit->data[4];
        i2 = xInit->size[1];
        emlrtDynamicBoundsCheckFastR2012b(6, 1, i2, &pb_emlrtBCI, &st);
        r1->data[5] = xInit->data[5];
        u[1] = 0.0;
        u[2] = 0.0;
      }

      i2 = xInit->size[0] * xInit->size[1];
      xInit->size[0] = 1;
      xInit->size[1] = r1->size[1];
      emxEnsureCapacity(&st, (emxArray__common *)xInit, i2, (int32_T)sizeof
                        (real_T), &d_emlrtRTEI);
      loop_ub = r1->size[0] * r1->size[1];
      for (i2 = 0; i2 < loop_ub; i2++) {
        xInit->data[i2] = r1->data[i2];
      }

      d0 = 6.0 * (1.0 + (real_T)n) + 1.0;
      x = 6.0 * ((1.0 + (real_T)n) + 1.0);
      if (d0 > x) {
        i2 = 1;
        ixstart = 1;
      } else {
        i2 = r2->size[1];
        ixstart = (int32_T)d0;
        i2 = emlrtDynamicBoundsCheckFastR2012b(ixstart, 1, i2, &qb_emlrtBCI, &st);
        ixstart = r2->size[1];
        ix = (int32_T)x;
        ixstart = emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &qb_emlrtBCI,
          &st) + 1;
      }

      ixstart -= i2;
      emlrtSizeEqCheck1DFastR2012b(ixstart, 6, &j_emlrtECI, &st);
      loop_ub = r1->size[1];
      for (ixstart = 0; ixstart < loop_ub; ixstart++) {
        r2->data[(i2 + ixstart) - 1] = r1->data[r1->size[0] * ixstart];
      }

      n++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    /* xInit = [zeros(1,3) xInitOrig 0 0]; */
    i2 = r4->size[0] * r4->size[1];
    r4->size[0] = 1;
    r4->size[1] = 5 + r1->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)r4, i2, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    for (i2 = 0; i2 < 3; i2++) {
      r4->data[r4->size[0] * i2] = 0.0;
    }

    loop_ub = r1->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      r4->data[r4->size[0] * (i2 + 3)] = r1->data[r1->size[0] * i2];
    }

    r4->data[r4->size[0] * (3 + r1->size[1])] = 0.0;
    r4->data[r4->size[0] * (4 + r1->size[1])] = 0.0;
    i2 = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = r4->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)r1, i2, (int32_T)sizeof(real_T),
                      &d_emlrtRTEI);
    loop_ub = r4->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      r1->data[r1->size[0] * i2] = r4->data[r4->size[0] * i2];
    }

    i2 = candStates->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &u_emlrtBCI, sp);
    loop_ub = candStates->size[1];
    i2 = r3->size[0];
    r3->size[0] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)r3, i2, (int32_T)sizeof(int32_T),
                      &d_emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      r3->data[i2] = i2;
    }

    iv6[0] = 1;
    iv6[1] = r3->size[0];
    emlrtSubAssignSizeCheckR2012b(iv6, 2, *(int32_T (*)[2])r1->size, 2,
      &c_emlrtECI, sp);
    loop_ub = r1->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      candStates->data[(i + candStates->size[0] * r3->data[i2]) - 1] = r1->
        data[r1->size[0] * i2];
    }

    i2 = candTransArrays->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &hb_emlrtBCI, sp);
    loop_ub = candTransArrays->size[1];
    i2 = r3->size[0];
    r3->size[0] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)r3, i2, (int32_T)sizeof(int32_T),
                      &d_emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      r3->data[i2] = i2;
    }

    iv7[0] = 1;
    iv7[1] = r3->size[0];
    emlrtSubAssignSizeCheckR2012b(iv7, 2, *(int32_T (*)[2])r2->size, 2,
      &d_emlrtECI, sp);
    loop_ub = r2->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      candTransArrays->data[(i + candTransArrays->size[0] * r3->data[i2]) - 1] =
        r2->data[r2->size[0] * i2];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    i2 = candStates->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &v_emlrtBCI, sp);
    for (i2 = 0; i2 < 3; i2++) {
      ixstart = candStates->size[1];
      ix = 7 + i2;
      emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &sb_emlrtBCI, sp);
    }

    i2 = candStates->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &w_emlrtBCI, sp);
    for (i2 = 0; i2 < 3; i2++) {
      ixstart = candStates->size[1];
      ix = 4 + i2;
      emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &tb_emlrtBCI, sp);
    }

    /* Calculate the distance between the candidate state and the random */
    /* state. */
    loop_ub = candStates->size[1];
    i2 = candStates->size[0];
    n = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &x_emlrtBCI, sp);
    i2 = b_candStates->size[0] * b_candStates->size[1];
    b_candStates->size[0] = 1;
    b_candStates->size[1] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)b_candStates, i2, (int32_T)sizeof
                      (real_T), &d_emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_candStates->data[b_candStates->size[0] * i2] = candStates->data[(n +
        candStates->size[0] * i2) - 1];
    }

    b_xRand_data.data = (real_T *)xRand_data;
    b_xRand_data.size = (int32_T *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    st.site = &db_emlrtRSI;
    hDiff = heuristicSingleLeg(&st, b_candStates, &b_xRand_data, kinematicConst);

    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    numIterations = HGAINS[2];
    i2 = xNear->size[1];
    emlrtDynamicBoundsCheckFastR2012b(1, 1, i2, &y_emlrtBCI, sp);
    if (xNear->data[0] == 1.0) {
      numIterations = 0.0;
    }

    /* Ss = [1000 1]; */
    /* qWDot = 1; */
    /* r = 0.378/2; */
    i2 = candStates->size[0];
    n = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &ab_emlrtBCI, sp);
    for (i2 = 0; i2 < 3; i2++) {
      ixstart = candStates->size[1];
      ix = 7 + i2;
      u[i2] = candStates->data[(n + candStates->size[0] *
        (emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &ub_emlrtBCI, sp) - 1))
        - 1];
    }

    i2 = candStates->size[0];
    n = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &bb_emlrtBCI, sp);
    for (i2 = 0; i2 < 3; i2++) {
      ixstart = candStates->size[1];
      ix = 4 + i2;
      c_candStates[i2] = candStates->data[(n + candStates->size[0] *
        (emlrtDynamicBoundsCheckFastR2012b(ix, 1, ixstart, &vb_emlrtBCI, sp) - 1))
        - 1];
    }

    calcPhi(u, c_candStates, kinematicConst, &d0, &x);
    i2 = candStates->size[1];
    emlrtDynamicBoundsCheckFastR2012b(10, 1, i2, &cb_emlrtBCI, sp);
    i2 = candStates->size[0];
    candStates->data[(emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &wb_emlrtBCI,
      sp) + candStates->size[0] * 9) - 1] = d0;
    i2 = candStates->size[1];
    emlrtDynamicBoundsCheckFastR2012b(11, 1, i2, &db_emlrtBCI, sp);
    i2 = candStates->size[0];
    candStates->data[(emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &xb_emlrtBCI,
      sp) + candStates->size[0] * 10) - 1] = x;

    /* ,Ss,qWDot,r); */
    i2 = candStates->size[1];
    emlrtDynamicBoundsCheckFastR2012b(10, 1, i2, &fb_emlrtBCI, sp);
    i2 = candStates->size[0];
    emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &eb_emlrtBCI, sp);

    /* angDiff Finds the angular difference between th1 and th2. */
    x = ((xNear->data[xNear->size[1] - 1] - candStates->data[(i +
           candStates->size[0] * 9) - 1]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(x - muDoubleScalarRound(x)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(x)) {
      x = 0.0;
    } else {
      x = (x - muDoubleScalarFloor(x)) * 6.2831853071795862;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    i2 = distance->size[1];
    if (muDoubleScalarAbs(x - 3.1415926535897931) > ankleThreshold) {
      i3 = 1;
    } else {
      i3 = 0;
    }

    distance->data[emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &yb_emlrtBCI, sp)
      - 1] = (1.0 - numIterations) * hDiff + numIterations * (real_T)i3;

    /* distance(i) = hDiff; */
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&r4);
  emxFree_real_T(&b_candStates);
  emxFree_real_T(&xInit);
  emxFree_int32_T(&r3);
  emxFree_real_T(&r2);
  emxFree_real_T(&r1);
  emxFree_real_T(&U_joint);
  st.site = &eb_emlrtRSI;
  b_st.site = &u_emlrtRSI;
  c_st.site = &v_emlrtRSI;
  if ((distance->size[1] == 1) || (distance->size[1] != 1)) {
    overflow = true;
  } else {
    overflow = false;
  }

  if (overflow) {
  } else {
    y = NULL;
    m2 = emlrtCreateCharArray(2, iv8);
    for (i = 0; i < 36; i++) {
      cv8[i] = cv9[i];
    }

    emlrtInitCharArrayR2013a(&c_st, 36, m2, cv8);
    emlrtAssign(&y, m2);
    d_st.site = &hb_emlrtRSI;
    e_st.site = &lb_emlrtRSI;
    error(&d_st, b_message(&e_st, y, &c_emlrtMCI), &d_emlrtMCI);
  }

  if (distance->size[1] > 0) {
  } else {
    b_y = NULL;
    m2 = emlrtCreateCharArray(2, iv9);
    for (i = 0; i < 39; i++) {
      cv10[i] = cv11[i];
    }

    emlrtInitCharArrayR2013a(&c_st, 39, m2, cv10);
    emlrtAssign(&b_y, m2);
    d_st.site = &gb_emlrtRSI;
    e_st.site = &kb_emlrtRSI;
    error(&d_st, b_message(&e_st, b_y, &e_emlrtMCI), &f_emlrtMCI);
  }

  d_st.site = &w_emlrtRSI;
  ixstart = 1;
  n = distance->size[1];
  x = distance->data[0];
  i = 1;
  if (distance->size[1] > 1) {
    if (muDoubleScalarIsNaN(distance->data[0])) {
      f_st.site = &y_emlrtRSI;
      overflow = (distance->size[1] > 2147483646);
      if (overflow) {
        g_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&g_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        ixstart = ix;
        if (!muDoubleScalarIsNaN(distance->data[ix - 1])) {
          x = distance->data[ix - 1];
          i = ix;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < distance->size[1]) {
      f_st.site = &x_emlrtRSI;
      if (ixstart + 1 > distance->size[1]) {
        overflow = false;
      } else {
        overflow = (distance->size[1] > 2147483646);
      }

      if (overflow) {
        g_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&g_st);
      }

      while (ixstart + 1 <= n) {
        if (distance->data[ixstart] < x) {
          x = distance->data[ixstart];
          i = ixstart + 1;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&distance);
  loop_ub = candStates->size[1];
  i2 = candStates->size[0];
  ixstart = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &gb_emlrtBCI, sp);
  i2 = xNew->size[0] * xNew->size[1];
  xNew->size[0] = 1;
  xNew->size[1] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)xNew, i2, (int32_T)sizeof(real_T),
                    &d_emlrtRTEI);
  for (i2 = 0; i2 < loop_ub; i2++) {
    xNew->data[xNew->size[0] * i2] = candStates->data[(ixstart +
      candStates->size[0] * i2) - 1];
  }

  emxFree_real_T(&candStates);
  loop_ub = candTransArrays->size[1];
  i2 = candTransArrays->size[0];
  i = emlrtDynamicBoundsCheckFastR2012b(i, 1, i2, &ib_emlrtBCI, sp);
  i2 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i2, (int32_T)sizeof
                    (real_T), &d_emlrtRTEI);
  for (i2 = 0; i2 < loop_ub; i2++) {
    transitionArray->data[transitionArray->size[0] * i2] = candTransArrays->
      data[(i + candTransArrays->size[0] * i2) - 1];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
