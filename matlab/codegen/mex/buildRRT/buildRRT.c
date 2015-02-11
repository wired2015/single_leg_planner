/*
 * buildRRT.c
 *
 * Code generation for function 'buildRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "buildRRT_emxutil.h"
#include "eml_error.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "nearestNeighbour.h"
#include "buildRRT_mexutil.h"
#include "buildRRT_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 26, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo b_emlrtRSI = { 27, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo c_emlrtRSI = { 35, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo d_emlrtRSI = { 36, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo e_emlrtRSI = { 44, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo g_emlrtRSI = { 59, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo h_emlrtRSI = { 60, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo i_emlrtRSI = { 61, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo j_emlrtRSI = { 66, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo k_emlrtRSI = { 29, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/randomState.m"
};

static emlrtRSInfo l_emlrtRSI = { 30, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/randomState.m"
};

static emlrtRSInfo m_emlrtRSI = { 31, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/randomState.m"
};

static emlrtRSInfo n_emlrtRSI = { 34, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 35, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 15, "asin",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/asin.m" };

static emlrtRTEInfo emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRTEInfo b_emlrtRTEI = { 47, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRTEInfo c_emlrtRTEI = { 57, 28, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRTEInfo l_emlrtRTEI = { 61, 6, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtBCInfo emlrtBCI = { -1, -1, 52, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtECInfo emlrtECI = { 2, 51, 16, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 50, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtDCInfo emlrtDCI = { 50, 18, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 1
};

static emlrtBCInfo c_emlrtBCI = { -1, -1, 50, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo d_emlrtBCI = { -1, -1, 49, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo e_emlrtBCI = { -1, -1, 46, 13, "xNearest", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtECInfo b_emlrtECI = { -1, 20, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 20, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtDCInfo b_emlrtDCI = { 18, 15, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 4
};

static emlrtDCInfo c_emlrtDCI = { 18, 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 4
};

static emlrtDCInfo d_emlrtDCI = { 20, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 1
};

static emlrtDCInfo e_emlrtDCI = { 20, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 4
};

static emlrtBCInfo ic_emlrtBCI = { -1, -1, 65, 15, "xNear", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo jc_emlrtBCI = { -1, -1, 64, 5, "xNew", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo kc_emlrtBCI = { -1, -1, 65, 5, "xNew", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo lc_emlrtBCI = { -1, -1, 66, 5, "xNew", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtBCInfo mc_emlrtBCI = { -1, -1, 68, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m", 0
};

static emlrtECInfo k_emlrtECI = { -1, 68, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/rrtSherpaTT4/buildRRT.m" };

static emlrtRSInfo mb_emlrtRSI = { 104, "mod",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/mod.m" };

/* Function Declarations */
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator);
static void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointRange[6], const real_T jointLimits[12], const real_T
                    kinematicConst[12], real_T K, const real_T U[10], real_T Dt,
                    real_T dt, int32_T NODE_SIZE, int32_T U_SIZE, const real_T
                    HGAINS[3], real_T ankleThreshold, real_T *nodeIDCount, const
                    real_T nGoal[11], int32_T goalSeedFreq);

/* Function Definitions */
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator)
{
  int32_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  boolean_T quotientNeedsNegation;
  uint32_T tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }

    emlrtDivisionByZeroErrorR2012b(NULL, sp);
  } else {
    if (numerator >= 0) {
      absNumerator = (uint32_T)numerator;
    } else {
      absNumerator = (uint32_T)-numerator;
    }

    if (denominator >= 0) {
      absDenominator = (uint32_T)denominator;
    } else {
      absDenominator = (uint32_T)-denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
    }

    if (quotientNeedsNegation) {
      quotient = -(int32_T)tempAbsQuotient;
    } else {
      quotient = (int32_T)tempAbsQuotient;
    }
  }

  return quotient;
}

static void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T
                    jointRange[6], const real_T jointLimits[12], const real_T
                    kinematicConst[12], real_T K, const real_T U[10], real_T Dt,
                    real_T dt, int32_T NODE_SIZE, int32_T U_SIZE, const real_T
                    HGAINS[3], real_T ankleThreshold, real_T *nodeIDCount, const
                    real_T nGoal[11], int32_T goalSeedFreq)
{
  real_T r;
  real_T gammaDotRand;
  real_T gammaRand;
  real_T x;
  real_T betaRand;
  real_T alphaDotRand;
  real_T dv0[9];
  int32_T ix;
  int32_T xRand_size[2];
  real_T xRand_data[11];
  int32_T xi;
  emxArray_real_T *d;
  int32_T loop_ub;
  emxArray_real_T *b_T;
  int32_T n;
  emxArray_real_T b_xRand_data;
  boolean_T b4;
  const mxArray *y;
  static const int32_T iv18[2] = { 1, 36 };

  const mxArray *m7;
  char_T cv16[36];
  static const char_T cv17[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  const mxArray *b_y;
  static const int32_T iv19[2] = { 1, 39 };

  char_T cv18[39];
  static const char_T cv19[39] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'e', 'm', 'l', '_', 'm', 'i', 'n', '_', 'o', 'r',
    '_', 'm', 'a', 'x', '_', 'v', 'a', 'r', 'D', 'i', 'm', 'Z', 'e', 'r', 'o' };

  int32_T itmp;
  boolean_T b5;
  boolean_T exitg1;
  boolean_T b_xi;
  int64_T i12;
  emxArray_real_T *xNew;
  emxArray_real_T *b_xNew;
  emxArray_real_T *transitionArray;
  emxArray_int32_T *r5;
  int32_T iv20[2];
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
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
  f_st.prev = &d_st;
  f_st.tls = d_st.tls;
  g_st.prev = &e_st;
  g_st.tls = e_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  st.site = &g_emlrtRSI;

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
  b_st.site = &k_emlrtRSI;
  emlrtRandu(&r, 1);
  b_st.site = &l_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  gammaRand = jointRange[1] * gammaDotRand + jointLimits[2];
  x = (((((K - kinematicConst[0]) + kinematicConst[3] * muDoubleScalarSin
          (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarSin(gammaRand
          + kinematicConst[8])) + kinematicConst[5]) + kinematicConst[7]) /
    kinematicConst[2];
  b_st.site = &m_emlrtRSI;
  if ((x < -1.0) || (1.0 < x)) {
    c_st.site = &p_emlrtRSI;
    eml_error(&c_st);
  }

  betaRand = -muDoubleScalarAsin(x);
  b_st.site = &n_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  alphaDotRand = jointRange[3] * gammaDotRand + jointLimits[6];
  b_st.site = &o_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  gammaDotRand = jointRange[4] * gammaDotRand + jointLimits[8];
  for (ix = 0; ix < 3; ix++) {
    dv0[ix] = 0.0;
  }

  dv0[3] = jointRange[0] * r + jointLimits[0];
  dv0[4] = gammaRand;
  dv0[5] = -muDoubleScalarAsin(x);
  dv0[6] = alphaDotRand;
  dv0[7] = gammaDotRand;
  dv0[8] = -(((((((((((((((((((((((((2.238E+31 * kinematicConst[1] *
    alphaDotRand - 2.238E+31 * kinematicConst[5] * alphaDotRand) - 1.827E+47 *
    kinematicConst[5] * gammaDotRand) + 2.238E+31 * kinematicConst[2] *
    alphaDotRand * muDoubleScalarCos(betaRand)) + 1.827E+47 * kinematicConst[2] *
    gammaDotRand * muDoubleScalarCos(betaRand)) - 2.238E+31 * kinematicConst[1] *
    alphaDotRand) + 2.238E+31 * kinematicConst[5] * alphaDotRand) - 1.37E+15 *
    kinematicConst[5] * gammaDotRand) + 2.238E+31 * kinematicConst[3] *
    alphaDotRand * muDoubleScalarCos(kinematicConst[8])) + 1.827E+47 *
    kinematicConst[3] * gammaDotRand * muDoubleScalarCos(kinematicConst[8])) +
    2.74E+15 * kinematicConst[6] * alphaDotRand * 0.0) + 2.74E+15 *
    kinematicConst[7] * alphaDotRand * 0.0) + 2.238E+31 * kinematicConst[6] *
    gammaDotRand * 0.0) + 2.238E+31 * kinematicConst[7] * gammaDotRand * 0.0) -
                        2.237E+31 * kinematicConst[2] * alphaDotRand *
                        muDoubleScalarCos(betaRand)) + 2.238E+31 *
                       kinematicConst[4] * alphaDotRand * muDoubleScalarCos
                       (gammaRand) * muDoubleScalarCos(kinematicConst[8])) +
                      1.827E+47 * kinematicConst[4] * gammaDotRand *
                      muDoubleScalarCos(gammaRand) * muDoubleScalarCos
                      (kinematicConst[8])) - 2.237E+31 * kinematicConst[3] *
                     alphaDotRand * muDoubleScalarCos(kinematicConst[8])) +
                    2.237E+31 * kinematicConst[2] * gammaDotRand *
                    muDoubleScalarSin(betaRand) * 0.0) - 2.238E+31 *
                   kinematicConst[4] * alphaDotRand * muDoubleScalarSin
                   (gammaRand) * muDoubleScalarSin(kinematicConst[8])) -
                  1.827E+47 * kinematicConst[4] * gammaDotRand *
                  muDoubleScalarSin(gammaRand) * muDoubleScalarSin
                  (kinematicConst[8])) + 2.237E+31 * kinematicConst[3] *
                 gammaDotRand * 0.0 * muDoubleScalarSin(kinematicConst[8])) -
                2.237E+31 * kinematicConst[4] * alphaDotRand * muDoubleScalarCos
                (gammaRand) * muDoubleScalarCos(kinematicConst[8])) + 2.237E+31 *
               kinematicConst[4] * alphaDotRand * muDoubleScalarSin(gammaRand) *
               muDoubleScalarSin(kinematicConst[8])) + 2.237E+31 *
              kinematicConst[4] * gammaDotRand * muDoubleScalarCos(gammaRand) *
              0.0 * muDoubleScalarSin(kinematicConst[8])) + 2.237E+31 *
             kinematicConst[4] * gammaDotRand * muDoubleScalarSin(gammaRand) *
             muDoubleScalarCos(kinematicConst[8]) * 0.0) / (((((((((1.827E+47 *
    kinematicConst[3] * muDoubleScalarCos(kinematicConst[8]) - 1.37E+15 *
    kinematicConst[5]) - 1.827E+47 * kinematicConst[5]) + 2.238E+31 *
    kinematicConst[6] * 0.0) + 2.238E+31 * kinematicConst[7] * 0.0) + 1.827E+47 *
    kinematicConst[4] * muDoubleScalarCos(gammaRand) * muDoubleScalarCos
    (kinematicConst[8])) - 1.827E+47 * kinematicConst[4] * muDoubleScalarSin
    (gammaRand) * muDoubleScalarSin(kinematicConst[8])) + 2.237E+31 *
    kinematicConst[3] * 0.0 * muDoubleScalarSin(kinematicConst[8])) + 2.237E+31 *
    kinematicConst[4] * muDoubleScalarCos(gammaRand) * 0.0 * muDoubleScalarSin
    (kinematicConst[8])) + 2.237E+31 * kinematicConst[4] * muDoubleScalarSin
    (gammaRand) * muDoubleScalarCos(kinematicConst[8]) * 0.0);
  xRand_size[0] = 1;
  xRand_size[1] = 9;
  for (ix = 0; ix < 9; ix++) {
    xRand_data[xRand_size[0] * ix] = dv0[ix];
  }

  gammaDotRand = muDoubleScalarRound(*nodeIDCount);
  if (gammaDotRand < 2.147483648E+9) {
    if (gammaDotRand >= -2.147483648E+9) {
      xi = (int32_T)gammaDotRand;
    } else {
      xi = MIN_int32_T;
    }
  } else if (gammaDotRand >= 2.147483648E+9) {
    xi = MAX_int32_T;
  } else {
    xi = 0;
  }

  if (xi == *nodeIDCount) {
    if (goalSeedFreq == 0) {
    } else {
      b_st.site = &mb_emlrtRSI;
      xi -= div_s32_floor(&b_st, xi, goalSeedFreq) * goalSeedFreq;
    }
  } else {
    if (goalSeedFreq == 0) {
      r = *nodeIDCount;
    } else {
      r = *nodeIDCount - muDoubleScalarFloor(*nodeIDCount / (real_T)goalSeedFreq)
        * (real_T)goalSeedFreq;
    }

    gammaDotRand = muDoubleScalarRound(r);
    if (gammaDotRand < 2.147483648E+9) {
      if (gammaDotRand >= -2.147483648E+9) {
        xi = (int32_T)gammaDotRand;
      } else {
        xi = MIN_int32_T;
      }
    } else if (gammaDotRand >= 2.147483648E+9) {
      xi = MAX_int32_T;
    } else {
      xi = 0;
    }
  }

  if (xi == 0) {
    xRand_size[0] = 1;
    xRand_size[1] = 11;
    memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(real_T));
  }

  emxInit_real_T(&st, &d, 2, &j_emlrtRTEI, true);
  st.site = &h_emlrtRSI;

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
  ix = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int32_T)*nodeIDCount;
  emxEnsureCapacity(&st, (emxArray__common *)d, ix, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  loop_ub = (int32_T)*nodeIDCount;
  for (ix = 0; ix < loop_ub; ix++) {
    d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, *nodeIDCount, mxDOUBLE_CLASS, (int32_T)*
    nodeIDCount, &o_emlrtRTEI, &st);
  xi = 1;
  emxInit_real_T(&st, &b_T, 2, &c_emlrtRTEI, true);
  while (xi - 1 <= (int32_T)*nodeIDCount - 1) {
    loop_ub = T->size[1];
    ix = T->size[0];
    n = emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &bc_emlrtBCI, &st);
    ix = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = loop_ub;
    emxEnsureCapacity(&st, (emxArray__common *)b_T, ix, (int32_T)sizeof(real_T),
                      &c_emlrtRTEI);
    for (ix = 0; ix < loop_ub; ix++) {
      b_T->data[b_T->size[0] * ix] = T->data[(n + T->size[0] * ix) - 1];
    }

    b_xRand_data.data = (real_T *)&xRand_data;
    b_xRand_data.size = (int32_T *)&xRand_size;
    b_xRand_data.allocatedSize = 11;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    ix = d->size[1];
    b_st.site = &r_emlrtRSI;
    d->data[emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &hc_emlrtBCI, &st) - 1]
      = heuristicSingleLeg(&b_st, &b_xRand_data, b_T, kinematicConst);
    xi++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  emxFree_real_T(&b_T);
  ix = d->size[1];
  xi = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &cc_emlrtBCI, &st);
  b_st.site = &q_emlrtRSI;
  c_st.site = &u_emlrtRSI;
  d_st.site = &v_emlrtRSI;
  if (((int32_T)*nodeIDCount == 1) || ((int32_T)*nodeIDCount != 1)) {
    b4 = true;
  } else {
    b4 = false;
  }

  if (b4) {
  } else {
    y = NULL;
    m7 = emlrtCreateCharArray(2, iv18);
    for (xi = 0; xi < 36; xi++) {
      cv16[xi] = cv17[xi];
    }

    emlrtInitCharArrayR2013a(&d_st, 36, m7, cv16);
    emlrtAssign(&y, m7);
    e_st.site = &hb_emlrtRSI;
    f_st.site = &lb_emlrtRSI;
    error(&e_st, b_message(&f_st, y, &c_emlrtMCI), &d_emlrtMCI);
  }

  if ((int32_T)*nodeIDCount > 0) {
  } else {
    b_y = NULL;
    m7 = emlrtCreateCharArray(2, iv19);
    for (xi = 0; xi < 39; xi++) {
      cv18[xi] = cv19[xi];
    }

    emlrtInitCharArrayR2013a(&d_st, 39, m7, cv18);
    emlrtAssign(&b_y, m7);
    e_st.site = &gb_emlrtRSI;
    f_st.site = &kb_emlrtRSI;
    error(&e_st, b_message(&f_st, b_y, &e_emlrtMCI), &f_emlrtMCI);
  }

  e_st.site = &w_emlrtRSI;
  xi = 1;
  n = (int32_T)*nodeIDCount;
  gammaDotRand = d->data[0];
  itmp = 1;
  if ((int32_T)*nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(gammaDotRand)) {
      g_st.site = &y_emlrtRSI;
      if (2 > (int32_T)*nodeIDCount) {
        b5 = false;
      } else {
        b5 = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b5) {
        h_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&h_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        xi = ix;
        if (!muDoubleScalarIsNaN(d->data[ix - 1])) {
          gammaDotRand = d->data[ix - 1];
          itmp = ix;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (xi < (int32_T)*nodeIDCount) {
      g_st.site = &x_emlrtRSI;
      if (xi + 1 > (int32_T)*nodeIDCount) {
        b_xi = false;
      } else {
        b_xi = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b_xi) {
        h_st.site = &f_emlrtRSI;
        b_check_forloop_overflow_error(&h_st);
      }

      for (ix = xi + 1; ix <= n; ix++) {
        if (d->data[ix - 1] < gammaDotRand) {
          gammaDotRand = d->data[ix - 1];
          itmp = ix;
        }
      }
    }
  }

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  if (1 > NODE_SIZE) {
    loop_ub = 0;
  } else {
    ix = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(1, 1, ix, &ec_emlrtBCI, &st);
    ix = T->size[1];
    loop_ub = emlrtDynamicBoundsCheckFastR2012b(NODE_SIZE, 1, ix, &ec_emlrtBCI,
      &st);
  }

  ix = T->size[0];
  xi = emlrtDynamicBoundsCheckFastR2012b(itmp, 1, ix, &dc_emlrtBCI, &st);
  ix = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = loop_ub;
  emxEnsureCapacity(&st, (emxArray__common *)d, ix, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  for (ix = 0; ix < loop_ub; ix++) {
    d->data[d->size[0] * ix] = T->data[(xi + T->size[0] * ix) - 1];
  }

  i12 = NODE_SIZE + 1L;
  if (i12 > 2147483647L) {
    i12 = 2147483647L;
  } else {
    if (i12 < -2147483648L) {
      i12 = -2147483648L;
    }
  }

  ix = (int32_T)i12;
  if (ix > T->size[1]) {
  } else {
    xi = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(ix, 1, xi, &gc_emlrtBCI, &st);
    ix = T->size[1];
    xi = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &gc_emlrtBCI, &st);
  }

  emxInit_real_T(&st, &xNew, 2, &l_emlrtRTEI, true);
  emxInit_real_T(&st, &b_xNew, 2, &c_emlrtRTEI, true);
  emxInit_real_T(&st, &transitionArray, 2, &c_emlrtRTEI, true);
  ix = T->size[0];
  emlrtDynamicBoundsCheckFastR2012b(itmp, 1, ix, &fc_emlrtBCI, &st);
  st.site = &i_emlrtRSI;
  selectInput(&st, d, xRand_data, xRand_size, U, dt, Dt, NODE_SIZE, U_SIZE,
              HGAINS, kinematicConst, ankleThreshold, jointLimits, b_xNew,
              transitionArray);
  ix = xNew->size[0] * xNew->size[1];
  xNew->size[0] = 1;
  xNew->size[1] = b_xNew->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)xNew, ix, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  xi = b_xNew->size[0] * b_xNew->size[1];
  for (ix = 0; ix < xi; ix++) {
    xNew->data[ix] = b_xNew->data[ix];
  }

  emxInit_int32_T(sp, &r5, 1, &c_emlrtRTEI, true);
  (*nodeIDCount)++;
  ix = b_xNew->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, ix, &jc_emlrtBCI, sp);
  xNew->data[0] = *nodeIDCount;

  /* Node ID */
  ix = b_xNew->size[1];
  emlrtDynamicBoundsCheckFastR2012b(2, 1, ix, &kc_emlrtBCI, sp);
  emlrtDynamicBoundsCheckFastR2012b(1, 1, loop_ub, &ic_emlrtBCI, sp);
  xNew->data[1] = T->data[itmp - 1];

  /* Parent ID */
  ix = b_xNew->size[1];
  emlrtDynamicBoundsCheckFastR2012b(3, 1, ix, &lc_emlrtBCI, sp);
  st.site = &j_emlrtRSI;
  xNew->data[2] = heuristicSingleLeg(&st, xNew, d, kinematicConst);

  /* Cost */
  ix = T->size[0];
  xi = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &mc_emlrtBCI, sp);
  loop_ub = T->size[1];
  ix = r5->size[0];
  r5->size[0] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)r5, ix, (int32_T)sizeof(int32_T),
                    &c_emlrtRTEI);
  emxFree_real_T(&b_xNew);
  for (ix = 0; ix < loop_ub; ix++) {
    r5->data[ix] = ix;
  }

  ix = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = xNew->size[1] + transitionArray->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)d, ix, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  loop_ub = xNew->size[1];
  for (ix = 0; ix < loop_ub; ix++) {
    d->data[d->size[0] * ix] = xNew->data[xNew->size[0] * ix];
  }

  loop_ub = transitionArray->size[1];
  for (ix = 0; ix < loop_ub; ix++) {
    d->data[d->size[0] * (ix + xNew->size[1])] = transitionArray->
      data[transitionArray->size[0] * ix];
  }

  emxFree_real_T(&transitionArray);
  emxFree_real_T(&xNew);
  iv20[0] = 1;
  iv20[1] = r5->size[0];
  emlrtSubAssignSizeCheckR2012b(iv20, 2, *(int32_T (*)[2])d->size, 2,
    &k_emlrtECI, sp);
  xi = (int32_T)*nodeIDCount;
  loop_ub = d->size[1];
  for (ix = 0; ix < loop_ub; ix++) {
    T->data[(xi + T->size[0] * r5->data[ix]) - 1] = d->data[d->size[0] * ix];
  }

  emxFree_real_T(&d);
  emxFree_int32_T(&r5);

  /* Append the new node to the tree.     */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRT(const emlrtStack *sp, const real_T nInit[11], const real_T nGoal
              [11], int32_T NUM_NODES, const real_T jointLimits[12], real_T K,
              const real_T HGAINS[3], int32_T NODE_SIZE, const real_T U[10],
              int32_T U_SIZE, real_T dt, real_T Dt, const real_T kinematicConst
              [12], real_T ankleThreshold, boolean_T exhaustive, real_T
              threshold, int32_T goalSeedFreq, emxArray_real_T *T,
              emxArray_real_T *path)
{
  real_T transitionArrayLength;
  int32_T i0;
  real_T dist;
  int32_T i;
  int32_T loop_ub;
  emxArray_int32_T *r0;
  emxArray_real_T *unusedU0;
  int32_T iv0[2];
  real_T jointRange[6];
  emxArray_real_T *next;
  boolean_T b0;
  real_T unusedU2;
  emxArray_real_T *b_path;
  boolean_T exitg1;
  int32_T b_loop_ub;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* buildRRT.m */
  /* author: wreid */
  /* date: 20150107 */
  /* buildRRT Icrementally builds a rapidly exploring random tree. */
  /*    An RRT is build by incrementally selecting a random state from the */
  /*    available state space as defined by the MIN and MAX vectors. The tree is */
  /*    started at xInit and is extended until the number of maximum nodes, K has */
  /*    been reached. A path is selected if the goal region as defined by xGoal */
  /*    has been reached by the RRT. */
  /* Constant Declaration                                                       */
  transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 6.0;

  /* Variable Initialization */
  i0 = T->size[0] * T->size[1];
  T->size[0] = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_NODES, &b_emlrtDCI,
    sp);
  dist = muDoubleScalarRound((real_T)NODE_SIZE + transitionArrayLength);
  if (dist < 2.147483648E+9) {
    if (dist >= -2.147483648E+9) {
      i = (int32_T)dist;
    } else {
      i = MIN_int32_T;
    }
  } else if (dist >= 2.147483648E+9) {
    i = MAX_int32_T;
  } else {
    i = 0;
  }

  T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(i, &c_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  dist = muDoubleScalarRound((real_T)NODE_SIZE + transitionArrayLength);
  if (dist < 2.147483648E+9) {
    if (dist >= -2.147483648E+9) {
      i0 = (int32_T)dist;
    } else {
      i0 = MIN_int32_T;
    }
  } else if (dist >= 2.147483648E+9) {
    i0 = MAX_int32_T;
  } else {
    i0 = 0;
  }

  loop_ub = (int32_T)emlrtNonNegativeCheckFastR2012b(NUM_NODES, &b_emlrtDCI, sp)
    * (int32_T)emlrtNonNegativeCheckFastR2012b(i0, &c_emlrtDCI, sp);
  for (i0 = 0; i0 < loop_ub; i0++) {
    T->data[i0] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &emlrtRTEI, true);

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  emlrtDynamicBoundsCheckFastR2012b(1, 1, NUM_NODES, &f_emlrtBCI, sp);
  dist = muDoubleScalarRound((real_T)NODE_SIZE + transitionArrayLength);
  if (dist < 2.147483648E+9) {
    if (dist >= -2.147483648E+9) {
      loop_ub = (int32_T)dist;
    } else {
      loop_ub = MIN_int32_T;
    }
  } else if (dist >= 2.147483648E+9) {
    loop_ub = MAX_int32_T;
  } else {
    loop_ub = 0;
  }

  i0 = r0->size[0];
  dist = muDoubleScalarRound((real_T)NODE_SIZE + transitionArrayLength);
  if (dist < 2.147483648E+9) {
    if (dist >= -2.147483648E+9) {
      i = (int32_T)dist;
    } else {
      i = MIN_int32_T;
    }
  } else if (dist >= 2.147483648E+9) {
    i = MAX_int32_T;
  } else {
    i = 0;
  }

  r0->size[0] = i;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i0, (int32_T)sizeof(int32_T),
                    &emlrtRTEI);
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0] = i0;
  }

  emxInit_real_T(sp, &unusedU0, 2, &emlrtRTEI, true);
  dist = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &e_emlrtDCI, sp);
  emlrtIntegerCheckFastR2012b(dist, &d_emlrtDCI, sp);
  i0 = unusedU0->size[0] * unusedU0->size[1];
  unusedU0->size[0] = 1;
  unusedU0->size[1] = 11 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)unusedU0, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  for (i0 = 0; i0 < 11; i0++) {
    unusedU0->data[unusedU0->size[0] * i0] = nInit[i0];
  }

  loop_ub = (int32_T)transitionArrayLength;
  for (i0 = 0; i0 < loop_ub; i0++) {
    unusedU0->data[unusedU0->size[0] * (i0 + 11)] = 0.0;
  }

  iv0[0] = 1;
  iv0[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv0, 2, *(int32_T (*)[2])unusedU0->size, 2,
    &b_emlrtECI, sp);
  loop_ub = unusedU0->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    T->data[T->size[0] * r0->data[i0]] = unusedU0->data[unusedU0->size[0] * i0];
  }

  emxFree_int32_T(&r0);

  /* Initialize the tree with initial state. */
  transitionArrayLength = 1.0;
  for (i0 = 0; i0 < 6; i0++) {
    jointRange[i0] = jointLimits[1 + (i0 << 1)] - jointLimits[i0 << 1];
  }

  emxInit_real_T(sp, &next, 2, &b_emlrtRTEI, true);
  if (!exhaustive) {
    st.site = &emlrtRSI;
    if (2 > NUM_NODES) {
      b0 = false;
    } else {
      b0 = (NUM_NODES > 2147483646);
    }

    if (b0) {
      b_st.site = &f_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    i = 2;
    while (i <= NUM_NODES) {
      st.site = &b_emlrtRSI;
      rrtLoop(&st, T, jointRange, jointLimits, kinematicConst, K, U, Dt, dt,
              NODE_SIZE, U_SIZE, HGAINS, ankleThreshold, &transitionArrayLength,
              nGoal, goalSeedFreq);
      i++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }
  } else {
    /* TODO: calculate maximum distance given the configuration space. */
    dist = 100.0;

    /* TODO: make the threshold distance */
    while ((dist > threshold) && (transitionArrayLength < NUM_NODES)) {
      st.site = &c_emlrtRSI;
      rrtLoop(&st, T, jointRange, jointLimits, kinematicConst, K, U, Dt, dt,
              NODE_SIZE, U_SIZE, HGAINS, ankleThreshold, &transitionArrayLength,
              nGoal, goalSeedFreq);
      st.site = &d_emlrtRSI;
      nearestNeighbour(&st, nGoal, T, kinematicConst, transitionArrayLength,
                       NODE_SIZE, unusedU0, next, &dist);
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    }
  }

  /* Find the closest node in the tree to the goal node. */
  st.site = &e_emlrtRSI;
  nearestNeighbour(&st, nGoal, T, kinematicConst, transitionArrayLength,
                   NODE_SIZE, next, unusedU0, &unusedU2);
  i0 = next->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i0, &e_emlrtBCI, sp);
  transitionArrayLength = next->data[0];
  i0 = path->size[0] * path->size[1];
  path->size[0] = 1;
  path->size[1] = next->size[1] + unusedU0->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)path, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  loop_ub = next->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    path->data[path->size[0] * i0] = next->data[next->size[0] * i0];
  }

  loop_ub = unusedU0->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    path->data[path->size[0] * (i0 + next->size[1])] = unusedU0->data
      [unusedU0->size[0] * i0];
  }

  emxFree_real_T(&unusedU0);
  emxInit_real_T(sp, &b_path, 2, &emlrtRTEI, true);
  exitg1 = false;
  while ((!exitg1) && (transitionArrayLength != 0.0)) {
    i0 = next->size[1];
    emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &d_emlrtBCI, sp);
    if (next->data[1] != 0.0) {
      i0 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &c_emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      loop_ub = T->size[1];
      i0 = T->size[0];
      i = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength, &emlrtDCI,
        sp);
      i = emlrtDynamicBoundsCheckFastR2012b(i, 1, i0, &b_emlrtBCI, sp);
      i0 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = loop_ub;
      emxEnsureCapacity(sp, (emxArray__common *)next, i0, (int32_T)sizeof(real_T),
                        &emlrtRTEI);
      for (i0 = 0; i0 < loop_ub; i0++) {
        next->data[next->size[0] * i0] = T->data[(i + T->size[0] * i0) - 1];
      }

      i0 = path->size[1];
      i = next->size[1];
      emlrtDimSizeEqCheckFastR2012b(i0, i, &emlrtECI, sp);
      i0 = b_path->size[0] * b_path->size[1];
      b_path->size[0] = path->size[0] + 1;
      b_path->size[1] = path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)b_path, i0, (int32_T)sizeof
                        (real_T), &emlrtRTEI);
      loop_ub = path->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_loop_ub = path->size[0];
        for (i = 0; i < b_loop_ub; i++) {
          b_path->data[i + b_path->size[0] * i0] = path->data[i + path->size[0] *
            i0];
        }
      }

      loop_ub = next->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_path->data[path->size[0] + b_path->size[0] * i0] = next->data
          [next->size[0] * i0];
      }

      i0 = path->size[0] * path->size[1];
      path->size[0] = b_path->size[0];
      path->size[1] = b_path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)path, i0, (int32_T)sizeof(real_T),
                        &emlrtRTEI);
      loop_ub = b_path->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_loop_ub = b_path->size[0];
        for (i = 0; i < b_loop_ub; i++) {
          path->data[i + path->size[0] * i0] = b_path->data[i + b_path->size[0] *
            i0];
        }
      }

      i0 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
    } else {
      exitg1 = true;
    }
  }

  emxFree_real_T(&b_path);
  emxFree_real_T(&next);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
