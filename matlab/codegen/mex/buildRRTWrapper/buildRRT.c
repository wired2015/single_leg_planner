/*
 * buildRRT.c
 *
 * Code generation for function 'buildRRT'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "eml_error.h"
#include "buildRRTWrapper_emxutil.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "extractKinematicConstants.h"
#include "buildRRTWrapper_mexutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo i_emlrtRSI = { 59, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 60, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 61, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo l_emlrtRSI = { 66, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 22, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo n_emlrtRSI = { 23, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 25, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 28, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 29, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo r_emlrtRSI = { 7, "getConstrainedBeta",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getConstrainedBeta.m"
};

static emlrtRSInfo s_emlrtRSI = { 15, "asin",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/asin.m" };

static emlrtRTEInfo c_emlrtRTEI = { 57, 28, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtECInfo f_emlrtECI = { -1, 68, 5, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo pb_emlrtBCI = { -1, -1, 68, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtRSInfo mb_emlrtRSI = { 104, "mod",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/mod.m" };

/* Function Declarations */
static int32_T div_s32_floor(const emlrtStack *sp, int32_T numerator, int32_T
  denominator);

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

void rrtLoop(const emlrtStack *sp, emxArray_real_T *T, const real_T jointRange[6],
             const real_T jointLimits[12], const real_T kinematicConst[16],
             real_T panHeight, const real_T U[10], real_T Dt, real_T dt, real_T *
             nodeIDCount, const real_T nGoal[11])
{
  real_T unusedU7;
  real_T unusedU6;
  real_T unusedU5;
  real_T unusedU4;
  real_T alphaDotRand;
  real_T betaRand;
  real_T unusedU1;
  real_T zeta;
  real_T L8;
  real_T L7;
  real_T L6;
  real_T L5;
  real_T L4;
  real_T L3;
  real_T L2;
  real_T gammaDotRand;
  real_T r;
  real_T gammaRand;
  real_T unusedU8;
  real_T b_zeta;
  real_T b_L8;
  real_T b_L6;
  real_T b_L5;
  real_T b_L4;
  real_T b_L3;
  real_T L1;
  real_T dv1[9];
  int32_T ix;
  int32_T xRand_size[2];
  real_T xRand_data[11];
  int32_T xi;
  int32_T b_xi;
  emxArray_real_T *d;
  int32_T n;
  emxArray_real_T *b_T;
  boolean_T b3;
  const mxArray *y;
  static const int32_T iv14[2] = { 1, 36 };

  const mxArray *m8;
  char_T cv6[36];
  static const char_T cv7[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  int32_T itmp;
  boolean_T b4;
  boolean_T exitg1;
  boolean_T c_xi;
  int32_T xNear_size[2];
  real_T xNear_data[11];
  emxArray_int32_T *r3;
  int32_T xNew_size[2];
  real_T xNew_data[11];
  emxArray_real_T b_xNear_data;
  emxArray_real_T *r4;
  int32_T iv15[2];
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
  st.site = &i_emlrtRSI;

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
  extractKinematicConstants(kinematicConst, &gammaDotRand, &L2, &L3, &L4, &L5,
    &L6, &L7, &L8, &zeta, &unusedU1, &betaRand, &alphaDotRand, &unusedU4,
    &unusedU5, &unusedU6, &unusedU7);
  b_st.site = &m_emlrtRSI;
  emlrtRandu(&r, 1);
  b_st.site = &n_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  gammaRand = jointRange[1] * gammaDotRand + jointLimits[2];

  /* betaRand = -asin((K-L1+L4*sin(zeta)+L5*sin(gammaRand+zeta)+L6+L8)/L3); */
  b_st.site = &o_emlrtRSI;

  /* GETCONSTRAINEDBETA Calculates the beta joint anglgiven a constrained body */
  /* height. */
  extractKinematicConstants(kinematicConst, &L1, &gammaDotRand, &b_L3, &b_L4,
    &b_L5, &b_L6, &unusedU1, &b_L8, &b_zeta, &betaRand, &alphaDotRand, &unusedU4,
    &unusedU5, &unusedU6, &unusedU7, &unusedU8);
  unusedU1 = (((((panHeight - L1) + b_L4 * muDoubleScalarSin(b_zeta)) + b_L5 *
                muDoubleScalarSin(gammaRand + b_zeta)) + b_L6) + b_L8) / b_L3;
  c_st.site = &r_emlrtRSI;
  if ((unusedU1 < -1.0) || (1.0 < unusedU1)) {
    d_st.site = &s_emlrtRSI;
    b_eml_error(&d_st);
  }

  betaRand = -muDoubleScalarAsin(unusedU1);
  b_st.site = &p_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  alphaDotRand = jointRange[3] * gammaDotRand + jointLimits[6];
  b_st.site = &q_emlrtRSI;
  emlrtRandu(&gammaDotRand, 1);
  gammaDotRand = jointRange[4] * gammaDotRand + jointLimits[8];
  for (ix = 0; ix < 3; ix++) {
    dv1[ix] = 0.0;
  }

  dv1[3] = jointRange[0] * r + jointLimits[0];
  dv1[4] = gammaRand;
  dv1[5] = -muDoubleScalarAsin(unusedU1);
  dv1[6] = alphaDotRand;
  dv1[7] = gammaDotRand;
  dv1[8] = -(((((((((((((((((((((((((2.238E+31 * L2 * alphaDotRand - 2.238E+31 *
    L6 * alphaDotRand) - 1.827E+47 * L6 * gammaDotRand) + 2.238E+31 * L3 *
    alphaDotRand * muDoubleScalarCos(betaRand)) + 1.827E+47 * L3 * gammaDotRand *
    muDoubleScalarCos(betaRand)) - 2.238E+31 * L2 * alphaDotRand) + 2.238E+31 *
    L6 * alphaDotRand) - 1.37E+15 * L6 * gammaDotRand) + 2.238E+31 * L4 *
    alphaDotRand * muDoubleScalarCos(zeta)) + 1.827E+47 * L4 * gammaDotRand *
    muDoubleScalarCos(zeta)) + 2.74E+15 * L7 * alphaDotRand * 0.0) + 2.74E+15 *
    L8 * alphaDotRand * 0.0) + 2.238E+31 * L7 * gammaDotRand * 0.0) + 2.238E+31 *
    L8 * gammaDotRand * 0.0) - 2.237E+31 * L3 * alphaDotRand * muDoubleScalarCos
                        (betaRand)) + 2.238E+31 * L5 * alphaDotRand *
                       muDoubleScalarCos(gammaRand) * muDoubleScalarCos(zeta)) +
                      1.827E+47 * L5 * gammaDotRand * muDoubleScalarCos
                      (gammaRand) * muDoubleScalarCos(zeta)) - 2.237E+31 * L4 *
                     alphaDotRand * muDoubleScalarCos(zeta)) + 2.237E+31 * L3 *
                    gammaDotRand * muDoubleScalarSin(betaRand) * 0.0) -
                   2.238E+31 * L5 * alphaDotRand * muDoubleScalarSin(gammaRand) *
                   muDoubleScalarSin(zeta)) - 1.827E+47 * L5 * gammaDotRand *
                  muDoubleScalarSin(gammaRand) * muDoubleScalarSin(zeta)) +
                 2.237E+31 * L4 * gammaDotRand * 0.0 * muDoubleScalarSin(zeta))
                - 2.237E+31 * L5 * alphaDotRand * muDoubleScalarCos(gammaRand) *
                muDoubleScalarCos(zeta)) + 2.237E+31 * L5 * alphaDotRand *
               muDoubleScalarSin(gammaRand) * muDoubleScalarSin(zeta)) +
              2.237E+31 * L5 * gammaDotRand * muDoubleScalarCos(gammaRand) * 0.0
              * muDoubleScalarSin(zeta)) + 2.237E+31 * L5 * gammaDotRand *
             muDoubleScalarSin(gammaRand) * muDoubleScalarCos(zeta) * 0.0) /
    (((((((((1.827E+47 * L4 * muDoubleScalarCos(zeta) - 1.37E+15 * L6) -
            1.827E+47 * L6) + 2.238E+31 * L7 * 0.0) + 2.238E+31 * L8 * 0.0) +
         1.827E+47 * L5 * muDoubleScalarCos(gammaRand) * muDoubleScalarCos(zeta))
        - 1.827E+47 * L5 * muDoubleScalarSin(gammaRand) * muDoubleScalarSin(zeta))
       + 2.237E+31 * L4 * 0.0 * muDoubleScalarSin(zeta)) + 2.237E+31 * L5 *
      muDoubleScalarCos(gammaRand) * 0.0 * muDoubleScalarSin(zeta)) + 2.237E+31 *
     L5 * muDoubleScalarSin(gammaRand) * muDoubleScalarCos(zeta) * 0.0);
  xRand_size[0] = 1;
  xRand_size[1] = 9;
  for (ix = 0; ix < 9; ix++) {
    xRand_data[ix] = dv1[ix];
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

  gammaDotRand = muDoubleScalarRound(*nodeIDCount - muDoubleScalarFloor
    (*nodeIDCount / 20.0) * 20.0);
  if (gammaDotRand < 2.147483648E+9) {
    if (gammaDotRand >= -2.147483648E+9) {
      ix = (int32_T)gammaDotRand;
    } else {
      ix = MIN_int32_T;
    }
  } else if (gammaDotRand >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  if (xi == *nodeIDCount) {
    b_st.site = &mb_emlrtRSI;
    b_xi = xi - div_s32_floor(&b_st, xi, 20) * 20;
  } else {
    b_xi = ix;
  }

  if (b_xi == 0) {
    xRand_size[0] = 1;
    xRand_size[1] = 11;
    memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(real_T));
  }

  emxInit_real_T(&st, &d, 2, &g_emlrtRTEI, true);
  st.site = &j_emlrtRSI;

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
  n = (int32_T)*nodeIDCount;
  for (ix = 0; ix < n; ix++) {
    d->data[ix] = 0.0;
  }

  /* parfor i = 1:nodeIDCount */
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, *nodeIDCount, mxDOUBLE_CLASS, (int32_T)*
    nodeIDCount, &k_emlrtRTEI, &st);
  xi = 1;
  emxInit_real_T(&st, &b_T, 2, &c_emlrtRTEI, true);
  while (xi - 1 <= (int32_T)*nodeIDCount - 1) {
    n = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(xi, 1, 1000, &jb_emlrtBCI, &st);
    ix = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = n;
    emxEnsureCapacity(&st, (emxArray__common *)b_T, ix, (int32_T)sizeof(real_T),
                      &c_emlrtRTEI);
    for (ix = 0; ix < n; ix++) {
      b_T->data[b_T->size[0] * ix] = T->data[(xi + T->size[0] * ix) - 1];
    }

    ix = d->size[1];
    b_st.site = &u_emlrtRSI;
    d->data[emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &ob_emlrtBCI, &st) - 1]
      = heuristicSingleLeg(&b_st, xRand_data, b_T, jointLimits, kinematicConst);
    xi++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  emxFree_real_T(&b_T);
  ix = d->size[1];
  xi = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &kb_emlrtBCI, &st);
  b_st.site = &t_emlrtRSI;
  c_st.site = &y_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  if (((int32_T)*nodeIDCount == 1) || ((int32_T)*nodeIDCount != 1)) {
    b3 = true;
  } else {
    b3 = false;
  }

  if (b3) {
  } else {
    y = NULL;
    m8 = emlrtCreateCharArray(2, iv14);
    for (xi = 0; xi < 36; xi++) {
      cv6[xi] = cv7[xi];
    }

    emlrtInitCharArrayR2013a(&d_st, 36, m8, cv6);
    emlrtAssign(&y, m8);
    e_st.site = &ib_emlrtRSI;
    f_st.site = &lb_emlrtRSI;
    error(&e_st, b_message(&f_st, y, &c_emlrtMCI), &d_emlrtMCI);
  }

  e_st.site = &bb_emlrtRSI;
  xi = 1;
  n = (int32_T)*nodeIDCount;
  gammaDotRand = d->data[0];
  itmp = 0;
  if ((int32_T)*nodeIDCount > 1) {
    if (muDoubleScalarIsNaN(gammaDotRand)) {
      g_st.site = &db_emlrtRSI;
      if (2 > (int32_T)*nodeIDCount) {
        b4 = false;
      } else {
        b4 = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (b4) {
        h_st.site = &eb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        xi = ix;
        if (!muDoubleScalarIsNaN(d->data[ix - 1])) {
          gammaDotRand = d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (xi < (int32_T)*nodeIDCount) {
      g_st.site = &cb_emlrtRSI;
      if (xi + 1 > (int32_T)*nodeIDCount) {
        c_xi = false;
      } else {
        c_xi = ((int32_T)*nodeIDCount > 2147483646);
      }

      if (c_xi) {
        h_st.site = &eb_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      while (xi + 1 <= n) {
        if (d->data[xi] < gammaDotRand) {
          gammaDotRand = d->data[xi];
          itmp = xi;
        }

        xi++;
      }
    }
  }

  /* [d,minIndex] = min(d(1:nodeIDCount)); */
  ix = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, ix, &mb_emlrtBCI, &st);
  ix = T->size[1];
  emlrtDynamicBoundsCheckFastR2012b(11, 1, ix, &mb_emlrtBCI, &st);
  emlrtDynamicBoundsCheckFastR2012b(itmp + 1, 1, 1000, &lb_emlrtBCI, &st);
  xNear_size[0] = 1;
  xNear_size[1] = 11;
  for (ix = 0; ix < 11; ix++) {
    xNear_data[xNear_size[0] * ix] = T->data[itmp + T->size[0] * ix];
  }

  if (12 > T->size[1]) {
  } else {
    ix = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(12, 1, ix, &nb_emlrtBCI, &st);
    ix = T->size[1];
    xi = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(xi, 1, ix, &nb_emlrtBCI, &st);
  }

  emxInit_int32_T(&st, &r3, 1, &c_emlrtRTEI, true);
  st.site = &k_emlrtRSI;
  selectInput(&st, xNear_data, xNear_size, xRand_data, xRand_size, U, dt, Dt,
              kinematicConst, jointLimits, xNew_data, xNew_size, d);
  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  /* Node ID */
  xNew_data[1] = T->data[itmp];

  /* Parent ID */
  b_xNear_data.data = (real_T *)&xNear_data;
  b_xNear_data.size = (int32_T *)&xNear_size;
  b_xNear_data.allocatedSize = 11;
  b_xNear_data.numDimensions = 2;
  b_xNear_data.canFreeData = false;
  st.site = &l_emlrtRSI;
  xNew_data[2] = heuristicSingleLeg(&st, xNew_data, &b_xNear_data, jointLimits,
    kinematicConst);

  /* Cost */
  ix = (int32_T)*nodeIDCount;
  emlrtDynamicBoundsCheckFastR2012b(ix, 1, 1000, &pb_emlrtBCI, sp);
  n = T->size[1];
  ix = r3->size[0];
  r3->size[0] = n;
  emxEnsureCapacity(sp, (emxArray__common *)r3, ix, (int32_T)sizeof(int32_T),
                    &c_emlrtRTEI);
  for (ix = 0; ix < n; ix++) {
    r3->data[ix] = ix;
  }

  emxInit_real_T(sp, &r4, 2, &c_emlrtRTEI, true);
  ix = r4->size[0] * r4->size[1];
  r4->size[0] = 1;
  r4->size[1] = 11 + d->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)r4, ix, (int32_T)sizeof(real_T),
                    &c_emlrtRTEI);
  for (ix = 0; ix < 11; ix++) {
    r4->data[r4->size[0] * ix] = xNew_data[xNew_size[0] * ix];
  }

  n = d->size[1];
  for (ix = 0; ix < n; ix++) {
    r4->data[r4->size[0] * (ix + 11)] = d->data[d->size[0] * ix];
  }

  emxFree_real_T(&d);
  iv15[0] = 1;
  iv15[1] = r3->size[0];
  emlrtSubAssignSizeCheckR2012b(iv15, 2, *(int32_T (*)[2])r4->size, 2,
    &f_emlrtECI, sp);
  xi = (int32_T)*nodeIDCount;
  n = r4->size[1];
  for (ix = 0; ix < n; ix++) {
    T->data[(xi + T->size[0] * r3->data[ix]) - 1] = r4->data[r4->size[0] * ix];
  }

  emxFree_real_T(&r4);
  emxFree_int32_T(&r3);

  /* Append the new node to the tree.     */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRT.c) */
