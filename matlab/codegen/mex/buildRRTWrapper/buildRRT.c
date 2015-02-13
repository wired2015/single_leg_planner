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
#include "buildRRTWrapper_emxutil.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "eml_int_forloop_overflow_check.h"
#include "nearestNeighbour.h"
#include "buildRRTWrapper_mexutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo h_emlrtRSI = { 47, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo i_emlrtRSI = { 30, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo j_emlrtRSI = { 62, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo k_emlrtRSI = { 63, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo l_emlrtRSI = { 64, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo m_emlrtRSI = { 69, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo n_emlrtRSI = { 25, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo o_emlrtRSI = { 26, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo p_emlrtRSI = { 27, "randomState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

static emlrtRSInfo q_emlrtRSI = { 19, "getConstrainedBeta",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getConstrainedBeta.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 5, 21, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo c_emlrtRTEI = { 50, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtECInfo emlrtECI = { -1, 71, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo q_emlrtBCI = { -1, -1, 71, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { -1, 23, 5, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo r_emlrtBCI = { -1, -1, 52, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtBCInfo s_emlrtBCI = { -1, -1, 53, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo e_emlrtDCI = { 53, 18, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 53, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo c_emlrtECI = { 2, 54, 16, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo u_emlrtBCI = { -1, -1, 55, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo f_emlrtDCI = { 21, 25, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo g_emlrtDCI = { 23, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtDCInfo h_emlrtDCI = { 23, 29, "buildRRT",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtRSInfo kb_emlrtRSI = { 104, "mod",
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

void buildRRT(const emlrtStack *sp, const real_T nInit[11], const real_T nGoal
              [11], const real_T jointLimits[12], real_T panHeight, const real_T
              U[10], real_T dt, real_T Dt, const struct0_T *kC, emxArray_real_T *
              T, emxArray_real_T *path)
{
  real_T transitionArrayLength;
  int32_T i3;
  real_T gammaRand;
  int32_T ix;
  int32_T loop_ub;
  emxArray_int32_T *r0;
  emxArray_real_T *d;
  int32_T iv1[2];
  uint32_T nodeIDCount;
  real_T jointRange[6];
  emxArray_real_T *next;
  emxArray_real_T *b_T;
  int32_T i;
  real_T r;
  real_T dv2[9];
  int32_T xRand_size[2];
  real_T xRand_data[11];
  uint32_T u0;
  int32_T xi;
  int32_T b_xi;
  boolean_T b0;
  const mxArray *y;
  static const int32_T iv2[2] = { 1, 36 };

  const mxArray *m0;
  char_T cv0[36];
  static const char_T cv1[36] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o',
    'l', 'b', 'o', 'x', ':', 'a', 'u', 't', 'o', 'D', 'i', 'm', 'I', 'n', 'c',
    'o', 'm', 'p', 'a', 't', 'i', 'b', 'i', 'l', 'i', 't', 'y' };

  boolean_T b1;
  boolean_T exitg2;
  boolean_T c_xi;
  int32_T xNearest_size[2];
  real_T xNearest_data[11];
  int32_T xNew_size[2];
  real_T xNew_data[11];
  emxArray_real_T b_xNearest_data;
  int32_T iv3[2];
  real_T unusedU2;
  emxArray_real_T *b_path;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
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
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &e_st;
  g_st.tls = e_st.tls;
  h_st.prev = &f_st;
  h_st.tls = f_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
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
  /* %TODO: Make a structure input for nInit and nGoal */
  /* %TEMPORARY definition of init and goal nodes. */
  /* nInit = makeNode(1,0,0,nInit(4:6),nInit(7:9),) */
  /* Constant Declaration                                                       */
  transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 6.0;

  /* Variable Initialization */
  i3 = T->size[0] * T->size[1];
  T->size[0] = 1000;
  gammaRand = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (gammaRand < 2.147483648E+9) {
    if (gammaRand >= -2.147483648E+9) {
      ix = (int32_T)gammaRand;
    } else {
      ix = MIN_int32_T;
    }
  } else if (gammaRand >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(ix, &f_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)T, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  gammaRand = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (gammaRand < 2.147483648E+9) {
    if (gammaRand >= -2.147483648E+9) {
      i3 = (int32_T)gammaRand;
    } else {
      i3 = MIN_int32_T;
    }
  } else if (gammaRand >= 2.147483648E+9) {
    i3 = MAX_int32_T;
  } else {
    i3 = 0;
  }

  loop_ub = 1000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i3, &f_emlrtDCI, sp);
  for (i3 = 0; i3 < loop_ub; i3++) {
    T->data[i3] = 0.0;
  }

  emxInit_int32_T(sp, &r0, 1, &b_emlrtRTEI, true);

  /* Define a zero array that will be used to  */
  /* store data from each tree node. */
  gammaRand = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (gammaRand < 2.147483648E+9) {
    if (gammaRand >= -2.147483648E+9) {
      loop_ub = (int32_T)gammaRand;
    } else {
      loop_ub = MIN_int32_T;
    }
  } else if (gammaRand >= 2.147483648E+9) {
    loop_ub = MAX_int32_T;
  } else {
    loop_ub = 0;
  }

  i3 = r0->size[0];
  gammaRand = muDoubleScalarRound(11.0 + transitionArrayLength);
  if (gammaRand < 2.147483648E+9) {
    if (gammaRand >= -2.147483648E+9) {
      ix = (int32_T)gammaRand;
    } else {
      ix = MIN_int32_T;
    }
  } else if (gammaRand >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  r0->size[0] = ix;
  emxEnsureCapacity(sp, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < loop_ub; i3++) {
    r0->data[i3] = i3;
  }

  emxInit_real_T(sp, &d, 2, &d_emlrtRTEI, true);
  gammaRand = emlrtNonNegativeCheckFastR2012b(transitionArrayLength, &h_emlrtDCI,
    sp);
  emlrtIntegerCheckFastR2012b(gammaRand, &g_emlrtDCI, sp);
  i3 = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = 11 + (int32_T)transitionArrayLength;
  emxEnsureCapacity(sp, (emxArray__common *)d, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 11; i3++) {
    d->data[d->size[0] * i3] = nInit[i3];
  }

  loop_ub = (int32_T)transitionArrayLength;
  for (i3 = 0; i3 < loop_ub; i3++) {
    d->data[d->size[0] * (i3 + 11)] = 0.0;
  }

  iv1[0] = 1;
  iv1[1] = r0->size[0];
  emlrtSubAssignSizeCheckR2012b(iv1, 2, *(int32_T (*)[2])d->size, 2, &b_emlrtECI,
    sp);
  loop_ub = d->size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    T->data[T->size[0] * r0->data[i3]] = d->data[d->size[0] * i3];
  }

  /* Initialize the tree with initial state. */
  nodeIDCount = 1U;
  for (i3 = 0; i3 < 6; i3++) {
    jointRange[i3] = jointLimits[1 + (i3 << 1)] - jointLimits[i3 << 1];
  }

  emxInit_real_T(sp, &next, 2, &c_emlrtRTEI, true);
  emxInit_real_T(sp, &b_T, 2, &b_emlrtRTEI, true);
  for (i = 0; i < 999; i++) {
    st.site = &i_emlrtRSI;
    b_st.site = &j_emlrtRSI;

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
    c_st.site = &n_emlrtRSI;
    emlrtRandu(&r, 1);
    c_st.site = &o_emlrtRSI;
    emlrtRandu(&transitionArrayLength, 1);
    gammaRand = jointRange[1] * transitionArrayLength + jointLimits[2];
    c_st.site = &p_emlrtRSI;

    /* GETCONSTRAINEDBETA Calculates the beta joint angle given a constrained body */
    /* height. */
    /* [L1,~,L3,L4,L5,L6,~,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); */
    /*      check = (panHeight-kC.l1+kC.l4*sin(kC.zeta)+kC.l5*sin(gamma+kC.zeta)+kC.l7+kC.l8+kC.r)/kC.l3; */
    /*   */
    /*      if check < -1 || check > 1 */
    /*          gammaMax = asin(1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta; */
    /*          gammaMin = asin(-1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta; */
    /*          gamma = gammaMin + rand*(gammaMax-gammaMin); */
    /*      end */
    transitionArrayLength = ((((((-panHeight + kC->l1) - kC->l4 *
      muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin(gammaRand +
      kC->zeta)) - kC->l6) - kC->l8) - kC->r) / kC->l3;
    while ((transitionArrayLength < -1.0) || (transitionArrayLength > 1.0)) {
      d_st.site = &q_emlrtRSI;
      emlrtRandu(&transitionArrayLength, 1);
      gammaRand = jointRange[1] * transitionArrayLength + jointLimits[2];
      transitionArrayLength = ((((((-panHeight + kC->l1) - kC->l4 *
        muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin(gammaRand +
        kC->zeta)) - kC->l6) - kC->l8) - kC->r) / kC->l3;

      /* disp('Imaginary'); */
      /* disp(count); */
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &c_st);
    }

    /* alphaDotRand = range(4)*rand+MIN(4); */
    /* gammaDotRand = range(5)*rand+MIN(5); */
    /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
    for (i3 = 0; i3 < 3; i3++) {
      dv2[i3] = 0.0;
    }

    dv2[3] = jointRange[0] * r + jointLimits[0];
    dv2[4] = gammaRand;
    dv2[5] = muDoubleScalarAsin(transitionArrayLength);
    dv2[6] = 0.0;
    dv2[7] = 0.0;
    dv2[8] = 0.0;
    xRand_size[0] = 1;
    xRand_size[1] = 9;
    for (i3 = 0; i3 < 9; i3++) {
      xRand_data[i3] = dv2[i3];
    }

    u0 = nodeIDCount;
    if (u0 > 2147483647U) {
      u0 = 2147483647U;
    }

    xi = (int32_T)u0;
    gammaRand = muDoubleScalarRound((real_T)nodeIDCount - muDoubleScalarFloor
      ((real_T)nodeIDCount / 20.0) * 20.0);
    if (gammaRand < 2.147483648E+9) {
      if (gammaRand >= -2.147483648E+9) {
        i3 = (int32_T)gammaRand;
      } else {
        i3 = MIN_int32_T;
      }
    } else {
      i3 = MAX_int32_T;
    }

    if ((uint32_T)xi == nodeIDCount) {
      c_st.site = &kb_emlrtRSI;
      b_xi = xi - div_s32_floor(&c_st, xi, 20) * 20;
    } else {
      b_xi = i3;
    }

    if (b_xi == 0) {
      xRand_size[0] = 1;
      xRand_size[1] = 11;
      memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(real_T));
    }

    b_st.site = &k_emlrtRSI;

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
    i3 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = (int32_T)nodeIDCount;
    emxEnsureCapacity(&b_st, (emxArray__common *)d, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    loop_ub = (int32_T)nodeIDCount;
    for (i3 = 0; i3 < loop_ub; i3++) {
      d->data[i3] = 0.0;
    }

    /* parfor i = 1:nodeIDCount */
    emlrtForLoopVectorCheckR2012b(1.0, 1.0, nodeIDCount, mxDOUBLE_CLASS,
      (int32_T)nodeIDCount, &j_emlrtRTEI, &b_st);
    xi = 1;
    while (xi - 1 <= (int32_T)nodeIDCount - 1) {
      loop_ub = T->size[1];
      emlrtDynamicBoundsCheckFastR2012b(xi, 1, 1000, &l_emlrtBCI, &b_st);
      i3 = b_T->size[0] * b_T->size[1];
      b_T->size[0] = 1;
      b_T->size[1] = loop_ub;
      emxEnsureCapacity(&b_st, (emxArray__common *)b_T, i3, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      for (i3 = 0; i3 < loop_ub; i3++) {
        b_T->data[b_T->size[0] * i3] = T->data[(xi + T->size[0] * i3) - 1];
      }

      i3 = d->size[1];
      c_st.site = &s_emlrtRSI;
      d->data[emlrtDynamicBoundsCheckFastR2012b(xi, 1, i3, &v_emlrtBCI, &b_st) -
        1] = heuristicSingleLeg(&c_st, xRand_data, b_T, jointLimits, kC->l2,
        kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);
      xi++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &b_st);
    }

    i3 = d->size[1];
    ix = (int32_T)nodeIDCount;
    emlrtDynamicBoundsCheckFastR2012b(ix, 1, i3, &m_emlrtBCI, &b_st);
    c_st.site = &r_emlrtRSI;
    d_st.site = &w_emlrtRSI;
    e_st.site = &x_emlrtRSI;
    if (((int32_T)nodeIDCount == 1) || ((int32_T)nodeIDCount != 1)) {
      b0 = true;
    } else {
      b0 = false;
    }

    if (b0) {
    } else {
      y = NULL;
      m0 = emlrtCreateCharArray(2, iv2);
      for (xi = 0; xi < 36; xi++) {
        cv0[xi] = cv1[xi];
      }

      emlrtInitCharArrayR2013a(&e_st, 36, m0, cv0);
      emlrtAssign(&y, m0);
      f_st.site = &gb_emlrtRSI;
      g_st.site = &jb_emlrtRSI;
      error(&f_st, b_message(&g_st, y, &c_emlrtMCI), &d_emlrtMCI);
    }

    f_st.site = &y_emlrtRSI;
    xi = 1;
    transitionArrayLength = d->data[0];
    loop_ub = 0;
    if ((int32_T)nodeIDCount > 1) {
      if (muDoubleScalarIsNaN(transitionArrayLength)) {
        h_st.site = &bb_emlrtRSI;
        if (2 > (int32_T)nodeIDCount) {
          b1 = false;
        } else {
          b1 = ((int32_T)nodeIDCount > 2147483646);
        }

        if (b1) {
          i_st.site = &cb_emlrtRSI;
          check_forloop_overflow_error(&i_st);
        }

        ix = 2;
        exitg2 = false;
        while ((!exitg2) && (ix <= (int32_T)nodeIDCount)) {
          xi = ix;
          if (!muDoubleScalarIsNaN(d->data[ix - 1])) {
            transitionArrayLength = d->data[ix - 1];
            loop_ub = ix - 1;
            exitg2 = true;
          } else {
            ix++;
          }
        }
      }

      if (xi < (int32_T)nodeIDCount) {
        h_st.site = &ab_emlrtRSI;
        if (xi + 1 > (int32_T)nodeIDCount) {
          c_xi = false;
        } else {
          c_xi = ((int32_T)nodeIDCount > 2147483646);
        }

        if (c_xi) {
          i_st.site = &cb_emlrtRSI;
          check_forloop_overflow_error(&i_st);
        }

        while (xi + 1 <= (int32_T)nodeIDCount) {
          if (d->data[xi] < transitionArrayLength) {
            transitionArrayLength = d->data[xi];
            loop_ub = xi;
          }

          xi++;
        }
      }
    }

    /* [d,minIndex] = min(d(1:nodeIDCount)); */
    i3 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(1, 1, i3, &o_emlrtBCI, &b_st);
    i3 = T->size[1];
    emlrtDynamicBoundsCheckFastR2012b(11, 1, i3, &o_emlrtBCI, &b_st);
    emlrtDynamicBoundsCheckFastR2012b(loop_ub + 1, 1, 1000, &n_emlrtBCI, &b_st);
    xNearest_size[0] = 1;
    xNearest_size[1] = 11;
    for (i3 = 0; i3 < 11; i3++) {
      xNearest_data[xNearest_size[0] * i3] = T->data[loop_ub + T->size[0] * i3];
    }

    if (12 > T->size[1]) {
    } else {
      i3 = T->size[1];
      ix = T->size[1];
      emlrtDynamicBoundsCheckFastR2012b(ix, 1, i3, &p_emlrtBCI, &b_st);
    }

    b_st.site = &l_emlrtRSI;
    selectInput(&b_st, xNearest_data, xNearest_size, xRand_data, xRand_size, U,
                dt, Dt, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                jointLimits, xNew_data, xNew_size, next);
    xNew_data[0] = (real_T)nodeIDCount + 1.0;

    /* Node ID */
    xNew_data[1] = T->data[loop_ub];

    /* Parent ID */
    b_xNearest_data.data = (real_T *)&xNearest_data;
    b_xNearest_data.size = (int32_T *)&xNearest_size;
    b_xNearest_data.allocatedSize = 11;
    b_xNearest_data.numDimensions = 2;
    b_xNearest_data.canFreeData = false;
    b_st.site = &m_emlrtRSI;
    xNew_data[2] = heuristicSingleLeg(&b_st, xNew_data, &b_xNearest_data,
      jointLimits, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);

    /* Cost */
    i3 = (int32_T)((real_T)nodeIDCount + 1.0);
    emlrtDynamicBoundsCheckFastR2012b(i3, 1, 1000, &q_emlrtBCI, &st);
    loop_ub = T->size[1];
    i3 = r0->size[0];
    r0->size[0] = loop_ub;
    emxEnsureCapacity(&st, (emxArray__common *)r0, i3, (int32_T)sizeof(int32_T),
                      &b_emlrtRTEI);
    for (i3 = 0; i3 < loop_ub; i3++) {
      r0->data[i3] = i3;
    }

    i3 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = 11 + next->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)d, i3, (int32_T)sizeof(real_T),
                      &b_emlrtRTEI);
    for (i3 = 0; i3 < 11; i3++) {
      d->data[d->size[0] * i3] = xNew_data[xNew_size[0] * i3];
    }

    loop_ub = next->size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      d->data[d->size[0] * (i3 + 11)] = next->data[next->size[0] * i3];
    }

    iv3[0] = 1;
    iv3[1] = r0->size[0];
    emlrtSubAssignSizeCheckR2012b(iv3, 2, *(int32_T (*)[2])d->size, 2, &emlrtECI,
      &st);
    loop_ub = d->size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      T->data[((int32_T)((real_T)nodeIDCount + 1.0) + T->size[0] * r0->data[i3])
        - 1] = d->data[d->size[0] * i3];
    }

    /* Append the new node to the tree.     */
    /* if mod(nodeIDCount,100) == 0 */
    /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
    /* end */
    nodeIDCount++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&b_T);
  emxFree_int32_T(&r0);

  /* Find the closest node in the tree to the goal node. */
  st.site = &h_emlrtRSI;
  nearestNeighbour(&st, nGoal, T, jointLimits, kC->l2, kC->l3, kC->l4, kC->l5,
                   kC->l7, kC->zeta, nodeIDCount, 11, xNearest_data,
                   xNearest_size, d, &unusedU2);
  transitionArrayLength = xNearest_data[0];
  i3 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 11;
  emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 11; i3++) {
    next->data[i3] = xNearest_data[i3];
  }

  i3 = path->size[0] * path->size[1];
  path->size[0] = 1;
  path->size[1] = 11 + d->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof(real_T),
                    &b_emlrtRTEI);
  for (i3 = 0; i3 < 11; i3++) {
    path->data[path->size[0] * i3] = xNearest_data[xNearest_size[0] * i3];
  }

  loop_ub = d->size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    path->data[path->size[0] * (i3 + 11)] = d->data[d->size[0] * i3];
  }

  emxFree_real_T(&d);
  emxInit_real_T(sp, &b_path, 2, &b_emlrtRTEI, true);
  exitg1 = false;
  while ((!exitg1) && (transitionArrayLength != 0.0)) {
    i3 = next->size[1];
    emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &r_emlrtBCI, sp);
    if (next->data[1] != 0.0) {
      i3 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &s_emlrtBCI, sp);
      transitionArrayLength = next->data[1];
      loop_ub = T->size[1];
      i3 = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength,
        &e_emlrtDCI, sp);
      emlrtDynamicBoundsCheckFastR2012b(i3, 1, 1000, &t_emlrtBCI, sp);
      i3 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = loop_ub;
      emxEnsureCapacity(sp, (emxArray__common *)next, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      for (i3 = 0; i3 < loop_ub; i3++) {
        next->data[next->size[0] * i3] = T->data[((int32_T)transitionArrayLength
          + T->size[0] * i3) - 1];
      }

      i3 = path->size[1];
      ix = next->size[1];
      emlrtDimSizeEqCheckFastR2012b(i3, ix, &c_emlrtECI, sp);
      i3 = b_path->size[0] * b_path->size[1];
      b_path->size[0] = path->size[0] + 1;
      b_path->size[1] = path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)b_path, i3, (int32_T)sizeof
                        (real_T), &b_emlrtRTEI);
      loop_ub = path->size[1];
      for (i3 = 0; i3 < loop_ub; i3++) {
        xi = path->size[0];
        for (ix = 0; ix < xi; ix++) {
          b_path->data[ix + b_path->size[0] * i3] = path->data[ix + path->size[0]
            * i3];
        }
      }

      loop_ub = next->size[1];
      for (i3 = 0; i3 < loop_ub; i3++) {
        b_path->data[path->size[0] + b_path->size[0] * i3] = next->data
          [next->size[0] * i3];
      }

      i3 = path->size[0] * path->size[1];
      path->size[0] = b_path->size[0];
      path->size[1] = b_path->size[1];
      emxEnsureCapacity(sp, (emxArray__common *)path, i3, (int32_T)sizeof(real_T),
                        &b_emlrtRTEI);
      loop_ub = b_path->size[1];
      for (i3 = 0; i3 < loop_ub; i3++) {
        xi = b_path->size[0];
        for (ix = 0; ix < xi; ix++) {
          path->data[ix + path->size[0] * i3] = b_path->data[ix + b_path->size[0]
            * i3];
        }
      }

      i3 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i3, &u_emlrtBCI, sp);
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
