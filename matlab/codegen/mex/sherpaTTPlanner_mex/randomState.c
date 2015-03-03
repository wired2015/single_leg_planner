/*
 * randomState.c
 *
 * Code generation for function 'randomState'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "randomState.h"
#include "sherpaTTIK.h"
#include "getXStar.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Function Definitions */
void randomState(const emlrtStack *sp, const real_T jointLimits[20], real_T
                 panHeight, real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T
                 kC_l4, real_T kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8,
                 real_T kC_zeta, real_T kC_r, real_T xRand[13])
{
  real_T r;
  real_T xMax;
  real_T xMin;
  const mxArray *y;
  static const int32_T iv2[2] = { 1, 17 };

  const mxArray *m0;
  char_T cv0[17];
  int32_T i;
  static const char_T cv1[17] = { 'z', ' ', 'i', 's', ' ', 'o', 'u', 't', ' ',
    'o', 'f', ' ', 'r', 'a', 'n', 'g', 'e' };

  real_T b_r;
  real_T b_xMin[3];
  real_T q[3];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

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
  st.site = &g_emlrtRSI;
  emlrtRandu(&r, 1);
  if ((panHeight <= -0.293) && (panHeight >= -0.671)) {
    st.site = &h_emlrtRSI;
    xMax = getXStar(&st, panHeight, jointLimits[4], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    st.site = &i_emlrtRSI;
    xMin = getXStar(&st, panHeight, jointLimits[2], true, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else if ((panHeight < -0.671) && (panHeight >= -0.7546)) {
    st.site = &j_emlrtRSI;
    xMax = getXStar(&st, panHeight, jointLimits[4], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    st.site = &k_emlrtRSI;
    xMin = getXStar(&st, panHeight, jointLimits[5], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else if ((panHeight < -0.7546) && (panHeight >= -1.1326)) {
    st.site = &l_emlrtRSI;
    xMax = getXStar(&st, panHeight, jointLimits[3], true, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
    st.site = &m_emlrtRSI;
    xMin = getXStar(&st, panHeight, jointLimits[5], false, kC_l1, kC_l2, kC_l3,
                    kC_l4, kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  } else {
    xMax = 0.0;
    xMin = 0.0;
    y = NULL;
    m0 = emlrtCreateCharArray(2, iv2);
    for (i = 0; i < 17; i++) {
      cv0[i] = cv1[i];
    }

    emlrtInitCharArrayR2013a(sp, 17, m0, cv0);
    emlrtAssign(&y, m0);
    st.site = &jc_emlrtRSI;
    disp(&st, y, &emlrtMCI);
  }

  st.site = &n_emlrtRSI;
  emlrtRandu(&b_r, 1);
  b_xMin[0] = xMin + (xMax - xMin) * b_r;
  b_xMin[1] = 0.0;
  b_xMin[2] = panHeight;
  st.site = &o_emlrtRSI;
  b_sherpaTTIK(&st, b_xMin, kC_l1, kC_l2, kC_l3, kC_l4, kC_l5, kC_l6, kC_l7,
               kC_l8, kC_zeta, kC_r, jointLimits, q);
  st.site = &p_emlrtRSI;
  emlrtRandu(&b_r, 1);
  st.site = &q_emlrtRSI;
  emlrtRandu(&xMax, 1);
  st.site = &r_emlrtRSI;
  emlrtRandu(&xMin, 1);

  /* betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); */
  for (i = 0; i < 3; i++) {
    xRand[i] = 0.0;
  }

  xRand[3] = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * r;
  xRand[4] = q[1];
  xRand[5] = q[2];
  xRand[6] = 0.0;
  xRand[7] = 0.0;
  xRand[8] = (jointLimits[9] - jointLimits[8]) * b_r + jointLimits[8];
  xRand[9] = (jointLimits[11] - jointLimits[10]) * xMax + jointLimits[10];
  xRand[10] = (jointLimits[13] - jointLimits[12]) * xMin + jointLimits[12];
  xRand[11] = 0.0;
  xRand[12] = 0.0;

  /* if mod(nodeIDCount,goalSeedFreq) == 0 */
  /*     xRand = nGoal; */
  /* end */
}

/* End of code generation (randomState.c) */
