/*
 * heuristicSingleLeg.c
 *
 * Code generation for function 'heuristicSingleLeg'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "heuristicSingleLeg.h"
#include "eml_error.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Function Definitions */
real_T heuristicSingleLeg(const emlrtStack *sp, const real_T xA_data[], const
  emxArray_real_T *xB, const real_T jointLimits[12], real_T kC_l2, real_T kC_l3,
  real_T kC_l4, real_T kC_l5, real_T kC_l7, real_T kC_zeta)
{
  real_T d;
  int32_T i7;
  real_T xStarMin;
  real_T dxStarMax;
  real_T dAlphaMax;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(4, 1, i7, &q_emlrtBCI, sp);
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(5, 1, i7, &p_emlrtBCI, sp);
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(6, 1, i7, &o_emlrtBCI, sp);
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(7, 1, i7, &n_emlrtBCI, sp);
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(8, 1, i7, &m_emlrtBCI, sp);
  i7 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(9, 1, i7, &l_emlrtBCI, sp);

  /* Calculate the distance between angular positions. */
  xStarMin = (((kC_l2 + kC_l3 * muDoubleScalarCos(jointLimits[2])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(kC_zeta +
    jointLimits[4])) - kC_l7;
  dxStarMax = ((((kC_l2 + kC_l3 * muDoubleScalarCos(jointLimits[3])) + kC_l4 *
                 muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(kC_zeta
    + jointLimits[5])) - kC_l7) - xStarMin;

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
  st.site = &cb_emlrtRSI;
  b_st.site = &eb_emlrtRSI;
  if (dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax * dAlphaMax) <
      0.0) {
    c_st.site = &g_emlrtRSI;
    eml_error(&c_st);
  }

  xStarMin = (((kC_l2 + kC_l3 * muDoubleScalarCos(xA_data[4])) + kC_l4 *
               muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(kC_zeta +
    xA_data[5])) - kC_l7;
  dxStarMax = ((((kC_l2 + kC_l3 * muDoubleScalarCos(xB->data[4])) + kC_l4 *
                 muDoubleScalarCos(kC_zeta)) + kC_l5 * muDoubleScalarCos(kC_zeta
    + xB->data[5])) - kC_l7) - xStarMin;

  /* angDiff Finds the angular difference between th1 and th2. */
  dAlphaMax = ((xA_data[3] - xB->data[3]) + 3.1415926535897931) /
    6.2831853071795862;
  if (muDoubleScalarAbs(dAlphaMax - muDoubleScalarRound(dAlphaMax)) <=
      2.2204460492503131E-16 * muDoubleScalarAbs(dAlphaMax)) {
    dAlphaMax = 0.0;
  } else {
    dAlphaMax = (dAlphaMax - muDoubleScalarFloor(dAlphaMax)) *
      6.2831853071795862;
  }

  dAlphaMax = muDoubleScalarAbs(dAlphaMax - 3.1415926535897931);
  st.site = &db_emlrtRSI;
  dAlphaMax = dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax *
    dAlphaMax);
  if (dAlphaMax < 0.0) {
    b_st.site = &g_emlrtRSI;
    eml_error(&b_st);
  }

  /* Calculate the total distance. */
  d = muDoubleScalarSqrt(dAlphaMax);

  /* dPosNorm+dVelNorm  */
  return d;
}

/* End of code generation (heuristicSingleLeg.c) */
