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
  emxArray_real_T *xB, const real_T jointLimits[12], const real_T
  kinematicConst[16])
{
  real_T d;
  int32_T i2;
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
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(4, 1, i2, &gb_emlrtBCI, sp);
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(5, 1, i2, &fb_emlrtBCI, sp);
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(6, 1, i2, &eb_emlrtBCI, sp);
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(7, 1, i2, &db_emlrtBCI, sp);
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(8, 1, i2, &cb_emlrtBCI, sp);
  i2 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(9, 1, i2, &bb_emlrtBCI, sp);

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
  st.site = &v_emlrtRSI;
  b_st.site = &x_emlrtRSI;
  if (dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax * dAlphaMax) <
      0.0) {
    c_st.site = &f_emlrtRSI;
    eml_error(&c_st);
  }

  xStarMin = (((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                (xA_data[4])) + kinematicConst[3] * muDoubleScalarCos
               (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarCos
              (kinematicConst[8] + xA_data[5])) - kinematicConst[6];
  dxStarMax = ((((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
                  (xB->data[4])) + kinematicConst[3] * muDoubleScalarCos
                 (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarCos
                (kinematicConst[8] + xB->data[5])) - kinematicConst[6]) -
    xStarMin;

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
  st.site = &w_emlrtRSI;
  dAlphaMax = dxStarMax * dxStarMax + xStarMin * xStarMin * (dAlphaMax *
    dAlphaMax);
  if (dAlphaMax < 0.0) {
    b_st.site = &f_emlrtRSI;
    eml_error(&b_st);
  }

  /* Calculate the total distance. */
  d = muDoubleScalarSqrt(dAlphaMax);

  /* dPosNorm+dVelNorm  */
  return d;
}

/* End of code generation (heuristicSingleLeg.c) */
