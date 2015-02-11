/*
 * heuristicSingleLeg.c
 *
 * Code generation for function 'heuristicSingleLeg'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "heuristicSingleLeg.h"
#include "eml_error.h"
#include "buildRRT_data.h"

/* Variable Definitions */
static emlrtBCInfo j_emlrtBCI = { -1, -1, 10, 14, "xA", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 9, 13, "xA", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

static emlrtBCInfo l_emlrtBCI = { -1, -1, 8, 14, "xA", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

/* Function Definitions */
real_T heuristicSingleLeg(const emlrtStack *sp, const emxArray_real_T *xA, const
  emxArray_real_T *xB, const real_T kinematicConst[12])
{
  int32_T i1;
  real_T xStarA;
  real_T dxStar;
  real_T dAlpha;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* heuristic Calculates the distance between states x1 and x2. */
  i1 = xA->size[1];
  emlrtDynamicBoundsCheckFastR2012b(4, 1, i1, &l_emlrtBCI, sp);
  i1 = xA->size[1];
  emlrtDynamicBoundsCheckFastR2012b(5, 1, i1, &k_emlrtBCI, sp);
  i1 = xA->size[1];
  emlrtDynamicBoundsCheckFastR2012b(6, 1, i1, &j_emlrtBCI, sp);
  i1 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(4, 1, i1, &i_emlrtBCI, sp);
  i1 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(5, 1, i1, &h_emlrtBCI, sp);
  i1 = xB->size[1];
  emlrtDynamicBoundsCheckFastR2012b(6, 1, i1, &g_emlrtBCI, sp);

  /* [x1,y1,z1] = fk(alpha1,beta1,gamma1,kinematicConst); */
  /* [x2,y2,z2] = fk(alpha2,beta2,gamma2,kinematicConst); */
  /* distMAX = [sqrt(range(1)^2+range(2)+range(3)^2) sqrt(range(4)^2+range(5)^2+range(6)^2)]; */
  /* distMAX = [10 1]; */
  /* d = HGAINS(1)*cartDist(xA(4:6),xB(4:6))/distMAX(1) +... */
  /*     HGAINS(2)*abs(z1-z2); */
  /* d = HGAINS(1)*cartDist([x1 y1 z1],[x2 y2 z2])/distMAX(1); */
  /*     HGAINS(2)*cartDist(x1(7:9),x2(7:9))/distMAX(2); */
  xStarA = (((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos(xA->
    data[4])) + kinematicConst[3] * muDoubleScalarCos(kinematicConst[8])) +
            kinematicConst[4] * muDoubleScalarCos(kinematicConst[8] + xA->data[5]))
    - kinematicConst[6];
  dxStar = ((((kinematicConst[1] + kinematicConst[2] * muDoubleScalarCos
               (xB->data[4])) + kinematicConst[3] * muDoubleScalarCos
              (kinematicConst[8])) + kinematicConst[4] * muDoubleScalarCos
             (kinematicConst[8] + xB->data[5])) - kinematicConst[6]) - xStarA;

  /* angDiff Finds the angular difference between th1 and th2. */
  dAlpha = ((xA->data[3] - xB->data[3]) + 3.1415926535897931) /
    6.2831853071795862;
  if (muDoubleScalarAbs(dAlpha - muDoubleScalarRound(dAlpha)) <=
      2.2204460492503131E-16 * muDoubleScalarAbs(dAlpha)) {
    dAlpha = 0.0;
  } else {
    dAlpha = (dAlpha - muDoubleScalarFloor(dAlpha)) * 6.2831853071795862;
  }

  dAlpha = muDoubleScalarAbs(dAlpha - 3.1415926535897931);
  st.site = &s_emlrtRSI;
  dAlpha = dxStar * dxStar + xStarA * xStarA * (dAlpha * dAlpha);
  if (dAlpha < 0.0) {
    b_st.site = &t_emlrtRSI;
    b_eml_error(&b_st);
  }

  return muDoubleScalarSqrt(dAlpha);
}

/* End of code generation (heuristicSingleLeg.c) */
