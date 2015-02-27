/*
 * getXStar.c
 *
 * Code generation for function 'getXStar'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "getXStar.h"
#include "eml_error.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo cb_emlrtRSI = { 30, "getXStar",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getXStar.m"
};

static emlrtRSInfo db_emlrtRSI = { 32, "getXStar",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getXStar.m"
};

static emlrtRSInfo eb_emlrtRSI = { 15, "asin",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/asin.m" };

/* Function Definitions */
real_T getXStar(const emlrtStack *sp, real_T z, real_T angle, boolean_T selector,
                real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T
                kC_l5, real_T kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta,
                real_T kC_r)
{
  real_T xStar;
  real_T A;
  real_T B;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* GETXSTAR Returns the xStar value given either the height of the wheel */
  /* contact               */
  /* point relative to the pan coordinate frame and either the beta or gamma */
  /* joint value. It is assumed that the angle input represents the beta joint */
  /* angle if selector = false, and the angle input is the gamma joint angle if */
  /* selector = true */
  /*  */
  /* Inputs: */
  /* -z: The height from the pan coordinate frame to the wheel contact point. */
  /* -angle: The angular value that is either beta or gamma, depending on the */
  /* selector input. */
  /* -selector: A logical that indicates that the angle value represents beta */
  /* -kC: A struct containing the kinematic constants of the Sherpa TT leg. */
  /*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   */
  /* if selector=true, or gamma if selector=false. */
  /* Outputs: */
  /* -xStar: The radius in a cylindrical coordinate representation that */
  /* connects the pan coordinate frame to the wheel contact coordinate frame. */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150216 */
  A = (kC_l2 + kC_l4 * muDoubleScalarCos(kC_zeta)) - kC_l7;
  B = (((kC_l1 - kC_l4 * muDoubleScalarSin(kC_zeta)) - kC_l6) - kC_l8) - kC_r;
  if (!selector) {
    B = ((-z + B) - kC_l5 * muDoubleScalarSin(kC_zeta + angle)) / kC_l3;
    st.site = &cb_emlrtRSI;
    if ((B < -1.0) || (1.0 < B)) {
      b_st.site = &eb_emlrtRSI;
      b_eml_error(&b_st);
    }

    xStar = (A + kC_l3 * muDoubleScalarCos(muDoubleScalarAsin(B))) + kC_l5 *
      muDoubleScalarCos(kC_zeta + angle);
  } else {
    B = ((B - kC_l3 * muDoubleScalarSin(angle)) - z) / kC_l5;
    st.site = &db_emlrtRSI;
    if ((B < -1.0) || (1.0 < B)) {
      b_st.site = &eb_emlrtRSI;
      b_eml_error(&b_st);
    }

    xStar = (A + kC_l3 * muDoubleScalarCos(angle)) + kC_l5 * muDoubleScalarCos
      (muDoubleScalarAsin(B));
  }

  return xStar;
}

/* End of code generation (getXStar.c) */
