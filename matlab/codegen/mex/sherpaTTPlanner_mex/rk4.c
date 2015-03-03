/*
 * rk4.c
 *
 * Code generation for function 'rk4'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "rk4.h"
#include "getPhiAndOmega.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo nb_emlrtRSI = { 112, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtECInfo b_emlrtECI = { -1, 139, 9, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

/* Function Declarations */
static void f(const real_T x[10], const real_T u[2], real_T kC_l3, real_T kC_l5,
              real_T kC_zeta, real_T xDot[10]);

/* Function Definitions */
static void f(const real_T x[10], const real_T u[2], real_T kC_l3, real_T kC_l5,
              real_T kC_zeta, real_T xDot[10])
{
  /* gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); */
  /* GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of */
  /* gamma given a pan height constraint and an independent beta angle. */
  xDot[0] = x[5];
  xDot[1] = x[6];
  xDot[2] = x[7];
  xDot[3] = 0.0;
  xDot[4] = 0.0;
  xDot[5] = u[0];
  xDot[6] = u[1];
  xDot[7] = ((-u[1] * kC_l3 * muDoubleScalarCos(x[1]) + x[6] * x[6] * kC_l3 *
              muDoubleScalarSin(x[1])) + x[7] * x[7] * kC_l5 * muDoubleScalarSin
             (kC_zeta + x[2])) / (kC_l5 * muDoubleScalarCos(kC_zeta + x[2]));
  xDot[8] = 0.0;
  xDot[9] = 0.0;
}

void rk4(const emlrtStack *sp, const real_T uIn[2], const real_T uBDot[6], const
         real_T xInit[13], const real_T jointLimits[20], const struct0_T *kC,
         int32_T legNum, real_T xNewFull[13], real_T transitionArray[80])
{
  real_T u[2];
  int32_T i10;
  real_T b_xInit[10];
  real_T k1[10];
  uint8_T tmp_data[80];
  int32_T i;
  real_T c_xInit[10];
  real_T k2[10];
  real_T k3[10];
  real_T dv17[10];
  real_T alpha;
  real_T beta;
  real_T b_gamma;
  real_T alphaDot;
  real_T betaDot;
  real_T gammaDot;
  real_T alphaDotDot;
  real_T betaDotDot;
  real_T b_alphaDot[4];
  real_T b_alpha[4];
  real_T omega;
  real_T phi;
  int32_T i11;
  int32_T i12;
  int32_T loop_ub;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* rk4.m */
  /* author: wreid */
  /* date: 20150107 */
  /* rk4 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i10 = 0; i10 < 2; i10++) {
    u[i10] = uIn[i10];
  }

  memcpy(&b_xInit[0], &xInit[3], 10U * sizeof(real_T));
  memset(&transitionArray[0], 0, 80U * sizeof(real_T));
  memcpy(&transitionArray[0], &xInit[3], 10U * sizeof(real_T));
  for (i = 0; i < 7; i++) {
    f(b_xInit, u, kC->l3, kC->l5, kC->zeta, k1);
    for (i10 = 0; i10 < 10; i10++) {
      c_xInit[i10] = b_xInit[i10] + 0.05 * k1[i10];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, k2);
    for (i10 = 0; i10 < 10; i10++) {
      c_xInit[i10] = b_xInit[i10] + 0.05 * k2[i10];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, k3);
    for (i10 = 0; i10 < 10; i10++) {
      c_xInit[i10] = b_xInit[i10] + 0.05 * k3[i10];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, dv17);
    for (i10 = 0; i10 < 10; i10++) {
      k1[i10] = b_xInit[i10] + 0.016666666666666666 * (((k1[i10] + 2.0 * k2[i10])
        + 2.0 * k3[i10]) + dv17[i10]);
    }

    alpha = k1[0];
    beta = k1[1];
    b_gamma = k1[2];
    alphaDot = k1[5];
    betaDot = k1[6];
    gammaDot = k1[7];
    alphaDotDot = u[0];
    betaDotDot = u[1];

    /* Check pan angular position limits */
    if ((k1[0] > jointLimits[1]) || (k1[0] < jointLimits[0])) {
      alpha = b_xInit[0];
      alphaDot = 0.0;
      alphaDotDot = 0.0;
    }

    /* Check inner and outer leg angular position limits */
    if ((k1[1] > jointLimits[3]) || (k1[1] < jointLimits[2]) || (k1[2] >
         jointLimits[5]) || (k1[2] < jointLimits[4])) {
      beta = b_xInit[1];
      b_gamma = b_xInit[2];
      betaDot = 0.0;
      gammaDot = 0.0;
      betaDotDot = 0.0;
    }

    /* Check pan angular velocity limits */
    if ((alphaDot > jointLimits[11]) || (alphaDot < jointLimits[10])) {
      alphaDot = b_xInit[5];
      alphaDotDot = 0.0;

      /* else */
      /*     alphaDotDot = uIn(1); */
    }

    /* Check the inner leg velocity limit. */
    if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
      betaDot = b_xInit[6];
      betaDotDot = 0.0;

      /* GETCONSTRAINEDGAMMADOT This function calculates the velocity of */
      /* gamma given a pan height constraint and an independent beta angle. */
      /*  */
      /* getConstrainedGammaDot.m */
      /* author: wreid */
      /* date: 20150224 */
      gammaDot = -b_xInit[6] * kC->l3 * muDoubleScalarCos(beta) / (kC->l5 *
        muDoubleScalarCos(kC->zeta + b_gamma));
    }

    /* Check the outer leg velocity limit. */
    if ((gammaDot > jointLimits[15]) || (gammaDot < jointLimits[14])) {
      gammaDot = b_xInit[7];

      /* GETCONSTRAINEDBETAADOT This function calculates the velocity of */
      /* gamma given a pan height constraint and an independent beta angle. */
      /*  */
      /* getConstrainedBetaDot.m */
      /* author: wreid */
      /* date: 20150224 */
      betaDot = -b_xInit[7] * kC->l5 * muDoubleScalarCos(kC->zeta + b_gamma) /
        (kC->l3 * muDoubleScalarCos(beta));

      /* GETCONSTRAINEDBETADOTDOT This function calculates the acceleration of */
      /* beta given a pan height constraint and an indpendent gamma angle. */
      betaDotDot = ((-0.0 * kC->l5 * muDoubleScalarCos(kC->zeta + b_gamma) +
                     b_xInit[7] * b_xInit[7] * kC->l5 * muDoubleScalarSin
                     (kC->zeta + b_gamma)) - betaDot * betaDot * kC->l3 *
                    muDoubleScalarSin(beta)) / (kC->l3 * muDoubleScalarCos(beta));
      if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
        betaDot = b_xInit[6];
        betaDotDot = 0.0;
      }
    }

    /*          if betaDot > betaDotMax || betaDot < betaDotMin || gammaDot > gammaDotMax || gammaDot < gammaDotMin */
    /*              betaDot = xInit(7); */
    /*              gammaDot = xInit(8); */
    /*              betaDotDot = 0; */
    /*          else */
    /*             betaDotDot = uIn(2); */
    /*          end */
    /* Check the outer leg velocity limit. */
    b_alphaDot[0] = alphaDot;
    b_alphaDot[1] = betaDot;
    b_alphaDot[2] = gammaDot;
    b_alphaDot[3] = 0.0;
    b_alpha[0] = alpha;
    b_alpha[1] = beta;
    b_alpha[2] = b_gamma;
    b_alpha[3] = 0.0;
    st.site = &nb_emlrtRSI;
    getPhiAndOmega(&st, uBDot, b_alphaDot, b_alpha, kC, legNum, &phi, &omega);

    /*          %Check if phi is above threshold, if so then stop the leg and drive */
    /*          %the steering joint until it is in the correct orientation. */
    /*          if abs(phi-phiInit)/dt > phiDotMax */
    /*            if phi > phiInit */
    /*              phiDot = phiDotMax; */
    /*            else */
    /*              phiDot = phiDotMin;   */
    /*            end */
    /*            alphaDotDot = 0; */
    /*            betaDotDot = 0; */
    /*            alphaDot = 0; */
    /*            betaDot = 0; */
    /*            gammaDot = 0; */
    /*            omega = 0; */
    /*            alpha = xInitOrig(1); */
    /*            beta = xInitOrig(2); */
    /*            gamma = xInitOrig(3); */
    /*          end */
    u[0] = alphaDotDot;
    u[1] = betaDotDot;
    alphaDotDot = k1[4];
    betaDotDot = k1[8];
    k1[0] = alpha;
    k1[1] = beta;
    k1[2] = b_gamma;
    k1[3] = phi;
    k1[4] = alphaDotDot + 0.1 * omega;
    k1[5] = alphaDot;
    k1[6] = betaDot;
    k1[7] = gammaDot;
    k1[8] = betaDotDot;
    k1[9] = omega;
    memcpy(&b_xInit[0], &k1[0], 10U * sizeof(real_T));
    i10 = 10 * (1 + i) + 1;
    i11 = 10 * (2 + i);
    if (i10 > i11) {
      i10 = 0;
      i11 = 0;
    } else {
      i10--;
    }

    i12 = i11 - i10;
    emlrtSizeEqCheck1DFastR2012b(i12, 10, &b_emlrtECI, sp);
    loop_ub = i11 - i10;
    for (i12 = 0; i12 < loop_ub; i12++) {
      tmp_data[i12] = (uint8_T)(i10 + i12);
    }

    loop_ub = i11 - i10;
    for (i10 = 0; i10 < loop_ub; i10++) {
      transitionArray[tmp_data[i10]] = k1[i10];
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  for (i10 = 0; i10 < 3; i10++) {
    xNewFull[i10] = 0.0;
  }

  memcpy(&xNewFull[3], &k1[0], 10U * sizeof(real_T));
}

/* End of code generation (rk4.c) */
