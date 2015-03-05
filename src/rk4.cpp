//
// File: rk4.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 15:01:21
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "rk4.h"
#include "getPhiAndOmega.h"
#include <stdio.h>

// Function Declarations
static void f(const double x[10], const double u[2], double kC_l3, double kC_l5,
              double kC_zeta, double xDot[10]);

// Function Definitions

//
// Arguments    : const double x[10]
//                const double u[2]
//                double kC_l3
//                double kC_l5
//                double kC_zeta
//                double xDot[10]
// Return Type  : void
//
static void f(const double x[10], const double u[2], double kC_l3, double kC_l5,
              double kC_zeta, double xDot[10])
{
  // gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); 
  // GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of
  // gamma given a pan height constraint and an independent beta angle.
  xDot[0] = x[5];
  xDot[1] = x[6];
  xDot[2] = x[7];
  xDot[3] = 0.0;
  xDot[4] = 0.0;
  xDot[5] = u[0];
  xDot[6] = u[1];
  xDot[7] = ((-u[1] * kC_l3 * std::cos(x[1]) + x[6] * x[6] * kC_l3 * std::sin(x
    [1])) + x[7] * x[7] * kC_l5 * std::sin(kC_zeta + x[2])) / (kC_l5 * std::cos
    (kC_zeta + x[2]));
  xDot[8] = 0.0;
  xDot[9] = 0.0;
}

//
// rk4 Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const double uIn[2]
//                const double uBDot[6]
//                const double xInit[13]
//                const double jointLimits[20]
//                const struct0_T *kC
//                int legNum
//                double xNewFull[13]
//                double transitionArray[80]
// Return Type  : void
//
void rk4(const double uIn[2], const double uBDot[6], const double xInit[13],
         const double jointLimits[20], const struct0_T *kC, int legNum, double
         xNewFull[13], double transitionArray[80])
{
  double u[2];
  int i13;
  double b_xInit[10];
  double k1[10];
  unsigned char tmp_data[80];
  int i;
  double c_xInit[10];
  double k2[10];
  double k3[10];
  double dv15[10];
  double alpha;
  double beta;
  double b_gamma;
  double alphaDot;
  double betaDot;
  double gammaDot;
  double alphaDotDot;
  double betaDotDot;
  double b_alphaDot[4];
  double b_alpha[4];
  double omega;
  double phi;
  int i14;
  int loop_ub;
  int i15;

  // rk4.m
  // author: wreid
  // date: 20150107
  for (i13 = 0; i13 < 2; i13++) {
    u[i13] = uIn[i13];
  }

  memcpy(&b_xInit[0], &xInit[3], 10U * sizeof(double));
  memset(&transitionArray[0], 0, 80U * sizeof(double));
  memcpy(&transitionArray[0], &xInit[3], 10U * sizeof(double));
  for (i = 0; i < 7; i++) {
    f(b_xInit, u, kC->l3, kC->l5, kC->zeta, k1);
    for (i13 = 0; i13 < 10; i13++) {
      c_xInit[i13] = b_xInit[i13] + 0.05 * k1[i13];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, k2);
    for (i13 = 0; i13 < 10; i13++) {
      c_xInit[i13] = b_xInit[i13] + 0.05 * k2[i13];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, k3);
    for (i13 = 0; i13 < 10; i13++) {
      c_xInit[i13] = b_xInit[i13] + 0.05 * k3[i13];
    }

    f(c_xInit, u, kC->l3, kC->l5, kC->zeta, dv15);
    for (i13 = 0; i13 < 10; i13++) {
      k1[i13] = b_xInit[i13] + 0.016666666666666666 * (((k1[i13] + 2.0 * k2[i13])
        + 2.0 * k3[i13]) + dv15[i13]);
    }

    alpha = k1[0];
    beta = k1[1];
    b_gamma = k1[2];
    alphaDot = k1[5];
    betaDot = k1[6];
    gammaDot = k1[7];
    alphaDotDot = u[0];
    betaDotDot = u[1];

    // Check pan angular position limits
    if ((k1[0] > jointLimits[1]) || (k1[0] < jointLimits[0])) {
      alpha = b_xInit[0];
      alphaDot = 0.0;
      alphaDotDot = 0.0;
    }

    // Check inner and outer leg angular position limits
    if ((k1[1] > jointLimits[3]) || (k1[1] < jointLimits[2]) || (k1[2] >
         jointLimits[5]) || (k1[2] < jointLimits[4])) {
      beta = b_xInit[1];
      b_gamma = b_xInit[2];
      betaDot = 0.0;
      gammaDot = 0.0;
      betaDotDot = 0.0;
    }

    // Check pan angular velocity limits
    if ((alphaDot > jointLimits[11]) || (alphaDot < jointLimits[10])) {
      alphaDot = b_xInit[5];
      alphaDotDot = 0.0;

      // else
      //     alphaDotDot = uIn(1);
    }

    // Check the inner leg velocity limit.
    if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
      betaDot = b_xInit[6];
      betaDotDot = 0.0;

      // GETCONSTRAINEDGAMMADOT This function calculates the velocity of
      // gamma given a pan height constraint and an independent beta angle.
      //
      // getConstrainedGammaDot.m
      // author: wreid
      // date: 20150224
      gammaDot = -b_xInit[6] * kC->l3 * std::cos(beta) / (kC->l5 * std::cos
        (kC->zeta + b_gamma));
    }

    // Check the outer leg velocity limit.
    if ((gammaDot > jointLimits[15]) || (gammaDot < jointLimits[14])) {
      gammaDot = b_xInit[7];

      // GETCONSTRAINEDBETAADOT This function calculates the velocity of
      // gamma given a pan height constraint and an independent beta angle.
      //
      // getConstrainedBetaDot.m
      // author: wreid
      // date: 20150224
      betaDot = -b_xInit[7] * kC->l5 * std::cos(kC->zeta + b_gamma) / (kC->l3 *
        std::cos(beta));

      // GETCONSTRAINEDBETADOTDOT This function calculates the acceleration of
      // beta given a pan height constraint and an indpendent gamma angle.
      betaDotDot = ((-0.0 * kC->l5 * std::cos(kC->zeta + b_gamma) + b_xInit[7] *
                     b_xInit[7] * kC->l5 * std::sin(kC->zeta + b_gamma)) -
                    betaDot * betaDot * kC->l3 * std::sin(beta)) / (kC->l3 * std::
        cos(beta));
      if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
        betaDot = b_xInit[6];
        betaDotDot = 0.0;
      }
    }

    //          if betaDot > betaDotMax || betaDot < betaDotMin || gammaDot > gammaDotMax || gammaDot < gammaDotMin 
    //              betaDot = xInit(7);
    //              gammaDot = xInit(8);
    //              betaDotDot = 0;
    //          else
    //             betaDotDot = uIn(2);
    //          end
    // Check the outer leg velocity limit.
    b_alphaDot[0] = alphaDot;
    b_alphaDot[1] = betaDot;
    b_alphaDot[2] = gammaDot;
    b_alphaDot[3] = 0.0;
    b_alpha[0] = alpha;
    b_alpha[1] = beta;
    b_alpha[2] = b_gamma;
    b_alpha[3] = 0.0;
    getPhiAndOmega(uBDot, b_alphaDot, b_alpha, kC, legNum, &phi, &omega);

    //          %Check if phi is above threshold, if so then stop the leg and drive 
    //          %the steering joint until it is in the correct orientation.
    //          if abs(phi-phiInit)/dt > phiDotMax
    //            if phi > phiInit
    //              phiDot = phiDotMax;
    //            else
    //              phiDot = phiDotMin;
    //            end
    //            alphaDotDot = 0;
    //            betaDotDot = 0;
    //            alphaDot = 0;
    //            betaDot = 0;
    //            gammaDot = 0;
    //            omega = 0;
    //            alpha = xInitOrig(1);
    //            beta = xInitOrig(2);
    //            gamma = xInitOrig(3);
    //          end
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
    memcpy(&b_xInit[0], &k1[0], 10U * sizeof(double));
    i13 = 10 * (1 + i) + 1;
    i14 = 10 * (2 + i);
    if (i13 > i14) {
      i13 = 0;
      i14 = 0;
    } else {
      i13--;
    }

    loop_ub = i14 - i13;
    for (i15 = 0; i15 < loop_ub; i15++) {
      tmp_data[i15] = (unsigned char)(i13 + i15);
    }

    loop_ub = i14 - i13;
    for (i13 = 0; i13 < loop_ub; i13++) {
      transitionArray[tmp_data[i13]] = k1[i13];
    }
  }

  for (i13 = 0; i13 < 3; i13++) {
    xNewFull[i13] = 0.0;
  }

  memcpy(&xNewFull[3], &k1[0], 10U * sizeof(double));
}

//
// File trailer for rk4.cpp
//
// [EOF]
//
