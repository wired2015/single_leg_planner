//
// File: rk4.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 25-Feb-2015 11:22:41
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "rk4.h"
#include "buildRRT.h"
#include "trInv.h"
#include "generateTrMatrices.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRTWrapper_rtwutil.h"
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
  xDot[7] = ((-u[1] * kC_l3 * cos(x[1]) + x[6] * x[6] * kC_l3 * sin(x[1])) + x[7]
             * x[7] * kC_l5 * sin(kC_zeta + x[2])) / (kC_l5 * cos(kC_zeta + x[2]));
  xDot[8] = 0.0;
  xDot[9] = 0.0;
}

//
// rk4 Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const double uIn[2]
//                const double uBDot[6]
//                double dt
//                double Dt
//                double xInit_data[]
//                const double jointLimits[20]
//                const struct0_T *kC
//                int legNum
//                double xNew_data[]
//                int xNew_size[2]
//                emxArray_real_T *transitionArray
// Return Type  : void
//
void rk4(const double uIn[2], const double uBDot[6], double dt, double Dt,
         double xInit_data[], const double jointLimits[20], const struct0_T *kC,
         int legNum, double xNew_data[], int xNew_size[2], emxArray_real_T
         *transitionArray)
{
  double u[2];
  int i4;
  double numIterations;
  double b_xInit_data[13];
  int xInit_size_idx_1;
  int loop_ub;
  int i;
  double c_xInit_data[10];
  double k2[10];
  double k3[10];
  double d_xInit_data[10];
  double e_xInit_data[10];
  double f_xInit_data[10];
  double g_xInit_data[10];
  double k1[10];
  double y;
  double scale;
  double alpha;
  double beta;
  double b_gamma;
  double alphaDot;
  double betaDot;
  double gammaDot;
  double alphaDotDot;
  double betaDotDot;
  double b_alpha[4];
  double TB2S[16];
  double TP2S[16];
  double TS2R[16];
  double TR2Q[16];
  double TQ2O[16];
  double TO2J[16];
  double TJ2I[16];
  double TI2P[16];
  double TI2S[16];
  double TO2S[16];
  double b_TI2S[16];
  double c_TI2S[16];
  double d_TI2S[16];
  int i5;
  double b_TI2P[16];
  double c_TI2P[16];
  double d_TI2P[16];
  double e_TI2P[16];
  double f_TI2P[16];
  double b_TJ2I[16];
  double c_TJ2I[16];
  double d_TJ2I[16];
  double e_TJ2I[16];
  double b_TQ2O[16];
  double c_TQ2O[16];
  double dv1[9];
  double dv2[9];
  double AdB2S[36];
  double dv3[9];
  double AdP2S[36];
  double dv4[9];
  double AdI2S[36];
  double dv5[9];
  double AdO2S[36];
  double b_AdB2S[6];
  double uPDot[6];
  double uIDot[6];
  double c_AdB2S[6];
  double b_AdP2S[6];
  double uODot[6];
  double b_AdO2S[6];
  double uSDot[6];
  double vS[3];
  double r;
  double absxk;
  double t;
  double c_alpha[10];

  // rk4.m
  // author: wreid
  // date: 20150107
  for (i4 = 0; i4 < 2; i4++) {
    u[i4] = uIn[i4];
  }

  numIterations = rt_roundd_snf(Dt / dt);
  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i4 = 0; i4 < 13; i4++) {
    xNew_data[i4] = 0.0;
  }

  for (i4 = 0; i4 < 10; i4++) {
    b_xInit_data[i4] = xInit_data[3 + i4];
  }

  xInit_size_idx_1 = 10;
  for (i4 = 0; i4 < 10; i4++) {
    xInit_data[i4] = b_xInit_data[i4];
  }

  i4 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = (int)((numIterations + 1.0) * 10.0);
  emxEnsureCapacity((emxArray__common *)transitionArray, i4, (int)sizeof(double));
  loop_ub = (int)((numIterations + 1.0) * 10.0);
  for (i4 = 0; i4 < loop_ub; i4++) {
    transitionArray->data[i4] = 0.0;
  }

  for (i4 = 0; i4 < 10; i4++) {
    transitionArray->data[i4] = xInit_data[i4];
  }

  for (i = 0; i < (int)numIterations; i++) {
    // gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); 
    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      c_xInit_data[i4] = xInit_data[i4];
    }

    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      k2[i4] = xInit_data[i4];
    }

    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      k3[i4] = xInit_data[i4];
    }

    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      d_xInit_data[i4] = xInit_data[i4];
    }

    // GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of
    // gamma given a pan height constraint and an independent beta angle.
    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      e_xInit_data[i4] = xInit_data[i4];
    }

    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      f_xInit_data[i4] = xInit_data[i4];
    }

    for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
      g_xInit_data[i4] = xInit_data[i4];
    }

    k1[0] = e_xInit_data[5];
    k1[1] = f_xInit_data[6];
    k1[2] = g_xInit_data[7];
    k1[3] = 0.0;
    k1[4] = 0.0;
    k1[5] = u[0];
    k1[6] = u[1];
    k1[7] = ((-u[1] * kC->l3 * cos(k3[1]) + c_xInit_data[6] * c_xInit_data[6] *
              kC->l3 * sin(k3[1])) + k2[7] * k2[7] * kC->l5 * sin(kC->zeta +
              d_xInit_data[2])) / (kC->l5 * cos(kC->zeta + d_xInit_data[2]));
    k1[8] = 0.0;
    k1[9] = 0.0;
    y = dt / 2.0;
    for (i4 = 0; i4 < 10; i4++) {
      c_xInit_data[i4] = xInit_data[i4] + y * k1[i4];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k2);
    y = dt / 2.0;
    for (i4 = 0; i4 < 10; i4++) {
      c_xInit_data[i4] = xInit_data[i4] + y * k2[i4];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k3);
    y = dt / 2.0;
    scale = dt / 6.0;
    for (i4 = 0; i4 < 10; i4++) {
      c_xInit_data[i4] = xInit_data[i4] + y * k3[i4];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, d_xInit_data);
    for (i4 = 0; i4 < 10; i4++) {
      c_xInit_data[i4] = xInit_data[i4] + y * k3[i4];
      d_xInit_data[i4] = xInit_data[i4] + scale * (((k1[i4] + 2.0 * k2[i4]) +
        2.0 * k3[i4]) + d_xInit_data[i4]);
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, e_xInit_data);
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i4 = 0; i4 < 10; i4++) {
      xNew_data[i4] = xInit_data[i4] + scale * (((k1[i4] + 2.0 * k2[i4]) + 2.0 *
        k3[i4]) + e_xInit_data[i4]);
    }

    alpha = d_xInit_data[0];
    beta = d_xInit_data[1];
    b_gamma = d_xInit_data[2];
    alphaDot = d_xInit_data[5];
    betaDot = d_xInit_data[6];
    gammaDot = d_xInit_data[7];
    alphaDotDot = u[0];
    betaDotDot = u[1];

    // Check pan angular position limits
    if ((xNew_data[0] > jointLimits[1]) || (xNew_data[0] < jointLimits[0])) {
      alpha = xInit_data[0];
      alphaDot = 0.0;
      alphaDotDot = 0.0;
    }

    // Check inner and outer leg angular position limits
    if ((xNew_data[1] > jointLimits[3]) || (xNew_data[1] < jointLimits[2]) ||
        (xNew_data[2] > jointLimits[5]) || (xNew_data[2] < jointLimits[4])) {
      beta = xInit_data[1];
      b_gamma = xInit_data[2];
      betaDot = 0.0;
      gammaDot = 0.0;
      betaDotDot = 0.0;
    }

    // Check pan angular velocity limits
    if ((alphaDot > jointLimits[11]) || (alphaDot < jointLimits[10])) {
      alphaDot = xInit_data[5];
      alphaDotDot = 0.0;

      // else
      //     alphaDotDot = uIn(1);
    }

    // Check the inner leg velocity limit.
    if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
      betaDot = xInit_data[6];
      betaDotDot = 0.0;

      // GETCONSTRAINEDGAMMADOT This function calculates the velocity of
      // gamma given a pan height constraint and an independent beta angle.
      //
      // getConstrainedGammaDot.m
      // author: wreid
      // date: 20150224
      gammaDot = -xInit_data[6] * kC->l3 * cos(beta) / (kC->l5 * cos(kC->zeta +
        b_gamma));
    }

    // Check the outer leg velocity limit.
    if ((gammaDot > jointLimits[15]) || (gammaDot < jointLimits[14])) {
      gammaDot = xInit_data[7];

      // GETCONSTRAINEDBETAADOT This function calculates the velocity of
      // gamma given a pan height constraint and an independent beta angle.
      //
      // getConstrainedBetaDot.m
      // author: wreid
      // date: 20150224
      betaDot = -xInit_data[7] * kC->l5 * cos(kC->zeta + b_gamma) / (kC->l3 *
        cos(beta));

      // GETCONSTRAINEDBETADOTDOT This function calculates the acceleration of
      // beta given a pan height constraint and an indpendent gamma angle.
      betaDotDot = ((-0.0 * kC->l5 * cos(kC->zeta + b_gamma) + xInit_data[7] *
                     xInit_data[7] * kC->l5 * sin(kC->zeta + b_gamma)) - betaDot
                    * betaDot * kC->l3 * sin(beta)) / (kC->l3 * cos(beta));
      if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
        betaDot = xInit_data[6];
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
    // Homogeneous transformation matrices.
    b_alpha[0] = alpha;
    b_alpha[1] = beta;
    b_alpha[2] = b_gamma;
    b_alpha[3] = 0.0;
    generateTrMatrices(b_alpha, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                       kC->l7, kC->l8, kC->zeta, kC->r, kC->B2PXOffset,
                       kC->B2PZOffset, kC->legAngleOffset, legNum, TO2S, TI2S,
                       TI2P, TJ2I, TO2J, TQ2O, TR2Q, TS2R, TP2S, TB2S);
    for (i4 = 0; i4 < 4; i4++) {
      for (i5 = 0; i5 < 4; i5++) {
        TO2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TO2S[i4 + (i5 << 2)] += TI2S[i4 + (xInit_size_idx_1 << 2)] *
            TI2P[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        TP2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TP2S[i4 + (i5 << 2)] += TO2S[i4 + (xInit_size_idx_1 << 2)] *
            TJ2I[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        TB2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TB2S[i4 + (i5 << 2)] += TP2S[i4 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        c_TI2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TI2S[i4 + (i5 << 2)] += TB2S[i4 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        d_TI2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TI2S[i4 + (i5 << 2)] += c_TI2S[i4 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        b_TI2S[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TI2S[i4 + (i5 << 2)] += d_TI2S[i4 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i5 << 2)];
        }

        b_TI2P[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TI2P[i4 + (i5 << 2)] += TI2P[i4 + (xInit_size_idx_1 << 2)] *
            TJ2I[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        c_TI2P[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TI2P[i4 + (i5 << 2)] += b_TI2P[i4 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        d_TI2P[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TI2P[i4 + (i5 << 2)] += c_TI2P[i4 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        e_TI2P[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          e_TI2P[i4 + (i5 << 2)] += d_TI2P[i4 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        f_TI2P[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          f_TI2P[i4 + (i5 << 2)] += e_TI2P[i4 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i5 << 2)];
        }

        b_TJ2I[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TJ2I[i4 + (i5 << 2)] += TJ2I[i4 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        c_TJ2I[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TJ2I[i4 + (i5 << 2)] += b_TJ2I[i4 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        d_TJ2I[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TJ2I[i4 + (i5 << 2)] += c_TJ2I[i4 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        e_TJ2I[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          e_TJ2I[i4 + (i5 << 2)] += d_TJ2I[i4 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i5 << 2)];
        }

        b_TQ2O[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TQ2O[i4 + (i5 << 2)] += TQ2O[i4 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i5 << 2)];
        }
      }

      for (i5 = 0; i5 < 4; i5++) {
        c_TQ2O[i4 + (i5 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TQ2O[i4 + (i5 << 2)] += b_TQ2O[i4 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i5 << 2)];
        }
      }
    }

    trInv(b_TI2S, TB2S);
    trInv(f_TI2P, TP2S);
    trInv(e_TJ2I, TI2S);
    trInv(c_TQ2O, TO2S);

    // Adjunct transformation matrices.
    // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
    // transformation matrix, T. The adjunct matrix serves to transform the
    // velocity from the one frame to another, as described by the homoegenous
    // transformation matrix.
    //
    // Inputs:
    // -T: The 4x4 homogeneous transformation matrix representing the
    // transformation from one frame to another.
    // Outputs:
    // -A: The adjunct matrix that transforms velocity vectors from one frame to 
    // another.
    dv1[0] = 0.0;
    dv1[3] = -TB2S[14];
    dv1[6] = TB2S[13];
    dv1[1] = TB2S[14];
    dv1[4] = 0.0;
    dv1[7] = -TB2S[12];
    dv1[2] = -TB2S[13];
    dv1[5] = TB2S[12];
    dv1[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        dv2[i4 + 3 * i5] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i4 + 3 * i5] += dv1[i4 + 3 * xInit_size_idx_1] *
            TB2S[xInit_size_idx_1 + (i5 << 2)];
        }

        AdB2S[i5 + 6 * i4] = TB2S[i5 + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdB2S[i5 + 6 * (i4 + 3)] = dv2[i5 + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdB2S[(i5 + 6 * i4) + 3] = 0.0;
      }
    }

    // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
    // transformation matrix, T. The adjunct matrix serves to transform the
    // velocity from the one frame to another, as described by the homoegenous
    // transformation matrix.
    //
    // Inputs:
    // -T: The 4x4 homogeneous transformation matrix representing the
    // transformation from one frame to another.
    // Outputs:
    // -A: The adjunct matrix that transforms velocity vectors from one frame to 
    // another.
    dv3[0] = 0.0;
    dv3[3] = -TP2S[14];
    dv3[6] = TP2S[13];
    dv3[1] = TP2S[14];
    dv3[4] = 0.0;
    dv3[7] = -TP2S[12];
    dv3[2] = -TP2S[13];
    dv3[5] = TP2S[12];
    dv3[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdB2S[(i5 + 6 * (i4 + 3)) + 3] = TB2S[i5 + (i4 << 2)];
        dv2[i4 + 3 * i5] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i4 + 3 * i5] += dv3[i4 + 3 * xInit_size_idx_1] *
            TP2S[xInit_size_idx_1 + (i5 << 2)];
        }

        AdP2S[i5 + 6 * i4] = TP2S[i5 + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdP2S[i5 + 6 * (i4 + 3)] = dv2[i5 + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdP2S[(i5 + 6 * i4) + 3] = 0.0;
      }
    }

    // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
    // transformation matrix, T. The adjunct matrix serves to transform the
    // velocity from the one frame to another, as described by the homoegenous
    // transformation matrix.
    //
    // Inputs:
    // -T: The 4x4 homogeneous transformation matrix representing the
    // transformation from one frame to another.
    // Outputs:
    // -A: The adjunct matrix that transforms velocity vectors from one frame to 
    // another.
    dv4[0] = 0.0;
    dv4[3] = -TI2S[14];
    dv4[6] = TI2S[13];
    dv4[1] = TI2S[14];
    dv4[4] = 0.0;
    dv4[7] = -TI2S[12];
    dv4[2] = -TI2S[13];
    dv4[5] = TI2S[12];
    dv4[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdP2S[(i5 + 6 * (i4 + 3)) + 3] = TP2S[i5 + (i4 << 2)];
        dv2[i4 + 3 * i5] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i4 + 3 * i5] += dv4[i4 + 3 * xInit_size_idx_1] *
            TI2S[xInit_size_idx_1 + (i5 << 2)];
        }

        AdI2S[i5 + 6 * i4] = TI2S[i5 + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdI2S[i5 + 6 * (i4 + 3)] = dv2[i5 + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdI2S[(i5 + 6 * i4) + 3] = 0.0;
      }
    }

    // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
    // transformation matrix, T. The adjunct matrix serves to transform the
    // velocity from the one frame to another, as described by the homoegenous
    // transformation matrix.
    //
    // Inputs:
    // -T: The 4x4 homogeneous transformation matrix representing the
    // transformation from one frame to another.
    // Outputs:
    // -A: The adjunct matrix that transforms velocity vectors from one frame to 
    // another.
    dv5[0] = 0.0;
    dv5[3] = -TO2S[14];
    dv5[6] = TO2S[13];
    dv5[1] = TO2S[14];
    dv5[4] = 0.0;
    dv5[7] = -TO2S[12];
    dv5[2] = -TO2S[13];
    dv5[5] = TO2S[12];
    dv5[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdI2S[(i5 + 6 * (i4 + 3)) + 3] = TI2S[i5 + (i4 << 2)];
        dv2[i4 + 3 * i5] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i4 + 3 * i5] += dv5[i4 + 3 * xInit_size_idx_1] *
            TO2S[xInit_size_idx_1 + (i5 << 2)];
        }

        AdO2S[i5 + 6 * i4] = TO2S[i5 + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdO2S[i5 + 6 * (i4 + 3)] = dv2[i5 + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdO2S[(i5 + 6 * i4) + 3] = 0.0;
      }
    }

    // Pan joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      for (i5 = 0; i5 < 3; i5++) {
        AdO2S[(i5 + 6 * (i4 + 3)) + 3] = TO2S[i5 + (i4 << 2)];
      }

      b_AdB2S[i4] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = alphaDot;
    for (i4 = 0; i4 < 6; i4++) {
      uPDot[i4] = b_AdB2S[i4];
    }

    // Beta joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      b_AdB2S[i4] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = betaDot;
    for (i4 = 0; i4 < 6; i4++) {
      uIDot[i4] = b_AdB2S[i4];
    }

    // Gamma joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      b_AdB2S[i4] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = gammaDot;

    // Velocity vector for the ankle frame.
    for (i4 = 0; i4 < 6; i4++) {
      uODot[i4] = b_AdB2S[i4];
      c_AdB2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        c_AdB2S[i4] += AdB2S[i4 + 6 * i5] * uBDot[i5];
      }

      b_AdP2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_AdP2S[i4] += AdP2S[i4 + 6 * i5] * uPDot[i5];
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      scale = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        scale += AdI2S[i4 + 6 * i5] * uIDot[i5];
      }

      b_AdB2S[i4] = (c_AdB2S[i4] + b_AdP2S[i4]) + scale;
    }

    for (i4 = 0; i4 < 6; i4++) {
      b_AdO2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_AdO2S[i4] += AdO2S[i4 + 6 * i5] * uODot[i5];
      }

      uSDot[i4] = b_AdB2S[i4] + b_AdO2S[i4];
      c_AdB2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        c_AdB2S[i4] += AdB2S[i4 + 6 * i5] * uBDot[i5];
      }

      b_AdP2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_AdP2S[i4] += AdP2S[i4 + 6 * i5] * uPDot[i5];
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      scale = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        scale += AdI2S[i4 + 6 * i5] * uIDot[i5];
      }

      b_AdB2S[i4] = (c_AdB2S[i4] + b_AdP2S[i4]) + scale;
    }

    for (i4 = 0; i4 < 6; i4++) {
      b_AdO2S[i4] = 0.0;
      for (i5 = 0; i5 < 6; i5++) {
        b_AdO2S[i4] += AdO2S[i4 + 6 * i5] * uODot[i5];
      }

      c_AdB2S[i4] = b_AdB2S[i4] + b_AdO2S[i4];
    }

    for (i4 = 0; i4 < 3; i4++) {
      vS[i4] = c_AdB2S[i4];
    }

    // [m/s]
    // [rad/s]
    // Calculate the required phi joint angle and the required wheel speed,
    // omega.
    // [rad]
    r = ((rt_atan2d_snf(uSDot[1], uSDot[0]) - 1.5707963267948966) +
         3.1415926535897931) / 6.2831853071795862;
    if (fabs(r - rt_roundd_snf(r)) <= 2.2204460492503131E-16 * fabs(r)) {
      r = 0.0;
    } else {
      r = (r - floor(r)) * 6.2831853071795862;
    }

    //      if phiDiff > pi/2
    //          phi = phi - pi;
    //          phi = mod(phi+pi,2*pi)-pi;
    //          sign = -1;
    //      end
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
      absxk = fabs(vS[xInit_size_idx_1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);
    t = y / kC->r;

    // [rad/s]
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
    scale = xNew_data[4];
    absxk = xNew_data[8];
    c_alpha[0] = alpha;
    c_alpha[1] = beta;
    c_alpha[2] = b_gamma;
    c_alpha[3] = r - 3.1415926535897931;
    c_alpha[4] = scale + dt * t;
    c_alpha[5] = alphaDot;
    c_alpha[6] = betaDot;
    c_alpha[7] = gammaDot;
    c_alpha[8] = absxk;
    c_alpha[9] = t;
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i4 = 0; i4 < 10; i4++) {
      xNew_data[xNew_size[0] * i4] = c_alpha[i4];
    }

    xInit_size_idx_1 = xNew_size[1];
    loop_ub = xNew_size[0] * xNew_size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      xInit_data[i4] = xNew_data[i4];
    }

    scale = 10.0 * (1.0 + (double)i) + 1.0;
    if (scale > 10.0 * ((1.0 + (double)i) + 1.0)) {
      i4 = 0;
    } else {
      i4 = (int)scale - 1;
    }

    loop_ub = xNew_size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      transitionArray->data[i4 + i5] = xNew_data[xNew_size[0] * i5];
    }
  }

  xInit_size_idx_1 = 3 + xNew_size[1];
  for (i4 = 0; i4 < 3; i4++) {
    TO2S[i4] = 0.0;
  }

  loop_ub = xNew_size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    TO2S[i4 + 3] = xNew_data[xNew_size[0] * i4];
  }

  xNew_size[0] = 1;
  xNew_size[1] = xInit_size_idx_1;
  for (i4 = 0; i4 < xInit_size_idx_1; i4++) {
    xNew_data[xNew_size[0] * i4] = TO2S[i4];
  }
}

//
// File trailer for rk4.cpp
//
// [EOF]
//
