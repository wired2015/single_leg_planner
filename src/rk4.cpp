//
// File: rk4.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 27-Feb-2015 15:48:27
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "rk4.h"
#include "norm.h"
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
  int i5;
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
  double alphaDotDot;
  double alpha;
  double beta;
  double b_gamma;
  double alphaDot;
  double betaDot;
  double gammaDot;
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
  int i6;
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
  double b_TB2S[36];
  double dv3[9];
  double b_TO2S[6];
  double b_TP2S[36];
  double uSDot[6];
  double c_TP2S[6];
  double dv4[9];
  double e_TI2S[36];
  double dv5[9];
  double dv6[6];
  double c_TO2S[36];
  double c_TB2S[6];
  double r;
  double omega;
  double c_alpha[10];

  // rk4.m
  // author: wreid
  // date: 20150107
  for (i5 = 0; i5 < 2; i5++) {
    u[i5] = uIn[i5];
  }

  numIterations = rt_roundd_snf(Dt / dt);
  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i5 = 0; i5 < 13; i5++) {
    xNew_data[i5] = 0.0;
  }

  for (i5 = 0; i5 < 10; i5++) {
    b_xInit_data[i5] = xInit_data[3 + i5];
  }

  xInit_size_idx_1 = 10;
  for (i5 = 0; i5 < 10; i5++) {
    xInit_data[i5] = b_xInit_data[i5];
  }

  i5 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = (int)((numIterations + 1.0) * 10.0);
  emxEnsureCapacity((emxArray__common *)transitionArray, i5, (int)sizeof(double));
  loop_ub = (int)((numIterations + 1.0) * 10.0);
  for (i5 = 0; i5 < loop_ub; i5++) {
    transitionArray->data[i5] = 0.0;
  }

  for (i5 = 0; i5 < 10; i5++) {
    transitionArray->data[i5] = xInit_data[i5];
  }

  for (i = 0; i < (int)numIterations; i++) {
    // gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); 
    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      c_xInit_data[i5] = xInit_data[i5];
    }

    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      k2[i5] = xInit_data[i5];
    }

    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      k3[i5] = xInit_data[i5];
    }

    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      d_xInit_data[i5] = xInit_data[i5];
    }

    // GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of
    // gamma given a pan height constraint and an independent beta angle.
    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      e_xInit_data[i5] = xInit_data[i5];
    }

    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      f_xInit_data[i5] = xInit_data[i5];
    }

    for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
      g_xInit_data[i5] = xInit_data[i5];
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
    for (i5 = 0; i5 < 10; i5++) {
      c_xInit_data[i5] = xInit_data[i5] + y * k1[i5];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k2);
    y = dt / 2.0;
    for (i5 = 0; i5 < 10; i5++) {
      c_xInit_data[i5] = xInit_data[i5] + y * k2[i5];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k3);
    y = dt / 2.0;
    alphaDotDot = dt / 6.0;
    for (i5 = 0; i5 < 10; i5++) {
      c_xInit_data[i5] = xInit_data[i5] + y * k3[i5];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, d_xInit_data);
    for (i5 = 0; i5 < 10; i5++) {
      c_xInit_data[i5] = xInit_data[i5] + y * k3[i5];
      d_xInit_data[i5] = xInit_data[i5] + alphaDotDot * (((k1[i5] + 2.0 * k2[i5])
        + 2.0 * k3[i5]) + d_xInit_data[i5]);
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, e_xInit_data);
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i5 = 0; i5 < 10; i5++) {
      xNew_data[i5] = xInit_data[i5] + alphaDotDot * (((k1[i5] + 2.0 * k2[i5]) +
        2.0 * k3[i5]) + e_xInit_data[i5]);
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
    for (i5 = 0; i5 < 4; i5++) {
      for (i6 = 0; i6 < 4; i6++) {
        TO2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TO2S[i5 + (i6 << 2)] += TI2S[i5 + (xInit_size_idx_1 << 2)] *
            TI2P[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        TP2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TP2S[i5 + (i6 << 2)] += TO2S[i5 + (xInit_size_idx_1 << 2)] *
            TJ2I[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        TB2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          TB2S[i5 + (i6 << 2)] += TP2S[i5 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        c_TI2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TI2S[i5 + (i6 << 2)] += TB2S[i5 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        d_TI2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TI2S[i5 + (i6 << 2)] += c_TI2S[i5 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        b_TI2S[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TI2S[i5 + (i6 << 2)] += d_TI2S[i5 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i6 << 2)];
        }

        b_TI2P[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TI2P[i5 + (i6 << 2)] += TI2P[i5 + (xInit_size_idx_1 << 2)] *
            TJ2I[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        c_TI2P[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TI2P[i5 + (i6 << 2)] += b_TI2P[i5 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        d_TI2P[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TI2P[i5 + (i6 << 2)] += c_TI2P[i5 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        e_TI2P[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          e_TI2P[i5 + (i6 << 2)] += d_TI2P[i5 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        f_TI2P[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          f_TI2P[i5 + (i6 << 2)] += e_TI2P[i5 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i6 << 2)];
        }

        b_TJ2I[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TJ2I[i5 + (i6 << 2)] += TJ2I[i5 + (xInit_size_idx_1 << 2)] *
            TO2J[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        c_TJ2I[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TJ2I[i5 + (i6 << 2)] += b_TJ2I[i5 + (xInit_size_idx_1 << 2)] *
            TQ2O[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        d_TJ2I[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          d_TJ2I[i5 + (i6 << 2)] += c_TJ2I[i5 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        e_TJ2I[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          e_TJ2I[i5 + (i6 << 2)] += d_TJ2I[i5 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i6 << 2)];
        }

        b_TQ2O[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          b_TQ2O[i5 + (i6 << 2)] += TQ2O[i5 + (xInit_size_idx_1 << 2)] *
            TR2Q[xInit_size_idx_1 + (i6 << 2)];
        }
      }

      for (i6 = 0; i6 < 4; i6++) {
        c_TQ2O[i5 + (i6 << 2)] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 4; xInit_size_idx_1++) {
          c_TQ2O[i5 + (i6 << 2)] += b_TQ2O[i5 + (xInit_size_idx_1 << 2)] *
            TS2R[xInit_size_idx_1 + (i6 << 2)];
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
    // Pan joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    // Beta joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    // Gamma joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    // Velocity vector for the ankle frame.
    dv1[0] = 0.0;
    dv1[3] = -TB2S[14];
    dv1[6] = TB2S[13];
    dv1[1] = TB2S[14];
    dv1[4] = 0.0;
    dv1[7] = -TB2S[12];
    dv1[2] = -TB2S[13];
    dv1[5] = TB2S[12];
    dv1[8] = 0.0;
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        dv2[i5 + 3 * i6] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i5 + 3 * i6] += dv1[i5 + 3 * xInit_size_idx_1] *
            TB2S[xInit_size_idx_1 + (i6 << 2)];
        }

        b_TB2S[i6 + 6 * i5] = TB2S[i6 + (i5 << 2)];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_TB2S[i6 + 6 * (i5 + 3)] = dv2[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_TB2S[(i6 + 6 * i5) + 3] = 0.0;
      }
    }

    dv3[0] = 0.0;
    dv3[3] = -TP2S[14];
    dv3[6] = TP2S[13];
    dv3[1] = TP2S[14];
    dv3[4] = 0.0;
    dv3[7] = -TP2S[12];
    dv3[2] = -TP2S[13];
    dv3[5] = TP2S[12];
    dv3[8] = 0.0;
    for (i5 = 0; i5 < 3; i5++) {
      b_TO2S[i5] = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        b_TB2S[(i6 + 6 * (i5 + 3)) + 3] = TB2S[i6 + (i5 << 2)];
        dv2[i5 + 3 * i6] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i5 + 3 * i6] += dv3[i5 + 3 * xInit_size_idx_1] *
            TP2S[xInit_size_idx_1 + (i6 << 2)];
        }

        b_TP2S[i6 + 6 * i5] = TP2S[i6 + (i5 << 2)];
      }
    }

    b_TO2S[3] = 0.0;
    b_TO2S[4] = 0.0;
    b_TO2S[5] = alphaDot;
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_TP2S[i6 + 6 * (i5 + 3)] = dv2[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_TP2S[(i6 + 6 * i5) + 3] = 0.0;
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        b_TP2S[(i6 + 6 * (i5 + 3)) + 3] = TP2S[i6 + (i5 << 2)];
      }
    }

    for (i5 = 0; i5 < 6; i5++) {
      uSDot[i5] = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        uSDot[i5] += b_TB2S[i5 + 6 * i6] * uBDot[i6];
      }

      c_TP2S[i5] = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        c_TP2S[i5] += b_TP2S[i5 + 6 * i6] * b_TO2S[i6];
      }
    }

    dv4[0] = 0.0;
    dv4[3] = -TI2S[14];
    dv4[6] = TI2S[13];
    dv4[1] = TI2S[14];
    dv4[4] = 0.0;
    dv4[7] = -TI2S[12];
    dv4[2] = -TI2S[13];
    dv4[5] = TI2S[12];
    dv4[8] = 0.0;
    for (i5 = 0; i5 < 3; i5++) {
      b_TO2S[i5] = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        dv2[i5 + 3 * i6] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i5 + 3 * i6] += dv4[i5 + 3 * xInit_size_idx_1] *
            TI2S[xInit_size_idx_1 + (i6 << 2)];
        }

        e_TI2S[i6 + 6 * i5] = TI2S[i6 + (i5 << 2)];
      }
    }

    b_TO2S[3] = 0.0;
    b_TO2S[4] = 0.0;
    b_TO2S[5] = betaDot;
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        e_TI2S[i6 + 6 * (i5 + 3)] = dv2[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        e_TI2S[(i6 + 6 * i5) + 3] = 0.0;
      }
    }

    dv5[0] = 0.0;
    dv5[3] = -TO2S[14];
    dv5[6] = TO2S[13];
    dv5[1] = TO2S[14];
    dv5[4] = 0.0;
    dv5[7] = -TO2S[12];
    dv5[2] = -TO2S[13];
    dv5[5] = TO2S[12];
    dv5[8] = 0.0;
    for (i5 = 0; i5 < 3; i5++) {
      dv6[i5] = 0.0;
      for (i6 = 0; i6 < 3; i6++) {
        e_TI2S[(i6 + 6 * (i5 + 3)) + 3] = TI2S[i6 + (i5 << 2)];
        dv2[i5 + 3 * i6] = 0.0;
        for (xInit_size_idx_1 = 0; xInit_size_idx_1 < 3; xInit_size_idx_1++) {
          dv2[i5 + 3 * i6] += dv5[i5 + 3 * xInit_size_idx_1] *
            TO2S[xInit_size_idx_1 + (i6 << 2)];
        }

        c_TO2S[i6 + 6 * i5] = TO2S[i6 + (i5 << 2)];
      }
    }

    dv6[3] = 0.0;
    dv6[4] = 0.0;
    dv6[5] = gammaDot;
    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        c_TO2S[i6 + 6 * (i5 + 3)] = dv2[i6 + 3 * i5];
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        c_TO2S[(i6 + 6 * i5) + 3] = 0.0;
      }
    }

    for (i5 = 0; i5 < 3; i5++) {
      for (i6 = 0; i6 < 3; i6++) {
        c_TO2S[(i6 + 6 * (i5 + 3)) + 3] = TO2S[i6 + (i5 << 2)];
      }
    }

    for (i5 = 0; i5 < 6; i5++) {
      y = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        y += e_TI2S[i5 + 6 * i6] * b_TO2S[i6];
      }

      c_TB2S[i5] = (uSDot[i5] + c_TP2S[i5]) + y;
    }

    for (i5 = 0; i5 < 6; i5++) {
      b_TO2S[i5] = 0.0;
      for (i6 = 0; i6 < 6; i6++) {
        b_TO2S[i5] += c_TO2S[i5 + 6 * i6] * dv6[i6];
      }

      uSDot[i5] = c_TB2S[i5] + b_TO2S[i5];
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
    omega = b_norm(*(double (*)[3])&uSDot[0]) / kC->r;

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
    y = xNew_data[4];
    alphaDotDot = xNew_data[8];
    c_alpha[0] = alpha;
    c_alpha[1] = beta;
    c_alpha[2] = b_gamma;
    c_alpha[3] = r - 3.1415926535897931;
    c_alpha[4] = y + dt * omega;
    c_alpha[5] = alphaDot;
    c_alpha[6] = betaDot;
    c_alpha[7] = gammaDot;
    c_alpha[8] = alphaDotDot;
    c_alpha[9] = omega;
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i5 = 0; i5 < 10; i5++) {
      xNew_data[xNew_size[0] * i5] = c_alpha[i5];
    }

    xInit_size_idx_1 = xNew_size[1];
    loop_ub = xNew_size[0] * xNew_size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      xInit_data[i5] = xNew_data[i5];
    }

    y = 10.0 * (1.0 + (double)i) + 1.0;
    if (y > 10.0 * ((1.0 + (double)i) + 1.0)) {
      i5 = 0;
    } else {
      i5 = (int)y - 1;
    }

    loop_ub = xNew_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      transitionArray->data[i5 + i6] = xNew_data[xNew_size[0] * i6];
    }
  }

  xInit_size_idx_1 = 3 + xNew_size[1];
  for (i5 = 0; i5 < 3; i5++) {
    TO2S[i5] = 0.0;
  }

  loop_ub = xNew_size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    TO2S[i5 + 3] = xNew_data[xNew_size[0] * i5];
  }

  xNew_size[0] = 1;
  xNew_size[1] = xInit_size_idx_1;
  for (i5 = 0; i5 < xInit_size_idx_1; i5++) {
    xNew_data[xNew_size[0] * i5] = TO2S[i5];
  }
}

//
// File trailer for rk4.cpp
//
// [EOF]
//
