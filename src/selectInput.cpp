//
// File: selectInput.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "selectInput.h"
#include "heuristicSingleLeg.h"
#include "trInv.h"
#include "generateTrMatrices.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// selectInput Selects the most appropriate control input.
//    A control input is selected from a set of control inputs, U. An input
//    is selected by applying each of the inputs to to state xNear, which
//    results in p candidate states, where p is the size of the input set.
//    The control input corresponding to candidate state that is closest to
//    x1 is returned as u.
// Arguments    : const double xNear_data[]
//                const int xNear_size[2]
//                const double xRand_data[]
//                const int xRand_size[2]
//                const double U[10]
//                double dt
//                double Dt
//                const struct0_T *kC
//                const double jointLimits[12]
//                const double uBDot[6]
//                int legNum
//                double xNew_data[]
//                int xNew_size[2]
//                emxArray_real_T *transitionArray
// Return Type  : void
//
void selectInput(const double xNear_data[], const int xNear_size[2], const
                 double xRand_data[], const int xRand_size[2], const double U[10],
                 double dt, double Dt, const struct0_T *kC, const double
                 jointLimits[12], const double uBDot[6], int legNum, double
                 xNew_data[], int xNew_size[2], emxArray_real_T *transitionArray)
{
  double candStates_data[55];
  emxArray_real_T *candTransArrays;
  double scale;
  int i4;
  int ixstart;
  double UJoint_data[15];
  int i;
  emxArray_real_T *r0;
  double distance_data[5];
  double vS[3];
  double numIterations;
  int ix;
  double tmp_data[16];
  double b_xNear_data[11];
  double xInit_data[11];
  double AdB2S[6];
  double k1[6];
  double y;
  double k2[6];
  double k3[6];
  double AdO2S[6];
  int itmp;
  double TO2S[16];
  emxArray_real_T b_xRand_data;
  double hDiff;
  double aGain;
  double candStates[4];
  double TB2S[16];
  double TP2S[16];
  double TS2R[16];
  double TR2Q[16];
  double TQ2O[16];
  double TO2J[16];
  double TJ2I[16];
  double TI2P[16];
  double TI2S[16];
  double b_TI2S[16];
  double c_TI2S[16];
  double d_TI2S[16];
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
  double dv0[9];
  double dv1[9];
  double b_AdB2S[36];
  double dv2[9];
  double AdP2S[36];
  double dv3[9];
  double AdI2S[36];
  double dv4[9];
  double b_AdO2S[36];
  double uPDot[6];
  double uIDot[6];
  double c_AdB2S[6];
  double b_AdP2S[6];
  double uODot[6];
  double uSDot[6];
  double t;
  int i5;
  boolean_T exitg1;

  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  memset(&candStates_data[0], 0, 55U * sizeof(double));
  emxInit_real_T(&candTransArrays, 2);
  scale = rt_roundd_snf(Dt / dt);
  i4 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  candTransArrays->size[1] = (int)((scale + 1.0) * 6.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i4, (int)sizeof(double));
  ixstart = 5 * (int)((scale + 1.0) * 6.0);
  for (i4 = 0; i4 < ixstart; i4++) {
    candTransArrays->data[i4] = 0.0;
  }

  // Transform the control inputs to joint space.
  for (i = 0; i < 5; i++) {
    // gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); 
    // getConstrainedGammaDotDot Summary of this function goes here
    //    Detailed explanation goes here
    // [~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); 
    for (i4 = 0; i4 < 2; i4++) {
      UJoint_data[i + 5 * i4] = U[i + 5 * i4];
    }

    UJoint_data[i + 10] = ((-U[5 + i] * kC->l3 * cos(xNear_data[4]) +
      xNear_data[7] * xNear_data[7] * kC->l3 * sin(xNear_data[4])) + xNear_data
      [8] * xNear_data[8] * kC->l5 * sin(kC->zeta + xNear_data[5])) / (kC->l5 *
      cos(kC->zeta + xNear_data[5]));
  }

  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  emxInit_real_T(&r0, 2);
  for (i = 0; i < 5; i++) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i4 = 0; i4 < 3; i4++) {
      vS[i4] = UJoint_data[i + 5 * i4];
    }

    // rk4 Summary of this function goes here
    //    Detailed explanation goes here
    // rk4.m
    // author: wreid
    // date: 20150107
    numIterations = rt_roundd_snf(Dt / dt);
    ix = 11;
    memset(&tmp_data[0], 0, 11U * sizeof(double));
    for (i4 = 0; i4 < 11; i4++) {
      b_xNear_data[i4] = xNear_data[xNear_size[0] * i4];
    }

    for (i4 = 0; i4 < 6; i4++) {
      xInit_data[i4] = b_xNear_data[3 + i4];
    }

    // xInitOrig = xInit;
    i4 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = (int)((numIterations + 1.0) * 6.0);
    emxEnsureCapacity((emxArray__common *)r0, i4, (int)sizeof(double));
    ixstart = (int)((numIterations + 1.0) * 6.0);
    for (i4 = 0; i4 < ixstart; i4++) {
      r0->data[i4] = 0.0;
    }

    for (i4 = 0; i4 < 6; i4++) {
      r0->data[i4] = xInit_data[i4];
    }

    for (ixstart = 0; ixstart < (int)numIterations; ixstart++) {
      for (i4 = 0; i4 < 6; i4++) {
        AdB2S[i4] = xInit_data[i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k1[i4] = AdB2S[3 + i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k1[i4 + 3] = vS[i4];
      }

      y = dt / 2.0;
      for (i4 = 0; i4 < 6; i4++) {
        AdB2S[i4] = xInit_data[i4] + y * k1[i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k2[i4] = AdB2S[3 + i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k2[i4 + 3] = vS[i4];
      }

      y = dt / 2.0;
      for (i4 = 0; i4 < 6; i4++) {
        AdB2S[i4] = xInit_data[i4] + y * k2[i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k3[i4] = AdB2S[3 + i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        k3[i4 + 3] = vS[i4];
      }

      y = dt / 2.0;
      scale = dt / 6.0;
      for (i4 = 0; i4 < 6; i4++) {
        AdB2S[i4] = xInit_data[i4] + y * k3[i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        AdO2S[i4] = AdB2S[3 + i4];
      }

      for (i4 = 0; i4 < 3; i4++) {
        AdO2S[i4 + 3] = vS[i4];
      }

      ix = 6;
      for (i4 = 0; i4 < 6; i4++) {
        tmp_data[i4] = xInit_data[i4] + scale * (((k1[i4] + 2.0 * k2[i4]) + 2.0 *
          k3[i4]) + AdO2S[i4]);
      }

      // Check pan angular position limits
      if ((tmp_data[0] > jointLimits[1]) || (tmp_data[0] < jointLimits[0])) {
        tmp_data[0] = xInit_data[0];
        tmp_data[3] = 0.0;
        vS[0] = 0.0;
      }

      // Check inner and outer leg angular position limits
      if ((tmp_data[1] > jointLimits[3]) || (tmp_data[1] < jointLimits[2]) ||
          (tmp_data[2] > jointLimits[5]) || (tmp_data[2] < jointLimits[4])) {
        tmp_data[1] = xInit_data[1];
        tmp_data[2] = xInit_data[2];
        tmp_data[4] = 0.0;
        tmp_data[5] = 0.0;
        vS[1] = 0.0;
        vS[2] = 0.0;
      }

      // Check pan angular velocity limits
      if ((tmp_data[3] > jointLimits[7]) || (tmp_data[3] < jointLimits[6])) {
        tmp_data[3] = xInit_data[3];
        vS[0] = 0.0;
      }

      // Check inner and outer leg angular velocity limits
      if ((tmp_data[4] > jointLimits[9]) || (tmp_data[4] < jointLimits[8]) ||
          (tmp_data[5] > jointLimits[11]) || (tmp_data[5] < jointLimits[10])) {
        tmp_data[4] = xInit_data[4];
        tmp_data[5] = xInit_data[5];
        vS[1] = 0.0;
        vS[2] = 0.0;
      }

      for (i4 = 0; i4 < 6; i4++) {
        xInit_data[i4] = tmp_data[i4];
      }

      scale = 6.0 * (1.0 + (double)ixstart) + 1.0;
      if (scale > 6.0 * ((1.0 + (double)ixstart) + 1.0)) {
        i4 = 0;
      } else {
        i4 = (int)scale - 1;
      }

      for (itmp = 0; itmp < 6; itmp++) {
        r0->data[i4 + itmp] = tmp_data[itmp];
      }
    }

    // xInit = [zeros(1,3) xInitOrig 0 0];
    for (i4 = 0; i4 < 3; i4++) {
      TO2S[i4] = 0.0;
    }

    for (i4 = 0; i4 < ix; i4++) {
      TO2S[i4 + 3] = tmp_data[i4];
    }

    TO2S[3 + ix] = 0.0;
    TO2S[4 + ix] = 0.0;
    ixstart = 5 + ix;
    for (i4 = 0; i4 < ixstart; i4++) {
      tmp_data[i4] = TO2S[i4];
    }

    ixstart = 5 + ix;
    for (i4 = 0; i4 < ixstart; i4++) {
      candStates_data[i + 5 * i4] = tmp_data[i4];
    }

    ixstart = r0->size[1];
    for (i4 = 0; i4 < ixstart; i4++) {
      candTransArrays->data[i + candTransArrays->size[0] * i4] = r0->data
        [r0->size[0] * i4];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); 
    // Calculate the distance between the candidate state and the random
    // state.
    for (i4 = 0; i4 < 11; i4++) {
      b_xNear_data[i4] = candStates_data[i + 5 * i4];
    }

    b_xRand_data.data = (double *)xRand_data;
    b_xRand_data.size = (int *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    hDiff = heuristicSingleLeg(b_xNear_data, &b_xRand_data, kC->l2, kC->l3,
      kC->l4, kC->l5, kC->l7, kC->zeta);

    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    aGain = 0.5;
    if (xNear_data[0] == 1.0) {
      aGain = 0.0;
    }

    // Homogeneous transformation matrices.
    for (i4 = 0; i4 < 3; i4++) {
      candStates[i4] = candStates_data[i + 5 * (3 + i4)];
    }

    candStates[3] = 0.0;
    generateTrMatrices(candStates, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                       kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, kC->B2PXOffset,
                       kC->B2PZOffset, kC->legAngleOffset, legNum, TO2S, TI2S,
                       TI2P, TJ2I, TO2J, TQ2O, TR2Q, TS2R, TP2S, TB2S);
    for (i4 = 0; i4 < 4; i4++) {
      for (itmp = 0; itmp < 4; itmp++) {
        TO2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          TO2S[i4 + (itmp << 2)] += TI2S[i4 + (ixstart << 2)] * TI2P[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        TP2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          TP2S[i4 + (itmp << 2)] += TO2S[i4 + (ixstart << 2)] * TJ2I[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        TB2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          TB2S[i4 + (itmp << 2)] += TP2S[i4 + (ixstart << 2)] * TO2J[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        c_TI2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          c_TI2S[i4 + (itmp << 2)] += TB2S[i4 + (ixstart << 2)] * TQ2O[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        d_TI2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          d_TI2S[i4 + (itmp << 2)] += c_TI2S[i4 + (ixstart << 2)] * TR2Q[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        b_TI2S[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          b_TI2S[i4 + (itmp << 2)] += d_TI2S[i4 + (ixstart << 2)] * TS2R[ixstart
            + (itmp << 2)];
        }

        b_TI2P[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          b_TI2P[i4 + (itmp << 2)] += TI2P[i4 + (ixstart << 2)] * TJ2I[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        c_TI2P[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          c_TI2P[i4 + (itmp << 2)] += b_TI2P[i4 + (ixstart << 2)] * TO2J[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        d_TI2P[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          d_TI2P[i4 + (itmp << 2)] += c_TI2P[i4 + (ixstart << 2)] * TQ2O[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        e_TI2P[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          e_TI2P[i4 + (itmp << 2)] += d_TI2P[i4 + (ixstart << 2)] * TR2Q[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        f_TI2P[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          f_TI2P[i4 + (itmp << 2)] += e_TI2P[i4 + (ixstart << 2)] * TS2R[ixstart
            + (itmp << 2)];
        }

        b_TJ2I[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          b_TJ2I[i4 + (itmp << 2)] += TJ2I[i4 + (ixstart << 2)] * TO2J[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        c_TJ2I[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          c_TJ2I[i4 + (itmp << 2)] += b_TJ2I[i4 + (ixstart << 2)] * TQ2O[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        d_TJ2I[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          d_TJ2I[i4 + (itmp << 2)] += c_TJ2I[i4 + (ixstart << 2)] * TR2Q[ixstart
            + (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        e_TJ2I[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          e_TJ2I[i4 + (itmp << 2)] += d_TJ2I[i4 + (ixstart << 2)] * TS2R[ixstart
            + (itmp << 2)];
        }

        b_TQ2O[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          b_TQ2O[i4 + (itmp << 2)] += TQ2O[i4 + (ixstart << 2)] * TR2Q[ixstart +
            (itmp << 2)];
        }
      }

      for (itmp = 0; itmp < 4; itmp++) {
        c_TQ2O[i4 + (itmp << 2)] = 0.0;
        for (ixstart = 0; ixstart < 4; ixstart++) {
          c_TQ2O[i4 + (itmp << 2)] += b_TQ2O[i4 + (ixstart << 2)] * TS2R[ixstart
            + (itmp << 2)];
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
    dv0[0] = 0.0;
    dv0[3] = -TB2S[14];
    dv0[6] = TB2S[13];
    dv0[1] = TB2S[14];
    dv0[4] = 0.0;
    dv0[7] = -TB2S[12];
    dv0[2] = -TB2S[13];
    dv0[5] = TB2S[12];
    dv0[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        dv1[i4 + 3 * itmp] = 0.0;
        for (ixstart = 0; ixstart < 3; ixstart++) {
          dv1[i4 + 3 * itmp] += dv0[i4 + 3 * ixstart] * TB2S[ixstart + (itmp <<
            2)];
        }

        b_AdB2S[itmp + 6 * i4] = TB2S[itmp + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdB2S[itmp + 6 * (i4 + 3)] = dv1[itmp + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdB2S[(itmp + 6 * i4) + 3] = 0.0;
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
    dv2[0] = 0.0;
    dv2[3] = -TP2S[14];
    dv2[6] = TP2S[13];
    dv2[1] = TP2S[14];
    dv2[4] = 0.0;
    dv2[7] = -TP2S[12];
    dv2[2] = -TP2S[13];
    dv2[5] = TP2S[12];
    dv2[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdB2S[(itmp + 6 * (i4 + 3)) + 3] = TB2S[itmp + (i4 << 2)];
        dv1[i4 + 3 * itmp] = 0.0;
        for (ixstart = 0; ixstart < 3; ixstart++) {
          dv1[i4 + 3 * itmp] += dv2[i4 + 3 * ixstart] * TP2S[ixstart + (itmp <<
            2)];
        }

        AdP2S[itmp + 6 * i4] = TP2S[itmp + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdP2S[itmp + 6 * (i4 + 3)] = dv1[itmp + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdP2S[(itmp + 6 * i4) + 3] = 0.0;
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
    dv3[3] = -TI2S[14];
    dv3[6] = TI2S[13];
    dv3[1] = TI2S[14];
    dv3[4] = 0.0;
    dv3[7] = -TI2S[12];
    dv3[2] = -TI2S[13];
    dv3[5] = TI2S[12];
    dv3[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdP2S[(itmp + 6 * (i4 + 3)) + 3] = TP2S[itmp + (i4 << 2)];
        dv1[i4 + 3 * itmp] = 0.0;
        for (ixstart = 0; ixstart < 3; ixstart++) {
          dv1[i4 + 3 * itmp] += dv3[i4 + 3 * ixstart] * TI2S[ixstart + (itmp <<
            2)];
        }

        AdI2S[itmp + 6 * i4] = TI2S[itmp + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdI2S[itmp + 6 * (i4 + 3)] = dv1[itmp + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdI2S[(itmp + 6 * i4) + 3] = 0.0;
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
    dv4[3] = -TO2S[14];
    dv4[6] = TO2S[13];
    dv4[1] = TO2S[14];
    dv4[4] = 0.0;
    dv4[7] = -TO2S[12];
    dv4[2] = -TO2S[13];
    dv4[5] = TO2S[12];
    dv4[8] = 0.0;
    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        AdI2S[(itmp + 6 * (i4 + 3)) + 3] = TI2S[itmp + (i4 << 2)];
        dv1[i4 + 3 * itmp] = 0.0;
        for (ixstart = 0; ixstart < 3; ixstart++) {
          dv1[i4 + 3 * itmp] += dv4[i4 + 3 * ixstart] * TO2S[ixstart + (itmp <<
            2)];
        }

        b_AdO2S[itmp + 6 * i4] = TO2S[itmp + (i4 << 2)];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdO2S[itmp + 6 * (i4 + 3)] = dv1[itmp + 3 * i4];
      }
    }

    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdO2S[(itmp + 6 * i4) + 3] = 0.0;
      }
    }

    // Pan joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      for (itmp = 0; itmp < 3; itmp++) {
        b_AdO2S[(itmp + 6 * (i4 + 3)) + 3] = TO2S[itmp + (i4 << 2)];
      }

      AdB2S[i4] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 30];
    for (i4 = 0; i4 < 6; i4++) {
      uPDot[i4] = AdB2S[i4];
    }

    // Beta joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      AdB2S[i4] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 35];
    for (i4 = 0; i4 < 6; i4++) {
      uIDot[i4] = AdB2S[i4];
    }

    // Gamma joint rate
    // [rad/s]
    // [m/s]
    // [rad/s]
    for (i4 = 0; i4 < 3; i4++) {
      AdB2S[i4] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 40];

    // Velocity vector for the ankle frame.
    for (i4 = 0; i4 < 6; i4++) {
      uODot[i4] = AdB2S[i4];
      c_AdB2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        c_AdB2S[i4] += b_AdB2S[i4 + 6 * itmp] * uBDot[itmp];
      }

      b_AdP2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        b_AdP2S[i4] += AdP2S[i4 + 6 * itmp] * uPDot[itmp];
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      scale = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        scale += AdI2S[i4 + 6 * itmp] * uIDot[itmp];
      }

      AdB2S[i4] = (c_AdB2S[i4] + b_AdP2S[i4]) + scale;
    }

    for (i4 = 0; i4 < 6; i4++) {
      AdO2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        AdO2S[i4] += b_AdO2S[i4 + 6 * itmp] * uODot[itmp];
      }

      uSDot[i4] = AdB2S[i4] + AdO2S[i4];
      c_AdB2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        c_AdB2S[i4] += b_AdB2S[i4 + 6 * itmp] * uBDot[itmp];
      }

      b_AdP2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        b_AdP2S[i4] += AdP2S[i4 + 6 * itmp] * uPDot[itmp];
      }
    }

    for (i4 = 0; i4 < 6; i4++) {
      scale = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        scale += AdI2S[i4 + 6 * itmp] * uIDot[itmp];
      }

      AdB2S[i4] = (c_AdB2S[i4] + b_AdP2S[i4]) + scale;
    }

    for (i4 = 0; i4 < 6; i4++) {
      AdO2S[i4] = 0.0;
      for (itmp = 0; itmp < 6; itmp++) {
        AdO2S[i4] += b_AdO2S[i4 + 6 * itmp] * uODot[itmp];
      }

      c_AdB2S[i4] = AdB2S[i4] + AdO2S[i4];
    }

    for (i4 = 0; i4 < 3; i4++) {
      vS[i4] = c_AdB2S[i4];
    }

    // [m/s]
    // [rad/s]
    // Calculate the required phi joint angle and the required wheel speed,
    // omega.
    // [rad]
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      numIterations = fabs(vS[ixstart]);
      if (numIterations > scale) {
        t = scale / numIterations;
        y = 1.0 + y * t * t;
        scale = numIterations;
      } else {
        t = numIterations / scale;
        y += t * t;
      }
    }

    y = scale * sqrt(y);

    // [rad/s]
    candStates_data[i + 45] = rt_atan2d_snf(uSDot[1], uSDot[0]);
    candStates_data[i + 50] = y / kC->r;

    // angDiff Finds the angular difference between th1 and th2.
    scale = ((xNear_data[10] - candStates_data[i + 45]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(scale - rt_roundd_snf(scale)) <= 2.2204460492503131E-16 * fabs
        (scale)) {
      scale = 0.0;
    } else {
      scale = (scale - floor(scale)) * 6.2831853071795862;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    if (fabs(scale - 3.1415926535897931) > 0.39269908169872414) {
      i5 = 1;
    } else {
      i5 = 0;
    }

    distance_data[i] = (1.0 - aGain) * hDiff + aGain * (double)i5;

    // distance(i) = hDiff;
  }

  emxFree_real_T(&r0);
  ixstart = 1;
  scale = distance_data[0];
  itmp = 0;
  if (rtIsNaN(distance_data[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 <= 5)) {
      ixstart = ix + 1;
      if (!rtIsNaN(distance_data[ix])) {
        scale = distance_data[ix];
        itmp = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < scale) {
        scale = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 11;
  for (i4 = 0; i4 < 11; i4++) {
    xNew_data[xNew_size[0] * i4] = candStates_data[itmp + 5 * i4];
  }

  ixstart = candTransArrays->size[1];
  i4 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ixstart;
  emxEnsureCapacity((emxArray__common *)transitionArray, i4, (int)sizeof(double));
  for (i4 = 0; i4 < ixstart; i4++) {
    transitionArray->data[transitionArray->size[0] * i4] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i4];
  }

  emxFree_real_T(&candTransArrays);

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
}

//
// File trailer for selectInput.cpp
//
// [EOF]
//
