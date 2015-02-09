//
// File: selectInput.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 09-Feb-2015 13:36:11
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "selectInput.h"
#include "heuristicSingleLeg.h"
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
//                const double kinematicConst[15]
//                const double jointLimits[12]
//                double xNew_data[]
//                int xNew_size[2]
//                emxArray_real_T *transitionArray
// Return Type  : void
//
void selectInput(const double xNear_data[], const int xNear_size[2], const
                 double xRand_data[], const int xRand_size[2], const double U[10],
                 double dt, double Dt, const double kinematicConst[15], const
                 double jointLimits[12], double xNew_data[], int xNew_size[2],
                 emxArray_real_T *transitionArray)
{
  double candStates_data[55];
  emxArray_real_T *candTransArrays;
  double scale;
  int i3;
  int ixstart;
  double U_joint_data[15];
  int i;
  emxArray_real_T *r0;
  double distance_data[5];
  double u[3];
  double numIterations;
  int itmp;
  double tmp_data[16];
  double b_xNear_data[11];
  double xInit_data[11];
  double b_xInit_data[6];
  double k1[6];
  double k2[6];
  double k3[6];
  double absxk;
  double xInit[6];
  double d0;
  int ix;
  double b_tmp_data[16];
  emxArray_real_T b_xRand_data;
  double hDiff;
  double aGain;
  double v_LS[2];
  int i4;
  boolean_T exitg1;

  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  memset(&candStates_data[0], 0, 55U * sizeof(double));
  emxInit_real_T(&candTransArrays, 2);
  scale = rt_roundd_snf(Dt / dt);
  i3 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  candTransArrays->size[1] = (int)((scale + 1.0) * 6.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i3, (int)sizeof(double));
  ixstart = 5 * (int)((scale + 1.0) * 6.0);
  for (i3 = 0; i3 < ixstart; i3++) {
    candTransArrays->data[i3] = 0.0;
  }

  // Transform the control inputs to joint space.
  for (i = 0; i < 5; i++) {
    for (i3 = 0; i3 < 2; i3++) {
      U_joint_data[i + 5 * i3] = U[i + 5 * i3];
    }

    U_joint_data[i + 10] = ((-U[5 + i] * kinematicConst[2] * cos(xNear_data[4])
      + xNear_data[7] * xNear_data[7] * kinematicConst[2] * sin(xNear_data[4]))
      + xNear_data[8] * xNear_data[8] * kinematicConst[4] * sin(kinematicConst[8]
      + xNear_data[5])) / (kinematicConst[4] * cos(kinematicConst[8] +
      xNear_data[5]));

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
  }

  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  emxInit_real_T(&r0, 2);
  for (i = 0; i < 5; i++) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i3 = 0; i3 < 3; i3++) {
      u[i3] = U_joint_data[i + 5 * i3];
    }

    // rk4 Summary of this function goes here
    //    Detailed explanation goes here
    // rk4.m
    // author: wreid
    // date: 20150107
    numIterations = rt_roundd_snf(Dt / dt);
    itmp = 11;
    memset(&tmp_data[0], 0, 11U * sizeof(double));
    for (i3 = 0; i3 < 11; i3++) {
      b_xNear_data[i3] = xNear_data[xNear_size[0] * i3];
    }

    for (i3 = 0; i3 < 6; i3++) {
      xInit_data[i3] = b_xNear_data[3 + i3];
    }

    // xInitOrig = xInit;
    i3 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = (int)((numIterations + 1.0) * 6.0);
    emxEnsureCapacity((emxArray__common *)r0, i3, (int)sizeof(double));
    ixstart = (int)((numIterations + 1.0) * 6.0);
    for (i3 = 0; i3 < ixstart; i3++) {
      r0->data[i3] = 0.0;
    }

    for (i3 = 0; i3 < 6; i3++) {
      r0->data[i3] = xInit_data[i3];
    }

    for (ixstart = 0; ixstart < (int)numIterations; ixstart++) {
      for (i3 = 0; i3 < 6; i3++) {
        b_xInit_data[i3] = xInit_data[i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k1[i3] = b_xInit_data[3 + i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k1[i3 + 3] = u[i3];
      }

      scale = dt / 2.0;
      for (i3 = 0; i3 < 6; i3++) {
        b_xInit_data[i3] = xInit_data[i3] + scale * k1[i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k2[i3] = b_xInit_data[3 + i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k2[i3 + 3] = u[i3];
      }

      scale = dt / 2.0;
      for (i3 = 0; i3 < 6; i3++) {
        b_xInit_data[i3] = xInit_data[i3] + scale * k2[i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k3[i3] = b_xInit_data[3 + i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        k3[i3 + 3] = u[i3];
      }

      scale = dt / 2.0;
      absxk = dt / 6.0;
      for (i3 = 0; i3 < 6; i3++) {
        b_xInit_data[i3] = xInit_data[i3] + scale * k3[i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        xInit[i3] = b_xInit_data[3 + i3];
      }

      for (i3 = 0; i3 < 3; i3++) {
        xInit[i3 + 3] = u[i3];
      }

      itmp = 6;
      for (i3 = 0; i3 < 6; i3++) {
        tmp_data[i3] = xInit_data[i3] + absxk * (((k1[i3] + 2.0 * k2[i3]) + 2.0 *
          k3[i3]) + xInit[i3]);
      }

      // Check pan angular position limits
      if ((tmp_data[0] > jointLimits[1]) || (tmp_data[0] < jointLimits[0])) {
        tmp_data[0] = xInit_data[0];
        tmp_data[3] = 0.0;
        u[0] = 0.0;
      }

      // Check inner and outer leg angular position limits
      if ((tmp_data[1] > jointLimits[3]) || (tmp_data[1] < jointLimits[2]) ||
          (tmp_data[2] > jointLimits[5]) || (tmp_data[2] < jointLimits[4])) {
        tmp_data[1] = xInit_data[1];
        tmp_data[2] = xInit_data[2];
        tmp_data[4] = 0.0;
        tmp_data[5] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
      }

      // Check pan angular velocity limits
      if ((tmp_data[3] > jointLimits[7]) || (tmp_data[3] < jointLimits[6])) {
        tmp_data[3] = xInit_data[3];
        u[0] = 0.0;
      }

      // Check inner and outer leg angular velocity limits
      if ((tmp_data[4] > jointLimits[9]) || (tmp_data[4] < jointLimits[8]) ||
          (tmp_data[5] > jointLimits[11]) || (tmp_data[5] < jointLimits[10])) {
        tmp_data[4] = xInit_data[4];
        tmp_data[5] = xInit_data[5];
        u[1] = 0.0;
        u[2] = 0.0;
      }

      for (i3 = 0; i3 < 6; i3++) {
        xInit_data[i3] = tmp_data[i3];
      }

      d0 = 6.0 * (1.0 + (double)ixstart) + 1.0;
      if (d0 > 6.0 * ((1.0 + (double)ixstart) + 1.0)) {
        i3 = 0;
      } else {
        i3 = (int)d0 - 1;
      }

      for (ix = 0; ix < 6; ix++) {
        r0->data[i3 + ix] = tmp_data[ix];
      }
    }

    // xInit = [zeros(1,3) xInitOrig 0 0];
    for (i3 = 0; i3 < 3; i3++) {
      b_tmp_data[i3] = 0.0;
    }

    for (i3 = 0; i3 < itmp; i3++) {
      b_tmp_data[i3 + 3] = tmp_data[i3];
    }

    b_tmp_data[3 + itmp] = 0.0;
    b_tmp_data[4 + itmp] = 0.0;
    ixstart = 5 + itmp;
    for (i3 = 0; i3 < ixstart; i3++) {
      tmp_data[i3] = b_tmp_data[i3];
    }

    ixstart = 5 + itmp;
    for (i3 = 0; i3 < ixstart; i3++) {
      candStates_data[i + 5 * i3] = tmp_data[i3];
    }

    ixstart = r0->size[1];
    for (i3 = 0; i3 < ixstart; i3++) {
      candTransArrays->data[i + candTransArrays->size[0] * i3] = r0->data
        [r0->size[0] * i3];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // Calculate the distance between the candidate state and the random
    // state.
    for (i3 = 0; i3 < 11; i3++) {
      b_xNear_data[i3] = candStates_data[i + 5 * i3];
    }

    b_xRand_data.data = (double *)xRand_data;
    b_xRand_data.size = (int *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    hDiff = heuristicSingleLeg(b_xNear_data, &b_xRand_data, kinematicConst);

    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    aGain = 0.5;
    if (xNear_data[0] == 1.0) {
      aGain = 0.0;
    }

    // Ss = [1000 1];
    // qWDot = 1;
    // r = 0.378/2;
    // calcPhi.m
    // function [qS,qWDot] = calcPhi(qDot,q,kinematicConst,sS,qWDot,r)
    // v_XS = sin(conj(alpha))*conj(L6)*conj(alphaDot) - sin(conj(alpha))*conj(L2)*conj(alphaDot) - cos(conj(alpha))*conj(L7)*conj(gammaDot) - cos(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(beta))*sin(conj(alpha))*conj(L3)*conj(alphaDot) - cos(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - sin(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) - cos(conj(gamma))*sin(conj(alpha))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot) + sin(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot); 
    // v_YS = cos(conj(alpha))*conj(L2)*conj(alphaDot) - sin(conj(alpha))*conj(L7)*conj(gammaDot) - sin(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(alpha))*conj(L6)*conj(alphaDot) + cos(conj(alpha))*cos(conj(beta))*conj(L3)*conj(alphaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - 1.0*sin(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot); 
    // qA1 = atan(sS(2)/sS(1));
    // v_WS = [qWDot*r*sign(sS(1))*cos(qA1); qWDot*r*sign(sS(1))*sin(qA1)];
    v_LS[0] = -kinematicConst[2] * candStates_data[i + 35] * sin
      (candStates_data[i + 20]) - candStates_data[i + 40] * kinematicConst[4] *
      sin(kinematicConst[8] + candStates_data[i + 25]);
    v_LS[1] = candStates_data[i + 30] * (((kinematicConst[1] + kinematicConst[2]
      * cos(candStates_data[i + 20])) + kinematicConst[3] * cos(kinematicConst[8]))
      + kinematicConst[4] * cos(kinematicConst[8] + candStates_data[i + 25]));

    //  + v_WS;
    d0 = 0.0;
    scale = 2.2250738585072014E-308;
    for (ixstart = 0; ixstart < 2; ixstart++) {
      absxk = fabs(v_LS[ixstart]);
      if (absxk > scale) {
        numIterations = scale / absxk;
        d0 = 1.0 + d0 * numIterations * numIterations;
        scale = absxk;
      } else {
        numIterations = absxk / scale;
        d0 += numIterations * numIterations;
      }
    }

    d0 = scale * sqrt(d0);
    candStates_data[i + 45] = atan(v_LS[0] / v_LS[1]);
    candStates_data[i + 50] = d0;

    // ,Ss,qWDot,r);
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
      i4 = 1;
    } else {
      i4 = 0;
    }

    distance_data[i] = (1.0 - aGain) * hDiff + aGain * (double)i4;

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
  for (i3 = 0; i3 < 11; i3++) {
    xNew_data[xNew_size[0] * i3] = candStates_data[itmp + 5 * i3];
  }

  ixstart = candTransArrays->size[1];
  i3 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ixstart;
  emxEnsureCapacity((emxArray__common *)transitionArray, i3, (int)sizeof(double));
  for (i3 = 0; i3 < ixstart; i3++) {
    transitionArray->data[transitionArray->size[0] * i3] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i3];
  }

  emxFree_real_T(&candTransArrays);

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
}

//
// File trailer for selectInput.cpp
//
// [EOF]
//
