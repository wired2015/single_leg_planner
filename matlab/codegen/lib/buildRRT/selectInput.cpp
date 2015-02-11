//
// File: selectInput.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "selectInput.h"
#include "buildRRT_emxutil.h"
#include "heuristicSingleLeg.h"
#include "buildRRT_rtwutil.h"

// Function Definitions

//
// selectInput Selects the most appropriate control input.
//    A control input is selected from a set of control inputs, U. An input
//    is selected by applying each of the inputs to to state xNear, which
//    results in p candidate states, where p is the size of the input set.
//    The control input corresponding to candidate state that is closest to
//    x1 is returned as u.
// Arguments    : const emxArray_real_T *xNear
//                const double xRand_data[]
//                const int xRand_size[2]
//                const double U[10]
//                double dt
//                double Dt
//                int NODE_SIZE
//                int U_SIZE
//                const double HGAINS[3]
//                const double kinematicConst[12]
//                double ankleThreshold
//                const double jointLimits[12]
//                emxArray_real_T *xNew
//                emxArray_real_T *transitionArray
// Return Type  : void
//
void selectInput(const emxArray_real_T *xNear, const double xRand_data[], const
                 int xRand_size[2], const double U[10], double dt, double Dt,
                 int NODE_SIZE, int U_SIZE, const double HGAINS[3], const double
                 kinematicConst[12], double ankleThreshold, const double
                 jointLimits[12], emxArray_real_T *xNew, emxArray_real_T
                 *transitionArray)
{
  emxArray_real_T *candStates;
  int i1;
  int n;
  emxArray_real_T *candTransArrays;
  double beta;
  emxArray_real_T *distance;
  emxArray_real_T *U_joint;
  int i;
  emxArray_real_T *r0;
  emxArray_real_T *r1;
  emxArray_real_T *xInit;
  emxArray_real_T *b_candStates;
  emxArray_real_T *r2;
  double u[3];
  double numIterations;
  int ixstart;
  int ix;
  double k1[6];
  double b_xInit[6];
  double k2[6];
  double k3[6];
  double absxk;
  double c_xInit[6];
  double d0;
  emxArray_real_T b_xRand_data;
  double hDiff;
  double aGain;
  double v_LS[2];
  int i2;
  boolean_T exitg1;
  emxInit_real_T(&candStates, 2);

  // Initialize arrays to store the candidate new state data and the
  // distances between each candidate state and the xNear state.
  i1 = candStates->size[0] * candStates->size[1];
  candStates->size[0] = U_SIZE;
  candStates->size[1] = NODE_SIZE;
  emxEnsureCapacity((emxArray__common *)candStates, i1, (int)sizeof(double));
  n = U_SIZE * NODE_SIZE;
  for (i1 = 0; i1 < n; i1++) {
    candStates->data[i1] = 0.0;
  }

  emxInit_real_T(&candTransArrays, 2);
  beta = rt_roundd_snf(Dt / dt);
  i1 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = U_SIZE;
  candTransArrays->size[1] = (int)((beta + 1.0) * 6.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i1, (int)sizeof(double));
  n = U_SIZE * (int)((beta + 1.0) * 6.0);
  for (i1 = 0; i1 < n; i1++) {
    candTransArrays->data[i1] = 0.0;
  }

  emxInit_real_T(&distance, 2);
  i1 = distance->size[0] * distance->size[1];
  distance->size[0] = 1;
  distance->size[1] = U_SIZE;
  emxEnsureCapacity((emxArray__common *)distance, i1, (int)sizeof(double));
  for (i1 = 0; i1 < U_SIZE; i1++) {
    distance->data[i1] = 0.0;
  }

  emxInit_real_T(&U_joint, 2);
  i1 = U_joint->size[0] * U_joint->size[1];
  U_joint->size[0] = U_SIZE;
  U_joint->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)U_joint, i1, (int)sizeof(double));
  n = U_SIZE * 3;
  for (i1 = 0; i1 < n; i1++) {
    U_joint->data[i1] = 0.0;
  }

  // Transform the control inputs to joint space.
  for (i = 0; i + 1 <= U_SIZE; i++) {
    for (i1 = 0; i1 < 2; i1++) {
      U_joint->data[i + U_joint->size[0] * i1] = U[i + 5 * i1];
    }

    U_joint->data[i + (U_joint->size[0] << 1)] = ((-U[5 + i] * kinematicConst[2]
      * cos(xNear->data[4]) + xNear->data[7] * xNear->data[7] * kinematicConst[2]
      * sin(xNear->data[4])) + xNear->data[8] * xNear->data[8] * kinematicConst
      [4] * sin(kinematicConst[8] + xNear->data[5])) / (kinematicConst[4] * cos
      (kinematicConst[8] + xNear->data[5]));

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
  }

  // Increment over the control vector. Generate a candidate state for each
  // possible control input.
  i = 0;
  emxInit_real_T(&r0, 2);
  emxInit_real_T(&r1, 2);
  emxInit_real_T(&xInit, 2);
  emxInit_real_T(&b_candStates, 2);
  emxInit_real_T(&r2, 2);
  while (i + 1 <= U_SIZE) {
    // Generate a candidate state using a fourth order Runge-Kutta
    // integration technique.
    for (i1 = 0; i1 < 3; i1++) {
      u[i1] = U_joint->data[i + U_joint->size[0] * i1];
    }

    // rk4 Summary of this function goes here
    //    Detailed explanation goes here
    // rk4.m
    // author: wreid
    // date: 20150107
    numIterations = rt_roundd_snf(Dt / dt);
    i1 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)r0, i1, (int)sizeof(double));
    ixstart = xNear->size[1];
    i1 = r0->size[0] * r0->size[1];
    r0->size[1] = ixstart;
    emxEnsureCapacity((emxArray__common *)r0, i1, (int)sizeof(double));
    n = xNear->size[1];
    for (i1 = 0; i1 < n; i1++) {
      r0->data[i1] = 0.0;
    }

    if (4 > xNear->size[1] - 2) {
      i1 = 0;
      ix = -1;
    } else {
      i1 = 3;
      ix = xNear->size[1] - 3;
    }

    ixstart = xInit->size[0] * xInit->size[1];
    xInit->size[0] = 1;
    xInit->size[1] = (ix - i1) + 1;
    emxEnsureCapacity((emxArray__common *)xInit, ixstart, (int)sizeof(double));
    n = ix - i1;
    for (ix = 0; ix <= n; ix++) {
      xInit->data[xInit->size[0] * ix] = xNear->data[i1 + ix];
    }

    // xInitOrig = xInit;
    i1 = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    r1->size[1] = (int)((numIterations + 1.0) * 6.0);
    emxEnsureCapacity((emxArray__common *)r1, i1, (int)sizeof(double));
    n = (int)((numIterations + 1.0) * 6.0);
    for (i1 = 0; i1 < n; i1++) {
      r1->data[i1] = 0.0;
    }

    for (i1 = 0; i1 < 6; i1++) {
      r1->data[i1] = xInit->data[i1];
    }

    for (ixstart = 0; ixstart < (int)numIterations; ixstart++) {
      for (i1 = 0; i1 < 3; i1++) {
        k1[i1] = xInit->data[i1 + 3];
      }

      for (i1 = 0; i1 < 3; i1++) {
        k1[i1 + 3] = u[i1];
      }

      beta = dt / 2.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_xInit[i1] = xInit->data[i1] + beta * k1[i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        k2[i1] = b_xInit[3 + i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        k2[i1 + 3] = u[i1];
      }

      beta = dt / 2.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_xInit[i1] = xInit->data[i1] + beta * k2[i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        k3[i1] = b_xInit[3 + i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        k3[i1 + 3] = u[i1];
      }

      beta = dt / 2.0;
      absxk = dt / 6.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_xInit[i1] = xInit->data[i1] + beta * k3[i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        c_xInit[i1] = b_xInit[3 + i1];
      }

      for (i1 = 0; i1 < 3; i1++) {
        c_xInit[i1 + 3] = u[i1];
      }

      i1 = r0->size[0] * r0->size[1];
      r0->size[0] = 1;
      r0->size[1] = 6;
      emxEnsureCapacity((emxArray__common *)r0, i1, (int)sizeof(double));
      for (i1 = 0; i1 < 6; i1++) {
        r0->data[r0->size[0] * i1] = xInit->data[i1] + absxk * (((k1[i1] + 2.0 *
          k2[i1]) + 2.0 * k3[i1]) + c_xInit[i1]);
      }

      // Check pan angular position limits
      if ((r0->data[0] > jointLimits[1]) || (r0->data[0] < jointLimits[0])) {
        r0->data[0] = xInit->data[0];
        r0->data[3] = 0.0;
        u[0] = 0.0;
      }

      // Check inner and outer leg angular position limits
      if ((r0->data[1] > jointLimits[3]) || (r0->data[1] < jointLimits[2]) ||
          (r0->data[2] > jointLimits[5]) || (r0->data[2] < jointLimits[4])) {
        r0->data[1] = xInit->data[1];
        r0->data[2] = xInit->data[2];
        r0->data[4] = 0.0;
        r0->data[5] = 0.0;
        u[1] = 0.0;
        u[2] = 0.0;
      }

      // Check pan angular velocity limits
      if ((r0->data[3] > jointLimits[7]) || (r0->data[3] < jointLimits[6])) {
        r0->data[3] = xInit->data[3];
        u[0] = 0.0;
      }

      // Check inner and outer leg angular velocity limits
      if ((r0->data[4] > jointLimits[9]) || (r0->data[4] < jointLimits[8]) ||
          (r0->data[5] > jointLimits[11]) || (r0->data[5] < jointLimits[10])) {
        r0->data[4] = xInit->data[4];
        r0->data[5] = xInit->data[5];
        u[1] = 0.0;
        u[2] = 0.0;
      }

      i1 = xInit->size[0] * xInit->size[1];
      xInit->size[0] = 1;
      xInit->size[1] = r0->size[1];
      emxEnsureCapacity((emxArray__common *)xInit, i1, (int)sizeof(double));
      n = r0->size[0] * r0->size[1];
      for (i1 = 0; i1 < n; i1++) {
        xInit->data[i1] = r0->data[i1];
      }

      d0 = 6.0 * (1.0 + (double)ixstart) + 1.0;
      if (d0 > 6.0 * ((1.0 + (double)ixstart) + 1.0)) {
        i1 = 0;
      } else {
        i1 = (int)d0 - 1;
      }

      n = r0->size[1];
      for (ix = 0; ix < n; ix++) {
        r1->data[i1 + ix] = r0->data[r0->size[0] * ix];
      }
    }

    // xInit = [zeros(1,3) xInitOrig 0 0];
    i1 = r2->size[0] * r2->size[1];
    r2->size[0] = 1;
    r2->size[1] = 5 + r0->size[1];
    emxEnsureCapacity((emxArray__common *)r2, i1, (int)sizeof(double));
    for (i1 = 0; i1 < 3; i1++) {
      r2->data[r2->size[0] * i1] = 0.0;
    }

    n = r0->size[1];
    for (i1 = 0; i1 < n; i1++) {
      r2->data[r2->size[0] * (i1 + 3)] = r0->data[r0->size[0] * i1];
    }

    r2->data[r2->size[0] * (3 + r0->size[1])] = 0.0;
    r2->data[r2->size[0] * (4 + r0->size[1])] = 0.0;
    i1 = r0->size[0] * r0->size[1];
    r0->size[0] = 1;
    r0->size[1] = r2->size[1];
    emxEnsureCapacity((emxArray__common *)r0, i1, (int)sizeof(double));
    n = r2->size[1];
    for (i1 = 0; i1 < n; i1++) {
      r0->data[r0->size[0] * i1] = r2->data[r2->size[0] * i1];
    }

    n = r0->size[1];
    for (i1 = 0; i1 < n; i1++) {
      candStates->data[i + candStates->size[0] * i1] = r0->data[r0->size[0] * i1];
    }

    n = r1->size[1];
    for (i1 = 0; i1 < n; i1++) {
      candTransArrays->data[i + candTransArrays->size[0] * i1] = r1->data
        [r1->size[0] * i1];
    }

    // U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) 
    // Calculate the distance between the candidate state and the random
    // state.
    n = candStates->size[1];
    i1 = b_candStates->size[0] * b_candStates->size[1];
    b_candStates->size[0] = 1;
    b_candStates->size[1] = n;
    emxEnsureCapacity((emxArray__common *)b_candStates, i1, (int)sizeof(double));
    for (i1 = 0; i1 < n; i1++) {
      b_candStates->data[b_candStates->size[0] * i1] = candStates->data[i +
        candStates->size[0] * i1];
    }

    b_xRand_data.data = (double *)xRand_data;
    b_xRand_data.size = (int *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    hDiff = heuristicSingleLeg(b_candStates, &b_xRand_data, kinematicConst);

    // Apply the ankle constraint to penalize any candidate state that
    // requires a change of ankle position greater than the allowed ankle
    // movement in a single time step.
    aGain = HGAINS[2];
    if (xNear->data[0] == 1.0) {
      aGain = 0.0;
    }

    // Ss = [1000 1];
    // qWDot = 1;
    // r = 0.378/2;
    // calcPhi.m
    // function [qS,qWDot] = calcPhi(qDot,q,kinematicConst,sS,qWDot,r)
    beta = candStates->data[i + (candStates->size[0] << 2)];

    // v_XS = sin(conj(alpha))*conj(L6)*conj(alphaDot) - sin(conj(alpha))*conj(L2)*conj(alphaDot) - cos(conj(alpha))*conj(L7)*conj(gammaDot) - cos(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(beta))*sin(conj(alpha))*conj(L3)*conj(alphaDot) - cos(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - sin(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) - cos(conj(gamma))*sin(conj(alpha))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot) + sin(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot); 
    // v_YS = cos(conj(alpha))*conj(L2)*conj(alphaDot) - sin(conj(alpha))*conj(L7)*conj(gammaDot) - sin(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(alpha))*conj(L6)*conj(alphaDot) + cos(conj(alpha))*cos(conj(beta))*conj(L3)*conj(alphaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - 1.0*sin(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot); 
    // qA1 = atan(sS(2)/sS(1));
    // v_WS = [qWDot*r*sign(sS(1))*cos(qA1); qWDot*r*sign(sS(1))*sin(qA1)];
    v_LS[0] = -kinematicConst[2] * candStates->data[i + candStates->size[0] * 7]
      * sin(beta) - candStates->data[i + (candStates->size[0] << 3)] *
      kinematicConst[4] * sin(kinematicConst[8] + candStates->data[i +
      candStates->size[0] * 5]);
    v_LS[1] = candStates->data[i + candStates->size[0] * 6] * (((kinematicConst
      [1] + kinematicConst[2] * cos(beta)) + kinematicConst[3] * cos
      (kinematicConst[8])) + kinematicConst[4] * cos(kinematicConst[8] +
      candStates->data[i + candStates->size[0] * 5]));

    //  + v_WS;
    d0 = 0.0;
    beta = 2.2250738585072014E-308;
    for (ixstart = 0; ixstart < 2; ixstart++) {
      absxk = fabs(v_LS[ixstart]);
      if (absxk > beta) {
        numIterations = beta / absxk;
        d0 = 1.0 + d0 * numIterations * numIterations;
        beta = absxk;
      } else {
        numIterations = absxk / beta;
        d0 += numIterations * numIterations;
      }
    }

    d0 = beta * sqrt(d0);
    candStates->data[i + candStates->size[0] * 9] = atan(v_LS[0] / v_LS[1]);
    candStates->data[i + candStates->size[0] * 10] = d0;

    // ,Ss,qWDot,r);
    // angDiff Finds the angular difference between th1 and th2.
    beta = ((xNear->data[xNear->size[1] - 1] - candStates->data[i +
             candStates->size[0] * 9]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(beta - rt_roundd_snf(beta)) <= 2.2204460492503131E-16 * fabs(beta))
    {
      beta = 0.0;
    } else {
      beta = (beta - floor(beta)) * 6.2831853071795862;
    }

    // Calculate a distance metric that includes the heurisitc distance
    // as well as any penalty due to ankle movements.
    if (fabs(beta - 3.1415926535897931) > ankleThreshold) {
      i2 = 1;
    } else {
      i2 = 0;
    }

    distance->data[i] = (1.0 - aGain) * hDiff + aGain * (double)i2;

    // distance(i) = hDiff;
    i++;
  }

  emxFree_real_T(&r2);
  emxFree_real_T(&b_candStates);
  emxFree_real_T(&xInit);
  emxFree_real_T(&r1);
  emxFree_real_T(&r0);
  emxFree_real_T(&U_joint);
  ixstart = 1;
  n = distance->size[1];
  beta = distance->data[0];
  i = 0;
  if (distance->size[1] > 1) {
    if (rtIsNaN(distance->data[0])) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        ixstart = ix;
        if (!rtIsNaN(distance->data[ix - 1])) {
          beta = distance->data[ix - 1];
          i = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < distance->size[1]) {
      while (ixstart + 1 <= n) {
        if (distance->data[ixstart] < beta) {
          beta = distance->data[ixstart];
          i = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&distance);
  n = candStates->size[1];
  i1 = xNew->size[0] * xNew->size[1];
  xNew->size[0] = 1;
  xNew->size[1] = n;
  emxEnsureCapacity((emxArray__common *)xNew, i1, (int)sizeof(double));
  for (i1 = 0; i1 < n; i1++) {
    xNew->data[xNew->size[0] * i1] = candStates->data[i + candStates->size[0] *
      i1];
  }

  emxFree_real_T(&candStates);
  n = candTransArrays->size[1];
  i1 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = n;
  emxEnsureCapacity((emxArray__common *)transitionArray, i1, (int)sizeof(double));
  for (i1 = 0; i1 < n; i1++) {
    transitionArray->data[transitionArray->size[0] * i1] = candTransArrays->
      data[i + candTransArrays->size[0] * i1];
  }

  emxFree_real_T(&candTransArrays);

  // velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)
}

//
// File trailer for selectInput.cpp
//
// [EOF]
//
