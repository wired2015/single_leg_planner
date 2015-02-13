//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 15:29:21
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxutil.h"
#include "flipud.h"
#include "buildRRT.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Variable Definitions
static double HGAINS[3];

// Function Declarations
static boolean_T validState(const double n[6], const double jointLimits[12]);

// Function Definitions

//
// Arguments    : const double n[6]
//                const double jointLimits[12]
// Return Type  : boolean_T
//
static boolean_T validState(const double n[6], const double jointLimits[12])
{
  boolean_T valid;
  if ((n[0] < jointLimits[0]) || (n[0] > jointLimits[1]) || (n[1] < jointLimits
       [2]) || (n[1] > jointLimits[3]) || (n[2] < jointLimits[4]) || (n[2] >
       jointLimits[5]) || (n[3] < jointLimits[6]) || (n[3] > jointLimits[7]) ||
      (n[4] < jointLimits[8]) || (n[4] > jointLimits[9]) || (n[5] < jointLimits
       [10]) || (n[5] > jointLimits[11])) {
    valid = false;
  } else {
    valid = true;
  }

  return valid;
}

//
// Arguments    : const double nInitCartesianB[6]
//                const double nGoalCartesianB[6]
//                const double jointLimits[12]
//                double bodyHeight
//                const double U[10]
//                double dt
//                double Dt
//                const struct0_T *kC
//                double threshold
//                int legNum
//                emxArray_real_T *T
//                emxArray_real_T *pathC
//                emxArray_real_T *pathJ
//                boolean_T *success
// Return Type  : void
//
void buildRRTWrapper(const double nInitCartesianB[6], const double
                     nGoalCartesianB[6], const double jointLimits[12], double
                     bodyHeight, const double U[10], double dt, double Dt, const
                     struct0_T *kC, double, int legNum, emxArray_real_T *T,
                     emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T
                     *success)
{
  double TP2B[16];
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  double b_TP2B[9];
  int i1;
  double c_TP2B[3];
  double TB2P[16];
  double b_TB2P[3];
  double d0;
  double qInit[3];
  double qGoal[3];
  double nInitJoint[6];
  double qDotInit[3];
  double nGoalJoint[6];
  emxArray_real_T *b_T;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_pathJ;
  emxArray_real_T *c_pathC;
  emxArray_real_T *c_pathJ;
  double dv0[11];
  double dv1[11];
  int loop_ub;
  double alpha;
  unsigned int count;
  double time;
  int j;
  double uP[3];
  double betaDot;
  double gammaDot;
  double alphaDot[3];

  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  // GETPANHEIGHT Summary of this function goes here
  //    Detailed explanation goes here
  // Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame 
  // to the pan coordinate frame.
  // TRP2B Calculates the homogeneous transformation matrix between the body
  // and pan coordinate frames.
  TP2B[0] = cos(kC->legAngleOffset[legNum - 1]);
  TP2B[4] = -sin(kC->legAngleOffset[legNum - 1]);
  TP2B[8] = sin(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[12] = kC->B2PXOffset * cos(kC->legAngleOffset[legNum - 1]);
  TP2B[1] = sin(kC->legAngleOffset[legNum - 1]);
  TP2B[5] = cos(kC->legAngleOffset[legNum - 1]);
  TP2B[9] = -cos(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[13] = kC->B2PXOffset * sin(kC->legAngleOffset[legNum - 1]);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = kC->B2PZOffset;
  for (i0 = 0; i0 < 4; i0++) {
    TP2B[3 + (i0 << 2)] = iv0[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_TP2B[i1 + 3 * i0] = -TP2B[i0 + (i1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * i1] * TP2B[12 + i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      TB2P[i1 + (i0 << 2)] = TP2B[i0 + (i1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    TB2P[12 + i0] = c_TP2B[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    TB2P[3 + (i0 << 2)] = iv0[i0];
  }

  // inv(TP2B);%
  // Transform the Cartesian goal and final positions in the pan coordinate
  // frame to the joint space.
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += TB2P[i0 + (i1 << 2)] * nInitCartesianB[i1];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[i1];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += TB2P[i0 + (i1 << 2)] * nInitCartesianB[3 + i1];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nInitJoint[i0] = qInit[i0];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                qDotInit);
  for (i0 = 0; i0 < 3; i0++) {
    nInitJoint[i0 + 3] = qDotInit[i0];
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[3 + i1];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nGoalJoint[i0] = qGoal[i0];
  }

  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                c_TP2B);
  for (i0 = 0; i0 < 3; i0++) {
    nGoalJoint[i0 + 3] = c_TP2B[i0];
  }

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  emxInit_real_T(&b_T, 2);
  emxInit_real_T(&b_pathC, 2);
  emxInit_real_T(&b_pathJ, 2);
  emxInit_real_T(&c_pathC, 2);
  emxInit_real_T(&c_pathJ, 2);
  if (validState(nInitJoint, jointLimits) && validState(nGoalJoint, jointLimits))
  {
    *success = true;

    // Run buildRRT.
    dv0[0] = 0.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      dv0[i0 + 3] = nInitJoint[i0];
    }

    dv0[9] = 0.0;
    dv0[10] = 0.0;
    dv1[0] = 0.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      dv1[i0 + 3] = nGoalJoint[i0];
    }

    dv1[9] = 0.0;
    dv1[10] = 0.0;
    buildRRT(dv0, dv1, jointLimits, -(bodyHeight + kC->B2PZOffset), U, dt, Dt,
             kC, b_T, pathJ);
    i0 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
    loop_ub = b_T->size[0] * b_T->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      T->data[i0] = b_T->data[i0];
    }

    // Transform path back to the Cartesian space.
    // Take the pathOld array and combine the general nodes and intermediate
    // states into a uniform path. The output path should be a npx6 array
    // that contains the n general nodes and the p intermediate nodes between
    // general nodes. Each row in the path matrix contains
    // [t,x,y,z,xDot,yDot,zDot] state data.
    alpha = rt_roundd_snf(Dt / dt);
    loop_ub = (int)(alpha * (double)pathJ->size[0]);
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = loop_ub;
    b_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    loop_ub = (int)(alpha * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = 0.0;
    }

    alpha = rt_roundd_snf(Dt / dt);
    loop_ub = (int)(alpha * (double)pathJ->size[0]);
    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = loop_ub;
    b_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathJ, i0, (int)sizeof(double));
    loop_ub = (int)(alpha * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathJ->data[i0] = 0.0;
    }

    count = 1U;
    time = rt_roundd_snf(Dt / dt) * (double)pathJ->size[0] * dt;
    for (loop_ub = 0; loop_ub < pathJ->size[0]; loop_ub++) {
      for (j = pathJ->size[1] - 6; j + 6 >= 18; j -= 6) {
        // sherpaTTFK Sherpa_TT Forward Kinematics
        //    Calculates the x,y,z position of the contact point given the alpha, 
        //    beta and gamma joint values.
        // sherpaTTFK.m
        // author: wreid
        // date: 20150122
        alpha = pathJ->data[loop_ub + pathJ->size[0] * j];
        uP[0] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[loop_ub + pathJ->size[0] *
                     (1 + j)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos
                  (pathJ->data[loop_ub + pathJ->size[0] * (2 + j)] + kC->zeta))
                 - kC->l7) * cos(alpha);
        uP[1] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[loop_ub + pathJ->size[0] *
                     (1 + j)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos
                  (pathJ->data[loop_ub + pathJ->size[0] * (2 + j)] + kC->zeta))
                 - kC->l7) * sin(alpha);
        uP[2] = ((((kC->l1 + kC->l3 * sin(-pathJ->data[loop_ub + pathJ->size[0] *
                     (1 + j)])) - kC->l4 * sin(kC->zeta)) - kC->l5 * sin
                  (pathJ->data[loop_ub + pathJ->size[0] * (2 + j)] + kC->zeta))
                 - kC->l6) - (kC->l8 + kC->r);

        // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
        // sherpaTTFKVel.m
        // author: wreid
        // date: 20150122
        alpha = pathJ->data[loop_ub + pathJ->size[0] * (3 + j)];
        betaDot = pathJ->data[loop_ub + pathJ->size[0] * (4 + j)];
        gammaDot = pathJ->data[loop_ub + pathJ->size[0] * (5 + j)];
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            d0 += TP2B[i0 + (i1 << 2)] * uP[i1];
          }

          c_TP2B[i0] = d0 + TP2B[12 + i0];
        }

        alphaDot[0] = (-alpha * sin(pathJ->data[loop_ub + pathJ->size[0] * j]) *
                       ((((kC->l2 - kC->l7) + kC->l5 * cos(pathJ->data[loop_ub +
          pathJ->size[0] * (2 + j)] + kC->zeta)) + kC->l3 * cos(pathJ->
          data[loop_ub + pathJ->size[0] * (1 + j)])) + kC->l4 * cos(kC->zeta)) -
                       betaDot * kC->l3 * cos(pathJ->data[loop_ub + pathJ->size
          [0] * j]) * sin(pathJ->data[loop_ub + pathJ->size[0] * (1 + j)])) -
          gammaDot * kC->l5 * sin(pathJ->data[loop_ub + pathJ->size[0] * (2 + j)]
          + kC->zeta) * cos(pathJ->data[loop_ub + pathJ->size[0] * j]);
        alphaDot[1] = (alpha * cos(pathJ->data[loop_ub + pathJ->size[0] * j]) *
                       ((((kC->l2 - kC->l7) + kC->l5 * cos(pathJ->data[loop_ub +
          pathJ->size[0] * (2 + j)] + kC->zeta)) + kC->l3 * cos(pathJ->
          data[loop_ub + pathJ->size[0] * (1 + j)])) + kC->l4 * cos(kC->zeta)) -
                       gammaDot * kC->l5 * sin(pathJ->data[loop_ub + pathJ->
          size[0] * (2 + j)] + kC->zeta) * sin(pathJ->data[loop_ub + pathJ->
          size[0] * j])) - betaDot * kC->l3 * sin(pathJ->data[loop_ub +
          pathJ->size[0] * j]) * sin(pathJ->data[loop_ub + pathJ->size[0] * (1 +
          j)]);
        alphaDot[2] = -betaDot * kC->l3 * cos(pathJ->data[loop_ub + pathJ->size
          [0] * (1 + j)]) - kC->l5 * gammaDot * cos(kC->zeta + pathJ->
          data[loop_ub + pathJ->size[0] * (2 + j)]);
        for (i0 = 0; i0 < 3; i0++) {
          b_TB2P[i0] = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            b_TB2P[i0] += TP2B[i0 + (i1 << 2)] * alphaDot[i1];
          }
        }

        b_pathC->data[(int)count - 1] = time;
        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 1)) - 1] =
            c_TP2B[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 4)) - 1] =
            b_TB2P[i0];
        }

        b_pathJ->data[(int)count - 1] = time;
        for (i0 = 0; i0 < 6; i0++) {
          b_pathJ->data[((int)count + b_pathJ->size[0] * (i0 + 1)) - 1] =
            pathJ->data[loop_ub + pathJ->size[0] * (i0 + j)];
        }

        time -= dt;
        count++;
      }
    }

    loop_ub = b_pathC->size[0];
    i0 = c_pathC->size[0] * c_pathC->size[1];
    c_pathC->size[0] = loop_ub;
    c_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)c_pathC, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        c_pathC->data[i1 + c_pathC->size[0] * i0] = b_pathC->data[i1 +
          b_pathC->size[0] * i0];
      }
    }

    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = c_pathC->size[0];
    b_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      loop_ub = c_pathC->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        b_pathC->data[i1 + b_pathC->size[0] * i0] = c_pathC->data[i1 +
          c_pathC->size[0] * i0];
      }
    }

    flipud(b_pathC);
    loop_ub = b_pathJ->size[0];
    i0 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = loop_ub;
    c_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)c_pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        c_pathJ->data[i1 + c_pathJ->size[0] * i0] = b_pathJ->data[i1 +
          b_pathJ->size[0] * i0];
      }
    }

    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = c_pathJ->size[0];
    b_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      loop_ub = c_pathJ->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        b_pathJ->data[i1 + b_pathJ->size[0] * i0] = c_pathJ->data[i1 +
          c_pathJ->size[0] * i0];
      }
    }

    flipud(b_pathJ);
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 1 + b_pathC->size[0];
    pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)pathC, i0, (int)sizeof(double));
    pathC->data[0] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      pathC->data[pathC->size[0] * (i0 + 1)] = nInitCartesianB[i0];
    }

    for (i0 = 0; i0 < 7; i0++) {
      loop_ub = b_pathC->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        pathC->data[(i1 + pathC->size[0] * i0) + 1] = b_pathC->data[i1 +
          b_pathC->size[0] * i0];
      }
    }

    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 1 + b_pathJ->size[0];
    pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    pathJ->data[0] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      pathJ->data[pathJ->size[0] * (i0 + 1)] = qInit[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      pathJ->data[pathJ->size[0] * (i0 + 4)] = qDotInit[i0];
    }

    for (i0 = 0; i0 < 7; i0++) {
      loop_ub = b_pathJ->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        pathJ->data[(i1 + pathJ->size[0] * i0) + 1] = b_pathJ->data[i1 +
          b_pathJ->size[0] * i0];
      }
    }
  } else {
    *success = false;
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 0;
    pathC->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathC, i0, (int)sizeof(double));
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    i0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
  }

  emxFree_real_T(&c_pathJ);
  emxFree_real_T(&c_pathC);
  emxFree_real_T(&b_pathJ);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&b_T);
}

//
// Arguments    : void
// Return Type  : void
//
void buildRRTWrapper_init()
{
  int i5;
  static const double dv3[3] = { 1.0, 0.0, 0.5 };

  for (i5 = 0; i5 < 3; i5++) {
    HGAINS[i5] = dv3[i5];
  }
}

//
// File trailer for buildRRTWrapper.cpp
//
// [EOF]
//
