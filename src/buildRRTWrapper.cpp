//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 09-Feb-2015 13:36:11
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRT.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "inv.h"
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
// buildRRTWrapper(nInitCartesianB,nGoalCartesianB,NUM_NODES,jointLimits,K,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kinematicConst,ankleThreshold,exhaustive,threshold,goalSeedFreq,legNum)
// Arguments    : const double nInitCartesianB[6]
//                const double nGoalCartesianB[6]
//                const double jointLimits[12]
//                double K
//                const double U[10]
//                double dt
//                double Dt
//                const double kinematicConst[15]
//                double threshold
//                int legNum
//                emxArray_real_T *T
//                emxArray_real_T *pathC
//                emxArray_real_T *pathJ
//                boolean_T *success
// Return Type  : void
//
void buildRRTWrapper(const double nInitCartesianB[6], const double
                     nGoalCartesianB[6], const double jointLimits[12], double K,
                     const double U[10], double dt, double Dt, const double
                     kinematicConst[15], double, int legNum, emxArray_real_T *T,
                     emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T
                     *success)
{
  long i0;
  double TP2B[16];
  int path_idx_0;
  static const signed char iv0[4] = { 0, 0, 1, 0 };

  static const signed char iv1[4] = { 0, 0, 0, 1 };

  double TB2P[16];
  double uInitP[3];
  double betaInit;
  int md2;
  double uGoalP[3];
  double gammaInit;
  double time;
  double gammaGoal;
  double betaGoal;
  double alphaGoal;
  double alphaInit[3];
  double b_TB2P[3];
  double c_TB2P[3];
  double d_TB2P[3];
  double b_alphaGoal[3];
  double nInitJoint[6];
  double nGoalJoint[6];
  emxArray_real_T *b_T;
  emxArray_real_T *path;
  emxArray_real_T *x;
  double dv0[11];
  double dv1[11];
  int m;
  unsigned int count;
  int i;
  int j;
  double b_pathJ[3];
  double c_pathJ[3];
  double b_kinematicConst[3];
  double b_uInitP[3];
  int b_path_idx_0;

  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  // Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame 
  // to the pan coordinate frame.
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[0] = cos(kinematicConst[(int)i0 - 1]);
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[4] = -sin(kinematicConst[(int)i0 - 1]);
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[8] = sin(kinematicConst[(int)i0 - 1]) * 0.0;
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[12] = kinematicConst[9] * cos(kinematicConst[(int)i0 - 1]);
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[1] = sin(kinematicConst[(int)i0 - 1]);
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[5] = cos(kinematicConst[(int)i0 - 1]);
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[9] = -cos(kinematicConst[(int)i0 - 1]) * 0.0;
  i0 = 11L + legNum;
  if (i0 > 2147483647L) {
    i0 = 2147483647L;
  } else {
    if (i0 < -2147483648L) {
      i0 = -2147483648L;
    }
  }

  TP2B[13] = kinematicConst[9] * sin(kinematicConst[(int)i0 - 1]);
  for (path_idx_0 = 0; path_idx_0 < 4; path_idx_0++) {
    TP2B[2 + (path_idx_0 << 2)] = iv0[path_idx_0];
    TP2B[3 + (path_idx_0 << 2)] = iv1[path_idx_0];
  }

  inv(TP2B, TB2P);

  // invHomoMatrix(TP2B);%
  for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
    betaInit = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      betaInit += TB2P[path_idx_0 + (md2 << 2)] * nInitCartesianB[md2];
    }

    uInitP[path_idx_0] = betaInit + TB2P[12 + path_idx_0];
  }

  for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
    betaInit = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      betaInit += TB2P[path_idx_0 + (md2 << 2)] * nGoalCartesianB[md2];
    }

    uGoalP[path_idx_0] = betaInit + TB2P[12 + path_idx_0];
  }

  // Transform the Cartesian goal and final positions in the pan coordinate
  // frame to the joint space.
  sherpaTTIK(uInitP[0], uInitP[1], uInitP[2], kinematicConst, jointLimits, &time,
             &betaInit, &gammaInit);
  sherpaTTIK(uGoalP[0], uGoalP[1], uGoalP[2], kinematicConst, jointLimits,
             &alphaGoal, &betaGoal, &gammaGoal);
  alphaInit[0] = time;
  alphaInit[1] = betaInit;
  alphaInit[2] = gammaInit;
  for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
    uGoalP[path_idx_0] = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      uGoalP[path_idx_0] += TB2P[path_idx_0 + (md2 << 2)] * nInitCartesianB[3 +
        md2];
    }

    b_TB2P[path_idx_0] = uGoalP[path_idx_0];
    d_TB2P[path_idx_0] = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      d_TB2P[path_idx_0] += TB2P[path_idx_0 + (md2 << 2)] * nGoalCartesianB[3 +
        md2];
    }

    c_TB2P[path_idx_0] = d_TB2P[path_idx_0];
  }

  sherpaTTIKVel(b_TB2P, alphaInit, kinematicConst, uInitP);
  b_alphaGoal[0] = alphaGoal;
  b_alphaGoal[1] = betaGoal;
  b_alphaGoal[2] = gammaGoal;
  sherpaTTIKVel(c_TB2P, b_alphaGoal, kinematicConst, uGoalP);
  nInitJoint[0] = time;
  nInitJoint[1] = betaInit;
  nInitJoint[2] = gammaInit;
  nInitJoint[3] = uInitP[0];
  nInitJoint[4] = uInitP[1];
  nInitJoint[5] = uInitP[2];
  nGoalJoint[0] = alphaGoal;
  nGoalJoint[1] = betaGoal;
  nGoalJoint[2] = gammaGoal;
  nGoalJoint[3] = uGoalP[0];
  nGoalJoint[4] = uGoalP[1];
  nGoalJoint[5] = uGoalP[2];

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  emxInit_real_T(&b_T, 2);
  emxInit_real_T(&path, 2);
  emxInit_real_T(&x, 2);
  if (validState(nInitJoint, jointLimits) && validState(nGoalJoint, jointLimits))
  {
    *success = true;

    // Run buildRRT.
    dv0[0] = 0.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 6; path_idx_0++) {
      dv0[path_idx_0 + 3] = nInitJoint[path_idx_0];
    }

    dv0[9] = 0.0;
    dv0[10] = 0.0;
    dv1[0] = 0.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 6; path_idx_0++) {
      dv1[path_idx_0 + 3] = nGoalJoint[path_idx_0];
    }

    dv1[9] = 0.0;
    dv1[10] = 0.0;
    buildRRT(dv0, dv1, jointLimits, K, U, dt, Dt, kinematicConst, b_T, pathJ);
    path_idx_0 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity((emxArray__common *)T, path_idx_0, (int)sizeof(double));
    m = b_T->size[0] * b_T->size[1];
    for (path_idx_0 = 0; path_idx_0 < m; path_idx_0++) {
      T->data[path_idx_0] = b_T->data[path_idx_0];
    }

    // Transform path back to the Cartesian space.
    // Take the pathOld array and combine the general nodes and intermediate
    // states into a uniform path. The output path should be a npx6 array
    // that contains the n general nodes and the p intermediate nodes between
    // general nodes. Each row in the path matrix contains
    // [t,x,y,z,xDot,yDot,zDot] state data.
    time = rt_roundd_snf(Dt / dt);
    m = (int)(time * (double)pathJ->size[0]);
    path_idx_0 = path->size[0] * path->size[1];
    path->size[0] = m;
    path->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)path, path_idx_0, (int)sizeof(double));
    m = (int)(time * (double)pathJ->size[0]) * 7;
    for (path_idx_0 = 0; path_idx_0 < m; path_idx_0++) {
      path->data[path_idx_0] = 0.0;
    }

    count = 1U;
    time = rt_roundd_snf(Dt / dt) * (double)pathJ->size[0] * dt;
    for (i = 0; i < pathJ->size[0]; i++) {
      for (j = pathJ->size[1] - 4; j + 4 >= 18; j -= 6) {
        // sherpaTTFK Sherpa_TT Forward Kinematics
        //    Calculates the x,y,z position of the contact point given the alpha, 
        //    beta and gamma joint values.
        // sherpaTTFK.m
        // author: wreid
        // date: 20150122
        b_pathJ[0] = pathJ->data[i + pathJ->size[0] * (j + 1)];
        b_pathJ[1] = pathJ->data[i + pathJ->size[0] * (j + 2)];
        b_pathJ[2] = pathJ->data[i + pathJ->size[0] * (j + 3)];
        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          uInitP[path_idx_0] = b_pathJ[path_idx_0];
        }

        c_pathJ[0] = pathJ->data[i + pathJ->size[0] * (j - 2)];
        c_pathJ[1] = pathJ->data[i + pathJ->size[0] * (j - 1)];
        c_pathJ[2] = pathJ->data[i + pathJ->size[0] * j];
        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          uGoalP[path_idx_0] = c_pathJ[path_idx_0];
        }

        // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
        // sherpaTTFKVel.m
        // author: wreid
        // date: 20150122
        b_kinematicConst[0] = ((((kinematicConst[1] + kinematicConst[2] * cos
          (-pathJ->data[i + pathJ->size[0] * (j - 1)])) + kinematicConst[3] *
          cos(kinematicConst[8])) + kinematicConst[4] * cos(pathJ->data[i +
          pathJ->size[0] * j] + kinematicConst[8])) - kinematicConst[6]) * cos
          (pathJ->data[i + pathJ->size[0] * (j - 2)]);
        b_kinematicConst[1] = ((((kinematicConst[1] + kinematicConst[2] * cos
          (-pathJ->data[i + pathJ->size[0] * (j - 1)])) + kinematicConst[3] *
          cos(kinematicConst[8])) + kinematicConst[4] * cos(pathJ->data[i +
          pathJ->size[0] * j] + kinematicConst[8])) - kinematicConst[6]) * sin
          (pathJ->data[i + pathJ->size[0] * (j - 2)]);
        b_kinematicConst[2] = ((((kinematicConst[0] + kinematicConst[2] * sin
          (-pathJ->data[i + pathJ->size[0] * (j - 1)])) - kinematicConst[3] *
          sin(kinematicConst[8])) - kinematicConst[4] * sin(pathJ->data[i +
          pathJ->size[0] * j] + kinematicConst[8])) - kinematicConst[5]) -
          kinematicConst[7];
        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          betaInit = 0.0;
          for (md2 = 0; md2 < 3; md2++) {
            betaInit += TP2B[path_idx_0 + (md2 << 2)] * b_kinematicConst[md2];
          }

          b_TB2P[path_idx_0] = betaInit + TP2B[12 + path_idx_0];
        }

        b_uInitP[0] = (-uInitP[0] * sin(uGoalP[0]) * ((((kinematicConst[1] -
          kinematicConst[6]) + kinematicConst[4] * cos(uGoalP[2] +
          kinematicConst[8])) + kinematicConst[2] * cos(uGoalP[1])) +
          kinematicConst[3] * cos(kinematicConst[8])) - uInitP[1] *
                       kinematicConst[2] * cos(uGoalP[0]) * sin(uGoalP[1])) -
          uInitP[2] * kinematicConst[4] * sin(uGoalP[2] + kinematicConst[8]) *
          cos(uGoalP[0]);
        b_uInitP[1] = (uInitP[0] * cos(uGoalP[0]) * ((((kinematicConst[1] -
          kinematicConst[6]) + kinematicConst[4] * cos(uGoalP[2] +
          kinematicConst[8])) + kinematicConst[2] * cos(uGoalP[1])) +
          kinematicConst[3] * cos(kinematicConst[8])) - uInitP[2] *
                       kinematicConst[4] * sin(uGoalP[2] + kinematicConst[8]) *
                       sin(uGoalP[0])) - uInitP[1] * kinematicConst[2] * sin
          (uGoalP[0]) * sin(uGoalP[1]);
        b_uInitP[2] = -uInitP[1] * kinematicConst[2] * cos(uGoalP[1]) -
          kinematicConst[4] * uInitP[2] * cos(kinematicConst[8] + uGoalP[2]);
        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          uGoalP[path_idx_0] = 0.0;
          for (md2 = 0; md2 < 3; md2++) {
            uGoalP[path_idx_0] += TP2B[path_idx_0 + (md2 << 2)] * b_uInitP[md2];
          }
        }

        path->data[(int)count - 1] = time;
        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          path->data[((int)count + path->size[0] * (path_idx_0 + 1)) - 1] =
            b_TB2P[path_idx_0];
        }

        for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
          path->data[((int)count + path->size[0] * (path_idx_0 + 4)) - 1] =
            uGoalP[path_idx_0];
        }

        time -= dt;
        count++;
      }
    }

    m = path->size[0];
    path_idx_0 = x->size[0] * x->size[1];
    x->size[0] = m;
    x->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)x, path_idx_0, (int)sizeof(double));
    for (path_idx_0 = 0; path_idx_0 < 7; path_idx_0++) {
      for (md2 = 0; md2 < m; md2++) {
        x->data[md2 + x->size[0] * path_idx_0] = path->data[md2 + path->size[0] *
          path_idx_0];
      }
    }

    path_idx_0 = path->size[0] * path->size[1];
    path->size[0] = x->size[0];
    path->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)path, path_idx_0, (int)sizeof(double));
    m = x->size[0] * x->size[1];
    for (path_idx_0 = 0; path_idx_0 < m; path_idx_0++) {
      path->data[path_idx_0] = x->data[path_idx_0];
    }

    m = x->size[0];
    md2 = x->size[0] >> 1;
    for (j = 0; j < 7; j++) {
      for (i = 1; i <= md2; i++) {
        path_idx_0 = path->size[0];
        time = path->data[(i + path_idx_0 * j) - 1];
        path_idx_0 = path->size[0];
        b_path_idx_0 = path->size[0];
        path->data[(i + path_idx_0 * j) - 1] = path->data[(m - i) + b_path_idx_0
          * j];
        path_idx_0 = path->size[0];
        path->data[(m - i) + path_idx_0 * j] = time;
      }
    }

    path_idx_0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 1 + path->size[0];
    pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)pathC, path_idx_0, (int)sizeof(double));
    pathC->data[0] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 6; path_idx_0++) {
      pathC->data[pathC->size[0] * (path_idx_0 + 1)] =
        nInitCartesianB[path_idx_0];
    }

    for (path_idx_0 = 0; path_idx_0 < 7; path_idx_0++) {
      m = path->size[0];
      for (md2 = 0; md2 < m; md2++) {
        pathC->data[(md2 + pathC->size[0] * path_idx_0) + 1] = path->data[md2 +
          path->size[0] * path_idx_0];
      }
    }
  } else {
    *success = false;
    path_idx_0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 0;
    pathC->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathC, path_idx_0, (int)sizeof(double));
    path_idx_0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathJ, path_idx_0, (int)sizeof(double));
    path_idx_0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T, path_idx_0, (int)sizeof(double));
  }

  emxFree_real_T(&x);
  emxFree_real_T(&path);
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
