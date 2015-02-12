//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 12-Feb-2015 09:24:14
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "extractKinematicConstants.h"
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
//                const double kinematicConst[16]
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
                     double kinematicConst[16], double, int legNum,
                     emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T
                     *pathJ, boolean_T *success)
{
  double unusedUe;
  double unusedUd;
  double unusedUc;
  double unusedUb;
  double B2PZOffset;
  double unusedUa;
  double unusedU9;
  double unusedU8;
  double unusedU7;
  double unusedU6;
  double unusedU5;
  double unusedU4;
  double unusedU3;
  double unusedU2;
  double unusedU1;
  double unusedU0;
  double legAngleOffset;
  double TP2B[16];
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  double b_TP2B[9];
  int i1;
  double c_TP2B[3];
  double TB2P[16];
  double uInitP[3];
  double uGoalP[3];
  double gammaInit;
  double betaInit;
  double alphaInit;
  double b_alphaInit[3];
  double b_TB2P[3];
  double qDotInit[3];
  double alphaGoal[3];
  double nInitJoint[6];
  double nGoalJoint[6];
  emxArray_real_T *b_T;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_pathJ;
  emxArray_real_T *c_pathC;
  emxArray_real_T *c_pathJ;
  double dv0[11];
  double dv1[11];
  int loop_ub;
  unsigned int count;
  double time;
  int j;
  double zeta;
  double L8;
  double d_pathJ[3];
  double e_pathJ[3];
  double b_zeta;
  double L7;
  double L5;
  double L4;
  double L3;
  double L2;
  double b_L2[3];
  double b_uInitP[3];

  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  // GETPANHEIGHT Summary of this function goes here
  //    Detailed explanation goes here
  extractKinematicConstants(kinematicConst, &unusedU0, &unusedU1, &unusedU2,
    &unusedU3, &unusedU4, &unusedU5, &unusedU6, &unusedU7, &unusedU8, &unusedU9,
    &unusedUa, &B2PZOffset, &unusedUb, &unusedUc, &unusedUd, &unusedUe);

  // Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame 
  // to the pan coordinate frame.
  // TRP2B Calculates the homogeneous transformation matrix between the body
  // and pan coordinate frames.
  extractKinematicConstants(kinematicConst, &unusedU0, &unusedU1, &unusedU2,
    &unusedU3, &unusedU4, &unusedU5, &unusedU6, &unusedU7, &unusedU8, &unusedU9,
    &unusedUa, &unusedUb, &unusedUc, &unusedUd, &unusedUe, &legAngleOffset);
  switch (legNum) {
   case 1:
    legAngleOffset = unusedUc;
    break;

   case 2:
    legAngleOffset = unusedUd;
    break;

   case 3:
    legAngleOffset = unusedUe;
    break;
  }

  TP2B[0] = cos(legAngleOffset);
  TP2B[4] = -sin(legAngleOffset);
  TP2B[8] = sin(legAngleOffset) * 0.0;
  TP2B[12] = unusedUa * cos(legAngleOffset);
  TP2B[1] = sin(legAngleOffset);
  TP2B[5] = cos(legAngleOffset);
  TP2B[9] = -cos(legAngleOffset) * 0.0;
  TP2B[13] = unusedUa * sin(legAngleOffset);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = unusedUb;
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
  for (i0 = 0; i0 < 3; i0++) {
    unusedUa = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      unusedUa += TB2P[i0 + (i1 << 2)] * nInitCartesianB[i1];
    }

    uInitP[i0] = unusedUa + TB2P[12 + i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    unusedUa = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      unusedUa += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[i1];
    }

    uGoalP[i0] = unusedUa + TB2P[12 + i0];
  }

  // Transform the Cartesian goal and final positions in the pan coordinate
  // frame to the joint space.
  sherpaTTIK(uInitP[0], uInitP[1], uInitP[2], kinematicConst, jointLimits,
             &alphaInit, &betaInit, &gammaInit);
  sherpaTTIK(uGoalP[0], uGoalP[1], uGoalP[2], kinematicConst, jointLimits,
             &unusedUa, &unusedUb, &unusedUc);
  b_alphaInit[0] = alphaInit;
  b_alphaInit[1] = betaInit;
  b_alphaInit[2] = gammaInit;
  for (i0 = 0; i0 < 3; i0++) {
    uGoalP[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      uGoalP[i0] += TB2P[i0 + (i1 << 2)] * nInitCartesianB[3 + i1];
    }

    c_TP2B[i0] = uGoalP[i0];
    qDotInit[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      qDotInit[i0] += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[3 + i1];
    }

    b_TB2P[i0] = qDotInit[i0];
  }

  sherpaTTIKVel(c_TP2B, b_alphaInit, kinematicConst, qDotInit);
  alphaGoal[0] = unusedUa;
  alphaGoal[1] = unusedUb;
  alphaGoal[2] = unusedUc;
  sherpaTTIKVel(b_TB2P, alphaGoal, kinematicConst, uInitP);
  nInitJoint[0] = alphaInit;
  nInitJoint[1] = betaInit;
  nInitJoint[2] = gammaInit;
  nInitJoint[3] = qDotInit[0];
  nInitJoint[4] = qDotInit[1];
  nInitJoint[5] = qDotInit[2];
  nGoalJoint[0] = unusedUa;
  nGoalJoint[1] = unusedUb;
  nGoalJoint[2] = unusedUc;
  nGoalJoint[3] = uInitP[0];
  nGoalJoint[4] = uInitP[1];
  nGoalJoint[5] = uInitP[2];

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
    buildRRT(dv0, dv1, jointLimits, -(bodyHeight + B2PZOffset), U, dt, Dt,
             kinematicConst, b_T, pathJ);
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
    unusedUa = rt_roundd_snf(Dt / dt);
    loop_ub = (int)(unusedUa * (double)pathJ->size[0]);
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = loop_ub;
    b_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    loop_ub = (int)(unusedUa * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = 0.0;
    }

    unusedUa = rt_roundd_snf(Dt / dt);
    loop_ub = (int)(unusedUa * (double)pathJ->size[0]);
    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = loop_ub;
    b_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathJ, i0, (int)sizeof(double));
    loop_ub = (int)(unusedUa * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathJ->data[i0] = 0.0;
    }

    count = 1U;
    time = rt_roundd_snf(Dt / dt) * (double)pathJ->size[0] * dt;
    for (loop_ub = 0; loop_ub < pathJ->size[0]; loop_ub++) {
      for (j = pathJ->size[1] - 4; j + 4 >= 18; j -= 6) {
        // sherpaTTFK Sherpa_TT Forward Kinematics
        //    Calculates the x,y,z position of the contact point given the alpha, 
        //    beta and gamma joint values.
        // sherpaTTFK.m
        // author: wreid
        // date: 20150122
        extractKinematicConstants(kinematicConst, &unusedUa, &unusedUb,
          &unusedUc, &unusedUd, &unusedUe, &legAngleOffset, &B2PZOffset, &L8,
          &zeta, &unusedU0, &unusedU1, &unusedU2, &unusedU3, &unusedU4,
          &unusedU5, &unusedU6);
        d_pathJ[0] = pathJ->data[loop_ub + pathJ->size[0] * (j + 1)];
        d_pathJ[1] = pathJ->data[loop_ub + pathJ->size[0] * (j + 2)];
        d_pathJ[2] = pathJ->data[loop_ub + pathJ->size[0] * (j + 3)];
        for (i0 = 0; i0 < 3; i0++) {
          uInitP[i0] = d_pathJ[i0];
        }

        e_pathJ[0] = pathJ->data[loop_ub + pathJ->size[0] * (j - 2)];
        e_pathJ[1] = pathJ->data[loop_ub + pathJ->size[0] * (j - 1)];
        e_pathJ[2] = pathJ->data[loop_ub + pathJ->size[0] * j];
        for (i0 = 0; i0 < 3; i0++) {
          uGoalP[i0] = e_pathJ[i0];
        }

        // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
        // sherpaTTFKVel.m
        // author: wreid
        // date: 20150122
        extractKinematicConstants(kinematicConst, &unusedU0, &L2, &L3, &L4, &L5,
          &unusedU1, &L7, &unusedU2, &b_zeta, &unusedU3, &unusedU4, &unusedU5,
          &unusedU6, &unusedU7, &unusedU8, &unusedU9);
        b_L2[0] = ((((unusedUb + unusedUc * cos(-pathJ->data[loop_ub +
          pathJ->size[0] * (j - 1)])) + unusedUd * cos(zeta)) + unusedUe * cos
                    (pathJ->data[loop_ub + pathJ->size[0] * j] + zeta)) -
                   B2PZOffset) * cos(pathJ->data[loop_ub + pathJ->size[0] * (j -
          2)]);
        b_L2[1] = ((((unusedUb + unusedUc * cos(-pathJ->data[loop_ub +
          pathJ->size[0] * (j - 1)])) + unusedUd * cos(zeta)) + unusedUe * cos
                    (pathJ->data[loop_ub + pathJ->size[0] * j] + zeta)) -
                   B2PZOffset) * sin(pathJ->data[loop_ub + pathJ->size[0] * (j -
          2)]);
        b_L2[2] = ((((unusedUa + unusedUc * sin(-pathJ->data[loop_ub +
          pathJ->size[0] * (j - 1)])) - unusedUd * sin(zeta)) - unusedUe * sin
                    (pathJ->data[loop_ub + pathJ->size[0] * j] + zeta)) -
                   legAngleOffset) - L8;
        for (i0 = 0; i0 < 3; i0++) {
          unusedUa = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            unusedUa += TP2B[i0 + (i1 << 2)] * b_L2[i1];
          }

          c_TP2B[i0] = unusedUa + TP2B[12 + i0];
        }

        b_uInitP[0] = (-uInitP[0] * sin(uGoalP[0]) * ((((L2 - L7) + L5 * cos
          (uGoalP[2] + b_zeta)) + L3 * cos(uGoalP[1])) + L4 * cos(b_zeta)) -
                       uInitP[1] * L3 * cos(uGoalP[0]) * sin(uGoalP[1])) -
          uInitP[2] * L5 * sin(uGoalP[2] + b_zeta) * cos(uGoalP[0]);
        b_uInitP[1] = (uInitP[0] * cos(uGoalP[0]) * ((((L2 - L7) + L5 * cos
          (uGoalP[2] + b_zeta)) + L3 * cos(uGoalP[1])) + L4 * cos(b_zeta)) -
                       uInitP[2] * L5 * sin(uGoalP[2] + b_zeta) * sin(uGoalP[0]))
          - uInitP[1] * L3 * sin(uGoalP[0]) * sin(uGoalP[1]);
        b_uInitP[2] = -uInitP[1] * L3 * cos(uGoalP[1]) - L5 * uInitP[2] * cos
          (b_zeta + uGoalP[2]);
        for (i0 = 0; i0 < 3; i0++) {
          uGoalP[i0] = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            uGoalP[i0] += TP2B[i0 + (i1 << 2)] * b_uInitP[i1];
          }
        }

        b_pathC->data[(int)count - 1] = time;
        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 1)) - 1] =
            c_TP2B[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 4)) - 1] =
            uGoalP[i0];
        }

        b_pathJ->data[(int)count - 1] = time;
        for (i0 = 0; i0 < 6; i0++) {
          b_pathJ->data[((int)count + b_pathJ->size[0] * (i0 + 1)) - 1] =
            pathJ->data[loop_ub + pathJ->size[0] * ((i0 + j) - 2)];
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
    pathJ->data[pathJ->size[0]] = alphaInit;
    pathJ->data[pathJ->size[0] << 1] = betaInit;
    pathJ->data[pathJ->size[0] * 3] = gammaInit;
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
