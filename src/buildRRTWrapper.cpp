//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 25-Feb-2015 11:22:41
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "buildRRTWrapper_emxutil.h"
#include "norm.h"
#include "sherpaTTIK.h"
#include "nearestNeighbour.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Variable Definitions
static double cartesianLimits[4];
static double HGAINS[3];

// Function Definitions

//
// Arguments    : const double nInitCartesianB[6]
//                const double nGoalCartesianB[6]
//                double phiInit
//                double omegaInit
//                const double jointLimits[20]
//                double bodyHeight
//                const double U[10]
//                double dt
//                double Dt
//                const struct0_T *kC
//                double threshold
//                int legNum
//                const double uBDot[6]
//                emxArray_real_T *T
//                emxArray_real_T *pathC
//                emxArray_real_T *pathJ
//                boolean_T *success
// Return Type  : void
//
void buildRRTWrapper(const double nInitCartesianB[6], const double
                     nGoalCartesianB[6], double phiInit, double omegaInit, const
                     double jointLimits[20], double bodyHeight, const double U
                     [10], double dt, double Dt, const struct0_T *kC, double,
                     int legNum, const double uBDot[6], emxArray_real_T *T,
                     emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T
                     *success)
{
  double panHeight;
  double TP2B[16];
  int i0;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  double b_TP2B[9];
  int cdiff;
  double c_TP2B[3];
  double TB2P[16];
  double b_TB2P[3];
  double newState;
  double qInit[3];
  double qGoal[3];
  double nInitJoint[10];
  double uB[3];
  double c_TB2P[3];
  double nGoalJoint[10];
  emxArray_real_T *b_T;
  emxArray_real_T *b_pathC;
  emxArray_real_T *next;
  emxArray_real_T *transitionArray;
  emxArray_real_T *transitionPath;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  double transitionArrayLength;
  int apnd;
  int absb;
  double dv0[13];
  double unusedU2;
  int xNearest_size[2];
  double xNearest_data[13];
  unsigned int i;
  int ndbl;
  double b_newState[3];
  double b_pathJ[3];
  double sGoalC[9];
  double pathCorrection[90];
  emxArray_real_T *c_pathJ;

  // BUILDRRTWRAPPER This function acts as a wrapper for the buildRRT function.
  // Code generation for the singleLegPlanner is performed using this function
  // as an entry point.
  //
  // Inputs:
  // -nInitCartesianB: the
  // -nGoalCartesianB:
  // -jointLimits:
  // -bodyHeight:
  // -U:
  // -dt:
  // -Dt:
  // -kC:
  // -threshold:
  // -legNum:
  //
  // Outputs:
  // -T:
  // -pathC:
  // -pathJ:
  // -success:
  //
  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  // if isempty(NUM_NODES)
  //     NUM_NODES = int32(1000);
  // end
  // GETPANHEIGHT Summary of this function goes here
  //    Detailed explanation goes here
  panHeight = -(bodyHeight + kC->B2PZOffset);

  // Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame 
  // to the pan coordinate frame.
  // TRP2B Generates the homogeneous transformation matrix between the body
  // and pan coordinate frames. kC is a struct containing the kinematic
  // constants of the leg. legNum indicates the number of the leg that is being
  // considered.
  //
  // Inputs:
  // -kC: Struct of kinematic constants of the Sherpa_TT leg
  // -legNum: The leg number identification.
  // Outputs:
  // -TP2B: The homogeneous transformation matrix that is used to transform
  // coordinates from the pan frame to the body frame.
  //
  // trP2B.m
  // author:    wreid
  // date:      20150214
  // TRDH Generates the homogeneous transformation matrix A using the
  // Denavit-Hartenberg parameters theta, d, a and alpha.
  //
  // trDH.m
  // author:    wreid
  // date:      20150214
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
    for (cdiff = 0; cdiff < 3; cdiff++) {
      b_TP2B[cdiff + 3 * i0] = -TP2B[i0 + (cdiff << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (cdiff = 0; cdiff < 3; cdiff++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * cdiff] * TP2B[12 + cdiff];
    }

    for (cdiff = 0; cdiff < 3; cdiff++) {
      TB2P[cdiff + (i0 << 2)] = TP2B[i0 + (cdiff << 2)];
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
    newState = 0.0;
    for (cdiff = 0; cdiff < 3; cdiff++) {
      newState += TB2P[i0 + (cdiff << 2)] * nInitCartesianB[cdiff];
    }

    b_TB2P[i0] = newState + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    newState = 0.0;
    for (cdiff = 0; cdiff < 3; cdiff++) {
      newState += TB2P[i0 + (cdiff << 2)] * nGoalCartesianB[cdiff];
    }

    b_TB2P[i0] = newState + TB2P[12 + i0];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (cdiff = 0; cdiff < 3; cdiff++) {
      c_TP2B[i0] += TB2P[i0 + (cdiff << 2)] * nInitCartesianB[3 + cdiff];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nInitJoint[i0] = qInit[i0];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nInitJoint[3] = phiInit;
  nInitJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nInitJoint[i0 + 5] = uB[i0];
    c_TB2P[i0] = 0.0;
    for (cdiff = 0; cdiff < 3; cdiff++) {
      c_TB2P[i0] += TB2P[i0 + (cdiff << 2)] * nGoalCartesianB[3 + cdiff];
    }

    b_TB2P[i0] = c_TB2P[i0];
    nGoalJoint[i0] = qGoal[i0];
  }

  nInitJoint[8] = 0.0;
  nInitJoint[9] = omegaInit;
  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nGoalJoint[3] = 0.0;
  nGoalJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nGoalJoint[i0 + 5] = uB[i0];
  }

  nGoalJoint[8] = 0.0;
  nGoalJoint[9] = 0.0;

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  emxInit_real_T(&b_T, 2);
  emxInit_real_T(&b_pathC, 2);
  emxInit_real_T(&next, 2);
  emxInit_real_T(&transitionArray, 2);
  emxInit_real_T(&transitionPath, 2);
  b_emxInit_real_T(&t, 1);
  emxInit_real_T(&path, 2);
  emxInit_real_T(&b_transitionPath, 2);
  emxInit_real_T(&c_transitionPath, 2);
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    // Run buildRRT.
    // buildRRT Icrementally builds a rapidly exploring random tree.
    //    An RRT is build by incrementally selecting a random state from the
    //    available state space as defined by the MIN and MAX vectors. The tree is 
    //    started at xInit and is extended until the number of maximum nodes, K has 
    //    been reached. A path is selected if the goal region as defined by xGoal 
    //    has been reached by the RRT.
    // buildRRT.m
    // author: wreid
    // date: 20150107
    // Constant Declaration                                                       
    transitionArrayLength = (rt_roundd_snf(Dt / dt) + 1.0) * 10.0;

    // Variable Initialization
    i0 = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1000;
    newState = rt_roundd_snf(13.0 + transitionArrayLength);
    if (newState < 2.147483648E+9) {
      if (newState >= -2.147483648E+9) {
        cdiff = (int)newState;
      } else {
        cdiff = MIN_int32_T;
      }
    } else if (newState >= 2.147483648E+9) {
      cdiff = MAX_int32_T;
    } else {
      cdiff = 0;
    }

    b_T->size[1] = cdiff;
    emxEnsureCapacity((emxArray__common *)b_T, i0, (int)sizeof(double));
    newState = rt_roundd_snf(13.0 + transitionArrayLength);
    if (newState < 2.147483648E+9) {
      if (newState >= -2.147483648E+9) {
        i0 = (int)newState;
      } else {
        i0 = MIN_int32_T;
      }
    } else if (newState >= 2.147483648E+9) {
      i0 = MAX_int32_T;
    } else {
      i0 = 0;
    }

    apnd = 1000 * i0;
    for (i0 = 0; i0 < apnd; i0++) {
      b_T->data[i0] = 0.0;
    }

    // Define a zero array that will be used to
    // store data from each tree node.
    b_T->data[0] = 1.0;
    b_T->data[b_T->size[0]] = 0.0;
    b_T->data[b_T->size[0] << 1] = 0.0;
    for (i0 = 0; i0 < 10; i0++) {
      b_T->data[b_T->size[0] * (i0 + 3)] = nInitJoint[i0];
    }

    apnd = (int)transitionArrayLength;
    for (i0 = 0; i0 < apnd; i0++) {
      b_T->data[b_T->size[0] * (i0 + 13)] = 0.0;
    }

    // Initialize the tree with initial state.
    transitionArrayLength = 1.0;
    for (absb = 0; absb < 999; absb++) {
      rrtLoop(b_T, jointLimits, kC, panHeight, U, Dt, dt, &transitionArrayLength,
              uBDot, legNum);
    }

    // Find the closest node in the tree to the goal node.
    dv0[0] = 0.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    memcpy(&dv0[3], &nGoalJoint[0], 10U * sizeof(double));
    nearestNeighbour(dv0, b_T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                     kC->l7, kC->l8, kC->zeta, kC->r, transitionArrayLength,
                     xNearest_data, xNearest_size, transitionArray, &unusedU2);
    transitionArrayLength = xNearest_data[0];
    i0 = next->size[0] * next->size[1];
    next->size[0] = 1;
    next->size[1] = 13;
    emxEnsureCapacity((emxArray__common *)next, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 13; i0++) {
      next->data[i0] = xNearest_data[i0];
    }

    i0 = path->size[0] * path->size[1];
    path->size[0] = 0;
    path->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)path, i0, (int)sizeof(double));

    // Iterate over the tree until the initial state has been found.
    while ((transitionArrayLength != 0.0) && (next->data[1] != 0.0)) {
      i0 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = 0;
      transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)transitionPath, i0, (int)sizeof
                        (double));
      i0 = (int)(((double)transitionArray->size[1] + 9.0) / 10.0);
      for (absb = 0; absb < i0; absb++) {
        i = absb * 10U + 1U;
        cdiff = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)c_transitionPath, cdiff, (int)
                          sizeof(double));
        for (cdiff = 0; cdiff < 10; cdiff++) {
          apnd = transitionPath->size[0];
          for (ndbl = 0; ndbl < apnd; ndbl++) {
            c_transitionPath->data[ndbl + c_transitionPath->size[0] * cdiff] =
              transitionPath->data[ndbl + transitionPath->size[0] * cdiff];
          }
        }

        for (cdiff = 0; cdiff < 10; cdiff++) {
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * cdiff] = transitionArray->data[(int)
            (cdiff + i) - 1];
        }

        cdiff = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)transitionPath, cdiff, (int)sizeof
                          (double));
        for (cdiff = 0; cdiff < 10; cdiff++) {
          apnd = c_transitionPath->size[0];
          for (ndbl = 0; ndbl < apnd; ndbl++) {
            transitionPath->data[ndbl + transitionPath->size[0] * cdiff] =
              c_transitionPath->data[ndbl + c_transitionPath->size[0] * cdiff];
          }
        }
      }

      i0 = b_transitionPath->size[0] * b_transitionPath->size[1];
      b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
      b_transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)b_transitionPath, i0, (int)sizeof
                        (double));
      for (i0 = 0; i0 < 10; i0++) {
        apnd = transitionPath->size[0];
        for (cdiff = 0; cdiff < apnd; cdiff++) {
          b_transitionPath->data[cdiff + b_transitionPath->size[0] * i0] =
            transitionPath->data[cdiff + transitionPath->size[0] * i0];
        }
      }

      for (i0 = 0; i0 < 10; i0++) {
        apnd = path->size[0];
        for (cdiff = 0; cdiff < apnd; cdiff++) {
          b_transitionPath->data[(cdiff + transitionPath->size[0]) +
            b_transitionPath->size[0] * i0] = path->data[cdiff + path->size[0] *
            i0];
        }
      }

      i0 = path->size[0] * path->size[1];
      path->size[0] = b_transitionPath->size[0];
      path->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)path, i0, (int)sizeof(double));
      for (i0 = 0; i0 < 10; i0++) {
        apnd = b_transitionPath->size[0];
        for (cdiff = 0; cdiff < apnd; cdiff++) {
          path->data[cdiff + path->size[0] * i0] = b_transitionPath->data[cdiff
            + b_transitionPath->size[0] * i0];
        }
      }

      transitionArrayLength = next->data[1];
      apnd = b_T->size[1];
      i0 = next->size[0] * next->size[1];
      next->size[0] = 1;
      next->size[1] = apnd;
      emxEnsureCapacity((emxArray__common *)next, i0, (int)sizeof(double));
      for (i0 = 0; i0 < apnd; i0++) {
        next->data[next->size[0] * i0] = b_T->data[((int)transitionArrayLength +
          b_T->size[0] * i0) - 1];
      }

      transitionArrayLength = next->data[1];
      if (14 > next->size[1]) {
        i0 = 0;
        cdiff = 0;
      } else {
        i0 = 13;
        cdiff = next->size[1];
      }

      ndbl = transitionArray->size[0] * transitionArray->size[1];
      transitionArray->size[0] = 1;
      transitionArray->size[1] = cdiff - i0;
      emxEnsureCapacity((emxArray__common *)transitionArray, ndbl, (int)sizeof
                        (double));
      apnd = cdiff - i0;
      for (cdiff = 0; cdiff < apnd; cdiff++) {
        transitionArray->data[transitionArray->size[0] * cdiff] = next->data[i0
          + cdiff];
      }
    }

    if (path->size[0] < 1) {
      absb = -1;
      apnd = 0;
    } else {
      ndbl = (int)floor(((double)path->size[0] - 1.0) + 0.5);
      apnd = ndbl + 1;
      cdiff = (ndbl - path->size[0]) + 1;
      absb = path->size[0];
      if (fabs((double)cdiff) < 4.4408920985006262E-16 * (double)absb) {
        ndbl++;
        apnd = path->size[0];
      } else if (cdiff > 0) {
        apnd = ndbl;
      } else {
        ndbl++;
      }

      absb = ndbl - 1;
    }

    i0 = next->size[0] * next->size[1];
    next->size[0] = 1;
    next->size[1] = absb + 1;
    emxEnsureCapacity((emxArray__common *)next, i0, (int)sizeof(double));
    if (absb + 1 > 0) {
      next->data[0] = 1.0;
      if (absb + 1 > 1) {
        next->data[absb] = apnd;
        ndbl = absb / 2;
        for (cdiff = 1; cdiff < ndbl; cdiff++) {
          next->data[cdiff] = 1.0 + (double)cdiff;
          next->data[absb - cdiff] = apnd - cdiff;
        }

        if (ndbl << 1 == absb) {
          next->data[ndbl] = (1.0 + (double)apnd) / 2.0;
        } else {
          next->data[ndbl] = 1.0 + (double)ndbl;
          next->data[ndbl + 1] = apnd - ndbl;
        }
      }
    }

    i0 = t->size[0];
    t->size[0] = next->size[1];
    emxEnsureCapacity((emxArray__common *)t, i0, (int)sizeof(double));
    apnd = next->size[1];
    for (i0 = 0; i0 < apnd; i0++) {
      t->data[i0] = dt * next->data[next->size[0] * i0];
    }

    ndbl = t->size[0];
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = ndbl;
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < ndbl; i0++) {
      pathJ->data[i0] = t->data[i0];
    }

    for (i0 = 0; i0 < 10; i0++) {
      apnd = path->size[0];
      for (cdiff = 0; cdiff < apnd; cdiff++) {
        pathJ->data[cdiff + pathJ->size[0] * (i0 + 1)] = path->data[cdiff +
          path->size[0] * i0];
      }
    }

    i0 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
    apnd = b_T->size[0] * b_T->size[1];
    for (i0 = 0; i0 < apnd; i0++) {
      T->data[i0] = b_T->data[i0];
    }

    // Transform path back to the Cartesian space.
    ndbl = pathJ->size[0];
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = ndbl;
    pathC->size[1] = 9;
    emxEnsureCapacity((emxArray__common *)pathC, i0, (int)sizeof(double));
    apnd = pathJ->size[0] * 9;
    for (i0 = 0; i0 < apnd; i0++) {
      pathC->data[i0] = 0.0;
    }

    transitionArrayLength = 0.0;
    for (absb = 0; absb < pathJ->size[0]; absb++) {
      // sherpaTTFK Sherpa_TT Forward Kinematics
      //    Calculates the x,y,z position of the contact point given the alpha,
      //    beta and gamma joint values.
      // SHERPATTFK Calcluates the Cartesian position of the wheel contact point 
      // relative to the pan coordinate frame for the SherpaTT Leg.
      //
      // Inputs:
      // -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] 
      // -kC: A struct containing the kinematic constants of the SherpaTT leg.
      // Outputs:
      //
      // sherpaTTFK.m
      // author: wreid
      // date: 20150122
      b_newState[0] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[absb + (pathJ->
        size[0] << 1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(pathJ->
        data[absb + pathJ->size[0] * 3] + kC->zeta)) - kC->l7) * cos(pathJ->
        data[absb + pathJ->size[0]]);
      b_newState[1] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[absb + (pathJ->
        size[0] << 1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(pathJ->
        data[absb + pathJ->size[0] * 3] + kC->zeta)) - kC->l7) * sin(pathJ->
        data[absb + pathJ->size[0]]);
      b_newState[2] = ((((kC->l1 + kC->l3 * sin(-pathJ->data[absb + (pathJ->
        size[0] << 1)])) - kC->l4 * sin(kC->zeta)) - kC->l5 * sin(pathJ->
        data[absb + pathJ->size[0] * 3] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

      // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
      // sherpaTTFKVel.m
      // author: wreid
      // date: 20150122
      for (i0 = 0; i0 < 3; i0++) {
        newState = 0.0;
        for (cdiff = 0; cdiff < 3; cdiff++) {
          newState += TP2B[i0 + (cdiff << 2)] * b_newState[cdiff];
        }

        uB[i0] = newState + TP2B[12 + i0];
      }

      if (1 + absb != 1) {
        for (i0 = 0; i0 < 3; i0++) {
          c_TP2B[i0] = uB[i0] - pathC->data[(absb + pathC->size[0] * (2 + i0)) -
            1];
        }

        transitionArrayLength += norm(c_TP2B);
      }

      b_pathJ[0] = (-pathJ->data[absb + pathJ->size[0] * 6] * sin(pathJ->
        data[absb + pathJ->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
        (pathJ->data[absb + pathJ->size[0] * 3] + kC->zeta)) + kC->l3 * cos
        (pathJ->data[absb + (pathJ->size[0] << 1)])) + kC->l4 * cos(kC->zeta)) -
                    pathJ->data[absb + pathJ->size[0] * 7] * kC->l3 * cos
                    (pathJ->data[absb + pathJ->size[0]]) * sin(pathJ->data[absb
        + (pathJ->size[0] << 1)])) - pathJ->data[absb + (pathJ->size[0] << 3)] *
        kC->l5 * sin(pathJ->data[absb + pathJ->size[0] * 3] + kC->zeta) * cos
        (pathJ->data[absb + pathJ->size[0]]);
      b_pathJ[1] = (pathJ->data[absb + pathJ->size[0] * 6] * cos(pathJ->
        data[absb + pathJ->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
        (pathJ->data[absb + pathJ->size[0] * 3] + kC->zeta)) + kC->l3 * cos
        (pathJ->data[absb + (pathJ->size[0] << 1)])) + kC->l4 * cos(kC->zeta)) -
                    pathJ->data[absb + (pathJ->size[0] << 3)] * kC->l5 * sin
                    (pathJ->data[absb + pathJ->size[0] * 3] + kC->zeta) * sin
                    (pathJ->data[absb + pathJ->size[0]])) - pathJ->data[absb +
        pathJ->size[0] * 7] * kC->l3 * sin(pathJ->data[absb + pathJ->size[0]]) *
        sin(pathJ->data[absb + (pathJ->size[0] << 1)]);
      b_pathJ[2] = -pathJ->data[absb + pathJ->size[0] * 7] * kC->l3 * cos
        (pathJ->data[absb + (pathJ->size[0] << 1)]) - kC->l5 * pathJ->data[absb
        + (pathJ->size[0] << 3)] * cos(kC->zeta + pathJ->data[absb + pathJ->
        size[0] * 3]);
      for (i0 = 0; i0 < 3; i0++) {
        c_TP2B[i0] = 0.0;
        for (cdiff = 0; cdiff < 3; cdiff++) {
          c_TP2B[i0] += TP2B[i0 + (cdiff << 2)] * b_pathJ[cdiff];
        }
      }

      pathC->data[absb] = pathJ->data[absb];
      pathC->data[absb + pathC->size[0]] = transitionArrayLength;
      for (i0 = 0; i0 < 3; i0++) {
        pathC->data[absb + pathC->size[0] * (i0 + 2)] = uB[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        pathC->data[absb + pathC->size[0] * (i0 + 5)] = c_TP2B[i0];
      }

      pathC->data[absb + (pathC->size[0] << 3)] = 0.0;
    }

    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = pathC->size[0];
    b_pathC->size[1] = 9;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    apnd = pathC->size[0] * pathC->size[1];
    for (i0 = 0; i0 < apnd; i0++) {
      b_pathC->data[i0] = pathC->data[i0];
    }
  } else {
    *success = false;
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = 0;
    b_pathC->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    i0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&transitionPath);
  emxFree_real_T(&transitionArray);
  emxFree_real_T(&next);
  emxFree_real_T(&b_T);

  // Linearly interpolate to the goal state from the final state.
  sGoalC[0] = 0.0;
  sGoalC[1] = 0.0;
  for (i0 = 0; i0 < 6; i0++) {
    sGoalC[i0 + 2] = nGoalCartesianB[i0];
  }

  sGoalC[8] = 1.0;

  // LININTERP Returns a Cartesian path that is linearly interpolated between
  // two points.
  //
  // Inputs:
  // s1: The initial state that is represented by a 1x8 vector. The vector has
  // the form [t x y z xDot yDot zDot interpBoolean]
  // s2: The goal state that is represented by a 1x8 vector. The vector has
  // the form [t x y z xDot yDot zDot interpBoolean]
  // N: The number of waypoints to be placed in the path.
  //
  // Outputs:
  // path: A 8xN matrix that returns a path from s2 to s1.
  //
  // linInterp.m
  // author: wreid
  // date: 20150224
  ndbl = b_pathC->size[0];
  transitionArrayLength = b_pathC->data[(ndbl + b_pathC->size[0]) - 1];
  apnd = b_pathC->size[1];
  ndbl = b_pathC->size[0];
  for (i0 = 0; i0 < apnd; i0++) {
    b_TP2B[i0] = b_pathC->data[(ndbl + b_pathC->size[0] * i0) - 1];
  }

  for (i0 = 0; i0 < 3; i0++) {
    qGoal[i0] = b_TP2B[i0 + 2];
    qInit[i0] = b_TP2B[i0 + 2];
  }

  for (absb = 0; absb < 10; absb++) {
    panHeight = (1.0 + (double)absb) / 10.0;
    for (i0 = 0; i0 < 3; i0++) {
      newState = qInit[i0] + panHeight * (sGoalC[2 + i0] - qInit[i0]);
      c_TP2B[i0] = newState - qGoal[i0];
      b_newState[i0] = newState;
    }

    transitionArrayLength += norm(c_TP2B);
    ndbl = b_pathC->size[0];
    pathCorrection[absb] = b_pathC->data[ndbl - 1] + (1.0 + (double)absb) / 10.0;
    pathCorrection[10 + absb] = transitionArrayLength;
    for (i0 = 0; i0 < 3; i0++) {
      pathCorrection[absb + 10 * (i0 + 2)] = b_newState[i0];
      qGoal[i0] = b_newState[i0];
    }

    pathCorrection[50 + absb] = 0.0;
    pathCorrection[60 + absb] = 0.0;
    pathCorrection[70 + absb] = 0.0;
    pathCorrection[80 + absb] = 1.0;
  }

  i0 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = b_pathC->size[0] + 10;
  pathC->size[1] = b_pathC->size[1];
  emxEnsureCapacity((emxArray__common *)pathC, i0, (int)sizeof(double));
  apnd = b_pathC->size[1];
  for (i0 = 0; i0 < apnd; i0++) {
    ndbl = b_pathC->size[0];
    for (cdiff = 0; cdiff < ndbl; cdiff++) {
      pathC->data[cdiff + pathC->size[0] * i0] = b_pathC->data[cdiff +
        b_pathC->size[0] * i0];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (cdiff = 0; cdiff < 10; cdiff++) {
      pathC->data[(cdiff + b_pathC->size[0]) + pathC->size[0] * i0] =
        pathCorrection[cdiff + 10 * i0];
    }
  }

  emxFree_real_T(&b_pathC);
  emxInit_real_T(&c_pathJ, 2);
  for (absb = 0; absb < 10; absb++) {
    for (i0 = 0; i0 < 3; i0++) {
      newState = 0.0;
      for (cdiff = 0; cdiff < 3; cdiff++) {
        newState += TB2P[i0 + (cdiff << 2)] * pathCorrection[absb + 10 * (2 +
          cdiff)];
      }

      b_TB2P[i0] = newState + TB2P[12 + i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_TP2B[i0] = b_TB2P[i0];
    }

    b_sherpaTTIK(c_TP2B, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
                 kC->l8, kC->zeta, kC->r, jointLimits, qInit);
    i0 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = pathJ->size[0] + 1;
    c_pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)c_pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 11; i0++) {
      apnd = pathJ->size[0];
      for (cdiff = 0; cdiff < apnd; cdiff++) {
        c_pathJ->data[cdiff + c_pathJ->size[0] * i0] = pathJ->data[cdiff +
          pathJ->size[0] * i0];
      }
    }

    c_pathJ->data[pathJ->size[0]] = pathCorrection[absb];
    for (i0 = 0; i0 < 3; i0++) {
      c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * (i0 + 1)] = qInit[i0];
    }

    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 2)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 5] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 6] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 7] = 0.0;
    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 3)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 9] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 10] = 0.0;
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = c_pathJ->size[0];
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 11; i0++) {
      apnd = c_pathJ->size[0];
      for (cdiff = 0; cdiff < apnd; cdiff++) {
        pathJ->data[cdiff + pathJ->size[0] * i0] = c_pathJ->data[cdiff +
          c_pathJ->size[0] * i0];
      }
    }
  }

  emxFree_real_T(&c_pathJ);
}

//
// Arguments    : void
// Return Type  : void
//
void buildRRTWrapper_init()
{
  int i8;
  static const double dv12[4] = { -0.293, -1.1326, -0.671, -0.7546 };

  static const double dv13[3] = { 1.0, 0.0, 0.5 };

  for (i8 = 0; i8 < 4; i8++) {
    cartesianLimits[i8] = dv12[i8];
  }

  for (i8 = 0; i8 < 3; i8++) {
    HGAINS[i8] = dv13[i8];
  }
}

//
// File trailer for buildRRTWrapper.cpp
//
// [EOF]
//
