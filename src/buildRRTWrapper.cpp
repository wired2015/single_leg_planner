//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 03-Mar-2015 11:19:40
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "randomState.h"
#include "sherpaTTPlanner_emxutil.h"
#include "norm.h"
#include "sherpaTTIK.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double nInitCartesianB[6]
//                const double nGoalCartesianB[6]
//                double phiInit
//                double omegaInit
//                const double jointLimits[20]
//                double bodyHeight
//                const struct0_T *kC
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
                     double jointLimits[20], double bodyHeight, const struct0_T *
                     kC, int legNum, const double uBDot[6], emxArray_real_T *T,
                     emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T
                     *success)
{
  double panHeight;
  double TP2B[16];
  int i20;
  static const signed char iv6[4] = { 0, 0, 0, 1 };

  double b_TP2B[9];
  int absb;
  double c_TP2B[3];
  double TB2P[16];
  double b_TB2P[3];
  double nodeIDCount;
  double qInit[3];
  double qGoal[3];
  double nInitJoint[10];
  double uB[3];
  double c_TB2P[3];
  double nGoalJoint[10];
  emxArray_real_T *b_pathC;
  emxArray_real_T *transitionPath;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *y;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  static double b_T[139500];
  int ndbl;
  double xRand[13];
  double check;
  double transitionArrayNearest[80];
  double xNear[13];
  double xNew[13];
  double dv16[13];
  double next_data[93];
  double transitionArray_data[80];
  int cdiff;
  int apnd;
  double newState[3];
  double b_pathJ[3];
  double sFinalC_data[9];
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
  // -kC:
  // -legNum:
  // -uBDot:
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
  for (i20 = 0; i20 < 4; i20++) {
    TP2B[3 + (i20 << 2)] = iv6[i20];
  }

  for (i20 = 0; i20 < 3; i20++) {
    for (absb = 0; absb < 3; absb++) {
      b_TP2B[absb + 3 * i20] = -TP2B[i20 + (absb << 2)];
    }
  }

  for (i20 = 0; i20 < 3; i20++) {
    c_TP2B[i20] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TP2B[i20] += b_TP2B[i20 + 3 * absb] * TP2B[12 + absb];
    }

    for (absb = 0; absb < 3; absb++) {
      TB2P[absb + (i20 << 2)] = TP2B[i20 + (absb << 2)];
    }
  }

  for (i20 = 0; i20 < 3; i20++) {
    TB2P[12 + i20] = c_TP2B[i20];
  }

  for (i20 = 0; i20 < 4; i20++) {
    TB2P[3 + (i20 << 2)] = iv6[i20];
  }

  // inv(TP2B);%
  // Transform the Cartesian goal and final positions in the pan coordinate
  // frame to the joint space.
  for (i20 = 0; i20 < 3; i20++) {
    nodeIDCount = 0.0;
    for (absb = 0; absb < 3; absb++) {
      nodeIDCount += TB2P[i20 + (absb << 2)] * nInitCartesianB[absb];
    }

    b_TB2P[i20] = nodeIDCount + TB2P[12 + i20];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i20 = 0; i20 < 3; i20++) {
    nodeIDCount = 0.0;
    for (absb = 0; absb < 3; absb++) {
      nodeIDCount += TB2P[i20 + (absb << 2)] * nGoalCartesianB[absb];
    }

    b_TB2P[i20] = nodeIDCount + TB2P[12 + i20];
  }

  sherpaTTIK(b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i20 = 0; i20 < 3; i20++) {
    c_TP2B[i20] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TP2B[i20] += TB2P[i20 + (absb << 2)] * nInitCartesianB[3 + absb];
    }

    b_TB2P[i20] = c_TP2B[i20];
    nInitJoint[i20] = qInit[i20];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nInitJoint[3] = phiInit;
  nInitJoint[4] = 0.0;
  for (i20 = 0; i20 < 3; i20++) {
    nInitJoint[i20 + 5] = uB[i20];
    c_TB2P[i20] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TB2P[i20] += TB2P[i20 + (absb << 2)] * nGoalCartesianB[3 + absb];
    }

    b_TB2P[i20] = c_TB2P[i20];
    nGoalJoint[i20] = qGoal[i20];
  }

  nInitJoint[8] = 0.0;
  nInitJoint[9] = omegaInit;
  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nGoalJoint[3] = 0.0;
  nGoalJoint[4] = 0.0;
  for (i20 = 0; i20 < 3; i20++) {
    nGoalJoint[i20 + 5] = uB[i20];
  }

  nGoalJoint[8] = 0.0;
  nGoalJoint[9] = 0.0;

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  emxInit_real_T(&b_pathC, 2);
  emxInit_real_T(&transitionPath, 2);
  b_emxInit_real_T(&t, 1);
  emxInit_real_T(&path, 2);
  emxInit_real_T(&y, 2);
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
    // Variable Initialization
    memset(&b_T[0], 0, 139500U * sizeof(double));

    // Define a zero array that will be used to
    // store data from each tree node.
    b_T[0] = 1.0;
    b_T[1500] = 0.0;
    b_T[3000] = 0.0;
    for (i20 = 0; i20 < 10; i20++) {
      b_T[1500 * (i20 + 3)] = nInitJoint[i20];
    }

    for (i20 = 0; i20 < 80; i20++) {
      b_T[1500 * (i20 + 13)] = 0.0;
    }

    // Initialize the tree with initial state.
    nodeIDCount = 1.0;
    for (ndbl = 0; ndbl < 1499; ndbl++) {
      randomState(jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                  kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);
      nearestNeighbour(xRand, b_T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                       kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount,
                       xNear, transitionArrayNearest, &check);
      selectInput(xNear, xRand, kC, 0.087266462599716474, jointLimits, uBDot,
                  legNum, xNew, transitionArrayNearest);
      xNew[0] = nodeIDCount + 1.0;

      // Node ID
      xNew[1] = xNear[0];

      // Parent ID
      xNew[2] = xNear[2] + b_heuristicSingleLeg(xNew, xNear, kC->l1, kC->l2,
        kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);

      // Cost
      for (i20 = 0; i20 < 13; i20++) {
        b_T[((int)(nodeIDCount + 1.0) + 1500 * i20) - 1] = xNew[i20];
      }

      for (i20 = 0; i20 < 80; i20++) {
        b_T[((int)(nodeIDCount + 1.0) + 1500 * (i20 + 13)) - 1] =
          transitionArrayNearest[i20];
      }

      // Append the new node to the tree.
      // if mod(nodeIDCount,100) == 0
      // fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
      // end
      nodeIDCount++;
    }

    // Find the closest node in the tree to the goal node.
    dv16[0] = 0.0;
    dv16[1] = 0.0;
    dv16[2] = 0.0;
    memcpy(&dv16[3], &nGoalJoint[0], 10U * sizeof(double));
    nearestNeighbour(dv16, b_T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                     kC->l7, kC->l8, kC->zeta, kC->r, nodeIDCount, xRand,
                     transitionArrayNearest, &check);
    check = xRand[0];
    memcpy(&next_data[0], &xRand[0], 13U * sizeof(double));
    i20 = path->size[0] * path->size[1];
    path->size[0] = 0;
    path->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)path, i20, (int)sizeof(double));
    memcpy(&transitionArray_data[0], &transitionArrayNearest[0], 80U * sizeof
           (double));

    // Iterate over the tree until the initial state has been found.
    while ((check != 0.0) && (next_data[1] != 0.0)) {
      i20 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = 0;
      transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)transitionPath, i20, (int)sizeof
                        (double));
      for (ndbl = 0; ndbl < 8; ndbl++) {
        cdiff = ndbl * 10;
        i20 = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)c_transitionPath, i20, (int)sizeof
                          (double));
        for (i20 = 0; i20 < 10; i20++) {
          apnd = transitionPath->size[0];
          for (absb = 0; absb < apnd; absb++) {
            c_transitionPath->data[absb + c_transitionPath->size[0] * i20] =
              transitionPath->data[absb + transitionPath->size[0] * i20];
          }
        }

        for (i20 = 0; i20 < 10; i20++) {
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * i20] = transitionArray_data[i20 + cdiff];
        }

        i20 = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)transitionPath, i20, (int)sizeof
                          (double));
        for (i20 = 0; i20 < 10; i20++) {
          apnd = c_transitionPath->size[0];
          for (absb = 0; absb < apnd; absb++) {
            transitionPath->data[absb + transitionPath->size[0] * i20] =
              c_transitionPath->data[absb + c_transitionPath->size[0] * i20];
          }
        }
      }

      i20 = b_transitionPath->size[0] * b_transitionPath->size[1];
      b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
      b_transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)b_transitionPath, i20, (int)sizeof
                        (double));
      for (i20 = 0; i20 < 10; i20++) {
        apnd = transitionPath->size[0];
        for (absb = 0; absb < apnd; absb++) {
          b_transitionPath->data[absb + b_transitionPath->size[0] * i20] =
            transitionPath->data[absb + transitionPath->size[0] * i20];
        }
      }

      for (i20 = 0; i20 < 10; i20++) {
        apnd = path->size[0];
        for (absb = 0; absb < apnd; absb++) {
          b_transitionPath->data[(absb + transitionPath->size[0]) +
            b_transitionPath->size[0] * i20] = path->data[absb + path->size[0] *
            i20];
        }
      }

      i20 = path->size[0] * path->size[1];
      path->size[0] = b_transitionPath->size[0];
      path->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)path, i20, (int)sizeof(double));
      for (i20 = 0; i20 < 10; i20++) {
        apnd = b_transitionPath->size[0];
        for (absb = 0; absb < apnd; absb++) {
          path->data[absb + path->size[0] * i20] = b_transitionPath->data[absb +
            b_transitionPath->size[0] * i20];
        }
      }

      check = next_data[1];
      for (i20 = 0; i20 < 93; i20++) {
        next_data[i20] = b_T[((int)check + 1500 * i20) - 1];
      }

      check = next_data[1];
      for (i20 = 0; i20 < 80; i20++) {
        transitionArray_data[i20] = next_data[13 + i20];
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

    i20 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = absb + 1;
    emxEnsureCapacity((emxArray__common *)y, i20, (int)sizeof(double));
    if (absb + 1 > 0) {
      y->data[0] = 1.0;
      if (absb + 1 > 1) {
        y->data[absb] = apnd;
        ndbl = absb / 2;
        for (cdiff = 1; cdiff < ndbl; cdiff++) {
          y->data[cdiff] = 1.0 + (double)cdiff;
          y->data[absb - cdiff] = apnd - cdiff;
        }

        if (ndbl << 1 == absb) {
          y->data[ndbl] = (1.0 + (double)apnd) / 2.0;
        } else {
          y->data[ndbl] = 1.0 + (double)ndbl;
          y->data[ndbl + 1] = apnd - ndbl;
        }
      }
    }

    i20 = t->size[0];
    t->size[0] = y->size[1];
    emxEnsureCapacity((emxArray__common *)t, i20, (int)sizeof(double));
    apnd = y->size[1];
    for (i20 = 0; i20 < apnd; i20++) {
      t->data[i20] = 0.1 * y->data[y->size[0] * i20];
    }

    ndbl = t->size[0];
    i20 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = ndbl;
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i20, (int)sizeof(double));
    for (i20 = 0; i20 < ndbl; i20++) {
      pathJ->data[i20] = t->data[i20];
    }

    for (i20 = 0; i20 < 10; i20++) {
      apnd = path->size[0];
      for (absb = 0; absb < apnd; absb++) {
        pathJ->data[absb + pathJ->size[0] * (i20 + 1)] = path->data[absb +
          path->size[0] * i20];
      }
    }

    i20 = T->size[0] * T->size[1];
    T->size[0] = 1500;
    T->size[1] = 93;
    emxEnsureCapacity((emxArray__common *)T, i20, (int)sizeof(double));
    for (i20 = 0; i20 < 139500; i20++) {
      T->data[i20] = b_T[i20];
    }

    // Transform path back to the Cartesian space.
    ndbl = pathJ->size[0];
    i20 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = ndbl;
    pathC->size[1] = 9;
    emxEnsureCapacity((emxArray__common *)pathC, i20, (int)sizeof(double));
    apnd = pathJ->size[0] * 9;
    for (i20 = 0; i20 < apnd; i20++) {
      pathC->data[i20] = 0.0;
    }

    check = 0.0;
    for (ndbl = 0; ndbl < pathJ->size[0]; ndbl++) {
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
      newState[0] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[ndbl + (pathJ->size[0]
        << 1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(pathJ->data[ndbl +
        pathJ->size[0] * 3] + kC->zeta)) - kC->l7) * cos(pathJ->data[ndbl +
        pathJ->size[0]]);
      newState[1] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[ndbl + (pathJ->size[0]
        << 1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(pathJ->data[ndbl +
        pathJ->size[0] * 3] + kC->zeta)) - kC->l7) * sin(pathJ->data[ndbl +
        pathJ->size[0]]);
      newState[2] = ((((kC->l1 + kC->l3 * sin(-pathJ->data[ndbl + (pathJ->size[0]
        << 1)])) - kC->l4 * sin(kC->zeta)) - kC->l5 * sin(pathJ->data[ndbl +
        pathJ->size[0] * 3] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

      // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
      // sherpaTTFKVel.m
      // author: wreid
      // date: 20150122
      for (i20 = 0; i20 < 3; i20++) {
        nodeIDCount = 0.0;
        for (absb = 0; absb < 3; absb++) {
          nodeIDCount += TP2B[i20 + (absb << 2)] * newState[absb];
        }

        uB[i20] = nodeIDCount + TP2B[12 + i20];
      }

      if (1 + ndbl != 1) {
        for (i20 = 0; i20 < 3; i20++) {
          c_TP2B[i20] = uB[i20] - pathC->data[(ndbl + pathC->size[0] * (2 + i20))
            - 1];
        }

        check += norm(c_TP2B);
      }

      b_pathJ[0] = (-pathJ->data[ndbl + pathJ->size[0] * 6] * sin(pathJ->
        data[ndbl + pathJ->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
        (pathJ->data[ndbl + pathJ->size[0] * 3] + kC->zeta)) + kC->l3 * cos
        (pathJ->data[ndbl + (pathJ->size[0] << 1)])) + kC->l4 * cos(kC->zeta)) -
                    pathJ->data[ndbl + pathJ->size[0] * 7] * kC->l3 * cos
                    (pathJ->data[ndbl + pathJ->size[0]]) * sin(pathJ->data[ndbl
        + (pathJ->size[0] << 1)])) - pathJ->data[ndbl + (pathJ->size[0] << 3)] *
        kC->l5 * sin(pathJ->data[ndbl + pathJ->size[0] * 3] + kC->zeta) * cos
        (pathJ->data[ndbl + pathJ->size[0]]);
      b_pathJ[1] = (pathJ->data[ndbl + pathJ->size[0] * 6] * cos(pathJ->
        data[ndbl + pathJ->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
        (pathJ->data[ndbl + pathJ->size[0] * 3] + kC->zeta)) + kC->l3 * cos
        (pathJ->data[ndbl + (pathJ->size[0] << 1)])) + kC->l4 * cos(kC->zeta)) -
                    pathJ->data[ndbl + (pathJ->size[0] << 3)] * kC->l5 * sin
                    (pathJ->data[ndbl + pathJ->size[0] * 3] + kC->zeta) * sin
                    (pathJ->data[ndbl + pathJ->size[0]])) - pathJ->data[ndbl +
        pathJ->size[0] * 7] * kC->l3 * sin(pathJ->data[ndbl + pathJ->size[0]]) *
        sin(pathJ->data[ndbl + (pathJ->size[0] << 1)]);
      b_pathJ[2] = -pathJ->data[ndbl + pathJ->size[0] * 7] * kC->l3 * cos
        (pathJ->data[ndbl + (pathJ->size[0] << 1)]) - kC->l5 * pathJ->data[ndbl
        + (pathJ->size[0] << 3)] * cos(kC->zeta + pathJ->data[ndbl + pathJ->
        size[0] * 3]);
      for (i20 = 0; i20 < 3; i20++) {
        c_TP2B[i20] = 0.0;
        for (absb = 0; absb < 3; absb++) {
          c_TP2B[i20] += TP2B[i20 + (absb << 2)] * b_pathJ[absb];
        }
      }

      pathC->data[ndbl] = pathJ->data[ndbl];
      pathC->data[ndbl + pathC->size[0]] = check;
      for (i20 = 0; i20 < 3; i20++) {
        pathC->data[ndbl + pathC->size[0] * (i20 + 2)] = uB[i20];
      }

      for (i20 = 0; i20 < 3; i20++) {
        pathC->data[ndbl + pathC->size[0] * (i20 + 5)] = c_TP2B[i20];
      }

      pathC->data[ndbl + (pathC->size[0] << 3)] = 0.0;
    }

    i20 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = pathC->size[0];
    b_pathC->size[1] = 9;
    emxEnsureCapacity((emxArray__common *)b_pathC, i20, (int)sizeof(double));
    apnd = pathC->size[0] * pathC->size[1];
    for (i20 = 0; i20 < apnd; i20++) {
      b_pathC->data[i20] = pathC->data[i20];
    }
  } else {
    *success = false;
    i20 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = 0;
    b_pathC->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)b_pathC, i20, (int)sizeof(double));
    i20 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i20, (int)sizeof(double));
    i20 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T, i20, (int)sizeof(double));
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&y);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&transitionPath);

  // Linearly interpolate to the goal state from the final state.
  if ((b_pathC->size[0] == 0) || (b_pathC->size[1] == 0)) {
    b_TP2B[0] = 0.0;
    b_TP2B[1] = 0.0;
    for (i20 = 0; i20 < 6; i20++) {
      b_TP2B[i20 + 2] = nInitCartesianB[i20];
    }

    b_TP2B[8] = 1.0;
    for (i20 = 0; i20 < 9; i20++) {
      sFinalC_data[i20] = b_TP2B[i20];
    }
  } else {
    apnd = b_pathC->size[1];
    ndbl = b_pathC->size[0];
    for (i20 = 0; i20 < apnd; i20++) {
      sFinalC_data[i20] = b_pathC->data[(ndbl + b_pathC->size[0] * i20) - 1];
    }
  }

  sGoalC[0] = 0.0;
  sGoalC[1] = 0.0;
  for (i20 = 0; i20 < 6; i20++) {
    sGoalC[i20 + 2] = nGoalCartesianB[i20];
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
  check = sFinalC_data[1];
  for (i20 = 0; i20 < 3; i20++) {
    qGoal[i20] = sFinalC_data[i20 + 2];
    qInit[i20] = sFinalC_data[i20 + 2];
  }

  for (ndbl = 0; ndbl < 10; ndbl++) {
    nodeIDCount = (1.0 + (double)ndbl) / 10.0;
    for (i20 = 0; i20 < 3; i20++) {
      panHeight = qInit[i20] + nodeIDCount * (sGoalC[2 + i20] - qInit[i20]);
      c_TP2B[i20] = panHeight - qGoal[i20];
      newState[i20] = panHeight;
    }

    check += norm(c_TP2B);
    pathCorrection[ndbl] = sFinalC_data[0] + (1.0 + (double)ndbl) / 10.0;
    pathCorrection[10 + ndbl] = check;
    for (i20 = 0; i20 < 3; i20++) {
      pathCorrection[ndbl + 10 * (i20 + 2)] = newState[i20];
      qGoal[i20] = newState[i20];
    }

    pathCorrection[50 + ndbl] = 0.0;
    pathCorrection[60 + ndbl] = 0.0;
    pathCorrection[70 + ndbl] = 0.0;
    pathCorrection[80 + ndbl] = 1.0;
  }

  i20 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = b_pathC->size[0] + 10;
  pathC->size[1] = b_pathC->size[1];
  emxEnsureCapacity((emxArray__common *)pathC, i20, (int)sizeof(double));
  apnd = b_pathC->size[1];
  for (i20 = 0; i20 < apnd; i20++) {
    ndbl = b_pathC->size[0];
    for (absb = 0; absb < ndbl; absb++) {
      pathC->data[absb + pathC->size[0] * i20] = b_pathC->data[absb +
        b_pathC->size[0] * i20];
    }
  }

  for (i20 = 0; i20 < 9; i20++) {
    for (absb = 0; absb < 10; absb++) {
      pathC->data[(absb + b_pathC->size[0]) + pathC->size[0] * i20] =
        pathCorrection[absb + 10 * i20];
    }
  }

  emxFree_real_T(&b_pathC);
  emxInit_real_T(&c_pathJ, 2);
  for (ndbl = 0; ndbl < 10; ndbl++) {
    for (i20 = 0; i20 < 3; i20++) {
      nodeIDCount = 0.0;
      for (absb = 0; absb < 3; absb++) {
        nodeIDCount += TB2P[i20 + (absb << 2)] * pathCorrection[ndbl + 10 * (2 +
          absb)];
      }

      b_TB2P[i20] = nodeIDCount + TB2P[12 + i20];
    }

    for (i20 = 0; i20 < 3; i20++) {
      c_TP2B[i20] = b_TB2P[i20];
    }

    b_sherpaTTIK(c_TP2B, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
                 kC->l8, kC->zeta, kC->r, jointLimits, qInit);
    i20 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = pathJ->size[0] + 1;
    c_pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)c_pathJ, i20, (int)sizeof(double));
    for (i20 = 0; i20 < 11; i20++) {
      apnd = pathJ->size[0];
      for (absb = 0; absb < apnd; absb++) {
        c_pathJ->data[absb + c_pathJ->size[0] * i20] = pathJ->data[absb +
          pathJ->size[0] * i20];
      }
    }

    c_pathJ->data[pathJ->size[0]] = pathCorrection[ndbl];
    for (i20 = 0; i20 < 3; i20++) {
      c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * (i20 + 1)] = qInit[i20];
    }

    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 2)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 5] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 6] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 7] = 0.0;
    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 3)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 9] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 10] = 0.0;
    i20 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = c_pathJ->size[0];
    pathJ->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)pathJ, i20, (int)sizeof(double));
    for (i20 = 0; i20 < 11; i20++) {
      apnd = c_pathJ->size[0];
      for (absb = 0; absb < apnd; absb++) {
        pathJ->data[absb + pathJ->size[0] * i20] = c_pathJ->data[absb +
          c_pathJ->size[0] * i20];
      }
    }
  }

  emxFree_real_T(&c_pathJ);
}

//
// File trailer for buildRRTWrapper.cpp
//
// [EOF]
//
