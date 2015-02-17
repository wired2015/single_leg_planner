//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 17-Feb-2015 14:05:36
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_emxutil.h"
#include "flipud.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Variable Definitions
static double HGAINS[3];
static double cartesianLimits[4];
static boolean_T cartesianLimits_not_empty;

// Function Definitions

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
//                const double uBDot[6]
//                emxArray_real_T *T
//                emxArray_real_T *pathC
//                emxArray_real_T *pathJ
//                boolean_T *success
// Return Type  : void
//
void buildRRTWrapper(const double nInitCartesianB[6], const double
                     nGoalCartesianB[6], const double jointLimits[12], double
                     bodyHeight, const double U[10], double dt, double Dt, const
                     struct0_T *kC, double, int legNum, const double uBDot[6],
                     emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T
                     *pathJ, boolean_T *success)
{
  int i0;
  double panHeight;
  double TP2B[16];
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
  emxArray_real_T *d;
  emxArray_real_T *c_pathC;
  emxArray_real_T *c_pathJ;
  emxArray_real_T *d_pathJ;
  double nGoal[11];
  double transitionArrayLength;
  int loop_ub;
  double nodeIDCount;
  int ixstart;
  double gammaDot;
  int itmp;
  boolean_T exitg1;
  double xNearest_data[11];
  unsigned int count;
  double uP[3];
  double alphaDot[3];

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
  if (!cartesianLimits_not_empty) {
    for (i0 = 0; i0 < 4; i0++) {
      cartesianLimits[i0] = 0.0;
    }

    cartesianLimits_not_empty = true;

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
    cartesianLimits[0] = ((((kC->l1 + kC->l3 * sin(-jointLimits[2])) - kC->l4 *
      sin(kC->zeta)) - kC->l5 * sin(jointLimits[4] + kC->zeta)) - kC->l6) -
      (kC->l8 + kC->r);

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
    cartesianLimits[1] = ((((kC->l1 + kC->l3 * sin(-jointLimits[3])) - kC->l4 *
      sin(kC->zeta)) - kC->l5 * sin(jointLimits[5] + kC->zeta)) - kC->l6) -
      (kC->l8 + kC->r);

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
    cartesianLimits[2] = ((((kC->l1 + kC->l3 * sin(-jointLimits[2])) - kC->l4 *
      sin(kC->zeta)) - kC->l5 * sin(jointLimits[5] + kC->zeta)) - kC->l6) -
      (kC->l8 + kC->r);

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
    cartesianLimits[3] = ((((kC->l1 + kC->l3 * sin(-jointLimits[3])) - kC->l4 *
      sin(kC->zeta)) - kC->l5 * sin(jointLimits[4] + kC->zeta)) - kC->l6) -
      (kC->l8 + kC->r);
  }

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
  emxInit_real_T(&d, 2);
  emxInit_real_T(&c_pathC, 2);
  emxInit_real_T(&c_pathJ, 2);
  emxInit_real_T(&d_pathJ, 2);
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    // Run buildRRT.
    nGoal[0] = 0.0;
    nGoal[1] = 0.0;
    nGoal[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      nGoal[i0 + 3] = nGoalJoint[i0];
    }

    nGoal[9] = 0.0;
    nGoal[10] = 0.0;

    // buildRRT Icrementally builds a rapidly exploring random tree.
    //    An RRT is build by incrementally selecting a random state from the
    //    available state space as defined by the MIN and MAX vectors. The tree is 
    //    started at xInit and is extended until the number of maximum nodes, K has 
    //    been reached. A path is selected if the goal region as defined by xGoal 
    //    has been reached by the RRT.
    // buildRRT.m
    // author: wreid
    // date: 20150107
    // %TODO: Make a structure input for nInit and nGoal
    // %TEMPORARY definition of init and goal nodes.
    // nInit = makeNode(1,0,0,nInit(4:6),nInit(7:9),)
    // Constant Declaration                                                       
    transitionArrayLength = (rt_roundd_snf(Dt / dt) + 1.0) * 6.0;

    // Variable Initialization
    i0 = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1000;
    d0 = rt_roundd_snf(11.0 + transitionArrayLength);
    if (d0 < 2.147483648E+9) {
      if (d0 >= -2.147483648E+9) {
        i1 = (int)d0;
      } else {
        i1 = MIN_int32_T;
      }
    } else if (d0 >= 2.147483648E+9) {
      i1 = MAX_int32_T;
    } else {
      i1 = 0;
    }

    b_T->size[1] = i1;
    emxEnsureCapacity((emxArray__common *)b_T, i0, (int)sizeof(double));
    d0 = rt_roundd_snf(11.0 + transitionArrayLength);
    if (d0 < 2.147483648E+9) {
      if (d0 >= -2.147483648E+9) {
        i0 = (int)d0;
      } else {
        i0 = MIN_int32_T;
      }
    } else if (d0 >= 2.147483648E+9) {
      i0 = MAX_int32_T;
    } else {
      i0 = 0;
    }

    loop_ub = 1000 * i0;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_T->data[i0] = 0.0;
    }

    // Define a zero array that will be used to
    // store data from each tree node.
    b_T->data[0] = 0.0;
    b_T->data[b_T->size[0]] = 0.0;
    b_T->data[b_T->size[0] << 1] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      b_T->data[b_T->size[0] * (i0 + 3)] = nInitJoint[i0];
    }

    b_T->data[b_T->size[0] * 9] = 0.0;
    b_T->data[b_T->size[0] * 10] = 0.0;
    loop_ub = (int)transitionArrayLength;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_T->data[b_T->size[0] * (i0 + 11)] = 0.0;
    }

    // Initialize the tree with initial state.
    nodeIDCount = 1.0;
    for (ixstart = 0; ixstart < 999; ixstart++) {
      rrtLoop(b_T, jointLimits, cartesianLimits, kC, panHeight, U, Dt, dt,
              &nodeIDCount, nGoal, uBDot, legNum);
    }

    // Find the closest node in the tree to the goal node.
    // nearestNeigbour Finds the node in the tree closest to x.
    //    This function scans each node within the tree and finds the node that
    //    is closest to the xRand node. The nearest node is returned by the
    //    function. A distance heuristic is  used
    //    Inputs:
    //        x:  The 1xn state that each node in the tree will be compared to,
    //            to find the node with the minimum distance to it. n refers to
    //            the number of dimensions within the state space.
    //        T:  The nxm tree being searched, m is the number of possible nodes 
    //            within the tree.
    //        HGAINS: The gains applied to the heuristic function.
    //    Outputs:
    //        xNear:  The node in the tree that is closet to x.
    // nearestNeigbour.m
    // author: wreid
    // date: 20150107
    // Iterate over the entire tree and apply the distance heuristic function
    // to each node.
    i0 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = (int)nodeIDCount;
    emxEnsureCapacity((emxArray__common *)d, i0, (int)sizeof(double));
    loop_ub = (int)nodeIDCount;
    for (i0 = 0; i0 < loop_ub; i0++) {
      d->data[i0] = 0.0;
    }

    // parfor i = 1:nodeIDCount
    for (ixstart = 0; ixstart < (int)nodeIDCount; ixstart++) {
      // heuristic Calculates the distance between states x1 and x2.
      // heuristicSingleLeg.m
      // author: wreid
      // date: 20150107
      // Calculate the distance between angular positions.
      panHeight = (((kC->l2 + kC->l3 * cos(nGoal[4])) + kC->l4 * cos(kC->zeta))
                   + kC->l5 * cos(kC->zeta + nGoal[5])) - kC->l7;
      gammaDot = ((((kC->l2 + kC->l3 * cos(b_T->data[ixstart + (b_T->size[0] <<
        2)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(kC->zeta + b_T->
        data[ixstart + b_T->size[0] * 5])) - kC->l7) - panHeight;

      // angDiff Finds the angular difference between th1 and th2.
      transitionArrayLength = ((nGoal[3] - b_T->data[ixstart + b_T->size[0] * 3])
        + 3.1415926535897931) / 6.2831853071795862;
      if (fabs(transitionArrayLength - rt_roundd_snf(transitionArrayLength)) <=
          2.2204460492503131E-16 * fabs(transitionArrayLength)) {
        transitionArrayLength = 0.0;
      } else {
        transitionArrayLength = (transitionArrayLength - floor
          (transitionArrayLength)) * 6.2831853071795862;
      }

      transitionArrayLength = fabs(transitionArrayLength - 3.1415926535897931);

      // Calculate the total distance.
      // dPosNorm+dVelNorm
      d->data[ixstart] = sqrt(gammaDot * gammaDot + panHeight * panHeight *
        (transitionArrayLength * transitionArrayLength));
    }

    ixstart = 1;
    transitionArrayLength = d->data[0];
    itmp = 0;
    if ((int)nodeIDCount > 1) {
      if (rtIsNaN(transitionArrayLength)) {
        loop_ub = 2;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= (int)nodeIDCount)) {
          ixstart = loop_ub;
          if (!rtIsNaN(d->data[loop_ub - 1])) {
            transitionArrayLength = d->data[loop_ub - 1];
            itmp = loop_ub - 1;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }

      if (ixstart < (int)nodeIDCount) {
        while (ixstart + 1 <= (int)nodeIDCount) {
          if (d->data[ixstart] < transitionArrayLength) {
            transitionArrayLength = d->data[ixstart];
            itmp = ixstart;
          }

          ixstart++;
        }
      }
    }

    // [d,minIndex] = min(d(1:nodeIDCount));
    for (i0 = 0; i0 < 11; i0++) {
      xNearest_data[i0] = b_T->data[itmp + b_T->size[0] * i0];
    }

    if (12 > b_T->size[1]) {
      i0 = -11;
      i1 = 0;
    } else {
      i0 = 0;
      i1 = b_T->size[1];
    }

    transitionArrayLength = b_T->data[itmp];
    ixstart = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = 11;
    emxEnsureCapacity((emxArray__common *)d, ixstart, (int)sizeof(double));
    for (ixstart = 0; ixstart < 11; ixstart++) {
      d->data[ixstart] = xNearest_data[ixstart];
    }

    ixstart = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 1;
    pathJ->size[1] = i1 - i0;
    emxEnsureCapacity((emxArray__common *)pathJ, ixstart, (int)sizeof(double));
    for (ixstart = 0; ixstart < 11; ixstart++) {
      pathJ->data[pathJ->size[0] * ixstart] = xNearest_data[ixstart];
    }

    loop_ub = i1 - i0;
    for (i1 = 0; i1 <= loop_ub - 12; i1++) {
      pathJ->data[pathJ->size[0] * (i1 + 11)] = b_T->data[itmp + b_T->size[0] *
        ((i0 + i1) + 11)];
    }

    while ((transitionArrayLength != 0.0) && (d->data[1] != 0.0)) {
      transitionArrayLength = d->data[1];
      loop_ub = b_T->size[1];
      i0 = d->size[0] * d->size[1];
      d->size[0] = 1;
      d->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)d, i0, (int)sizeof(double));
      for (i0 = 0; i0 < loop_ub; i0++) {
        d->data[d->size[0] * i0] = b_T->data[((int)transitionArrayLength +
          b_T->size[0] * i0) - 1];
      }

      i0 = d_pathJ->size[0] * d_pathJ->size[1];
      d_pathJ->size[0] = pathJ->size[0] + 1;
      d_pathJ->size[1] = pathJ->size[1];
      emxEnsureCapacity((emxArray__common *)d_pathJ, i0, (int)sizeof(double));
      loop_ub = pathJ->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        ixstart = pathJ->size[0];
        for (i1 = 0; i1 < ixstart; i1++) {
          d_pathJ->data[i1 + d_pathJ->size[0] * i0] = pathJ->data[i1 +
            pathJ->size[0] * i0];
        }
      }

      loop_ub = d->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        d_pathJ->data[pathJ->size[0] + d_pathJ->size[0] * i0] = d->data[d->size
          [0] * i0];
      }

      i0 = pathJ->size[0] * pathJ->size[1];
      pathJ->size[0] = d_pathJ->size[0];
      pathJ->size[1] = d_pathJ->size[1];
      emxEnsureCapacity((emxArray__common *)pathJ, i0, (int)sizeof(double));
      loop_ub = d_pathJ->size[1];
      for (i0 = 0; i0 < loop_ub; i0++) {
        ixstart = d_pathJ->size[0];
        for (i1 = 0; i1 < ixstart; i1++) {
          pathJ->data[i1 + pathJ->size[0] * i0] = d_pathJ->data[i1 +
            d_pathJ->size[0] * i0];
        }
      }

      transitionArrayLength = d->data[1];
    }

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
    transitionArrayLength = rt_roundd_snf(Dt / dt);
    ixstart = (int)(transitionArrayLength * (double)pathJ->size[0]);
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = ixstart;
    b_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathC, i0, (int)sizeof(double));
    loop_ub = (int)(transitionArrayLength * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = 0.0;
    }

    transitionArrayLength = rt_roundd_snf(Dt / dt);
    ixstart = (int)(transitionArrayLength * (double)pathJ->size[0]);
    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = ixstart;
    b_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)b_pathJ, i0, (int)sizeof(double));
    loop_ub = (int)(transitionArrayLength * (double)pathJ->size[0]) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathJ->data[i0] = 0.0;
    }

    count = 1U;
    nodeIDCount = rt_roundd_snf(Dt / dt) * (double)pathJ->size[0] * dt;
    for (ixstart = 0; ixstart < pathJ->size[0]; ixstart++) {
      for (loop_ub = pathJ->size[1] - 6; loop_ub + 6 >= 18; loop_ub -= 6) {
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
        transitionArrayLength = pathJ->data[ixstart + pathJ->size[0] * loop_ub];
        uP[0] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[ixstart + pathJ->size[0] *
                     (1 + loop_ub)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos
                  (pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)] +
                   kC->zeta)) - kC->l7) * cos(transitionArrayLength);
        uP[1] = ((((kC->l2 + kC->l3 * cos(-pathJ->data[ixstart + pathJ->size[0] *
                     (1 + loop_ub)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos
                  (pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)] +
                   kC->zeta)) - kC->l7) * sin(transitionArrayLength);
        uP[2] = ((((kC->l1 + kC->l3 * sin(-pathJ->data[ixstart + pathJ->size[0] *
                     (1 + loop_ub)])) - kC->l4 * sin(kC->zeta)) - kC->l5 * sin
                  (pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)] +
                   kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

        // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
        // sherpaTTFKVel.m
        // author: wreid
        // date: 20150122
        transitionArrayLength = pathJ->data[ixstart + pathJ->size[0] * (3 +
          loop_ub)];
        panHeight = pathJ->data[ixstart + pathJ->size[0] * (4 + loop_ub)];
        gammaDot = pathJ->data[ixstart + pathJ->size[0] * (5 + loop_ub)];
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            d0 += TP2B[i0 + (i1 << 2)] * uP[i1];
          }

          c_TP2B[i0] = d0 + TP2B[12 + i0];
        }

        alphaDot[0] = (-transitionArrayLength * sin(pathJ->data[ixstart +
          pathJ->size[0] * loop_ub]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
          (pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)] + kC->zeta)) +
          kC->l3 * cos(pathJ->data[ixstart + pathJ->size[0] * (1 + loop_ub)])) +
          kC->l4 * cos(kC->zeta)) - panHeight * kC->l3 * cos(pathJ->data[ixstart
          + pathJ->size[0] * loop_ub]) * sin(pathJ->data[ixstart + pathJ->size[0]
          * (1 + loop_ub)])) - gammaDot * kC->l5 * sin(pathJ->data[ixstart +
          pathJ->size[0] * (2 + loop_ub)] + kC->zeta) * cos(pathJ->data[ixstart
          + pathJ->size[0] * loop_ub]);
        alphaDot[1] = (transitionArrayLength * cos(pathJ->data[ixstart +
          pathJ->size[0] * loop_ub]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
          (pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)] + kC->zeta)) +
          kC->l3 * cos(pathJ->data[ixstart + pathJ->size[0] * (1 + loop_ub)])) +
          kC->l4 * cos(kC->zeta)) - gammaDot * kC->l5 * sin(pathJ->data[ixstart
          + pathJ->size[0] * (2 + loop_ub)] + kC->zeta) * sin(pathJ->
          data[ixstart + pathJ->size[0] * loop_ub])) - panHeight * kC->l3 * sin
          (pathJ->data[ixstart + pathJ->size[0] * loop_ub]) * sin(pathJ->
          data[ixstart + pathJ->size[0] * (1 + loop_ub)]);
        alphaDot[2] = -panHeight * kC->l3 * cos(pathJ->data[ixstart +
          pathJ->size[0] * (1 + loop_ub)]) - kC->l5 * gammaDot * cos(kC->zeta +
          pathJ->data[ixstart + pathJ->size[0] * (2 + loop_ub)]);
        for (i0 = 0; i0 < 3; i0++) {
          b_TB2P[i0] = 0.0;
          for (i1 = 0; i1 < 3; i1++) {
            b_TB2P[i0] += TP2B[i0 + (i1 << 2)] * alphaDot[i1];
          }
        }

        b_pathC->data[(int)count - 1] = nodeIDCount;
        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 1)) - 1] =
            c_TP2B[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int)count + b_pathC->size[0] * (i0 + 4)) - 1] =
            b_TB2P[i0];
        }

        b_pathJ->data[(int)count - 1] = nodeIDCount;
        for (i0 = 0; i0 < 6; i0++) {
          b_pathJ->data[((int)count + b_pathJ->size[0] * (i0 + 1)) - 1] =
            pathJ->data[ixstart + pathJ->size[0] * (i0 + loop_ub)];
        }

        nodeIDCount -= dt;
        count++;
      }
    }

    ixstart = b_pathC->size[0];
    i0 = c_pathC->size[0] * c_pathC->size[1];
    c_pathC->size[0] = ixstart;
    c_pathC->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)c_pathC, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      for (i1 = 0; i1 < ixstart; i1++) {
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
    ixstart = b_pathJ->size[0];
    i0 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = ixstart;
    c_pathJ->size[1] = 7;
    emxEnsureCapacity((emxArray__common *)c_pathJ, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 7; i0++) {
      for (i1 = 0; i1 < ixstart; i1++) {
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

  emxFree_real_T(&d_pathJ);
  emxFree_real_T(&c_pathJ);
  emxFree_real_T(&c_pathC);
  emxFree_real_T(&d);
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
  int i8;
  static const double dv11[3] = { 1.0, 0.0, 0.5 };

  for (i8 = 0; i8 < 3; i8++) {
    HGAINS[i8] = dv11[i8];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void cartesianLimits_not_empty_init()
{
  cartesianLimits_not_empty = false;
}

//
// File trailer for buildRRTWrapper.cpp
//
// [EOF]
//
