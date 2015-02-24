//
// File: nearestNeighbour.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 24-Feb-2015 15:18:00
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "nearestNeighbour.h"
#include "norm.h"
#include "buildRRTWrapper_emxutil.h"
#include <stdio.h>

// Function Definitions

//
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
// Arguments    : const double x[13]
//                const emxArray_real_T *T
//                double kC_l1
//                double kC_l2
//                double kC_l3
//                double kC_l4
//                double kC_l5
//                double kC_l6
//                double kC_l7
//                double kC_l8
//                double kC_zeta
//                double kC_r
//                double nodeIDCount
//                double xNear_data[]
//                int xNear_size[2]
//                emxArray_real_T *transitionArray
//                double *d
// Return Type  : void
//
void nearestNeighbour(const double x[13], const emxArray_real_T *T, double kC_l1,
                      double kC_l2, double kC_l3, double kC_l4, double kC_l5,
                      double kC_l6, double kC_l7, double kC_l8, double kC_zeta,
                      double kC_r, double nodeIDCount, double xNear_data[], int
                      xNear_size[2], emxArray_real_T *transitionArray, double *d)
{
  emxArray_real_T *b_d;
  int i3;
  int ixstart;
  double uA[3];
  double mtmp;
  double q_idx_1;
  double q_idx_2;
  double uB[3];
  double b_uB[3];
  int itmp;
  int ix;
  boolean_T exitg1;
  emxInit_real_T(&b_d, 2);

  // nearestNeigbour.m
  // author: wreid
  // date: 20150107
  // Iterate over the entire tree and apply the distance heuristic function
  // to each node.
  i3 = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = (int)nodeIDCount;
  emxEnsureCapacity((emxArray__common *)b_d, i3, (int)sizeof(double));
  ixstart = (int)nodeIDCount;
  for (i3 = 0; i3 < ixstart; i3++) {
    b_d->data[i3] = 0.0;
  }

  // parfor i = 1:nodeIDCount
  for (ixstart = 0; ixstart < (int)nodeIDCount; ixstart++) {
    // heuristic Calculates the distance between states x1 and x2.
    // heuristicSingleLeg.m
    // author: wreid
    // date: 20150107
    // Calculate the distance between angular positions.
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
    uA[0] = ((((kC_l2 + kC_l3 * cos(-x[4])) + kC_l4 * cos(kC_zeta)) + kC_l5 *
              cos(x[5] + kC_zeta)) - kC_l7) * cos(x[3]);
    uA[1] = ((((kC_l2 + kC_l3 * cos(-x[4])) + kC_l4 * cos(kC_zeta)) + kC_l5 *
              cos(x[5] + kC_zeta)) - kC_l7) * sin(x[3]);
    uA[2] = ((((kC_l1 + kC_l3 * sin(-x[4])) - kC_l4 * sin(kC_zeta)) - kC_l5 *
              sin(x[5] + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);
    mtmp = T->data[ixstart + T->size[0] * 3];
    q_idx_1 = T->data[ixstart + (T->size[0] << 2)];
    q_idx_2 = T->data[ixstart + T->size[0] * 5];

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
    uB[0] = ((((kC_l2 + kC_l3 * cos(-q_idx_1)) + kC_l4 * cos(kC_zeta)) + kC_l5 *
              cos(q_idx_2 + kC_zeta)) - kC_l7) * cos(mtmp);
    uB[1] = ((((kC_l2 + kC_l3 * cos(-q_idx_1)) + kC_l4 * cos(kC_zeta)) + kC_l5 *
              cos(q_idx_2 + kC_zeta)) - kC_l7) * sin(mtmp);
    uB[2] = ((((kC_l1 + kC_l3 * sin(-q_idx_1)) - kC_l4 * sin(kC_zeta)) - kC_l5 *
              sin(q_idx_2 + kC_zeta)) - kC_l6) - (kC_l8 + kC_r);

    // dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); 
    // dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); 
    //     uA = sherpaTTFK(xA(4:6),kC);
    //     uB = sherpaTTFK(xB(4:6),kC);
    // dPos = norm(uA-uB);
    // Calculate the total distance.
    // d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;
    for (i3 = 0; i3 < 3; i3++) {
      b_uB[i3] = uB[i3] - uA[i3];
    }

    b_d->data[ixstart] = norm(b_uB);
  }

  ixstart = 1;
  mtmp = b_d->data[0];
  itmp = 0;
  if ((int)nodeIDCount > 1) {
    if (rtIsNaN(mtmp)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int)nodeIDCount)) {
        ixstart = ix;
        if (!rtIsNaN(b_d->data[ix - 1])) {
          mtmp = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int)nodeIDCount) {
      while (ixstart + 1 <= (int)nodeIDCount) {
        if (b_d->data[ixstart] < mtmp) {
          mtmp = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = mtmp;

  // [d,minIndex] = min(d(1:nodeIDCount));
  xNear_size[0] = 1;
  xNear_size[1] = 13;
  for (i3 = 0; i3 < 13; i3++) {
    xNear_data[xNear_size[0] * i3] = T->data[itmp + T->size[0] * i3];
  }

  if (14 > T->size[1]) {
    i3 = 0;
    ix = 0;
  } else {
    i3 = 13;
    ix = T->size[1];
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ix - i3;
  emxEnsureCapacity((emxArray__common *)transitionArray, ixstart, (int)sizeof
                    (double));
  ixstart = ix - i3;
  for (ix = 0; ix < ixstart; ix++) {
    transitionArray->data[transitionArray->size[0] * ix] = T->data[itmp +
      T->size[0] * (i3 + ix)];
  }
}

//
// File trailer for nearestNeighbour.cpp
//
// [EOF]
//
