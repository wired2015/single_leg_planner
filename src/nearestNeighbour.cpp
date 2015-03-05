//
// File: nearestNeighbour.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 11:17:25
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "nearestNeighbour.h"
#include "heuristicSingleLeg.h"
#include "sherpaTTPlanner_emxutil.h"
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
//                const double T[139500]
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
//                double xNear[13]
//                double transitionArray[80]
//                double *d
// Return Type  : void
//
void nearestNeighbour(const double x[13], const double T[139500], double kC_l1,
                      double kC_l2, double kC_l3, double kC_l4, double kC_l5,
                      double kC_l6, double kC_l7, double kC_l8, double kC_zeta,
                      double kC_r, double nodeIDCount, double xNear[13], double
                      transitionArray[80], double *d)
{
  emxArray_real_T *b_d;
  int ix;
  int ixstart;
  double b_T[93];
  int itmp;
  boolean_T exitg1;
  emxInit_real_T(&b_d, 2);

  // nearestNeigbour.m
  // author: wreid
  // date: 20150107
  // Iterate over the entire tree and apply the distance heuristic function
  // to each node.
  ix = b_d->size[0] * b_d->size[1];
  b_d->size[0] = 1;
  b_d->size[1] = (int)nodeIDCount;
  emxEnsureCapacity((emxArray__common *)b_d, ix, (int)sizeof(double));
  ixstart = (int)nodeIDCount;
  for (ix = 0; ix < ixstart; ix++) {
    b_d->data[ix] = 0.0;
  }

  // parfor i = 1:nodeIDCount
  for (ixstart = 0; ixstart < (int)nodeIDCount; ixstart++) {
    for (ix = 0; ix < 93; ix++) {
      b_T[ix] = T[ixstart + 1500 * ix];
    }

    b_d->data[ixstart] = heuristicSingleLeg(x, b_T, kC_l1, kC_l2, kC_l3, kC_l4,
      kC_l5, kC_l6, kC_l7, kC_l8, kC_zeta, kC_r);
  }

  ixstart = 1;
  *d = b_d->data[0];
  itmp = 0;
  if ((int)nodeIDCount > 1) {
    if (rtIsNaN(*d)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int)nodeIDCount)) {
        ixstart = ix;
        if (!rtIsNaN(b_d->data[ix - 1])) {
          *d = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int)nodeIDCount) {
      while (ixstart + 1 <= (int)nodeIDCount) {
        if (b_d->data[ixstart] < *d) {
          *d = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);

  // [d,minIndex] = min(d(1:nodeIDCount));
  for (ix = 0; ix < 13; ix++) {
    xNear[ix] = T[itmp + 1500 * ix];
  }

  for (ix = 0; ix < 80; ix++) {
    transitionArray[ix] = T[itmp + 1500 * (13 + ix)];
  }
}

//
// File trailer for nearestNeighbour.cpp
//
// [EOF]
//
