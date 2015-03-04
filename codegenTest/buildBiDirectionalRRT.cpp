//
// File: buildBiDirectionalRRT.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Mar-2015 14:16:20
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "buildBiDirectionalRRT.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "sherpaTTPlanner_emxutil.h"
#include "randomState.h"
#include "norm.h"
#include "flipud.h"
#include <stdio.h>

// Function Declarations
static void traceBranch(const double T[69750], const double midPoint_data[],
  emxArray_real_T *path);

// Function Definitions

//
// Assignn the
// Arguments    : const double T[69750]
//                const double midPoint_data[]
//                emxArray_real_T *path
// Return Type  : void
//
static void traceBranch(const double T[69750], const double midPoint_data[],
  emxArray_real_T *path)
{
  double check;
  double next_data[93];
  int i18;
  double transitionArray_data[80];
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int i;
  int b_i;
  int loop_ub;
  int i19;
  check = midPoint_data[0];
  memcpy(&next_data[0], &midPoint_data[0], 13U * sizeof(double));
  i18 = path->size[0] * path->size[1];
  path->size[0] = 0;
  path->size[1] = 10;
  emxEnsureCapacity((emxArray__common *)path, i18, (int)sizeof(double));
  for (i18 = 0; i18 < 80; i18++) {
    transitionArray_data[i18] = T[((int)midPoint_data[0] + 750 * (13 + i18)) - 1];
  }

  // Iterate over the tree until the initial state has been found.
  emxInit_real_T(&transitionPath, 2);
  emxInit_real_T(&b_transitionPath, 2);
  emxInit_real_T(&c_transitionPath, 2);
  while ((check != 0.0) && (next_data[1] != 0.0)) {
    i18 = transitionPath->size[0] * transitionPath->size[1];
    transitionPath->size[0] = 0;
    transitionPath->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)transitionPath, i18, (int)sizeof
                      (double));
    for (i = 0; i < 8; i++) {
      b_i = i * 10;
      i18 = c_transitionPath->size[0] * c_transitionPath->size[1];
      c_transitionPath->size[0] = transitionPath->size[0] + 1;
      c_transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)c_transitionPath, i18, (int)sizeof
                        (double));
      for (i18 = 0; i18 < 10; i18++) {
        loop_ub = transitionPath->size[0];
        for (i19 = 0; i19 < loop_ub; i19++) {
          c_transitionPath->data[i19 + c_transitionPath->size[0] * i18] =
            transitionPath->data[i19 + transitionPath->size[0] * i18];
        }
      }

      for (i18 = 0; i18 < 10; i18++) {
        c_transitionPath->data[transitionPath->size[0] + c_transitionPath->size
          [0] * i18] = transitionArray_data[i18 + b_i];
      }

      i18 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = c_transitionPath->size[0];
      transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)transitionPath, i18, (int)sizeof
                        (double));
      for (i18 = 0; i18 < 10; i18++) {
        loop_ub = c_transitionPath->size[0];
        for (i19 = 0; i19 < loop_ub; i19++) {
          transitionPath->data[i19 + transitionPath->size[0] * i18] =
            c_transitionPath->data[i19 + c_transitionPath->size[0] * i18];
        }
      }
    }

    i18 = b_transitionPath->size[0] * b_transitionPath->size[1];
    b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
    b_transitionPath->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)b_transitionPath, i18, (int)sizeof
                      (double));
    for (i18 = 0; i18 < 10; i18++) {
      loop_ub = transitionPath->size[0];
      for (i19 = 0; i19 < loop_ub; i19++) {
        b_transitionPath->data[i19 + b_transitionPath->size[0] * i18] =
          transitionPath->data[i19 + transitionPath->size[0] * i18];
      }
    }

    for (i18 = 0; i18 < 10; i18++) {
      loop_ub = path->size[0];
      for (i19 = 0; i19 < loop_ub; i19++) {
        b_transitionPath->data[(i19 + transitionPath->size[0]) +
          b_transitionPath->size[0] * i18] = path->data[i19 + path->size[0] *
          i18];
      }
    }

    i18 = path->size[0] * path->size[1];
    path->size[0] = b_transitionPath->size[0];
    path->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)path, i18, (int)sizeof(double));
    for (i18 = 0; i18 < 10; i18++) {
      loop_ub = b_transitionPath->size[0];
      for (i19 = 0; i19 < loop_ub; i19++) {
        path->data[i19 + path->size[0] * i18] = b_transitionPath->data[i19 +
          b_transitionPath->size[0] * i18];
      }
    }

    check = next_data[1];
    for (i18 = 0; i18 < 93; i18++) {
      next_data[i18] = T[((int)check + 750 * i18) - 1];
    }

    check = next_data[1];
    for (i18 = 0; i18 < 80; i18++) {
      transitionArray_data[i18] = next_data[13 + i18];
    }
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
}

//
// buildRRT Icrementally builds a rapidly exploring random tree.
//    An RRT is build by incrementally selecting a random state from the
//    available state space as defined by the MIN and MAX vectors. The tree is
//    started at xInit and is extended until the number of maximum nodes, K has
//    been reached. A path is selected if the goal region as defined by xGoal
//    has been reached by the RRT.
// Arguments    : const double nInit[13]
//                const double nGoal[13]
//                const double jointLimits[20]
//                double panHeight
//                const struct0_T *kC
//                const double uBDot[6]
//                int legNum
//                const double TP2B[16]
//                double T1[69750]
//                double T2[69750]
//                emxArray_real_T *pathJ
//                emxArray_real_T *pathC
// Return Type  : void
//
void buildBiDirectionalRRT(const double nInit[13], const double nGoal[13], const
  double jointLimits[20], double panHeight, const struct0_T *kC, const double
  uBDot[6], int legNum, const double TP2B[16], double T1[69750], double T2[69750],
  emxArray_real_T *pathJ, emxArray_real_T *pathC)
{
  int i5;
  unsigned int nodeIDCount1;
  unsigned int nodeIDCount2;
  double pathLengthMin;
  emxArray_real_T *d;
  int i;
  static double b_T1[69750];
  double xRand[13];
  int ixstart;
  double b_T2[93];
  double dist2Go;
  int absb;
  int cdiff;
  boolean_T exitg2;
  double c_T1[13];
  double transitionArray[80];
  double unusedU1[13];
  unsigned int nodeIDCountTemp;
  emxArray_real_T *pathT1;
  emxArray_real_T *pathT2;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_t;
  double b_d[750];
  boolean_T exitg1;
  int apnd;
  double uP[3];
  double uB[3];
  double d1;
  double b_uB[3];
  double b_path[3];

  // buildBiDirectionalRRT.m
  // author: wreid
  // date: 20150107
  // Constant Declaration
  for (i5 = 0; i5 < 69750; i5++) {
    T1[i5] = 0.0;
  }

  for (i5 = 0; i5 < 69750; i5++) {
    T2[i5] = 0.0;
  }

  for (i5 = 0; i5 < 13; i5++) {
    T1[750 * i5] = nInit[i5];
  }

  for (i5 = 0; i5 < 80; i5++) {
    T1[750 * (i5 + 13)] = 0.0;
  }

  for (i5 = 0; i5 < 13; i5++) {
    T2[750 * i5] = nGoal[i5];
  }

  for (i5 = 0; i5 < 80; i5++) {
    T2[750 * (i5 + 13)] = 0.0;
  }

  nodeIDCount1 = 1U;
  nodeIDCount2 = 1U;
  pathLengthMin = 100.0;
  i5 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = 0;
  pathC->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)pathC, i5, (int)sizeof(double));
  i5 = pathJ->size[0] * pathJ->size[1];
  pathJ->size[0] = 0;
  pathJ->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)pathJ, i5, (int)sizeof(double));
  emxInit_real_T(&d, 2);
  for (i = 0; i < 1498; i++) {
    for (i5 = 0; i5 < 69750; i5++) {
      b_T1[i5] = T1[i5];
    }

    randomState(jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
                kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);

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
    i5 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = (int)nodeIDCount1;
    emxEnsureCapacity((emxArray__common *)d, i5, (int)sizeof(double));
    ixstart = (int)nodeIDCount1;
    for (i5 = 0; i5 < ixstart; i5++) {
      d->data[i5] = 0.0;
    }

    // parfor i = 1:nodeIDCount
    for (ixstart = 0; ixstart < (int)nodeIDCount1; ixstart++) {
      for (i5 = 0; i5 < 93; i5++) {
        b_T2[i5] = T1[ixstart + 750 * i5];
      }

      d->data[ixstart] = heuristicSingleLeg(xRand, b_T2, kC->l1, kC->l2, kC->l3,
        kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
    }

    ixstart = 1;
    dist2Go = d->data[0];
    absb = 0;
    if ((int)nodeIDCount1 > 1) {
      if (rtIsNaN(dist2Go)) {
        cdiff = 2;
        exitg2 = false;
        while ((!exitg2) && (cdiff <= (int)nodeIDCount1)) {
          ixstart = cdiff;
          if (!rtIsNaN(d->data[cdiff - 1])) {
            dist2Go = d->data[cdiff - 1];
            absb = cdiff - 1;
            exitg2 = true;
          } else {
            cdiff++;
          }
        }
      }

      if (ixstart < (int)nodeIDCount1) {
        for (cdiff = ixstart + 1; cdiff <= (int)nodeIDCount1; cdiff++) {
          if (d->data[cdiff - 1] < dist2Go) {
            dist2Go = d->data[cdiff - 1];
            absb = cdiff - 1;
          }
        }
      }
    }

    // [d,minIndex] = min(d(1:nodeIDCount));
    for (i5 = 0; i5 < 13; i5++) {
      c_T1[i5] = T1[absb + 750 * i5];
    }

    selectInput(c_T1, xRand, kC, 0.39269908169872414, jointLimits, uBDot, legNum,
                unusedU1, transitionArray);
    unusedU1[0] = (double)nodeIDCount1 + 1.0;

    // Node ID
    unusedU1[1] = T1[absb];

    // Parent ID
    for (i5 = 0; i5 < 13; i5++) {
      c_T1[i5] = T1[absb + 750 * i5];
    }

    unusedU1[2] = T1[1500 + absb] + b_heuristicSingleLeg(unusedU1, c_T1, kC->l1,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);

    // Cost
    ixstart = (int)nodeIDCount1;
    for (i5 = 0; i5 < 13; i5++) {
      b_T1[ixstart + 750 * i5] = unusedU1[i5];
    }

    for (i5 = 0; i5 < 80; i5++) {
      b_T1[ixstart + 750 * (i5 + 13)] = transitionArray[i5];
    }

    // Append the new node to the tree.
    // if mod(nodeIDCount,100) == 0
    // fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
    // end
    // Swap the trees.
    for (i5 = 0; i5 < 69750; i5++) {
      T1[i5] = T2[i5];
    }

    memcpy(&T2[0], &b_T1[0], 69750U * sizeof(double));

    // Swap the trees.
    nodeIDCountTemp = nodeIDCount1;
    nodeIDCount1 = nodeIDCount2;
    nodeIDCount2 = nodeIDCountTemp + 1U;
  }

  emxInit_real_T(&pathT1, 2);
  emxInit_real_T(&pathT2, 2);
  b_emxInit_real_T(&t, 1);
  emxInit_real_T(&path, 2);
  emxInit_real_T(&b_pathC, 2);
  emxInit_real_T(&b_t, 2);
  for (i = 0; i < 750; i++) {
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
    // parfor i = 1:nodeIDCount
    for (ixstart = 0; ixstart < 750; ixstart++) {
      for (i5 = 0; i5 < 13; i5++) {
        xRand[i5] = T1[i + 750 * i5];
      }

      for (i5 = 0; i5 < 93; i5++) {
        b_T2[i5] = T2[ixstart + 750 * i5];
      }

      b_d[ixstart] = c_heuristicSingleLeg(xRand, b_T2, kC->l1, kC->l2, kC->l3,
        kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
    }

    ixstart = 1;
    dist2Go = b_d[0];
    absb = 0;
    if (rtIsNaN(b_d[0])) {
      cdiff = 2;
      exitg1 = false;
      while ((!exitg1) && (cdiff < 751)) {
        ixstart = cdiff;
        if (!rtIsNaN(b_d[cdiff - 1])) {
          dist2Go = b_d[cdiff - 1];
          absb = cdiff - 1;
          exitg1 = true;
        } else {
          cdiff++;
        }
      }
    }

    if (ixstart < 750) {
      while (ixstart + 1 < 751) {
        if (b_d[ixstart] < dist2Go) {
          dist2Go = b_d[ixstart];
          absb = ixstart;
        }

        ixstart++;
      }
    }

    // [d,minIndex] = min(d(1:nodeIDCount));
    if (dist2Go < 0.04) {
      for (i5 = 0; i5 < 13; i5++) {
        xRand[i5] = T1[i + 750 * i5];
      }

      traceBranch(T1, xRand, pathT1);
      for (i5 = 0; i5 < 13; i5++) {
        xRand[i5] = T2[absb + 750 * i5];
      }

      traceBranch(T2, xRand, pathT2);
      if ((T1[2250] == nInit[3]) && (T1[3000] == nInit[4]) && (T1[3750] ==
           nInit[5])) {
        flipud(pathT2);
        i5 = path->size[0] * path->size[1];
        path->size[0] = pathT1->size[0] + pathT2->size[0];
        path->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)path, i5, (int)sizeof(double));
        for (i5 = 0; i5 < 10; i5++) {
          ixstart = pathT1->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[absb + path->size[0] * i5] = pathT1->data[absb +
              pathT1->size[0] * i5];
          }
        }

        for (i5 = 0; i5 < 10; i5++) {
          ixstart = pathT2->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[(absb + pathT1->size[0]) + path->size[0] * i5] =
              pathT2->data[absb + pathT2->size[0] * i5];
          }
        }
      } else {
        flipud(pathT1);
        i5 = path->size[0] * path->size[1];
        path->size[0] = pathT2->size[0] + pathT1->size[0];
        path->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)path, i5, (int)sizeof(double));
        for (i5 = 0; i5 < 10; i5++) {
          ixstart = pathT2->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[absb + path->size[0] * i5] = pathT2->data[absb +
              pathT2->size[0] * i5];
          }
        }

        for (i5 = 0; i5 < 10; i5++) {
          ixstart = pathT1->size[0];
          for (absb = 0; absb < ixstart; absb++) {
            path->data[(absb + pathT2->size[0]) + path->size[0] * i5] =
              pathT1->data[absb + pathT1->size[0] * i5];
          }
        }
      }

      if (path->size[0] < 1) {
        absb = -1;
        apnd = 0;
      } else {
        ixstart = (int)std::floor(((double)path->size[0] - 1.0) + 0.5);
        apnd = ixstart + 1;
        cdiff = (ixstart - path->size[0]) + 1;
        absb = path->size[0];
        if (std::abs((double)cdiff) < 4.4408920985006262E-16 * (double)absb) {
          ixstart++;
          apnd = path->size[0];
        } else if (cdiff > 0) {
          apnd = ixstart;
        } else {
          ixstart++;
        }

        absb = ixstart - 1;
      }

      i5 = d->size[0] * d->size[1];
      d->size[0] = 1;
      d->size[1] = absb + 1;
      emxEnsureCapacity((emxArray__common *)d, i5, (int)sizeof(double));
      if (absb + 1 > 0) {
        d->data[0] = 1.0;
        if (absb + 1 > 1) {
          d->data[absb] = apnd;
          ixstart = absb / 2;
          for (cdiff = 1; cdiff < ixstart; cdiff++) {
            d->data[cdiff] = 1.0 + (double)cdiff;
            d->data[absb - cdiff] = apnd - cdiff;
          }

          if (ixstart << 1 == absb) {
            d->data[ixstart] = (1.0 + (double)apnd) / 2.0;
          } else {
            d->data[ixstart] = 1.0 + (double)ixstart;
            d->data[ixstart + 1] = apnd - ixstart;
          }
        }
      }

      i5 = t->size[0];
      t->size[0] = d->size[1];
      emxEnsureCapacity((emxArray__common *)t, i5, (int)sizeof(double));
      ixstart = d->size[1];
      for (i5 = 0; i5 < ixstart; i5++) {
        t->data[i5] = 0.1 * d->data[d->size[0] * i5];
      }

      ixstart = t->size[0];
      i5 = b_t->size[0] * b_t->size[1];
      b_t->size[0] = ixstart;
      b_t->size[1] = 1 + path->size[1];
      emxEnsureCapacity((emxArray__common *)b_t, i5, (int)sizeof(double));
      for (i5 = 0; i5 < ixstart; i5++) {
        b_t->data[i5] = t->data[i5];
      }

      ixstart = path->size[1];
      for (i5 = 0; i5 < ixstart; i5++) {
        cdiff = path->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          b_t->data[absb + b_t->size[0] * (i5 + 1)] = path->data[absb +
            path->size[0] * i5];
        }
      }

      i5 = path->size[0] * path->size[1];
      path->size[0] = b_t->size[0];
      path->size[1] = b_t->size[1];
      emxEnsureCapacity((emxArray__common *)path, i5, (int)sizeof(double));
      ixstart = b_t->size[1];
      for (i5 = 0; i5 < ixstart; i5++) {
        cdiff = b_t->size[0];
        for (absb = 0; absb < cdiff; absb++) {
          path->data[absb + path->size[0] * i5] = b_t->data[absb + b_t->size[0] *
            i5];
        }
      }

      ixstart = path->size[0];
      i5 = b_pathC->size[0] * b_pathC->size[1];
      b_pathC->size[0] = ixstart;
      b_pathC->size[1] = 9;
      emxEnsureCapacity((emxArray__common *)b_pathC, i5, (int)sizeof(double));
      ixstart = path->size[0] * 9;
      for (i5 = 0; i5 < ixstart; i5++) {
        b_pathC->data[i5] = 0.0;
      }

      dist2Go = 0.0;
      for (ixstart = 0; ixstart < path->size[0]; ixstart++) {
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
        uP[0] = ((((kC->l2 + kC->l3 * std::cos(-path->data[ixstart + (path->
          size[0] << 1)])) + kC->l4 * std::cos(kC->zeta)) + kC->l5 * std::cos
                  (path->data[ixstart + path->size[0] * 3] + kC->zeta)) - kC->l7)
          * std::cos(path->data[ixstart + path->size[0]]);
        uP[1] = ((((kC->l2 + kC->l3 * std::cos(-path->data[ixstart + (path->
          size[0] << 1)])) + kC->l4 * std::cos(kC->zeta)) + kC->l5 * std::cos
                  (path->data[ixstart + path->size[0] * 3] + kC->zeta)) - kC->l7)
          * std::sin(path->data[ixstart + path->size[0]]);
        uP[2] = ((((kC->l1 + kC->l3 * std::sin(-path->data[ixstart + (path->
          size[0] << 1)])) - kC->l4 * std::sin(kC->zeta)) - kC->l5 * std::sin
                  (path->data[ixstart + path->size[0] * 3] + kC->zeta)) - kC->l6)
          - (kC->l8 + kC->r);

        // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
        // sherpaTTFKVel.m
        // author: wreid
        // date: 20150122
        for (i5 = 0; i5 < 3; i5++) {
          d1 = 0.0;
          for (absb = 0; absb < 3; absb++) {
            d1 += TP2B[i5 + (absb << 2)] * uP[absb];
          }

          uB[i5] = d1 + TP2B[12 + i5];
        }

        if (1 + ixstart != 1) {
          for (i5 = 0; i5 < 3; i5++) {
            b_uB[i5] = uB[i5] - b_pathC->data[(ixstart + b_pathC->size[0] * (2 +
              i5)) - 1];
          }

          dist2Go += norm(b_uB);
        }

        b_path[0] = (-path->data[ixstart + path->size[0] * 6] * std::sin
                     (path->data[ixstart + path->size[0]]) * ((((kC->l2 - kC->l7)
          + kC->l5 * std::cos(path->data[ixstart + path->size[0] * 3] + kC->zeta))
          + kC->l3 * std::cos(path->data[ixstart + (path->size[0] << 1)])) +
          kC->l4 * std::cos(kC->zeta)) - path->data[ixstart + path->size[0] * 7]
                     * kC->l3 * std::cos(path->data[ixstart + path->size[0]]) *
                     std::sin(path->data[ixstart + (path->size[0] << 1)])) -
          path->data[ixstart + (path->size[0] << 3)] * kC->l5 * std::sin
          (path->data[ixstart + path->size[0] * 3] + kC->zeta) * std::cos
          (path->data[ixstart + path->size[0]]);
        b_path[1] = (path->data[ixstart + path->size[0] * 6] * std::cos
                     (path->data[ixstart + path->size[0]]) * ((((kC->l2 - kC->l7)
          + kC->l5 * std::cos(path->data[ixstart + path->size[0] * 3] + kC->zeta))
          + kC->l3 * std::cos(path->data[ixstart + (path->size[0] << 1)])) +
          kC->l4 * std::cos(kC->zeta)) - path->data[ixstart + (path->size[0] <<
          3)] * kC->l5 * std::sin(path->data[ixstart + path->size[0] * 3] +
          kC->zeta) * std::sin(path->data[ixstart + path->size[0]])) -
          path->data[ixstart + path->size[0] * 7] * kC->l3 * std::sin(path->
          data[ixstart + path->size[0]]) * std::sin(path->data[ixstart +
          (path->size[0] << 1)]);
        b_path[2] = -path->data[ixstart + path->size[0] * 7] * kC->l3 * std::cos
          (path->data[ixstart + (path->size[0] << 1)]) - kC->l5 * path->
          data[ixstart + (path->size[0] << 3)] * std::cos(kC->zeta + path->
          data[ixstart + path->size[0] * 3]);
        for (i5 = 0; i5 < 3; i5++) {
          b_uB[i5] = 0.0;
          for (absb = 0; absb < 3; absb++) {
            b_uB[i5] += TP2B[i5 + (absb << 2)] * b_path[absb];
          }
        }

        b_pathC->data[ixstart] = path->data[ixstart];
        b_pathC->data[ixstart + b_pathC->size[0]] = dist2Go;
        for (i5 = 0; i5 < 3; i5++) {
          b_pathC->data[ixstart + b_pathC->size[0] * (i5 + 2)] = uB[i5];
        }

        for (i5 = 0; i5 < 3; i5++) {
          b_pathC->data[ixstart + b_pathC->size[0] * (i5 + 5)] = b_uB[i5];
        }

        b_pathC->data[ixstart + (b_pathC->size[0] << 3)] = 0.0;
      }

      if (b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1] <
          pathLengthMin) {
        pathLengthMin = b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1];
        i5 = pathC->size[0] * pathC->size[1];
        pathC->size[0] = b_pathC->size[0];
        pathC->size[1] = 9;
        emxEnsureCapacity((emxArray__common *)pathC, i5, (int)sizeof(double));
        ixstart = b_pathC->size[0] * b_pathC->size[1];
        for (i5 = 0; i5 < ixstart; i5++) {
          pathC->data[i5] = b_pathC->data[i5];
        }

        i5 = pathJ->size[0] * pathJ->size[1];
        pathJ->size[0] = path->size[0];
        pathJ->size[1] = path->size[1];
        emxEnsureCapacity((emxArray__common *)pathJ, i5, (int)sizeof(double));
        ixstart = path->size[0] * path->size[1];
        for (i5 = 0; i5 < ixstart; i5++) {
          pathJ->data[i5] = path->data[i5];
        }
      }
    }
  }

  emxFree_real_T(&b_t);
  emxFree_real_T(&d);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&pathT2);
  emxFree_real_T(&pathT1);
}

//
// File trailer for buildBiDirectionalRRT.cpp
//
// [EOF]
//
