//
// File: buildRRT.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 13-Feb-2015 14:11:02
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "rand.h"
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_emxutil.h"
#include "selectInput.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Declarations
static int div_s32_floor(int numerator, int denominator);

// Function Definitions

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator >= 0) {
      absNumerator = (unsigned int)numerator;
    } else {
      absNumerator = (unsigned int)-numerator;
    }

    if (denominator >= 0) {
      absDenominator = (unsigned int)denominator;
    } else {
      absDenominator = (unsigned int)-denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
    }

    if (quotientNeedsNegation) {
      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
    }
  }

  return quotient;
}

//
// buildRRT Icrementally builds a rapidly exploring random tree.
//    An RRT is build by incrementally selecting a random state from the
//    available state space as defined by the MIN and MAX vectors. The tree is
//    started at xInit and is extended until the number of maximum nodes, K has
//    been reached. A path is selected if the goal region as defined by xGoal
//    has been reached by the RRT.
// Arguments    : const double nInit[11]
//                const double nGoal[11]
//                const double jointLimits[12]
//                double panHeight
//                const double U[10]
//                double dt
//                double Dt
//                const struct0_T *kC
//                emxArray_real_T *T
//                emxArray_real_T *path
// Return Type  : void
//
void buildRRT(const double nInit[11], const double nGoal[11], const double
              jointLimits[12], double panHeight, const double U[10], double dt,
              double Dt, const struct0_T *kC, emxArray_real_T *T,
              emxArray_real_T *path)
{
  double transitionArrayLength;
  int i2;
  double xStarA;
  int ix;
  int loop_ub;
  unsigned int nodeIDCount;
  double jointRange[6];
  emxArray_real_T *b_T;
  emxArray_real_T *d;
  emxArray_real_T *c_T;
  int i;
  double dxStar;
  double dv2[9];
  int xRand_size[2];
  double xRand_data[11];
  unsigned int u0;
  int ixstart;
  int b_ixstart;
  int itmp;
  boolean_T exitg2;
  int xNearest_size[2];
  double xNearest_data[11];
  int xNew_size[2];
  double xNew_data[11];
  emxArray_real_T b_xNearest_data;
  boolean_T exitg1;
  emxArray_real_T *b_path;

  // buildRRT.m
  // author: wreid
  // date: 20150107
  // %TODO: Make a structure input for nInit and nGoal
  // %TEMPORARY definition of init and goal nodes.
  // nInit = makeNode(1,0,0,nInit(4:6),nInit(7:9),)
  // Constant Declaration
  transitionArrayLength = (rt_roundd_snf(Dt / dt) + 1.0) * 6.0;

  // Variable Initialization
  i2 = T->size[0] * T->size[1];
  T->size[0] = 1000;
  xStarA = rt_roundd_snf(11.0 + transitionArrayLength);
  if (xStarA < 2.147483648E+9) {
    if (xStarA >= -2.147483648E+9) {
      ix = (int)xStarA;
    } else {
      ix = MIN_int32_T;
    }
  } else if (xStarA >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  T->size[1] = ix;
  emxEnsureCapacity((emxArray__common *)T, i2, (int)sizeof(double));
  xStarA = rt_roundd_snf(11.0 + transitionArrayLength);
  if (xStarA < 2.147483648E+9) {
    if (xStarA >= -2.147483648E+9) {
      i2 = (int)xStarA;
    } else {
      i2 = MIN_int32_T;
    }
  } else if (xStarA >= 2.147483648E+9) {
    i2 = MAX_int32_T;
  } else {
    i2 = 0;
  }

  loop_ub = 1000 * i2;
  for (i2 = 0; i2 < loop_ub; i2++) {
    T->data[i2] = 0.0;
  }

  // Define a zero array that will be used to
  // store data from each tree node.
  for (i2 = 0; i2 < 11; i2++) {
    T->data[T->size[0] * i2] = nInit[i2];
  }

  loop_ub = (int)transitionArrayLength;
  for (i2 = 0; i2 < loop_ub; i2++) {
    T->data[T->size[0] * (i2 + 11)] = 0.0;
  }

  // Initialize the tree with initial state.
  nodeIDCount = 1U;
  for (i2 = 0; i2 < 6; i2++) {
    jointRange[i2] = jointLimits[1 + (i2 << 1)] - jointLimits[i2 << 1];
  }

  emxInit_real_T(&b_T, 2);
  emxInit_real_T(&d, 2);
  emxInit_real_T(&c_T, 2);
  for (i = 0; i < 999; i++) {
    i2 = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1000;
    b_T->size[1] = T->size[1];
    emxEnsureCapacity((emxArray__common *)b_T, i2, (int)sizeof(double));
    loop_ub = T->size[0] * T->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_T->data[i2] = T->data[i2];
    }

    // randomState Picks a random state from the state space.
    //    A random state is selected from the state space within the boundaries of 
    //    the state space as defined by the MIN and MAX vectors. The state space has 
    //    a dimension n.
    //    Inputs:
    //        MIN:    The 1xn vector containing the minimum boundaries for the state 
    //                space.
    //        MAX:    The 1xn vector containing the maximum boundaries for the state 
    //                space.
    //    Outputs:
    //        xRand:  The 1xn vector describing the selected random state.
    // randomState.m
    // author: wreid
    // date: 20150107
    // [~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); 
    xStarA = jointRange[0] * b_rand() + jointLimits[0];
    dxStar = jointRange[1] * b_rand() + jointLimits[2];

    // GETCONSTRAINEDBETA Calculates the beta joint angle given a constrained body 
    // height.
    // [L1,~,L3,L4,L5,L6,~,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); 
    //      check = (panHeight-kC.l1+kC.l4*sin(kC.zeta)+kC.l5*sin(gamma+kC.zeta)+kC.l7+kC.l8+kC.r)/kC.l3; 
    //
    //      if check < -1 || check > 1
    //          gammaMax = asin(1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta; 
    //          gammaMin = asin(-1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta; 
    //          gamma = gammaMin + rand*(gammaMax-gammaMin);
    //      end
    transitionArrayLength = ((((((-panHeight + kC->l1) - kC->l4 * sin(kC->zeta))
      - kC->l5 * sin(dxStar + kC->zeta)) - kC->l6) - kC->l8) - kC->r) / kC->l3;
    while ((transitionArrayLength < -1.0) || (transitionArrayLength > 1.0)) {
      dxStar = jointRange[1] * b_rand() + jointLimits[2];
      transitionArrayLength = ((((((-panHeight + kC->l1) - kC->l4 * sin(kC->zeta))
        - kC->l5 * sin(dxStar + kC->zeta)) - kC->l6) - kC->l8) - kC->r) / kC->l3;

      // disp('Imaginary');
      // disp(count);
    }

    // alphaDotRand = range(4)*rand+MIN(4);
    // gammaDotRand = range(5)*rand+MIN(5);
    // betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); 
    for (i2 = 0; i2 < 3; i2++) {
      dv2[i2] = 0.0;
    }

    dv2[3] = xStarA;
    dv2[4] = dxStar;
    dv2[5] = asin(transitionArrayLength);
    dv2[6] = 0.0;
    dv2[7] = 0.0;
    dv2[8] = 0.0;
    xRand_size[0] = 1;
    xRand_size[1] = 9;
    for (i2 = 0; i2 < 9; i2++) {
      xRand_data[i2] = dv2[i2];
    }

    u0 = nodeIDCount;
    if (u0 > 2147483647U) {
      u0 = 2147483647U;
    }

    ixstart = (int)u0;
    xStarA = rt_roundd_snf((double)nodeIDCount - floor((double)nodeIDCount /
      20.0) * 20.0);
    if (xStarA < 2.147483648E+9) {
      if (xStarA >= -2.147483648E+9) {
        i2 = (int)xStarA;
      } else {
        i2 = MIN_int32_T;
      }
    } else {
      i2 = MAX_int32_T;
    }

    if ((unsigned int)ixstart == nodeIDCount) {
      b_ixstart = ixstart - div_s32_floor(ixstart, 20) * 20;
    } else {
      b_ixstart = i2;
    }

    if (b_ixstart == 0) {
      xRand_size[0] = 1;
      xRand_size[1] = 11;
      memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(double));
    }

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
    i2 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = (int)nodeIDCount;
    emxEnsureCapacity((emxArray__common *)d, i2, (int)sizeof(double));
    loop_ub = (int)nodeIDCount;
    for (i2 = 0; i2 < loop_ub; i2++) {
      d->data[i2] = 0.0;
    }

    // parfor i = 1:nodeIDCount
    for (ixstart = 0; ixstart < (int)nodeIDCount; ixstart++) {
      loop_ub = T->size[1];
      i2 = c_T->size[0] * c_T->size[1];
      c_T->size[0] = 1;
      c_T->size[1] = loop_ub;
      emxEnsureCapacity((emxArray__common *)c_T, i2, (int)sizeof(double));
      for (i2 = 0; i2 < loop_ub; i2++) {
        c_T->data[c_T->size[0] * i2] = T->data[ixstart + T->size[0] * i2];
      }

      d->data[ixstart] = heuristicSingleLeg(xRand_data, c_T, kC->l2, kC->l3,
        kC->l4, kC->l5, kC->l7, kC->zeta);
    }

    ixstart = 1;
    transitionArrayLength = d->data[0];
    itmp = 0;
    if ((int)nodeIDCount > 1) {
      if (rtIsNaN(transitionArrayLength)) {
        ix = 2;
        exitg2 = false;
        while ((!exitg2) && (ix <= (int)nodeIDCount)) {
          ixstart = ix;
          if (!rtIsNaN(d->data[ix - 1])) {
            transitionArrayLength = d->data[ix - 1];
            itmp = ix - 1;
            exitg2 = true;
          } else {
            ix++;
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
    xNearest_size[0] = 1;
    xNearest_size[1] = 11;
    for (i2 = 0; i2 < 11; i2++) {
      xNearest_data[xNearest_size[0] * i2] = T->data[itmp + T->size[0] * i2];
    }

    selectInput(xNearest_data, xNearest_size, xRand_data, xRand_size, U, dt, Dt,
                kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta, jointLimits,
                xNew_data, xNew_size, d);
    xNew_data[0] = (double)nodeIDCount + 1.0;

    // Node ID
    xNew_data[1] = T->data[itmp];

    // Parent ID
    b_xNearest_data.data = (double *)&xNearest_data;
    b_xNearest_data.size = (int *)&xNearest_size;
    b_xNearest_data.allocatedSize = 11;
    b_xNearest_data.numDimensions = 2;
    b_xNearest_data.canFreeData = false;
    xNew_data[2] = heuristicSingleLeg(xNew_data, &b_xNearest_data, kC->l2,
      kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);

    // Cost
    for (i2 = 0; i2 < 11; i2++) {
      b_T->data[((int)((double)nodeIDCount + 1.0) + b_T->size[0] * i2) - 1] =
        xNew_data[xNew_size[0] * i2];
    }

    loop_ub = d->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_T->data[((int)((double)nodeIDCount + 1.0) + b_T->size[0] * (i2 + 11)) -
        1] = d->data[d->size[0] * i2];
    }

    // Append the new node to the tree.
    // if mod(nodeIDCount,100) == 0
    // fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
    // end
    i2 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity((emxArray__common *)T, i2, (int)sizeof(double));
    loop_ub = b_T->size[0] * b_T->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      T->data[i2] = b_T->data[i2];
    }

    nodeIDCount++;
  }

  emxFree_real_T(&c_T);
  emxFree_real_T(&b_T);

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
  i2 = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int)nodeIDCount;
  emxEnsureCapacity((emxArray__common *)d, i2, (int)sizeof(double));
  loop_ub = (int)nodeIDCount;
  for (i2 = 0; i2 < loop_ub; i2++) {
    d->data[i2] = 0.0;
  }

  // parfor i = 1:nodeIDCount
  for (i = 0; i < (int)nodeIDCount; i++) {
    // heuristic Calculates the distance between states x1 and x2.
    // heuristicSingleLeg.m
    // author: wreid
    // date: 20150107
    // Calculate the distance between angular positions.
    xStarA = (((kC->l2 + kC->l3 * cos(nGoal[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(kC->zeta + nGoal[5])) - kC->l7;
    dxStar = ((((kC->l2 + kC->l3 * cos(T->data[i + (T->size[0] << 2)])) + kC->l4
                * cos(kC->zeta)) + kC->l5 * cos(kC->zeta + T->data[i + T->size[0]
                * 5])) - kC->l7) - xStarA;

    // angDiff Finds the angular difference between th1 and th2.
    transitionArrayLength = ((nGoal[3] - T->data[i + T->size[0] * 3]) +
      3.1415926535897931) / 6.2831853071795862;
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
    d->data[i] = sqrt(dxStar * dxStar + xStarA * xStarA * (transitionArrayLength
      * transitionArrayLength));
  }

  ixstart = 1;
  transitionArrayLength = d->data[0];
  itmp = 0;
  if ((int)nodeIDCount > 1) {
    if (rtIsNaN(transitionArrayLength)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int)nodeIDCount)) {
        ixstart = ix;
        if (!rtIsNaN(d->data[ix - 1])) {
          transitionArrayLength = d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
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
  xNearest_size[0] = 1;
  xNearest_size[1] = 11;
  for (i2 = 0; i2 < 11; i2++) {
    xNearest_data[xNearest_size[0] * i2] = T->data[itmp + T->size[0] * i2];
  }

  if (12 > T->size[1]) {
    i2 = -11;
    ix = 0;
  } else {
    i2 = 0;
    ix = T->size[1];
  }

  transitionArrayLength = T->data[itmp];
  ixstart = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = 11;
  emxEnsureCapacity((emxArray__common *)d, ixstart, (int)sizeof(double));
  for (ixstart = 0; ixstart < 11; ixstart++) {
    d->data[ixstart] = xNearest_data[ixstart];
  }

  ixstart = path->size[0] * path->size[1];
  path->size[0] = 1;
  path->size[1] = ix - i2;
  emxEnsureCapacity((emxArray__common *)path, ixstart, (int)sizeof(double));
  for (ixstart = 0; ixstart < 11; ixstart++) {
    path->data[path->size[0] * ixstart] = xNearest_data[xNearest_size[0] *
      ixstart];
  }

  loop_ub = ix - i2;
  for (ix = 0; ix <= loop_ub - 12; ix++) {
    path->data[path->size[0] * (ix + 11)] = T->data[itmp + T->size[0] * ((i2 +
      ix) + 11)];
  }

  emxInit_real_T(&b_path, 2);
  while ((transitionArrayLength != 0.0) && (d->data[1] != 0.0)) {
    transitionArrayLength = d->data[1];
    loop_ub = T->size[1];
    i2 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)d, i2, (int)sizeof(double));
    for (i2 = 0; i2 < loop_ub; i2++) {
      d->data[d->size[0] * i2] = T->data[((int)transitionArrayLength + T->size[0]
        * i2) - 1];
    }

    i2 = b_path->size[0] * b_path->size[1];
    b_path->size[0] = path->size[0] + 1;
    b_path->size[1] = path->size[1];
    emxEnsureCapacity((emxArray__common *)b_path, i2, (int)sizeof(double));
    loop_ub = path->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      ixstart = path->size[0];
      for (ix = 0; ix < ixstart; ix++) {
        b_path->data[ix + b_path->size[0] * i2] = path->data[ix + path->size[0] *
          i2];
      }
    }

    loop_ub = d->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_path->data[path->size[0] + b_path->size[0] * i2] = d->data[d->size[0] *
        i2];
    }

    i2 = path->size[0] * path->size[1];
    path->size[0] = b_path->size[0];
    path->size[1] = b_path->size[1];
    emxEnsureCapacity((emxArray__common *)path, i2, (int)sizeof(double));
    loop_ub = b_path->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      ixstart = b_path->size[0];
      for (ix = 0; ix < ixstart; ix++) {
        path->data[ix + path->size[0] * i2] = b_path->data[ix + b_path->size[0] *
          i2];
      }
    }

    transitionArrayLength = d->data[1];
  }

  emxFree_real_T(&b_path);
  emxFree_real_T(&d);
}

//
// File trailer for buildRRT.cpp
//
// [EOF]
//
