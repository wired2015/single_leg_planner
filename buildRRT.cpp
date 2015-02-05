//
// File: buildRRT.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Feb-2015 15:38:22
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_emxutil.h"
#include "selectInput.h"
#include "rand.h"
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
// Arguments    : emxArray_real_T *T
//                const double jointRange[6]
//                const double jointLimits[12]
//                const double kinematicConst[12]
//                double K
//                const double U[10]
//                double Dt
//                double dt
//                int NODE_SIZE
//                int U_SIZE
//                const double HGAINS[3]
//                double ankleThreshold
//                double *nodeIDCount
//                const double nGoal[11]
//                int goalSeedFreq
// Return Type  : void
//
void rrtLoop(emxArray_real_T *T, const double jointRange[6], const double
             jointLimits[12], const double kinematicConst[12], double K, const
             double U[10], double Dt, double dt, int NODE_SIZE, int U_SIZE,
             const double HGAINS[3], double ankleThreshold, double *nodeIDCount,
             const double nGoal[11], int goalSeedFreq)
{
  double alphaRand;
  double gammaRand;
  double y;
  double betaRand;
  double alphaDotRand;
  double gammaDotRand;
  double dv0[9];
  int n;
  int xRand_size[2];
  double xRand_data[11];
  int xi;
  emxArray_real_T *d;
  int loop_ub;
  emxArray_real_T *b_T;
  emxArray_real_T b_xRand_data;
  int itmp;
  boolean_T exitg1;
  emxArray_real_T *xNew;
  emxArray_real_T *b_xNew;
  emxArray_real_T *transitionArray;

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
  alphaRand = jointRange[0] * b_rand() + jointLimits[0];
  gammaRand = jointRange[1] * b_rand() + jointLimits[2];
  y = (((((K - kinematicConst[0]) + kinematicConst[3] * sin(kinematicConst[8]))
         + kinematicConst[4] * sin(gammaRand + kinematicConst[8])) +
        kinematicConst[5]) + kinematicConst[7]) / kinematicConst[2];
  betaRand = -asin(y);
  alphaDotRand = jointRange[3] * b_rand() + jointLimits[6];
  gammaDotRand = jointRange[4] * b_rand() + jointLimits[8];
  for (n = 0; n < 3; n++) {
    dv0[n] = 0.0;
  }

  dv0[3] = alphaRand;
  dv0[4] = gammaRand;
  dv0[5] = -asin(y);
  dv0[6] = alphaDotRand;
  dv0[7] = gammaDotRand;
  dv0[8] = -(((((((((((((((((((((((((2.238E+31 * kinematicConst[1] *
    alphaDotRand - 2.238E+31 * kinematicConst[5] * alphaDotRand) - 1.827E+47 *
    kinematicConst[5] * gammaDotRand) + 2.238E+31 * kinematicConst[2] *
    alphaDotRand * cos(betaRand)) + 1.827E+47 * kinematicConst[2] * gammaDotRand
    * cos(betaRand)) - 2.238E+31 * kinematicConst[1] * alphaDotRand) + 2.238E+31
    * kinematicConst[5] * alphaDotRand) - 1.37E+15 * kinematicConst[5] *
    gammaDotRand) + 2.238E+31 * kinematicConst[3] * alphaDotRand * cos
    (kinematicConst[8])) + 1.827E+47 * kinematicConst[3] * gammaDotRand * cos
    (kinematicConst[8])) + 2.74E+15 * kinematicConst[6] * alphaDotRand * 0.0) +
    2.74E+15 * kinematicConst[7] * alphaDotRand * 0.0) + 2.238E+31 *
    kinematicConst[6] * gammaDotRand * 0.0) + 2.238E+31 * kinematicConst[7] *
    gammaDotRand * 0.0) - 2.237E+31 * kinematicConst[2] * alphaDotRand * cos
                        (betaRand)) + 2.238E+31 * kinematicConst[4] *
                       alphaDotRand * cos(gammaRand) * cos(kinematicConst[8])) +
                      1.827E+47 * kinematicConst[4] * gammaDotRand * cos
                      (gammaRand) * cos(kinematicConst[8])) - 2.237E+31 *
                     kinematicConst[3] * alphaDotRand * cos(kinematicConst[8]))
                    + 2.237E+31 * kinematicConst[2] * gammaDotRand * sin
                    (betaRand) * 0.0) - 2.238E+31 * kinematicConst[4] *
                   alphaDotRand * sin(gammaRand) * sin(kinematicConst[8])) -
                  1.827E+47 * kinematicConst[4] * gammaDotRand * sin(gammaRand) *
                  sin(kinematicConst[8])) + 2.237E+31 * kinematicConst[3] *
                 gammaDotRand * 0.0 * sin(kinematicConst[8])) - 2.237E+31 *
                kinematicConst[4] * alphaDotRand * cos(gammaRand) * cos
                (kinematicConst[8])) + 2.237E+31 * kinematicConst[4] *
               alphaDotRand * sin(gammaRand) * sin(kinematicConst[8])) +
              2.237E+31 * kinematicConst[4] * gammaDotRand * cos(gammaRand) *
              0.0 * sin(kinematicConst[8])) + 2.237E+31 * kinematicConst[4] *
             gammaDotRand * sin(gammaRand) * cos(kinematicConst[8]) * 0.0) /
    (((((((((1.827E+47 * kinematicConst[3] * cos(kinematicConst[8]) - 1.37E+15 *
             kinematicConst[5]) - 1.827E+47 * kinematicConst[5]) + 2.238E+31 *
           kinematicConst[6] * 0.0) + 2.238E+31 * kinematicConst[7] * 0.0) +
         1.827E+47 * kinematicConst[4] * cos(gammaRand) * cos(kinematicConst[8]))
        - 1.827E+47 * kinematicConst[4] * sin(gammaRand) * sin(kinematicConst[8]))
       + 2.237E+31 * kinematicConst[3] * 0.0 * sin(kinematicConst[8])) +
      2.237E+31 * kinematicConst[4] * cos(gammaRand) * 0.0 * sin(kinematicConst
       [8])) + 2.237E+31 * kinematicConst[4] * sin(gammaRand) * cos
     (kinematicConst[8]) * 0.0);
  xRand_size[0] = 1;
  xRand_size[1] = 9;
  for (n = 0; n < 9; n++) {
    xRand_data[xRand_size[0] * n] = dv0[n];
  }

  alphaRand = rt_roundd_snf(*nodeIDCount);
  if (alphaRand < 2.147483648E+9) {
    if (alphaRand >= -2.147483648E+9) {
      xi = (int)alphaRand;
    } else {
      xi = MIN_int32_T;
    }
  } else if (alphaRand >= 2.147483648E+9) {
    xi = MAX_int32_T;
  } else {
    xi = 0;
  }

  if (xi == *nodeIDCount) {
    if (goalSeedFreq == 0) {
    } else {
      xi -= div_s32_floor(xi, goalSeedFreq) * goalSeedFreq;
    }
  } else {
    if (goalSeedFreq == 0) {
      alphaRand = *nodeIDCount;
    } else {
      alphaRand = *nodeIDCount - floor(*nodeIDCount / (double)goalSeedFreq) *
        (double)goalSeedFreq;
    }

    alphaRand = rt_roundd_snf(alphaRand);
    if (alphaRand < 2.147483648E+9) {
      if (alphaRand >= -2.147483648E+9) {
        xi = (int)alphaRand;
      } else {
        xi = MIN_int32_T;
      }
    } else if (alphaRand >= 2.147483648E+9) {
      xi = MAX_int32_T;
    } else {
      xi = 0;
    }
  }

  if (xi == 0) {
    xRand_size[0] = 1;
    xRand_size[1] = 11;
    memcpy(&xRand_data[0], &nGoal[0], 11U * sizeof(double));
  }

  emxInit_real_T(&d, 2);

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
  n = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int)*nodeIDCount;
  emxEnsureCapacity((emxArray__common *)d, n, (int)sizeof(double));
  loop_ub = (int)*nodeIDCount;
  for (n = 0; n < loop_ub; n++) {
    d->data[n] = 0.0;
  }

  // parfor i = 1:nodeIDCount
  xi = 0;
  emxInit_real_T(&b_T, 2);
  while (xi <= (int)*nodeIDCount - 1) {
    loop_ub = T->size[1];
    n = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)b_T, n, (int)sizeof(double));
    for (n = 0; n < loop_ub; n++) {
      b_T->data[b_T->size[0] * n] = T->data[xi + T->size[0] * n];
    }

    b_xRand_data.data = (double *)&xRand_data;
    b_xRand_data.size = (int *)&xRand_size;
    b_xRand_data.allocatedSize = 11;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    d->data[xi] = heuristicSingleLeg(&b_xRand_data, b_T, kinematicConst);
    xi++;
  }

  emxFree_real_T(&b_T);
  xi = 1;
  n = (int)*nodeIDCount;
  alphaRand = d->data[0];
  itmp = 0;
  if ((int)*nodeIDCount > 1) {
    if (rtIsNaN(alphaRand)) {
      loop_ub = 2;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= n)) {
        xi = loop_ub;
        if (!rtIsNaN(d->data[loop_ub - 1])) {
          alphaRand = d->data[loop_ub - 1];
          itmp = loop_ub - 1;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
    }

    if (xi < (int)*nodeIDCount) {
      for (loop_ub = xi + 1; loop_ub <= n; loop_ub++) {
        if (d->data[loop_ub - 1] < alphaRand) {
          alphaRand = d->data[loop_ub - 1];
          itmp = loop_ub - 1;
        }
      }
    }
  }

  // [d,minIndex] = min(d(1:nodeIDCount));
  if (1 > NODE_SIZE) {
    loop_ub = 0;
  } else {
    loop_ub = NODE_SIZE;
  }

  n = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = loop_ub;
  emxEnsureCapacity((emxArray__common *)d, n, (int)sizeof(double));
  for (n = 0; n < loop_ub; n++) {
    d->data[d->size[0] * n] = T->data[itmp + T->size[0] * n];
  }

  emxInit_real_T(&xNew, 2);
  emxInit_real_T(&b_xNew, 2);
  emxInit_real_T(&transitionArray, 2);
  selectInput(d, xRand_data, xRand_size, U, dt, Dt, NODE_SIZE, U_SIZE, HGAINS,
              kinematicConst, ankleThreshold, jointLimits, b_xNew,
              transitionArray);
  n = xNew->size[0] * xNew->size[1];
  xNew->size[0] = 1;
  xNew->size[1] = b_xNew->size[1];
  emxEnsureCapacity((emxArray__common *)xNew, n, (int)sizeof(double));
  loop_ub = b_xNew->size[0] * b_xNew->size[1];
  for (n = 0; n < loop_ub; n++) {
    xNew->data[n] = b_xNew->data[n];
  }

  emxFree_real_T(&b_xNew);
  (*nodeIDCount)++;
  xNew->data[0] = *nodeIDCount;

  // Node ID
  xNew->data[1] = T->data[itmp];

  // Parent ID
  xNew->data[2] = heuristicSingleLeg(xNew, d, kinematicConst);

  // Cost
  xi = (int)*nodeIDCount - 1;
  loop_ub = xNew->size[1];
  emxFree_real_T(&d);
  for (n = 0; n < loop_ub; n++) {
    T->data[xi + T->size[0] * n] = xNew->data[xNew->size[0] * n];
  }

  loop_ub = transitionArray->size[1];
  for (n = 0; n < loop_ub; n++) {
    T->data[xi + T->size[0] * (n + xNew->size[1])] = transitionArray->
      data[transitionArray->size[0] * n];
  }

  emxFree_real_T(&transitionArray);
  emxFree_real_T(&xNew);

  // Append the new node to the tree.
}

//
// File trailer for buildRRT.cpp
//
// [EOF]
//
