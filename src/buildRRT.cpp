//
// File: buildRRT.cpp
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
#include "selectInput.h"
#include "sherpaTTIK.h"
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
//                const double jointLimits[12]
//                const double b_cartesianLimits[4]
//                const struct0_T *kC
//                double panHeight
//                const double U[10]
//                double Dt
//                double dt
//                double *nodeIDCount
//                const double nGoal[11]
//                const double uBDot[6]
//                int legNum
// Return Type  : void
//
void rrtLoop(emxArray_real_T *T, const double jointLimits[12], const double
             b_cartesianLimits[4], const struct0_T *kC, double panHeight, const
             double U[10], double Dt, double dt, double *nodeIDCount, const
             double nGoal[11], const double uBDot[6], int legNum)
{
  double alphaRand;
  double xMax;
  double xMin;
  double xRand[3];
  double q[3];
  double dv12[9];
  int ix;
  int xRand_size[2];
  double xRand_data[11];
  int xi;
  int b_xi;
  emxArray_real_T *d;
  int n;
  emxArray_real_T *b_T;
  int itmp;
  boolean_T exitg1;
  int xNear_size[2];
  double xNear_data[11];
  int xNew_size[2];
  double xNew_data[11];
  emxArray_real_T b_xNear_data;

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
  alphaRand = jointLimits[0] + (jointLimits[1] - jointLimits[0]) * b_rand();
  if ((panHeight <= b_cartesianLimits[0]) && (panHeight >= b_cartesianLimits[2]))
  {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[4])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[4]);

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos
            (jointLimits[2])) + kC->l5 * cos(asin(((((((kC->l1 - kC->l4 * sin
      (kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * sin(jointLimits[2])) -
      panHeight) / kC->l5));
  } else if ((panHeight < b_cartesianLimits[2]) && (panHeight >=
              b_cartesianLimits[3])) {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[4])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[4]);

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[5])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[5]);
  } else if ((panHeight < b_cartesianLimits[3]) && (panHeight >=
              b_cartesianLimits[1])) {
    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMax = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos
            (jointLimits[3])) + kC->l5 * cos(asin(((((((kC->l1 - kC->l4 * sin
      (kC->zeta)) - kC->l6) - kC->l8) - kC->r) - kC->l3 * sin(jointLimits[3])) -
      panHeight) / kC->l5));

    // GETXSTAR Returns the xStar value given either the height of the wheel contact 
    // point relative to the pan coordinate frame and either the beta or gamma
    // joint value. It is assumed that the angle input represents the beta joint 
    // angle if selector = false, and the angle input is the gamma joint angle if 
    // selector = true
    //
    // Inputs:
    // -z: The height from the pan coordinate frame to the wheel contact point.
    // -angle: The angular value that is either beta or gamma, depending on the
    // selector input.
    // -selector: A logical that indicates that the angle value represents beta
    // -kC: A struct containing the kinematic constants of the Sherpa TT leg.
    // if selector=true, or gamma if selector=false.
    // Outputs:
    // -xStar: The radius in a cylindrical coordinate representation that
    // connects the pan coordinate frame to the wheel contact coordinate frame.
    //
    // sherpaTTFK.m
    // author: wreid
    // date: 20150216
    xMin = (((kC->l2 + kC->l4 * cos(kC->zeta)) - kC->l7) + kC->l3 * cos(asin
             (((-panHeight + ((((kC->l1 - kC->l4 * sin(kC->zeta)) - kC->l6) -
      kC->l8) - kC->r)) - kC->l5 * sin(kC->zeta + jointLimits[5])) / kC->l3))) +
      kC->l5 * cos(kC->zeta + jointLimits[5]);
  } else {
    xMax = 0.0;
    xMin = 0.0;
  }

  xMax = xMin + (xMax - xMin) * b_rand();
  xRand[0] = xMax;
  xRand[1] = 0.0;
  xRand[2] = panHeight;
  b_sherpaTTIK(xRand, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
               kC->l8, kC->zeta, kC->r, jointLimits, q);

  // alphaDotRand = range(4)*rand+MIN(4);
  // gammaDotRand = range(5)*rand+MIN(5);
  // betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi)); 
  for (ix = 0; ix < 3; ix++) {
    dv12[ix] = 0.0;
  }

  dv12[3] = alphaRand;
  dv12[4] = q[1];
  dv12[5] = q[2];
  dv12[6] = 0.0;
  dv12[7] = 0.0;
  dv12[8] = 0.0;
  xRand_size[0] = 1;
  xRand_size[1] = 9;
  for (ix = 0; ix < 9; ix++) {
    xRand_data[ix] = dv12[ix];
  }

  xMax = rt_roundd_snf(*nodeIDCount);
  if (xMax < 2.147483648E+9) {
    if (xMax >= -2.147483648E+9) {
      xi = (int)xMax;
    } else {
      xi = MIN_int32_T;
    }
  } else if (xMax >= 2.147483648E+9) {
    xi = MAX_int32_T;
  } else {
    xi = 0;
  }

  xMax = rt_roundd_snf(*nodeIDCount - floor(*nodeIDCount / 20.0) * 20.0);
  if (xMax < 2.147483648E+9) {
    if (xMax >= -2.147483648E+9) {
      ix = (int)xMax;
    } else {
      ix = MIN_int32_T;
    }
  } else if (xMax >= 2.147483648E+9) {
    ix = MAX_int32_T;
  } else {
    ix = 0;
  }

  if (xi == *nodeIDCount) {
    b_xi = xi - div_s32_floor(xi, 20) * 20;
  } else {
    b_xi = ix;
  }

  if (b_xi == 0) {
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
  ix = d->size[0] * d->size[1];
  d->size[0] = 1;
  d->size[1] = (int)*nodeIDCount;
  emxEnsureCapacity((emxArray__common *)d, ix, (int)sizeof(double));
  n = (int)*nodeIDCount;
  for (ix = 0; ix < n; ix++) {
    d->data[ix] = 0.0;
  }

  // parfor i = 1:nodeIDCount
  xi = 0;
  emxInit_real_T(&b_T, 2);
  while (xi <= (int)*nodeIDCount - 1) {
    n = T->size[1];
    ix = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1;
    b_T->size[1] = n;
    emxEnsureCapacity((emxArray__common *)b_T, ix, (int)sizeof(double));
    for (ix = 0; ix < n; ix++) {
      b_T->data[b_T->size[0] * ix] = T->data[xi + T->size[0] * ix];
    }

    d->data[xi] = heuristicSingleLeg(xRand_data, b_T, kC->l2, kC->l3, kC->l4,
      kC->l5, kC->l7, kC->zeta);
    xi++;
  }

  emxFree_real_T(&b_T);
  xi = 1;
  n = (int)*nodeIDCount;
  xMax = d->data[0];
  itmp = 0;
  if ((int)*nodeIDCount > 1) {
    if (rtIsNaN(xMax)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= n)) {
        xi = ix;
        if (!rtIsNaN(d->data[ix - 1])) {
          xMax = d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (xi < (int)*nodeIDCount) {
      while (xi + 1 <= n) {
        if (d->data[xi] < xMax) {
          xMax = d->data[xi];
          itmp = xi;
        }

        xi++;
      }
    }
  }

  // [d,minIndex] = min(d(1:nodeIDCount));
  xNear_size[0] = 1;
  xNear_size[1] = 11;
  for (ix = 0; ix < 11; ix++) {
    xNear_data[xNear_size[0] * ix] = T->data[itmp + T->size[0] * ix];
  }

  selectInput(xNear_data, xNear_size, xRand_data, xRand_size, U, dt, Dt, kC,
              jointLimits, uBDot, legNum, xNew_data, xNew_size, d);
  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  // Node ID
  xNew_data[1] = T->data[itmp];

  // Parent ID
  b_xNear_data.data = (double *)&xNear_data;
  b_xNear_data.size = (int *)&xNear_size;
  b_xNear_data.allocatedSize = 11;
  b_xNear_data.numDimensions = 2;
  b_xNear_data.canFreeData = false;
  xNew_data[2] = heuristicSingleLeg(xNew_data, &b_xNear_data, kC->l2, kC->l3,
    kC->l4, kC->l5, kC->l7, kC->zeta);

  // Cost
  xi = (int)*nodeIDCount - 1;
  for (ix = 0; ix < 11; ix++) {
    T->data[xi + T->size[0] * ix] = xNew_data[xNew_size[0] * ix];
  }

  n = d->size[1];
  for (ix = 0; ix < n; ix++) {
    T->data[xi + T->size[0] * (ix + 11)] = d->data[d->size[0] * ix];
  }

  emxFree_real_T(&d);

  // Append the new node to the tree.
  // if mod(nodeIDCount,100) == 0
  // fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
  // end
}

//
// File trailer for buildRRT.cpp
//
// [EOF]
//
