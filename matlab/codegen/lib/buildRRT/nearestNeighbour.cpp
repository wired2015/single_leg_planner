//
// File: nearestNeighbour.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 04-Feb-2015 23:15:06
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "nearestNeighbour.h"
#include "heuristicSingleLeg.h"
#include "buildRRT_emxutil.h"
#include "buildRRT_rtwutil.h"

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
// Arguments    : const double x[11]
//                const emxArray_real_T *T
//                const double kinematicConst[12]
//                double nodeIDCount
//                int NODE_SIZE
//                emxArray_real_T *xNear
//                emxArray_real_T *transitionArray
//                double *d
// Return Type  : void
//
void nearestNeighbour(const double x[11], const emxArray_real_T *T, const double
                      kinematicConst[12], double nodeIDCount, int NODE_SIZE,
                      emxArray_real_T *xNear, emxArray_real_T *transitionArray,
                      double *d)
{
  emxArray_real_T *b_d;
  int ix;
  int ixstart;
  double xStarA;
  double dxStar;
  double dAlpha;
  int itmp;
  boolean_T exitg1;
  long i3;
  int i4;
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
    // heuristic Calculates the distance between states x1 and x2.
    // heuristicSingleLeg.m
    // author: wreid
    // date: 20150107
    // [x1,y1,z1] = fk(alpha1,beta1,gamma1,kinematicConst);
    // [x2,y2,z2] = fk(alpha2,beta2,gamma2,kinematicConst);
    // distMAX = [sqrt(range(1)^2+range(2)+range(3)^2) sqrt(range(4)^2+range(5)^2+range(6)^2)]; 
    // distMAX = [10 1];
    // d = HGAINS(1)*cartDist(xA(4:6),xB(4:6))/distMAX(1) +...
    //     HGAINS(2)*abs(z1-z2);
    // d = HGAINS(1)*cartDist([x1 y1 z1],[x2 y2 z2])/distMAX(1);
    //     HGAINS(2)*cartDist(x1(7:9),x2(7:9))/distMAX(2);
    xStarA = (((kinematicConst[1] + kinematicConst[2] * cos(x[4])) +
               kinematicConst[3] * cos(kinematicConst[8])) + kinematicConst[4] *
              cos(kinematicConst[8] + x[5])) - kinematicConst[6];
    dxStar = ((((kinematicConst[1] + kinematicConst[2] * cos(T->data[ixstart +
      (T->size[0] << 2)])) + kinematicConst[3] * cos(kinematicConst[8])) +
               kinematicConst[4] * cos(kinematicConst[8] + T->data[ixstart +
                T->size[0] * 5])) - kinematicConst[6]) - xStarA;

    // angDiff Finds the angular difference between th1 and th2.
    dAlpha = ((x[3] - T->data[ixstart + T->size[0] * 3]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(dAlpha - rt_roundd_snf(dAlpha)) <= 2.2204460492503131E-16 * fabs
        (dAlpha)) {
      dAlpha = 0.0;
    } else {
      dAlpha = (dAlpha - floor(dAlpha)) * 6.2831853071795862;
    }

    dAlpha = fabs(dAlpha - 3.1415926535897931);
    b_d->data[ixstart] = sqrt(dxStar * dxStar + xStarA * xStarA * (dAlpha *
      dAlpha));
  }

  ixstart = 1;
  dAlpha = b_d->data[0];
  itmp = 0;
  if ((int)nodeIDCount > 1) {
    if (rtIsNaN(dAlpha)) {
      ix = 2;
      exitg1 = false;
      while ((!exitg1) && (ix <= (int)nodeIDCount)) {
        ixstart = ix;
        if (!rtIsNaN(b_d->data[ix - 1])) {
          dAlpha = b_d->data[ix - 1];
          itmp = ix - 1;
          exitg1 = true;
        } else {
          ix++;
        }
      }
    }

    if (ixstart < (int)nodeIDCount) {
      while (ixstart + 1 <= (int)nodeIDCount) {
        if (b_d->data[ixstart] < dAlpha) {
          dAlpha = b_d->data[ixstart];
          itmp = ixstart;
        }

        ixstart++;
      }
    }
  }

  emxFree_real_T(&b_d);
  *d = dAlpha;

  // [d,minIndex] = min(d(1:nodeIDCount));
  if (1 > NODE_SIZE) {
    ixstart = 0;
  } else {
    ixstart = NODE_SIZE;
  }

  ix = xNear->size[0] * xNear->size[1];
  xNear->size[0] = 1;
  xNear->size[1] = ixstart;
  emxEnsureCapacity((emxArray__common *)xNear, ix, (int)sizeof(double));
  for (ix = 0; ix < ixstart; ix++) {
    xNear->data[xNear->size[0] * ix] = T->data[itmp + T->size[0] * ix];
  }

  i3 = NODE_SIZE + 1L;
  if (i3 > 2147483647L) {
    i3 = 2147483647L;
  } else {
    if (i3 < -2147483648L) {
      i3 = -2147483648L;
    }
  }

  ix = (int)i3;
  if (ix > T->size[1]) {
    ix = 0;
    i4 = 0;
  } else {
    ix--;
    i4 = T->size[1];
  }

  ixstart = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = i4 - ix;
  emxEnsureCapacity((emxArray__common *)transitionArray, ixstart, (int)sizeof
                    (double));
  ixstart = i4 - ix;
  for (i4 = 0; i4 < ixstart; i4++) {
    transitionArray->data[transitionArray->size[0] * i4] = T->data[itmp +
      T->size[0] * (ix + i4)];
  }
}

//
// File trailer for nearestNeighbour.cpp
//
// [EOF]
//
