//
// File: buildRRTWrapper.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Feb-2015 15:38:22
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRT.h"
#include "buildRRTWrapper_emxutil.h"
#include "heuristicSingleLeg.h"
#include "nearestNeighbour.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Declarations
static boolean_T validState(const double n[6], const double jointLimits[12]);

// Function Definitions

//
// Arguments    : const double n[6]
//                const double jointLimits[12]
// Return Type  : boolean_T
//
static boolean_T validState(const double n[6], const double jointLimits[12])
{
  boolean_T valid;
  if ((n[0] < jointLimits[0]) || (n[0] > jointLimits[1]) || (n[1] < jointLimits
       [2]) || (n[1] > jointLimits[3]) || (n[2] < jointLimits[4]) || (n[2] >
       jointLimits[5]) || (n[3] < jointLimits[6]) || (n[3] > jointLimits[7]) ||
      (n[4] < jointLimits[8]) || (n[4] > jointLimits[9]) || (n[5] < jointLimits
       [10]) || (n[5] > jointLimits[11])) {
    valid = false;
  } else {
    valid = true;
  }

  return valid;
}

//
// Transform the nInit and nGoal variables to joint space.
// Arguments    : const double nInit[6]
//                const double nGoal[6]
//                int NUM_NODES
//                const double jointLimits[12]
//                double K
//                const double HGAINS[3]
//                int NODE_SIZE
//                const double U[10]
//                int U_SIZE
//                double dt
//                double Dt
//                const double kinematicConst[12]
//                double ankleThreshold
//                boolean_T exhaustive
//                double threshold
//                int goalSeedFreq
//                emxArray_real_T *T
//                emxArray_real_T *path
//                emxArray_real_T *pathJoint
// Return Type  : void
//
void buildRRTWrapper(const double nInit[6], const double nGoal[6], int NUM_NODES,
                     const double jointLimits[12], double K, const double
                     HGAINS[3], int NODE_SIZE, const double U[10], int U_SIZE,
                     double dt, double Dt, const double kinematicConst[12],
                     double ankleThreshold, boolean_T exhaustive, double
                     threshold, int goalSeedFreq, emxArray_real_T *T,
                     emxArray_real_T *path, emxArray_real_T *pathJoint)
{
  double gammaInit;
  double betaInit;
  double alphaInit;
  double gammaGoal;
  double transitionArrayLength;
  double dist;
  double qDotGoal[3];
  double qDotInit[3];
  double alphaGoal[3];
  double jointRange[6];
  emxArray_real_T *b_path;
  emxArray_real_T *next;
  emxArray_real_T *unusedU0;
  emxArray_real_T *r0;
  emxArray_real_T *c_path;
  emxArray_real_T *b_pathJoint;
  boolean_T guard1 = false;
  double b_alphaGoal[6];
  double b_nGoal[11];
  int i0;
  int unnamed_idx_0;
  int md2;
  int i;
  double unusedU2;
  int loop_ub;
  unsigned int count;
  long i1;
  int j;
  double c_pathJoint[3];
  double d_pathJoint[3];
  double b_qDotInit[3];
  long i2;

  // buildRRTWrapper.m
  // author: wreid
  // date: 20150502
  sherpaTTIK(nInit[0], nInit[1], nInit[2], kinematicConst, jointLimits,
             &alphaInit, &betaInit, &gammaInit);
  sherpaTTIK(nGoal[0], nGoal[1], nGoal[2], kinematicConst, jointLimits, &dist,
             &transitionArrayLength, &gammaGoal);
  qDotGoal[0] = alphaInit;
  qDotGoal[1] = betaInit;
  qDotGoal[2] = gammaInit;
  sherpaTTIKVel(*(double (*)[3])&nInit[3], qDotGoal, kinematicConst, qDotInit);
  alphaGoal[0] = dist;
  alphaGoal[1] = transitionArrayLength;
  alphaGoal[2] = gammaGoal;
  sherpaTTIKVel(*(double (*)[3])&nGoal[3], alphaGoal, kinematicConst, qDotGoal);

  // Check that the initial and final positions are valid. If they are not
  // return failure and an empty path.
  jointRange[0] = alphaInit;
  jointRange[1] = betaInit;
  jointRange[2] = gammaInit;
  jointRange[3] = qDotInit[0];
  jointRange[4] = qDotInit[1];
  jointRange[5] = qDotInit[2];
  emxInit_real_T(&b_path, 2);
  emxInit_real_T(&next, 2);
  emxInit_real_T(&unusedU0, 2);
  emxInit_real_T(&r0, 2);
  b_emxInit_real_T(&c_path, 1);
  emxInit_real_T(&b_pathJoint, 2);
  guard1 = false;
  if (validState(jointRange, jointLimits)) {
    b_alphaGoal[0] = dist;
    b_alphaGoal[1] = transitionArrayLength;
    b_alphaGoal[2] = gammaGoal;
    b_alphaGoal[3] = qDotGoal[0];
    b_alphaGoal[4] = qDotGoal[1];
    b_alphaGoal[5] = qDotGoal[2];
    if (validState(b_alphaGoal, jointLimits)) {
      // success = true;
      // Run buildRRT.
      b_nGoal[0] = 0.0;
      b_nGoal[1] = 0.0;
      b_nGoal[2] = 0.0;
      b_nGoal[3] = dist;
      b_nGoal[4] = transitionArrayLength;
      b_nGoal[5] = gammaGoal;
      b_nGoal[6] = qDotGoal[0];
      b_nGoal[7] = qDotGoal[1];
      b_nGoal[8] = qDotGoal[2];
      b_nGoal[9] = 0.0;
      b_nGoal[10] = 0.0;

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
      transitionArrayLength = (rt_roundd_snf(Dt / dt) + 1.0) * 6.0;

      // Variable Initialization
      i0 = T->size[0] * T->size[1];
      T->size[0] = NUM_NODES;
      dist = rt_roundd_snf((double)NODE_SIZE + transitionArrayLength);
      if (dist < 2.147483648E+9) {
        if (dist >= -2.147483648E+9) {
          unnamed_idx_0 = (int)dist;
        } else {
          unnamed_idx_0 = MIN_int32_T;
        }
      } else if (dist >= 2.147483648E+9) {
        unnamed_idx_0 = MAX_int32_T;
      } else {
        unnamed_idx_0 = 0;
      }

      T->size[1] = unnamed_idx_0;
      emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
      dist = rt_roundd_snf((double)NODE_SIZE + transitionArrayLength);
      if (dist < 2.147483648E+9) {
        if (dist >= -2.147483648E+9) {
          i0 = (int)dist;
        } else {
          i0 = MIN_int32_T;
        }
      } else if (dist >= 2.147483648E+9) {
        i0 = MAX_int32_T;
      } else {
        i0 = 0;
      }

      md2 = NUM_NODES * i0;
      for (i0 = 0; i0 < md2; i0++) {
        T->data[i0] = 0.0;
      }

      // Define a zero array that will be used to
      // store data from each tree node.
      T->data[0] = 0.0;
      T->data[T->size[0]] = 0.0;
      T->data[T->size[0] << 1] = 0.0;
      T->data[T->size[0] * 3] = alphaInit;
      T->data[T->size[0] << 2] = betaInit;
      T->data[T->size[0] * 5] = gammaInit;
      T->data[T->size[0] * 6] = qDotInit[0];
      T->data[T->size[0] * 7] = qDotInit[1];
      T->data[T->size[0] << 3] = qDotInit[2];
      T->data[T->size[0] * 9] = 0.0;
      T->data[T->size[0] * 10] = 0.0;
      md2 = (int)transitionArrayLength;
      for (i0 = 0; i0 < md2; i0++) {
        T->data[T->size[0] * (i0 + 11)] = 0.0;
      }

      // Initialize the tree with initial state.
      transitionArrayLength = 1.0;
      for (i0 = 0; i0 < 6; i0++) {
        jointRange[i0] = jointLimits[1 + (i0 << 1)] - jointLimits[i0 << 1];
      }

      if (!exhaustive) {
        for (i = 2; i <= NUM_NODES; i++) {
          rrtLoop(T, jointRange, jointLimits, kinematicConst, K, U, Dt, dt,
                  NODE_SIZE, U_SIZE, HGAINS, ankleThreshold,
                  &transitionArrayLength, b_nGoal, goalSeedFreq);
        }
      } else {
        // TODO: calculate maximum distance given the configuration space.
        dist = 100.0;

        // TODO: make the threshold distance
        while ((dist > threshold) && (transitionArrayLength < NUM_NODES)) {
          rrtLoop(T, jointRange, jointLimits, kinematicConst, K, U, Dt, dt,
                  NODE_SIZE, U_SIZE, HGAINS, ankleThreshold,
                  &transitionArrayLength, b_nGoal, goalSeedFreq);
          nearestNeighbour(b_nGoal, T, kinematicConst, transitionArrayLength,
                           NODE_SIZE, unusedU0, next, &dist);
        }
      }

      // Find the closest node in the tree to the goal node.
      nearestNeighbour(b_nGoal, T, kinematicConst, transitionArrayLength,
                       NODE_SIZE, next, unusedU0, &unusedU2);
      transitionArrayLength = next->data[0];
      i0 = pathJoint->size[0] * pathJoint->size[1];
      pathJoint->size[0] = 1;
      pathJoint->size[1] = next->size[1] + unusedU0->size[1];
      emxEnsureCapacity((emxArray__common *)pathJoint, i0, (int)sizeof(double));
      md2 = next->size[1];
      for (i0 = 0; i0 < md2; i0++) {
        pathJoint->data[pathJoint->size[0] * i0] = next->data[next->size[0] * i0];
      }

      md2 = unusedU0->size[1];
      for (i0 = 0; i0 < md2; i0++) {
        pathJoint->data[pathJoint->size[0] * (i0 + next->size[1])] =
          unusedU0->data[unusedU0->size[0] * i0];
      }

      while ((transitionArrayLength != 0.0) && (next->data[1] != 0.0)) {
        transitionArrayLength = next->data[1];
        md2 = T->size[1];
        i0 = next->size[0] * next->size[1];
        next->size[0] = 1;
        next->size[1] = md2;
        emxEnsureCapacity((emxArray__common *)next, i0, (int)sizeof(double));
        for (i0 = 0; i0 < md2; i0++) {
          next->data[next->size[0] * i0] = T->data[((int)transitionArrayLength +
            T->size[0] * i0) - 1];
        }

        i0 = b_pathJoint->size[0] * b_pathJoint->size[1];
        b_pathJoint->size[0] = pathJoint->size[0] + 1;
        b_pathJoint->size[1] = pathJoint->size[1];
        emxEnsureCapacity((emxArray__common *)b_pathJoint, i0, (int)sizeof
                          (double));
        md2 = pathJoint->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          loop_ub = pathJoint->size[0];
          for (unnamed_idx_0 = 0; unnamed_idx_0 < loop_ub; unnamed_idx_0++) {
            b_pathJoint->data[unnamed_idx_0 + b_pathJoint->size[0] * i0] =
              pathJoint->data[unnamed_idx_0 + pathJoint->size[0] * i0];
          }
        }

        md2 = next->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          b_pathJoint->data[pathJoint->size[0] + b_pathJoint->size[0] * i0] =
            next->data[next->size[0] * i0];
        }

        i0 = pathJoint->size[0] * pathJoint->size[1];
        pathJoint->size[0] = b_pathJoint->size[0];
        pathJoint->size[1] = b_pathJoint->size[1];
        emxEnsureCapacity((emxArray__common *)pathJoint, i0, (int)sizeof(double));
        md2 = b_pathJoint->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          loop_ub = b_pathJoint->size[0];
          for (unnamed_idx_0 = 0; unnamed_idx_0 < loop_ub; unnamed_idx_0++) {
            pathJoint->data[unnamed_idx_0 + pathJoint->size[0] * i0] =
              b_pathJoint->data[unnamed_idx_0 + b_pathJoint->size[0] * i0];
          }
        }

        transitionArrayLength = next->data[1];
      }

      // Transform path back to the Cartesian space.
      // Take the pathOld array and combine the general nodes and intermediate
      // states into a uniform path. The output path should be a npx6 array
      // that contains the n general nodes and the p intermediate nodes between
      // general nodes. Each row in the path matrix contains
      // [t,x,y,z,xDot,yDot,zDot] state data.
      transitionArrayLength = rt_roundd_snf(Dt / dt);
      md2 = (int)(transitionArrayLength * (double)pathJoint->size[0]);
      i0 = b_path->size[0] * b_path->size[1];
      b_path->size[0] = md2;
      b_path->size[1] = 7;
      emxEnsureCapacity((emxArray__common *)b_path, i0, (int)sizeof(double));
      md2 = (int)(transitionArrayLength * (double)pathJoint->size[0]) * 7;
      for (i0 = 0; i0 < md2; i0++) {
        b_path->data[i0] = 0.0;
      }

      count = 1U;
      for (i = 0; i < pathJoint->size[0]; i++) {
        i0 = pathJoint->size[1];
        i1 = NODE_SIZE + 7L;
        if (i1 > 2147483647L) {
          i1 = 2147483647L;
        } else {
          if (i1 < -2147483648L) {
            i1 = -2147483648L;
          }
        }

        for (j = (int)i1; j <= i0; j += 6) {
          // sherpaTTFK Sherpa_TT Forward Kinematics
          //    Calculates the x,y,z position of the contact point given the alpha, 
          //    beta and gamma joint values.
          // sherpaTTFK.m
          // author: wreid
          // date: 20150122
          i1 = j + 3L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          c_pathJoint[0] = pathJoint->data[i + pathJoint->size[0] * ((int)i1 - 1)];
          i1 = j + 4L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          c_pathJoint[1] = pathJoint->data[i + pathJoint->size[0] * ((int)i1 - 1)];
          i1 = j + 5L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          c_pathJoint[2] = pathJoint->data[i + pathJoint->size[0] * ((int)i1 - 1)];
          for (unnamed_idx_0 = 0; unnamed_idx_0 < 3; unnamed_idx_0++) {
            qDotInit[unnamed_idx_0] = c_pathJoint[unnamed_idx_0];
          }

          d_pathJoint[0] = pathJoint->data[i + pathJoint->size[0] * (j - 1)];
          i1 = j + 1L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          d_pathJoint[1] = pathJoint->data[i + pathJoint->size[0] * ((int)i1 - 1)];
          i1 = j + 2L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          d_pathJoint[2] = pathJoint->data[i + pathJoint->size[0] * ((int)i1 - 1)];
          for (unnamed_idx_0 = 0; unnamed_idx_0 < 3; unnamed_idx_0++) {
            qDotGoal[unnamed_idx_0] = d_pathJoint[unnamed_idx_0];
          }

          // sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
          // sherpaTTFKVel.m
          // author: wreid
          // date: 20150122
          b_qDotInit[0] = (-qDotInit[0] * sin(qDotGoal[0]) * ((((kinematicConst
            [1] - kinematicConst[6]) + kinematicConst[4] * cos(qDotGoal[2] +
            kinematicConst[8])) + kinematicConst[2] * cos(qDotGoal[1])) +
            kinematicConst[3] * cos(kinematicConst[8])) - qDotInit[1] *
                           kinematicConst[2] * cos(qDotGoal[0]) * sin(qDotGoal[1]))
            - qDotInit[2] * kinematicConst[4] * sin(qDotGoal[2] +
            kinematicConst[8]) * cos(qDotGoal[0]);
          b_qDotInit[1] = (qDotInit[0] * cos(qDotGoal[0]) * ((((kinematicConst[1]
            - kinematicConst[6]) + kinematicConst[4] * cos(qDotGoal[2] +
            kinematicConst[8])) + kinematicConst[2] * cos(qDotGoal[1])) +
            kinematicConst[3] * cos(kinematicConst[8])) - qDotInit[2] *
                           kinematicConst[4] * sin(qDotGoal[2] + kinematicConst
            [8]) * sin(qDotGoal[0])) - qDotInit[1] * kinematicConst[2] * sin
            (qDotGoal[0]) * sin(qDotGoal[1]);
          b_qDotInit[2] = -qDotInit[1] * kinematicConst[2] * cos(qDotGoal[1]) -
            kinematicConst[4] * qDotInit[2] * cos(kinematicConst[8] + qDotGoal[2]);
          b_path->data[(int)count - 1] = (double)count * dt;
          i1 = j + 1L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          i2 = j + 2L;
          if (i2 > 2147483647L) {
            i2 = 2147483647L;
          } else {
            if (i2 < -2147483648L) {
              i2 = -2147483648L;
            }
          }

          b_path->data[((int)count + b_path->size[0]) - 1] = ((((kinematicConst
            [1] + kinematicConst[2] * cos(-pathJoint->data[i + pathJoint->size[0]
            * ((int)i1 - 1)])) + kinematicConst[3] * cos(kinematicConst[8])) +
            kinematicConst[4] * cos(pathJoint->data[i + pathJoint->size[0] *
            ((int)i2 - 1)] + kinematicConst[8])) - kinematicConst[6]) * cos
            (pathJoint->data[i + pathJoint->size[0] * (j - 1)]);
          i1 = j + 1L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          i2 = j + 2L;
          if (i2 > 2147483647L) {
            i2 = 2147483647L;
          } else {
            if (i2 < -2147483648L) {
              i2 = -2147483648L;
            }
          }

          b_path->data[((int)count + (b_path->size[0] << 1)) - 1] =
            ((((kinematicConst[1] + kinematicConst[2] * cos(-pathJoint->data[i +
                 pathJoint->size[0] * ((int)i1 - 1)])) + kinematicConst[3] * cos
               (kinematicConst[8])) + kinematicConst[4] * cos(pathJoint->data[i
               + pathJoint->size[0] * ((int)i2 - 1)] + kinematicConst[8])) -
             kinematicConst[6]) * sin(pathJoint->data[i + pathJoint->size[0] *
            (j - 1)]);
          i1 = j + 1L;
          if (i1 > 2147483647L) {
            i1 = 2147483647L;
          } else {
            if (i1 < -2147483648L) {
              i1 = -2147483648L;
            }
          }

          i2 = j + 2L;
          if (i2 > 2147483647L) {
            i2 = 2147483647L;
          } else {
            if (i2 < -2147483648L) {
              i2 = -2147483648L;
            }
          }

          b_path->data[((int)count + b_path->size[0] * 3) - 1] =
            ((((kinematicConst[0] + kinematicConst[2] * sin(-pathJoint->data[i +
                 pathJoint->size[0] * ((int)i1 - 1)])) - kinematicConst[3] * sin
               (kinematicConst[8])) - kinematicConst[4] * sin(pathJoint->data[i
               + pathJoint->size[0] * ((int)i2 - 1)] + kinematicConst[8])) -
             kinematicConst[5]) - kinematicConst[7];
          for (unnamed_idx_0 = 0; unnamed_idx_0 < 3; unnamed_idx_0++) {
            b_path->data[((int)count + b_path->size[0] * (unnamed_idx_0 + 4)) -
              1] = b_qDotInit[unnamed_idx_0];
          }

          count++;
        }
      }

      md2 = b_path->size[0];
      i0 = r0->size[0] * r0->size[1];
      r0->size[0] = md2;
      r0->size[1] = 6;
      emxEnsureCapacity((emxArray__common *)r0, i0, (int)sizeof(double));
      for (i0 = 0; i0 < 6; i0++) {
        for (unnamed_idx_0 = 0; unnamed_idx_0 < md2; unnamed_idx_0++) {
          r0->data[unnamed_idx_0 + r0->size[0] * i0] = b_path->
            data[unnamed_idx_0 + b_path->size[0] * (1 + i0)];
        }
      }

      i0 = b_path->size[0];
      unnamed_idx_0 = b_path->size[0];
      md2 = unnamed_idx_0 >> 1;
      for (j = 0; j < 6; j++) {
        for (i = 1; i <= md2; i++) {
          loop_ub = r0->size[0];
          transitionArrayLength = r0->data[(i + loop_ub * j) - 1];
          loop_ub = r0->size[0];
          unnamed_idx_0 = r0->size[0];
          r0->data[(i + loop_ub * j) - 1] = r0->data[(i0 - i) + unnamed_idx_0 *
            j];
          loop_ub = r0->size[0];
          r0->data[(i0 - i) + loop_ub * j] = transitionArrayLength;
        }
      }

      md2 = b_path->size[0];
      loop_ub = b_path->size[0];
      i0 = c_path->size[0];
      c_path->size[0] = loop_ub;
      emxEnsureCapacity((emxArray__common *)c_path, i0, (int)sizeof(double));
      for (i0 = 0; i0 < loop_ub; i0++) {
        c_path->data[i0] = b_path->data[i0];
      }

      i0 = path->size[0] * path->size[1];
      path->size[0] = 1 + md2;
      path->size[1] = 7;
      emxEnsureCapacity((emxArray__common *)path, i0, (int)sizeof(double));
      path->data[0] = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        path->data[path->size[0] * (i0 + 1)] = nInit[i0];
      }

      for (i0 = 0; i0 < md2; i0++) {
        path->data[i0 + 1] = c_path->data[i0];
      }

      for (i0 = 0; i0 < 6; i0++) {
        md2 = r0->size[0];
        for (unnamed_idx_0 = 0; unnamed_idx_0 < md2; unnamed_idx_0++) {
          path->data[(unnamed_idx_0 + path->size[0] * (i0 + 1)) + 1] = r0->
            data[unnamed_idx_0 + r0->size[0] * i0];
        }
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    // success = false;
    i0 = path->size[0] * path->size[1];
    path->size[0] = 0;
    path->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)path, i0, (int)sizeof(double));
    i0 = pathJoint->size[0] * pathJoint->size[1];
    pathJoint->size[0] = 0;
    pathJoint->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)pathJoint, i0, (int)sizeof(double));
    i0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)T, i0, (int)sizeof(double));
  }

  emxFree_real_T(&b_pathJoint);
  emxFree_real_T(&c_path);
  emxFree_real_T(&r0);
  emxFree_real_T(&unusedU0);
  emxFree_real_T(&next);
  emxFree_real_T(&b_path);
}

//
// File trailer for buildRRTWrapper.cpp
//
// [EOF]
//
