/*
 * File: buildBiDirectionalRRT.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildBiDirectionalRRT.h"
#include "buildBiDirectionalRRTWrapper_emxutil.h"
#include "norm.h"
#include "rk4.h"
#include "nearestNeighbour.h"
#include "randomState.h"
#include "heuristicSingleLeg.h"
#include "flipud.h"
#include "buildBiDirectionalRRTWrapper_rtwutil.h"
#include <stdio.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_1x13
#define struct_emxArray_real_T_1x13

struct emxArray_real_T_1x13
{
  double data[13];
  int size[2];
};

#endif                                 /*struct_emxArray_real_T_1x13*/

#ifndef typedef_emxArray_real_T_1x13
#define typedef_emxArray_real_T_1x13

typedef struct emxArray_real_T_1x13 emxArray_real_T_1x13;

#endif                                 /*typedef_emxArray_real_T_1x13*/

/* Function Declarations */
static void rrtLoop(emxArray_real_T *T, const double jointLimits[20], const
                    struct0_T *kC, double panHeight, const double U[18], double
                    Dt, double dt, double *nodeIDCount, const double uBDot[6],
                    int legNum, double xNew_data[], int xNew_size[2]);
static void traceBranch(const emxArray_real_T *T, const double midPoint_data[],
  emxArray_real_T *path);

/* Function Definitions */

/*
 * Arguments    : emxArray_real_T *T
 *                const double jointLimits[20]
 *                const struct0_T *kC
 *                double panHeight
 *                const double U[18]
 *                double Dt
 *                double dt
 *                double *nodeIDCount
 *                const double uBDot[6]
 *                int legNum
 *                double xNew_data[]
 *                int xNew_size[2]
 * Return Type  : void
 */
static void rrtLoop(emxArray_real_T *T, const double jointLimits[20], const
                    struct0_T *kC, double panHeight, const double U[18], double
                    Dt, double dt, double *nodeIDCount, const double uBDot[6],
                    int legNum, double xNew_data[], int xNew_size[2])
{
  emxArray_real_T *unusedU6;
  double xRand[13];
  double unusedU7;
  int xNear_size[2];
  double xNear_data[13];
  emxArray_real_T *candTransArrays;
  int i15;
  int loop_ub;
  double candStates_data[65];
  double distance_data[5];
  int i;
  double b_U[2];
  double b_xNear_data[13];
  int tmp_size[2];
  double tmp_data[16];
  double uA[3];
  double uB[3];
  int ixstart;
  double qDot[3];
  double b_qDot[3];
  double b_uB[3];
  double c_qDot[3];
  int itmp;
  boolean_T exitg1;
  double b_uA[3];
  double c_uB[3];
  double d_qDot[3];
  double e_qDot[3];
  emxInit_real_T(&unusedU6, 2);
  randomState(jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5,
              kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);
  nearestNeighbour(xRand, T, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                   kC->l7, kC->l8, kC->zeta, kC->r, *nodeIDCount, xNear_data,
                   xNear_size, unusedU6, &unusedU7);

  /* selectInput Selects the most appropriate control input. */
  /*    A control input is selected from a set of control inputs, U. An input */
  /*    is selected by applying each of the inputs to to state xNear, which */
  /*    results in p candidate states, where p is the size of the input set. */
  /*    The control input corresponding to candidate state that is closest to */
  /*    x1 is returned as u. */
  /* Initialize arrays to store the candidate new state data and the */
  /* distances between each candidate state and the xNear state. */
  emxInit_real_T(&candTransArrays, 2);
  unusedU7 = rt_roundd_snf(Dt / dt);
  i15 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  candTransArrays->size[1] = (int)((unusedU7 + 1.0) * 10.0);
  emxEnsureCapacity((emxArray__common *)candTransArrays, i15, (int)sizeof(double));
  loop_ub = 5 * (int)((unusedU7 + 1.0) * 10.0);
  for (i15 = 0; i15 < loop_ub; i15++) {
    candTransArrays->data[i15] = 0.0;
  }

  /* UJoint = zeros(U_SIZE,3); */
  /* Transform the control inputs to joint space. */
  /* for i = 1:U_SIZE */
  /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
  /* gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6)); */
  /* UJoint(i,:) = [U(i,:) gammaDotDot]; */
  /* end     */
  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  for (i = 0; i < 5; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    for (i15 = 0; i15 < 2; i15++) {
      b_U[i15] = U[i + 9 * i15];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 13U * sizeof(double));
    rk4(b_U, uBDot, dt, Dt, b_xNear_data, jointLimits, kC, legNum, tmp_data,
        tmp_size, unusedU6);
    loop_ub = tmp_size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      candStates_data[i + 5 * i15] = tmp_data[tmp_size[0] * i15];
    }

    loop_ub = unusedU6->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      candTransArrays->data[i + candTransArrays->size[0] * i15] = unusedU6->
        data[unusedU6->size[0] * i15];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    /* heuristic Calculates the distance between states x1 and x2. */
    /* heuristicSingleLeg.m */
    /* author: wreid */
    /* date: 20150107 */
    /* Calculate the distance between angular positions. */
    /*      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC); */
    /*      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC); */
    /*       */
    /*      dxStarMax = xStarMax-xStarMin; */
    /*      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2)); */
    /*       */
    /*      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax); */
    /*       */
    /*      xStarA = legRadius(betaA,gammaA,kC); */
    /*      xStarB = legRadius(betaB,gammaB,kC); */
    /*       */
    /*      dxStar = xStarB-xStarA; */
    /*      dAlpha = angDiff(alphaA,alphaB); */
    /*       */
    /*      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2); */
    /*       */
    /*      dPosNorm = dPos/dPosMax; */
    /* sherpaTTFK Sherpa_TT Forward Kinematics */
    /*    Calculates the x,y,z position of the contact point given the alpha, */
    /*    beta and gamma joint values. */
    /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
    /* relative to the pan coordinate frame for the SherpaTT Leg. */
    /*  */
    /* Inputs: */
    /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
    /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
    /* Outputs: */
    /*  */
    /* sherpaTTFK.m */
    /* author: wreid */
    /* date: 20150122 */
    uA[0] = ((((kC->l2 + kC->l3 * cos(-candStates_data[i + 20])) + kC->l4 * cos
               (kC->zeta)) + kC->l5 * cos(candStates_data[i + 25] + kC->zeta)) -
             kC->l7) * cos(candStates_data[i + 15]);
    uA[1] = ((((kC->l2 + kC->l3 * cos(-candStates_data[i + 20])) + kC->l4 * cos
               (kC->zeta)) + kC->l5 * cos(candStates_data[i + 25] + kC->zeta)) -
             kC->l7) * sin(candStates_data[i + 15]);
    uA[2] = ((((kC->l1 + kC->l3 * sin(-candStates_data[i + 20])) - kC->l4 * sin
               (kC->zeta)) - kC->l5 * sin(candStates_data[i + 25] + kC->zeta)) -
             kC->l6) - (kC->l8 + kC->r);

    /* sherpaTTFK Sherpa_TT Forward Kinematics */
    /*    Calculates the x,y,z position of the contact point given the alpha, */
    /*    beta and gamma joint values. */
    /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
    /* relative to the pan coordinate frame for the SherpaTT Leg. */
    /*  */
    /* Inputs: */
    /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
    /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
    /* Outputs: */
    /*  */
    /* sherpaTTFK.m */
    /* author: wreid */
    /* date: 20150122 */
    uB[0] = ((((kC->l2 + kC->l3 * cos(-xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(xRand[5] + kC->zeta)) - kC->l7) * cos(xRand[3]);
    uB[1] = ((((kC->l2 + kC->l3 * cos(-xRand[4])) + kC->l4 * cos(kC->zeta)) +
              kC->l5 * cos(xRand[5] + kC->zeta)) - kC->l7) * sin(xRand[3]);
    uB[2] = ((((kC->l1 + kC->l3 * sin(-xRand[4])) - kC->l4 * sin(kC->zeta)) -
              kC->l5 * sin(xRand[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
    /* sherpaTTFKVel.m */
    /* author: wreid */
    /* date: 20150122 */
    /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
    /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
    /*     uA = sherpaTTFK(xA(4:6),kC); */
    /*     uB = sherpaTTFK(xB(4:6),kC); */
    /* dPos = norm(uA-uB); */
    /* Calculate the total distance. */
    /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    /* angDiff Finds the angular difference between th1 and th2. */
    unusedU7 = ((xNear_data[3] - candStates_data[i + 15]) + 3.1415926535897931) /
      6.2831853071795862;
    if (fabs(unusedU7 - rt_roundd_snf(unusedU7)) <= 2.2204460492503131E-16 *
        fabs(unusedU7)) {
      unusedU7 = 0.0;
    } else {
      unusedU7 = (unusedU7 - floor(unusedU7)) * 6.2831853071795862;
    }

    if (fabs(unusedU7 - 3.1415926535897931) > 0.39269908169872414) {
      ixstart = 1;
    } else {
      /* aDiff = abs(aDiff/ankleDiffMax); */
      ixstart = 0;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    qDot[0] = (-xRand[8] * sin(xRand[3]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
      (xRand[5] + kC->zeta)) + kC->l3 * cos(xRand[4])) + kC->l4 * cos(kC->zeta))
               - xRand[9] * kC->l3 * cos(xRand[3]) * sin(xRand[4])) - xRand[10] *
      kC->l5 * sin(xRand[5] + kC->zeta) * cos(xRand[3]);
    qDot[1] = (xRand[8] * cos(xRand[3]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
      (xRand[5] + kC->zeta)) + kC->l3 * cos(xRand[4])) + kC->l4 * cos(kC->zeta))
               - xRand[10] * kC->l5 * sin(xRand[5] + kC->zeta) * sin(xRand[3]))
      - xRand[9] * kC->l3 * sin(xRand[3]) * sin(xRand[4]);
    qDot[2] = -xRand[9] * kC->l3 * cos(xRand[4]) - kC->l5 * xRand[10] * cos
      (kC->zeta + xRand[5]);
    b_qDot[0] = (-candStates_data[i + 40] * sin(candStates_data[i + 15]) *
                 ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[i + 25] +
      kC->zeta)) + kC->l3 * cos(candStates_data[i + 20])) + kC->l4 * cos
                  (kC->zeta)) - candStates_data[i + 45] * kC->l3 * cos
                 (candStates_data[i + 15]) * sin(candStates_data[i + 20])) -
      candStates_data[i + 50] * kC->l5 * sin(candStates_data[i + 25] + kC->zeta)
      * cos(candStates_data[i + 15]);
    b_qDot[1] = (candStates_data[i + 40] * cos(candStates_data[i + 15]) *
                 ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[i + 25] +
      kC->zeta)) + kC->l3 * cos(candStates_data[i + 20])) + kC->l4 * cos
                  (kC->zeta)) - candStates_data[i + 50] * kC->l5 * sin
                 (candStates_data[i + 25] + kC->zeta) * sin(candStates_data[i +
      15])) - candStates_data[i + 45] * kC->l3 * sin(candStates_data[i + 15]) *
      sin(candStates_data[i + 20]);
    b_qDot[2] = -candStates_data[i + 45] * kC->l3 * cos(candStates_data[i + 20])
      - kC->l5 * candStates_data[i + 50] * cos(kC->zeta + candStates_data[i + 25]);
    for (i15 = 0; i15 < 3; i15++) {
      b_uB[i15] = uB[i15] - uA[i15];
      c_qDot[i15] = qDot[i15] - b_qDot[i15];
    }

    distance_data[i] = 0.5 * (norm(b_uB) + 0.0 * b_norm(c_qDot)) + 0.5 * (double)
      ixstart;

    /* distance(i) = hDiff; */
  }

  emxFree_real_T(&unusedU6);
  ixstart = 1;
  unusedU7 = distance_data[0];
  itmp = 0;
  if (rtIsNaN(distance_data[0])) {
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 <= 5)) {
      ixstart = i + 1;
      if (!rtIsNaN(distance_data[i])) {
        unusedU7 = distance_data[i];
        itmp = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < unusedU7) {
        unusedU7 = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i15 = 0; i15 < 13; i15++) {
    xNew_data[i15] = candStates_data[itmp + 5 * i15];
  }

  (*nodeIDCount)++;
  xNew_data[0] = *nodeIDCount;

  /* Node ID */
  xNew_data[1] = xNear_data[0];

  /* Parent ID */
  /* heuristic Calculates the distance between states x1 and x2. */
  /* heuristicSingleLeg.m */
  /* author: wreid */
  /* date: 20150107 */
  /* Calculate the distance between angular positions. */
  /*      xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC); */
  /*      xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC); */
  /*       */
  /*      dxStarMax = xStarMax-xStarMin; */
  /*      dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2)); */
  /*       */
  /*      dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax); */
  /*       */
  /*      xStarA = legRadius(betaA,gammaA,kC); */
  /*      xStarB = legRadius(betaB,gammaB,kC); */
  /*       */
  /*      dxStar = xStarB-xStarA; */
  /*      dAlpha = angDiff(alphaA,alphaB); */
  /*       */
  /*      dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2); */
  /*       */
  /*      dPosNorm = dPos/dPosMax; */
  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  b_uA[0] = ((((kC->l2 + kC->l3 * cos(-candStates_data[itmp + 20])) + kC->l4 *
               cos(kC->zeta)) + kC->l5 * cos(candStates_data[itmp + 25] +
    kC->zeta)) - kC->l7) * cos(candStates_data[itmp + 15]);
  b_uA[1] = ((((kC->l2 + kC->l3 * cos(-candStates_data[itmp + 20])) + kC->l4 *
               cos(kC->zeta)) + kC->l5 * cos(candStates_data[itmp + 25] +
    kC->zeta)) - kC->l7) * sin(candStates_data[itmp + 15]);
  b_uA[2] = ((((kC->l1 + kC->l3 * sin(-candStates_data[itmp + 20])) - kC->l4 *
               sin(kC->zeta)) - kC->l5 * sin(candStates_data[itmp + 25] +
    kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

  /* sherpaTTFK Sherpa_TT Forward Kinematics */
  /*    Calculates the x,y,z position of the contact point given the alpha, */
  /*    beta and gamma joint values. */
  /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
  /* relative to the pan coordinate frame for the SherpaTT Leg. */
  /*  */
  /* Inputs: */
  /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
  /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
  /* Outputs: */
  /*  */
  /* sherpaTTFK.m */
  /* author: wreid */
  /* date: 20150122 */
  c_uB[0] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * cos
    (xNear_data[3]);
  c_uB[1] = ((((kC->l2 + kC->l3 * cos(-xNear_data[4])) + kC->l4 * cos(kC->zeta))
              + kC->l5 * cos(xNear_data[5] + kC->zeta)) - kC->l7) * sin
    (xNear_data[3]);
  c_uB[2] = ((((kC->l1 + kC->l3 * sin(-xNear_data[4])) - kC->l4 * sin(kC->zeta))
              - kC->l5 * sin(xNear_data[5] + kC->zeta)) - kC->l6) - (kC->l8 +
    kC->r);

  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA); */
  /* dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8); */
  /*     uA = sherpaTTFK(xA(4:6),kC); */
  /*     uB = sherpaTTFK(xB(4:6),kC); */
  /* dPos = norm(uA-uB); */
  /* Calculate the total distance. */
  /* d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm;  */
  d_qDot[0] = (-xNear_data[8] * sin(xNear_data[3]) * ((((kC->l2 - kC->l7) +
    kC->l5 * cos(xNear_data[5] + kC->zeta)) + kC->l3 * cos(xNear_data[4])) +
    kC->l4 * cos(kC->zeta)) - xNear_data[9] * kC->l3 * cos(xNear_data[3]) * sin
               (xNear_data[4])) - xNear_data[10] * kC->l5 * sin(xNear_data[5] +
    kC->zeta) * cos(xNear_data[3]);
  d_qDot[1] = (xNear_data[8] * cos(xNear_data[3]) * ((((kC->l2 - kC->l7) +
    kC->l5 * cos(xNear_data[5] + kC->zeta)) + kC->l3 * cos(xNear_data[4])) +
    kC->l4 * cos(kC->zeta)) - xNear_data[10] * kC->l5 * sin(xNear_data[5] +
    kC->zeta) * sin(xNear_data[3])) - xNear_data[9] * kC->l3 * sin(xNear_data[3])
    * sin(xNear_data[4]);
  d_qDot[2] = -xNear_data[9] * kC->l3 * cos(xNear_data[4]) - kC->l5 *
    xNear_data[10] * cos(kC->zeta + xNear_data[5]);
  e_qDot[0] = (-candStates_data[itmp + 40] * sin(candStates_data[itmp + 15]) *
               ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[itmp + 25] +
    kC->zeta)) + kC->l3 * cos(candStates_data[itmp + 20])) + kC->l4 * cos
                (kC->zeta)) - candStates_data[itmp + 45] * kC->l3 * cos
               (candStates_data[itmp + 15]) * sin(candStates_data[itmp + 20])) -
    candStates_data[itmp + 50] * kC->l5 * sin(candStates_data[itmp + 25] +
    kC->zeta) * cos(candStates_data[itmp + 15]);
  e_qDot[1] = (candStates_data[itmp + 40] * cos(candStates_data[itmp + 15]) *
               ((((kC->l2 - kC->l7) + kC->l5 * cos(candStates_data[itmp + 25] +
    kC->zeta)) + kC->l3 * cos(candStates_data[itmp + 20])) + kC->l4 * cos
                (kC->zeta)) - candStates_data[itmp + 50] * kC->l5 * sin
               (candStates_data[itmp + 25] + kC->zeta) * sin
               (candStates_data[itmp + 15])) - candStates_data[itmp + 45] *
    kC->l3 * sin(candStates_data[itmp + 15]) * sin(candStates_data[itmp + 20]);
  e_qDot[2] = -candStates_data[itmp + 45] * kC->l3 * cos(candStates_data[itmp +
    20]) - kC->l5 * candStates_data[itmp + 50] * cos(kC->zeta +
    candStates_data[itmp + 25]);
  for (i15 = 0; i15 < 3; i15++) {
    uB[i15] = c_uB[i15] - b_uA[i15];
    qDot[i15] = d_qDot[i15] - e_qDot[i15];
  }

  xNew_data[2] = xNear_data[2] + (norm(uB) + 0.0 * b_norm(qDot));

  /* Cost */
  ixstart = (int)*nodeIDCount - 1;
  loop_ub = candTransArrays->size[1] - 1;
  for (i15 = 0; i15 < 13; i15++) {
    T->data[ixstart + T->size[0] * i15] = xNew_data[i15];
  }

  for (i15 = 0; i15 <= loop_ub; i15++) {
    T->data[ixstart + T->size[0] * (i15 + 13)] = candTransArrays->data[itmp +
      candTransArrays->size[0] * i15];
  }

  emxFree_real_T(&candTransArrays);

  /* Append the new node to the tree.     */
  /* if mod(nodeIDCount,100) == 0 */
  /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
  /* end */
}

/*
 * Assignn the
 * Arguments    : const emxArray_real_T *T
 *                const double midPoint_data[]
 *                emxArray_real_T *path
 * Return Type  : void
 */
static void traceBranch(const emxArray_real_T *T, const double midPoint_data[],
  emxArray_real_T *path)
{
  emxArray_real_T *next;
  double check;
  int i11;
  int i12;
  emxArray_real_T *transitionArray;
  int i13;
  int loop_ub;
  emxArray_real_T *transitionPath;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int i;
  unsigned int b_i;
  emxInit_real_T(&next, 2);
  check = midPoint_data[0];
  i11 = next->size[0] * next->size[1];
  next->size[0] = 1;
  next->size[1] = 13;
  emxEnsureCapacity((emxArray__common *)next, i11, (int)sizeof(double));
  for (i11 = 0; i11 < 13; i11++) {
    next->data[i11] = midPoint_data[i11];
  }

  i11 = path->size[0] * path->size[1];
  path->size[0] = 0;
  path->size[1] = 10;
  emxEnsureCapacity((emxArray__common *)path, i11, (int)sizeof(double));
  if (14 > T->size[1]) {
    i11 = 0;
    i12 = 0;
  } else {
    i11 = 13;
    i12 = T->size[1];
  }

  emxInit_real_T(&transitionArray, 2);
  i13 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = i12 - i11;
  emxEnsureCapacity((emxArray__common *)transitionArray, i13, (int)sizeof(double));
  loop_ub = i12 - i11;
  for (i12 = 0; i12 < loop_ub; i12++) {
    transitionArray->data[transitionArray->size[0] * i12] = T->data[((int)
      midPoint_data[0] + T->size[0] * (i11 + i12)) - 1];
  }

  /* Iterate over the tree until the initial state has been found. */
  emxInit_real_T(&transitionPath, 2);
  emxInit_real_T(&b_transitionPath, 2);
  emxInit_real_T(&c_transitionPath, 2);
  while ((check != 0.0) && (next->data[1] != 0.0)) {
    i11 = transitionPath->size[0] * transitionPath->size[1];
    transitionPath->size[0] = 0;
    transitionPath->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)transitionPath, i11, (int)sizeof
                      (double));
    i11 = (int)(((double)transitionArray->size[1] + 9.0) / 10.0);
    for (i = 0; i < i11; i++) {
      b_i = i * 10U + 1U;
      i12 = c_transitionPath->size[0] * c_transitionPath->size[1];
      c_transitionPath->size[0] = transitionPath->size[0] + 1;
      c_transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)c_transitionPath, i12, (int)sizeof
                        (double));
      for (i12 = 0; i12 < 10; i12++) {
        loop_ub = transitionPath->size[0];
        for (i13 = 0; i13 < loop_ub; i13++) {
          c_transitionPath->data[i13 + c_transitionPath->size[0] * i12] =
            transitionPath->data[i13 + transitionPath->size[0] * i12];
        }
      }

      for (i12 = 0; i12 < 10; i12++) {
        c_transitionPath->data[transitionPath->size[0] + c_transitionPath->size
          [0] * i12] = transitionArray->data[(int)(i12 + b_i) - 1];
      }

      i12 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = c_transitionPath->size[0];
      transitionPath->size[1] = 10;
      emxEnsureCapacity((emxArray__common *)transitionPath, i12, (int)sizeof
                        (double));
      for (i12 = 0; i12 < 10; i12++) {
        loop_ub = c_transitionPath->size[0];
        for (i13 = 0; i13 < loop_ub; i13++) {
          transitionPath->data[i13 + transitionPath->size[0] * i12] =
            c_transitionPath->data[i13 + c_transitionPath->size[0] * i12];
        }
      }
    }

    i11 = b_transitionPath->size[0] * b_transitionPath->size[1];
    b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
    b_transitionPath->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)b_transitionPath, i11, (int)sizeof
                      (double));
    for (i11 = 0; i11 < 10; i11++) {
      loop_ub = transitionPath->size[0];
      for (i12 = 0; i12 < loop_ub; i12++) {
        b_transitionPath->data[i12 + b_transitionPath->size[0] * i11] =
          transitionPath->data[i12 + transitionPath->size[0] * i11];
      }
    }

    for (i11 = 0; i11 < 10; i11++) {
      loop_ub = path->size[0];
      for (i12 = 0; i12 < loop_ub; i12++) {
        b_transitionPath->data[(i12 + transitionPath->size[0]) +
          b_transitionPath->size[0] * i11] = path->data[i12 + path->size[0] *
          i11];
      }
    }

    i11 = path->size[0] * path->size[1];
    path->size[0] = b_transitionPath->size[0];
    path->size[1] = 10;
    emxEnsureCapacity((emxArray__common *)path, i11, (int)sizeof(double));
    for (i11 = 0; i11 < 10; i11++) {
      loop_ub = b_transitionPath->size[0];
      for (i12 = 0; i12 < loop_ub; i12++) {
        path->data[i12 + path->size[0] * i11] = b_transitionPath->data[i12 +
          b_transitionPath->size[0] * i11];
      }
    }

    check = next->data[1];
    loop_ub = T->size[1];
    i11 = next->size[0] * next->size[1];
    next->size[0] = 1;
    next->size[1] = loop_ub;
    emxEnsureCapacity((emxArray__common *)next, i11, (int)sizeof(double));
    for (i11 = 0; i11 < loop_ub; i11++) {
      next->data[next->size[0] * i11] = T->data[((int)check + T->size[0] * i11)
        - 1];
    }

    check = next->data[1];
    if (14 > next->size[1]) {
      i11 = 0;
      i12 = 0;
    } else {
      i11 = 13;
      i12 = next->size[1];
    }

    i13 = transitionArray->size[0] * transitionArray->size[1];
    transitionArray->size[0] = 1;
    transitionArray->size[1] = i12 - i11;
    emxEnsureCapacity((emxArray__common *)transitionArray, i13, (int)sizeof
                      (double));
    loop_ub = i12 - i11;
    for (i12 = 0; i12 < loop_ub; i12++) {
      transitionArray->data[transitionArray->size[0] * i12] = next->data[i11 +
        i12];
    }
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&transitionPath);
  emxFree_real_T(&transitionArray);
  emxFree_real_T(&next);
}

/*
 * buildRRT Icrementally builds a rapidly exploring random tree.
 *    An RRT is build by incrementally selecting a random state from the
 *    available state space as defined by the MIN and MAX vectors. The tree is
 *    started at xInit and is extended until the number of maximum nodes, K has
 *    been reached. A path is selected if the goal region as defined by xGoal
 *    has been reached by the RRT.
 * Arguments    : const double nInit[13]
 *                const double nGoal[13]
 *                const double jointLimits[20]
 *                double panHeight
 *                const double U[18]
 *                double dt
 *                double Dt
 *                const struct0_T *kC
 *                const double uBDot[6]
 *                int legNum
 *                const double TP2B[16]
 *                emxArray_real_T *T1
 *                emxArray_real_T *T2
 *                emxArray_real_T *pathJ
 *                emxArray_real_T *pathC
 * Return Type  : void
 */
void buildBiDirectionalRRT(const double nInit[13], const double nGoal[13], const
  double jointLimits[20], double panHeight, const double U[18], double dt,
  double Dt, const struct0_T *kC, const double uBDot[6], int legNum, const
  double TP2B[16], emxArray_real_T *T1, emxArray_real_T *T2, emxArray_real_T
  *pathJ, emxArray_real_T *pathC)
{
  double transitionArrayLength;
  int i3;
  double nodeIDCount1;
  int cdiff;
  int absb;
  emxArray_real_T *b_T1;
  double nodeIDCount2;
  double pathLengthMin;
  int i;
  emxArray_real_T_1x13 c_T1;
  emxArray_real_T *pathT1;
  emxArray_real_T *pathT2;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *y;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_T2;
  emxArray_real_T *d;
  emxArray_real_T *b_d;
  emxArray_real_T *c_d;
  emxArray_real_T *b_t;
  emxArray_real_T *d_d;
  emxArray_real_T *e_d;
  double nMid1_data[13];
  double d_data[1000];
  int ixstart;
  boolean_T exitg1;
  double T2_data[13];
  int apnd;
  double uP[3];
  double uB[3];
  double b_uB[3];
  double b_path[3];

  /* buildBiDirectionalRRT.m */
  /* author: wreid */
  /* date: 20150107 */
  /* Constant Declaration                                                       */
  transitionArrayLength = (rt_roundd_snf(Dt / dt) + 1.0) * 10.0;
  i3 = T1->size[0] * T1->size[1];
  T1->size[0] = 1000;
  nodeIDCount1 = rt_roundd_snf(13.0 + transitionArrayLength);
  if (nodeIDCount1 < 2.147483648E+9) {
    if (nodeIDCount1 >= -2.147483648E+9) {
      cdiff = (int)nodeIDCount1;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (nodeIDCount1 >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  T1->size[1] = cdiff;
  emxEnsureCapacity((emxArray__common *)T1, i3, (int)sizeof(double));
  nodeIDCount1 = rt_roundd_snf(13.0 + transitionArrayLength);
  if (nodeIDCount1 < 2.147483648E+9) {
    if (nodeIDCount1 >= -2.147483648E+9) {
      i3 = (int)nodeIDCount1;
    } else {
      i3 = MIN_int32_T;
    }
  } else if (nodeIDCount1 >= 2.147483648E+9) {
    i3 = MAX_int32_T;
  } else {
    i3 = 0;
  }

  absb = 1000 * i3;
  for (i3 = 0; i3 < absb; i3++) {
    T1->data[i3] = 0.0;
  }

  i3 = T2->size[0] * T2->size[1];
  T2->size[0] = 1000;
  nodeIDCount1 = rt_roundd_snf(13.0 + transitionArrayLength);
  if (nodeIDCount1 < 2.147483648E+9) {
    if (nodeIDCount1 >= -2.147483648E+9) {
      cdiff = (int)nodeIDCount1;
    } else {
      cdiff = MIN_int32_T;
    }
  } else if (nodeIDCount1 >= 2.147483648E+9) {
    cdiff = MAX_int32_T;
  } else {
    cdiff = 0;
  }

  T2->size[1] = cdiff;
  emxEnsureCapacity((emxArray__common *)T2, i3, (int)sizeof(double));
  nodeIDCount1 = rt_roundd_snf(13.0 + transitionArrayLength);
  if (nodeIDCount1 < 2.147483648E+9) {
    if (nodeIDCount1 >= -2.147483648E+9) {
      i3 = (int)nodeIDCount1;
    } else {
      i3 = MIN_int32_T;
    }
  } else if (nodeIDCount1 >= 2.147483648E+9) {
    i3 = MAX_int32_T;
  } else {
    i3 = 0;
  }

  absb = 1000 * i3;
  for (i3 = 0; i3 < absb; i3++) {
    T2->data[i3] = 0.0;
  }

  for (i3 = 0; i3 < 13; i3++) {
    T1->data[T1->size[0] * i3] = nInit[i3];
  }

  absb = (int)transitionArrayLength;
  for (i3 = 0; i3 < absb; i3++) {
    T1->data[T1->size[0] * (i3 + 13)] = 0.0;
  }

  for (i3 = 0; i3 < 13; i3++) {
    T2->data[T2->size[0] * i3] = nGoal[i3];
  }

  absb = (int)transitionArrayLength;
  for (i3 = 0; i3 < absb; i3++) {
    T2->data[T2->size[0] * (i3 + 13)] = 0.0;
  }

  emxInit_real_T(&b_T1, 2);
  nodeIDCount1 = 1.0;
  nodeIDCount2 = 1.0;
  pathLengthMin = 100.0;
  i3 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = 0;
  pathC->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)pathC, i3, (int)sizeof(double));
  i3 = pathJ->size[0] * pathJ->size[1];
  pathJ->size[0] = 0;
  pathJ->size[1] = 0;
  emxEnsureCapacity((emxArray__common *)pathJ, i3, (int)sizeof(double));
  for (i = 0; i < 1998; i++) {
    i3 = b_T1->size[0] * b_T1->size[1];
    b_T1->size[0] = 1000;
    b_T1->size[1] = T1->size[1];
    emxEnsureCapacity((emxArray__common *)b_T1, i3, (int)sizeof(double));
    absb = T1->size[0] * T1->size[1];
    for (i3 = 0; i3 < absb; i3++) {
      b_T1->data[i3] = T1->data[i3];
    }

    transitionArrayLength = nodeIDCount1;
    rrtLoop(b_T1, jointLimits, kC, panHeight, U, Dt, dt, &transitionArrayLength,
            uBDot, legNum, c_T1.data, c_T1.size);
    nodeIDCount1++;

    /* Swap the trees. */
    i3 = T1->size[0] * T1->size[1];
    T1->size[0] = 1000;
    T1->size[1] = T2->size[1];
    emxEnsureCapacity((emxArray__common *)T1, i3, (int)sizeof(double));
    absb = T2->size[0] * T2->size[1];
    for (i3 = 0; i3 < absb; i3++) {
      T1->data[i3] = T2->data[i3];
    }

    i3 = T2->size[0] * T2->size[1];
    T2->size[0] = 1000;
    T2->size[1] = b_T1->size[1];
    emxEnsureCapacity((emxArray__common *)T2, i3, (int)sizeof(double));
    absb = b_T1->size[0] * b_T1->size[1];
    for (i3 = 0; i3 < absb; i3++) {
      T2->data[i3] = b_T1->data[i3];
    }

    /* Swap the trees. */
    transitionArrayLength = nodeIDCount1;
    nodeIDCount1 = nodeIDCount2;
    nodeIDCount2 = transitionArrayLength;
  }

  emxFree_real_T(&b_T1);
  emxInit_real_T(&pathT1, 2);
  emxInit_real_T(&pathT2, 2);
  b_emxInit_real_T(&t, 1);
  emxInit_real_T(&path, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b_pathC, 2);
  emxInit_real_T(&b_T2, 2);
  emxInit_real_T(&d, 2);
  emxInit_real_T(&b_d, 2);
  emxInit_real_T(&c_d, 2);
  emxInit_real_T(&b_t, 2);
  emxInit_real_T(&d_d, 2);
  emxInit_real_T(&e_d, 2);
  for (i = 0; i < 1000; i++) {
    for (i3 = 0; i3 < 13; i3++) {
      nMid1_data[i3] = T1->data[i + T1->size[0] * i3];
    }

    /* nearestNeigbour Finds the node in the tree closest to x. */
    /*    This function scans each node within the tree and finds the node that */
    /*    is closest to the xRand node. The nearest node is returned by the */
    /*    function. A distance heuristic is  used */
    /*    Inputs: */
    /*        x:  The 1xn state that each node in the tree will be compared to, */
    /*            to find the node with the minimum distance to it. n refers to */
    /*            the number of dimensions within the state space. */
    /*        T:  The nxm tree being searched, m is the number of possible nodes */
    /*            within the tree. */
    /*        HGAINS: The gains applied to the heuristic function. */
    /*    Outputs: */
    /*        xNear:  The node in the tree that is closet to x. */
    /* nearestNeigbour.m */
    /* author: wreid */
    /* date: 20150107 */
    /* Iterate over the entire tree and apply the distance heuristic function */
    /* to each node. */
    /* parfor i = 1:nodeIDCount */
    for (ixstart = 0; ixstart < 1000; ixstart++) {
      absb = T2->size[1];
      i3 = b_T2->size[0] * b_T2->size[1];
      b_T2->size[0] = 1;
      b_T2->size[1] = absb;
      emxEnsureCapacity((emxArray__common *)b_T2, i3, (int)sizeof(double));
      for (i3 = 0; i3 < absb; i3++) {
        b_T2->data[b_T2->size[0] * i3] = T2->data[ixstart + T2->size[0] * i3];
      }

      d_data[ixstart] = heuristicSingleLeg(nMid1_data, b_T2, kC->l1, kC->l2,
        kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);
    }

    ixstart = 1;
    i3 = d->size[0] * d->size[1];
    d->size[0] = 1;
    d->size[1] = 1000;
    emxEnsureCapacity((emxArray__common *)d, i3, (int)sizeof(double));
    for (i3 = 0; i3 < 1000; i3++) {
      d->data[d->size[0] * i3] = d_data[i3];
    }

    transitionArrayLength = d->data[0];
    cdiff = 0;
    if (rtIsNaN(transitionArrayLength)) {
      absb = 2;
      exitg1 = false;
      while ((!exitg1) && (absb <= 1000)) {
        ixstart = absb;
        i3 = b_d->size[0] * b_d->size[1];
        b_d->size[0] = 1;
        b_d->size[1] = 1000;
        emxEnsureCapacity((emxArray__common *)b_d, i3, (int)sizeof(double));
        for (i3 = 0; i3 < 1000; i3++) {
          b_d->data[b_d->size[0] * i3] = d_data[i3];
        }

        if (!rtIsNaN(b_d->data[absb - 1])) {
          i3 = c_d->size[0] * c_d->size[1];
          c_d->size[0] = 1;
          c_d->size[1] = 1000;
          emxEnsureCapacity((emxArray__common *)c_d, i3, (int)sizeof(double));
          for (i3 = 0; i3 < 1000; i3++) {
            c_d->data[c_d->size[0] * i3] = d_data[i3];
          }

          transitionArrayLength = c_d->data[absb - 1];
          cdiff = absb - 1;
          exitg1 = true;
        } else {
          absb++;
        }
      }
    }

    if (ixstart < 1000) {
      while (ixstart + 1 <= 1000) {
        i3 = d_d->size[0] * d_d->size[1];
        d_d->size[0] = 1;
        d_d->size[1] = 1000;
        emxEnsureCapacity((emxArray__common *)d_d, i3, (int)sizeof(double));
        for (i3 = 0; i3 < 1000; i3++) {
          d_d->data[d_d->size[0] * i3] = d_data[i3];
        }

        if (d_d->data[ixstart] < transitionArrayLength) {
          i3 = e_d->size[0] * e_d->size[1];
          e_d->size[0] = 1;
          e_d->size[1] = 1000;
          emxEnsureCapacity((emxArray__common *)e_d, i3, (int)sizeof(double));
          for (i3 = 0; i3 < 1000; i3++) {
            e_d->data[e_d->size[0] * i3] = d_data[i3];
          }

          transitionArrayLength = e_d->data[ixstart];
          cdiff = ixstart;
        }

        ixstart++;
      }
    }

    /* [d,minIndex] = min(d(1:nodeIDCount)); */
    if (transitionArrayLength < 0.04) {
      traceBranch(T1, nMid1_data, pathT1);
      for (i3 = 0; i3 < 13; i3++) {
        T2_data[i3] = T2->data[cdiff + T2->size[0] * i3];
      }

      traceBranch(T2, T2_data, pathT2);
      if ((T1->data[T1->size[0] * 3] == nInit[3]) && (T1->data[T1->size[0] << 2]
           == nInit[4]) && (T1->data[T1->size[0] * 5] == nInit[5])) {
        flipud(pathT2);
        i3 = path->size[0] * path->size[1];
        path->size[0] = pathT1->size[0] + pathT2->size[0];
        path->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)path, i3, (int)sizeof(double));
        for (i3 = 0; i3 < 10; i3++) {
          absb = pathT1->size[0];
          for (cdiff = 0; cdiff < absb; cdiff++) {
            path->data[cdiff + path->size[0] * i3] = pathT1->data[cdiff +
              pathT1->size[0] * i3];
          }
        }

        for (i3 = 0; i3 < 10; i3++) {
          absb = pathT2->size[0];
          for (cdiff = 0; cdiff < absb; cdiff++) {
            path->data[(cdiff + pathT1->size[0]) + path->size[0] * i3] =
              pathT2->data[cdiff + pathT2->size[0] * i3];
          }
        }
      } else {
        flipud(pathT1);
        i3 = path->size[0] * path->size[1];
        path->size[0] = pathT2->size[0] + pathT1->size[0];
        path->size[1] = 10;
        emxEnsureCapacity((emxArray__common *)path, i3, (int)sizeof(double));
        for (i3 = 0; i3 < 10; i3++) {
          absb = pathT2->size[0];
          for (cdiff = 0; cdiff < absb; cdiff++) {
            path->data[cdiff + path->size[0] * i3] = pathT2->data[cdiff +
              pathT2->size[0] * i3];
          }
        }

        for (i3 = 0; i3 < 10; i3++) {
          absb = pathT1->size[0];
          for (cdiff = 0; cdiff < absb; cdiff++) {
            path->data[(cdiff + pathT2->size[0]) + path->size[0] * i3] =
              pathT1->data[cdiff + pathT1->size[0] * i3];
          }
        }
      }

      if (path->size[0] < 1) {
        absb = -1;
        apnd = 0;
      } else {
        ixstart = (int)floor(((double)path->size[0] - 1.0) + 0.5);
        apnd = ixstart + 1;
        cdiff = (ixstart - path->size[0]) + 1;
        absb = path->size[0];
        if (fabs(cdiff) < 4.4408920985006262E-16 * (double)absb) {
          ixstart++;
          apnd = path->size[0];
        } else if (cdiff > 0) {
          apnd = ixstart;
        } else {
          ixstart++;
        }

        absb = ixstart - 1;
      }

      i3 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = absb + 1;
      emxEnsureCapacity((emxArray__common *)y, i3, (int)sizeof(double));
      if (absb + 1 > 0) {
        y->data[0] = 1.0;
        if (absb + 1 > 1) {
          y->data[absb] = apnd;
          ixstart = absb / 2;
          for (cdiff = 1; cdiff < ixstart; cdiff++) {
            y->data[cdiff] = 1.0 + (double)cdiff;
            y->data[absb - cdiff] = apnd - cdiff;
          }

          if (ixstart << 1 == absb) {
            y->data[ixstart] = (1.0 + (double)apnd) / 2.0;
          } else {
            y->data[ixstart] = 1.0 + (double)ixstart;
            y->data[ixstart + 1] = apnd - ixstart;
          }
        }
      }

      i3 = t->size[0];
      t->size[0] = y->size[1];
      emxEnsureCapacity((emxArray__common *)t, i3, (int)sizeof(double));
      absb = y->size[1];
      for (i3 = 0; i3 < absb; i3++) {
        t->data[i3] = dt * y->data[y->size[0] * i3];
      }

      ixstart = t->size[0];
      i3 = b_t->size[0] * b_t->size[1];
      b_t->size[0] = ixstart;
      b_t->size[1] = 1 + path->size[1];
      emxEnsureCapacity((emxArray__common *)b_t, i3, (int)sizeof(double));
      for (i3 = 0; i3 < ixstart; i3++) {
        b_t->data[i3] = t->data[i3];
      }

      absb = path->size[1];
      for (i3 = 0; i3 < absb; i3++) {
        ixstart = path->size[0];
        for (cdiff = 0; cdiff < ixstart; cdiff++) {
          b_t->data[cdiff + b_t->size[0] * (i3 + 1)] = path->data[cdiff +
            path->size[0] * i3];
        }
      }

      i3 = path->size[0] * path->size[1];
      path->size[0] = b_t->size[0];
      path->size[1] = b_t->size[1];
      emxEnsureCapacity((emxArray__common *)path, i3, (int)sizeof(double));
      absb = b_t->size[1];
      for (i3 = 0; i3 < absb; i3++) {
        ixstart = b_t->size[0];
        for (cdiff = 0; cdiff < ixstart; cdiff++) {
          path->data[cdiff + path->size[0] * i3] = b_t->data[cdiff + b_t->size[0]
            * i3];
        }
      }

      ixstart = path->size[0];
      i3 = b_pathC->size[0] * b_pathC->size[1];
      b_pathC->size[0] = ixstart;
      b_pathC->size[1] = 9;
      emxEnsureCapacity((emxArray__common *)b_pathC, i3, (int)sizeof(double));
      absb = path->size[0] * 9;
      for (i3 = 0; i3 < absb; i3++) {
        b_pathC->data[i3] = 0.0;
      }

      transitionArrayLength = 0.0;
      for (ixstart = 0; ixstart < path->size[0]; ixstart++) {
        /* sherpaTTFK Sherpa_TT Forward Kinematics */
        /*    Calculates the x,y,z position of the contact point given the alpha, */
        /*    beta and gamma joint values. */
        /* SHERPATTFK Calcluates the Cartesian position of the wheel contact point */
        /* relative to the pan coordinate frame for the SherpaTT Leg. */
        /*  */
        /* Inputs: */
        /* -q: A 1x3 vector containing the joint angular positions [alpha beta gamma] */
        /* -kC: A struct containing the kinematic constants of the SherpaTT leg. */
        /* Outputs: */
        /*  */
        /* sherpaTTFK.m */
        /* author: wreid */
        /* date: 20150122 */
        uP[0] = ((((kC->l2 + kC->l3 * cos(-path->data[ixstart + (path->size[0] <<
          1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(path->data[ixstart +
                   path->size[0] * 3] + kC->zeta)) - kC->l7) * cos(path->
          data[ixstart + path->size[0]]);
        uP[1] = ((((kC->l2 + kC->l3 * cos(-path->data[ixstart + (path->size[0] <<
          1)])) + kC->l4 * cos(kC->zeta)) + kC->l5 * cos(path->data[ixstart +
                   path->size[0] * 3] + kC->zeta)) - kC->l7) * sin(path->
          data[ixstart + path->size[0]]);
        uP[2] = ((((kC->l1 + kC->l3 * sin(-path->data[ixstart + (path->size[0] <<
          1)])) - kC->l4 * sin(kC->zeta)) - kC->l5 * sin(path->data[ixstart +
                   path->size[0] * 3] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

        /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
        /* sherpaTTFKVel.m */
        /* author: wreid */
        /* date: 20150122 */
        for (i3 = 0; i3 < 3; i3++) {
          nodeIDCount1 = 0.0;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            nodeIDCount1 += TP2B[i3 + (cdiff << 2)] * uP[cdiff];
          }

          uB[i3] = nodeIDCount1 + TP2B[12 + i3];
        }

        if (1 + ixstart != 1) {
          for (i3 = 0; i3 < 3; i3++) {
            b_uB[i3] = uB[i3] - b_pathC->data[(ixstart + b_pathC->size[0] * (2 +
              i3)) - 1];
          }

          transitionArrayLength += norm(b_uB);
        }

        b_path[0] = (-path->data[ixstart + path->size[0] * 6] * sin(path->
          data[ixstart + path->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
          (path->data[ixstart + path->size[0] * 3] + kC->zeta)) + kC->l3 * cos
          (path->data[ixstart + (path->size[0] << 1)])) + kC->l4 * cos(kC->zeta))
                     - path->data[ixstart + path->size[0] * 7] * kC->l3 * cos
                     (path->data[ixstart + path->size[0]]) * sin(path->
          data[ixstart + (path->size[0] << 1)])) - path->data[ixstart +
          (path->size[0] << 3)] * kC->l5 * sin(path->data[ixstart + path->size[0]
          * 3] + kC->zeta) * cos(path->data[ixstart + path->size[0]]);
        b_path[1] = (path->data[ixstart + path->size[0] * 6] * cos(path->
          data[ixstart + path->size[0]]) * ((((kC->l2 - kC->l7) + kC->l5 * cos
          (path->data[ixstart + path->size[0] * 3] + kC->zeta)) + kC->l3 * cos
          (path->data[ixstart + (path->size[0] << 1)])) + kC->l4 * cos(kC->zeta))
                     - path->data[ixstart + (path->size[0] << 3)] * kC->l5 * sin
                     (path->data[ixstart + path->size[0] * 3] + kC->zeta) * sin
                     (path->data[ixstart + path->size[0]])) - path->data[ixstart
          + path->size[0] * 7] * kC->l3 * sin(path->data[ixstart + path->size[0]])
          * sin(path->data[ixstart + (path->size[0] << 1)]);
        b_path[2] = -path->data[ixstart + path->size[0] * 7] * kC->l3 * cos
          (path->data[ixstart + (path->size[0] << 1)]) - kC->l5 * path->
          data[ixstart + (path->size[0] << 3)] * cos(kC->zeta + path->
          data[ixstart + path->size[0] * 3]);
        for (i3 = 0; i3 < 3; i3++) {
          b_uB[i3] = 0.0;
          for (cdiff = 0; cdiff < 3; cdiff++) {
            b_uB[i3] += TP2B[i3 + (cdiff << 2)] * b_path[cdiff];
          }
        }

        b_pathC->data[ixstart] = path->data[ixstart];
        b_pathC->data[ixstart + b_pathC->size[0]] = transitionArrayLength;
        for (i3 = 0; i3 < 3; i3++) {
          b_pathC->data[ixstart + b_pathC->size[0] * (i3 + 2)] = uB[i3];
        }

        for (i3 = 0; i3 < 3; i3++) {
          b_pathC->data[ixstart + b_pathC->size[0] * (i3 + 5)] = b_uB[i3];
        }

        b_pathC->data[ixstart + (b_pathC->size[0] << 3)] = 0.0;
      }

      if (b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1] <
          pathLengthMin) {
        pathLengthMin = b_pathC->data[(b_pathC->size[0] + b_pathC->size[0]) - 1];
        i3 = pathC->size[0] * pathC->size[1];
        pathC->size[0] = b_pathC->size[0];
        pathC->size[1] = 9;
        emxEnsureCapacity((emxArray__common *)pathC, i3, (int)sizeof(double));
        absb = b_pathC->size[0] * b_pathC->size[1];
        for (i3 = 0; i3 < absb; i3++) {
          pathC->data[i3] = b_pathC->data[i3];
        }

        i3 = pathJ->size[0] * pathJ->size[1];
        pathJ->size[0] = path->size[0];
        pathJ->size[1] = path->size[1];
        emxEnsureCapacity((emxArray__common *)pathJ, i3, (int)sizeof(double));
        absb = path->size[0] * path->size[1];
        for (i3 = 0; i3 < absb; i3++) {
          pathJ->data[i3] = path->data[i3];
        }
      }
    }
  }

  emxFree_real_T(&e_d);
  emxFree_real_T(&d_d);
  emxFree_real_T(&b_t);
  emxFree_real_T(&c_d);
  emxFree_real_T(&b_d);
  emxFree_real_T(&d);
  emxFree_real_T(&b_T2);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&y);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&pathT2);
  emxFree_real_T(&pathT1);
}

/*
 * File trailer for buildBiDirectionalRRT.c
 *
 * [EOF]
 */
