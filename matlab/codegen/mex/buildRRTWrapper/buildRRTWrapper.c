/*
 * buildRRTWrapper.c
 *
 * Code generation for function 'buildRRTWrapper'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxutil.h"
#include "norm.h"
#include "buildRRT.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static real_T cartesianLimits[4];
static real_T HGAINS[3];
static emlrtRSInfo emlrtRSI = { 70, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo b_emlrtRSI = { 79, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo c_emlrtRSI = { 80, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo d_emlrtRSI = { 94, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo e_emlrtRSI = { 96, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo emlrtRTEI = { 27, 36, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtBCInfo emlrtBCI = { 1, 4, 18, 17, "kC.legAngleOffset", "trP2B",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/trP2B.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 111, 31, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 112, 37, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 112, 50, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 116, 48, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 118, 15, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { -1, -1, 118, 29, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

/* Function Definitions */
void HGAINS_not_empty_init(void)
{
}

void NODE_SIZE_not_empty_init(void)
{
}

void NUM_NODES_not_empty_init(void)
{
}

void U_SIZE_not_empty_init(void)
{
}

void ankleThreshold_not_empty_init(void)
{
}

void buildRRTWrapper(const emlrtStack *sp, const real_T nInitCartesianB[6],
                     const real_T nGoalCartesianB[6], real_T phiInit, real_T
                     omegaInit, const real_T jointLimits[20], real_T bodyHeight,
                     const real_T U[10], real_T dt, real_T Dt, const struct0_T
                     *kC, real_T threshold, int32_T legNum, const real_T uBDot[6],
                     emxArray_real_T *T, emxArray_real_T *pathC, emxArray_real_T
                     *pathJ, boolean_T *success)
{
  real_T TP2B[16];
  int32_T i0;
  static const int8_T iv0[4] = { 0, 0, 0, 1 };

  real_T b_TP2B[9];
  int32_T loop_ub;
  real_T c_TP2B[3];
  real_T TB2P[16];
  real_T b_TB2P[3];
  real_T d0;
  real_T qInit[3];
  real_T qGoal[3];
  real_T nInitJoint[10];
  real_T uB[3];
  real_T c_TB2P[3];
  real_T nGoalJoint[10];
  emxArray_real_T *b_T;
  emxArray_real_T *b_pathC;
  real_T dv0[13];
  real_T dv1[13];
  real_T dist2Go;
  int32_T i;
  real_T uP[3];
  real_T b_pathJ[3];
  emlrtStack st;
  (void)threshold;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* BUILDRRTWRAPPER This function acts as a wrapper for the buildRRT function. */
  /* Code generation for the singleLegPlanner is performed using this function */
  /* as an entry point. */
  /*    */
  /* Inputs: */
  /* -nInitCartesianB: the */
  /* -nGoalCartesianB: */
  /* -jointLimits: */
  /* -bodyHeight: */
  /* -U: */
  /* -dt: */
  /* -Dt: */
  /* -kC: */
  /* -threshold: */
  /* -legNum: */
  /*  */
  /* Outputs: */
  /* -T: */
  /* -pathC: */
  /* -pathJ: */
  /* -success: */
  /*  */
  /* buildRRTWrapper.m */
  /* author: wreid */
  /* date: 20150502 */
  /* if isempty(NUM_NODES) */
  /*     NUM_NODES = int32(1000); */
  /* end */
  /* GETPANHEIGHT Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame */
  /* to the pan coordinate frame. */
  st.site = &emlrtRSI;

  /* TRP2B Generates the homogeneous transformation matrix between the body */
  /* and pan coordinate frames. kC is a struct containing the kinematic */
  /* constants of the leg. legNum indicates the number of the leg that is being */
  /* considered. */
  /*  */
  /* Inputs: */
  /* -kC: Struct of kinematic constants of the Sherpa_TT leg */
  /* -legNum: The leg number identification. */
  /* Outputs: */
  /* -TP2B: The homogeneous transformation matrix that is used to transform */
  /* coordinates from the pan frame to the body frame. */
  /*  */
  /* trP2B.m */
  /* author:    wreid */
  /* date:      20150214 */
  emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &emlrtBCI, &st);

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TP2B[0] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  TP2B[4] = -muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  TP2B[8] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[12] = kC->B2PXOffset * muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  TP2B[1] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  TP2B[5] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  TP2B[9] = -muDoubleScalarCos(kC->legAngleOffset[legNum - 1]) * 0.0;
  TP2B[13] = kC->B2PXOffset * muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = kC->B2PZOffset;
  for (i0 = 0; i0 < 4; i0++) {
    TP2B[3 + (i0 << 2)] = iv0[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      b_TP2B[loop_ub + 3 * i0] = -TP2B[i0 + (loop_ub << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * loop_ub] * TP2B[12 + loop_ub];
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      TB2P[loop_ub + (i0 << 2)] = TP2B[i0 + (loop_ub << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    TB2P[12 + i0] = c_TP2B[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    TB2P[3 + (i0 << 2)] = iv0[i0];
  }

  /* inv(TP2B);% */
  /* Transform the Cartesian goal and final positions in the pan coordinate */
  /* frame to the joint space. */
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      d0 += TB2P[i0 + (loop_ub << 2)] * nInitCartesianB[loop_ub];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  st.site = &b_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      d0 += TB2P[i0 + (loop_ub << 2)] * nGoalCartesianB[loop_ub];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  st.site = &c_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      c_TP2B[i0] += TB2P[i0 + (loop_ub << 2)] * nInitCartesianB[3 + loop_ub];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nInitJoint[i0] = qInit[i0];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nInitJoint[3] = phiInit;
  nInitJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nInitJoint[i0 + 5] = uB[i0];
    c_TB2P[i0] = 0.0;
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      c_TB2P[i0] += TB2P[i0 + (loop_ub << 2)] * nGoalCartesianB[3 + loop_ub];
    }

    b_TB2P[i0] = c_TB2P[i0];
    nGoalJoint[i0] = qGoal[i0];
  }

  nInitJoint[8] = 0.0;
  nInitJoint[9] = omegaInit;
  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nGoalJoint[3] = 0.0;
  nGoalJoint[4] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    nGoalJoint[i0 + 5] = uB[i0];
  }

  nGoalJoint[8] = 0.0;
  nGoalJoint[9] = 0.0;

  /* Check that the initial and final positions are valid. If they are not */
  /* return failure and an empty path. */
  emxInit_real_T(sp, &b_T, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathC, 2, &emlrtRTEI, true);
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    /* Run buildRRT. */
    dv0[0] = 1.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    dv1[0] = 0.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    for (i0 = 0; i0 < 10; i0++) {
      dv0[i0 + 3] = nInitJoint[i0];
      dv1[i0 + 3] = nGoalJoint[i0];
    }

    st.site = &d_emlrtRSI;
    buildRRT(&st, dv0, dv1, jointLimits, -(bodyHeight + kC->B2PZOffset), U, dt,
             Dt, kC, uBDot, legNum, b_T, pathJ);
    i0 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)T, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    loop_ub = b_T->size[0] * b_T->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      T->data[i0] = b_T->data[i0];
    }

    /* Transform path back to the Cartesian space. */
    st.site = &e_emlrtRSI;
    loop_ub = pathJ->size[0];
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = loop_ub;
    b_pathC->size[1] = 8;
    emxEnsureCapacity(&st, (emxArray__common *)b_pathC, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    loop_ub = pathJ->size[0] << 3;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = 0.0;
    }

    dist2Go = 0.0;
    i = 0;
    while (i <= pathJ->size[0] - 1) {
      i0 = pathJ->size[0];
      loop_ub = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i0, &b_emlrtBCI, &st);

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
      /* sherpaTTFK Sherpa_TT Forward Kinematics */
      /*    Calculates the x,y,z position of the contact point given the alpha, */
      /*    beta and gamma joint values. */
      uP[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
        (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] +
                 kC->zeta)) - kC->l7) * muDoubleScalarCos(pathJ->data[i +
        pathJ->size[0]]);
      uP[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
        (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] +
                 kC->zeta)) - kC->l7) * muDoubleScalarSin(pathJ->data[i +
        pathJ->size[0]]);
      uP[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-pathJ->data[i +
        (pathJ->size[0] << 1)])) - kC->l4 * muDoubleScalarSin(kC->zeta)) -
                kC->l5 * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * 3] +
                 kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
      i0 = pathJ->size[0];
      loop_ub = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i0, &c_emlrtBCI, &st);
      i0 = pathJ->size[0];
      loop_ub = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i0, &d_emlrtBCI, &st);

      /* sherpaTTFKVel.m */
      /* author: wreid */
      /* date: 20150122 */
      /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
      for (i0 = 0; i0 < 3; i0++) {
        d0 = 0.0;
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          d0 += TP2B[i0 + (loop_ub << 2)] * uP[loop_ub];
        }

        uB[i0] = d0 + TP2B[12 + i0];
      }

      if (1 + i != 1) {
        i0 = b_pathC->size[0];
        loop_ub = emlrtDynamicBoundsCheckFastR2012b(i, 1, i0, &e_emlrtBCI, &st);
        for (i0 = 0; i0 < 3; i0++) {
          c_TP2B[i0] = uB[i0] - b_pathC->data[(loop_ub + b_pathC->size[0] * (2 +
            i0)) - 1];
        }

        dist2Go += norm(c_TP2B);
      }

      loop_ub = b_pathC->size[0];
      i0 = 1 + i;
      emlrtDynamicBoundsCheckFastR2012b(i0, 1, loop_ub, &f_emlrtBCI, &st);
      b_pathJ[0] = (-pathJ->data[i + pathJ->size[0] * 6] * muDoubleScalarSin
                    (pathJ->data[i + pathJ->size[0]]) * ((((kC->l2 - kC->l7) +
        kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] +
        kC->zeta)) + kC->l3 * muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] <<
        1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - pathJ->data[i +
                    pathJ->size[0] * 7] * kC->l3 * muDoubleScalarCos(pathJ->
        data[i + pathJ->size[0]]) * muDoubleScalarSin(pathJ->data[i +
        (pathJ->size[0] << 1)])) - pathJ->data[i + (pathJ->size[0] << 3)] *
        kC->l5 * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * 3] +
        kC->zeta) * muDoubleScalarCos(pathJ->data[i + pathJ->size[0]]);
      b_pathJ[1] = (pathJ->data[i + pathJ->size[0] * 6] * muDoubleScalarCos
                    (pathJ->data[i + pathJ->size[0]]) * ((((kC->l2 - kC->l7) +
        kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * 3] +
        kC->zeta)) + kC->l3 * muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] <<
        1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - pathJ->data[i +
                    (pathJ->size[0] << 3)] * kC->l5 * muDoubleScalarSin
                    (pathJ->data[i + pathJ->size[0] * 3] + kC->zeta) *
                    muDoubleScalarSin(pathJ->data[i + pathJ->size[0]])) -
        pathJ->data[i + pathJ->size[0] * 7] * kC->l3 * muDoubleScalarSin
        (pathJ->data[i + pathJ->size[0]]) * muDoubleScalarSin(pathJ->data[i +
        (pathJ->size[0] << 1)]);
      b_pathJ[2] = -pathJ->data[i + pathJ->size[0] * 7] * kC->l3 *
        muDoubleScalarCos(pathJ->data[i + (pathJ->size[0] << 1)]) - kC->l5 *
        pathJ->data[i + (pathJ->size[0] << 3)] * muDoubleScalarCos(kC->zeta +
        pathJ->data[i + pathJ->size[0] * 3]);
      for (i0 = 0; i0 < 3; i0++) {
        c_TP2B[i0] = 0.0;
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          c_TP2B[i0] += TP2B[i0 + (loop_ub << 2)] * b_pathJ[loop_ub];
        }
      }

      i0 = pathJ->size[0];
      loop_ub = 1 + i;
      b_pathC->data[i] = pathJ->data[emlrtDynamicBoundsCheckFastR2012b(loop_ub,
        1, i0, &g_emlrtBCI, &st) - 1];
      b_pathC->data[i + b_pathC->size[0]] = dist2Go;
      for (i0 = 0; i0 < 3; i0++) {
        b_pathC->data[i + b_pathC->size[0] * (i0 + 2)] = uB[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        b_pathC->data[i + b_pathC->size[0] * (i0 + 5)] = c_TP2B[i0];
      }

      i++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = b_pathC->size[0];
    pathC->size[1] = 8;
    emxEnsureCapacity(sp, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    loop_ub = b_pathC->size[0] * b_pathC->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      pathC->data[i0] = b_pathC->data[i0];
    }
  } else {
    *success = false;
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 0;
    pathC->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    i0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)T, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
  }

  emxFree_real_T(&b_pathC);
  emxFree_real_T(&b_T);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRTWrapper_init(void)
{
  int32_T i8;
  static const real_T dv13[4] = { -0.293, -1.1326, -0.671, -0.7546 };

  static const real_T dv14[3] = { 1.0, 0.0, 0.5 };

  for (i8 = 0; i8 < 4; i8++) {
    cartesianLimits[i8] = dv13[i8];
  }

  for (i8 = 0; i8 < 3; i8++) {
    HGAINS[i8] = dv14[i8];
  }
}

void cartesianLimits_not_empty_init(void)
{
}

void exhaustive_not_empty_init(void)
{
}

void goalSeedFreq_not_empty_init(void)
{
}

/* End of code generation (buildRRTWrapper.c) */
