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
#include "sherpaTTIK.h"
#include "buildRRT.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
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

static emlrtRSInfo f_emlrtRSI = { 107, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo g_emlrtRSI = { 112, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo emlrtRTEI = { 27, 36, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 27, 13, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtBCInfo emlrtBCI = { -1, -1, 21, 15, "s1", "linInterp",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/linearInterpolator/linInterp.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 20, 14, "s1", "linInterp",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/linearInterpolator/linInterp.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 130, 15, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 128, 48, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 124, 50, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 124, 37, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { -1, -1, 123, 31, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo h_emlrtBCI = { 1, 4, 18, 17, "kC.legAngleOffset", "trP2B",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/trP2B.m",
  0 };

static emlrtECInfo emlrtECI = { 2, 108, 13, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtBCInfo i_emlrtBCI = { -1, -1, 105, 21, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 22, 10, "s1", "linInterp",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/linearInterpolator/linInterp.m",
  0 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 130, 29, "pathJ", "buildRRTWrapper",
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
  int32_T i1;
  real_T c_TP2B[3];
  real_T TB2P[16];
  real_T b_TB2P[3];
  real_T y;
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
  int32_T loop_ub;
  int32_T pathJ_idx_0;
  real_T dist2Go;
  int32_T i;
  real_T newState[3];
  real_T b_pathJ[3];
  real_T sGoalC[9];
  real_T pathCorrection[90];
  real_T b_newState;
  emxArray_real_T *c_pathJ;
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
  emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &h_emlrtBCI, &st);

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
    for (i1 = 0; i1 < 3; i1++) {
      b_TP2B[i1 + 3 * i0] = -TP2B[i0 + (i1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * i1] * TP2B[12 + i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      TB2P[i1 + (i0 << 2)] = TP2B[i0 + (i1 << 2)];
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
    y = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      y += TB2P[i0 + (i1 << 2)] * nInitCartesianB[i1];
    }

    b_TB2P[i0] = y + TB2P[12 + i0];
  }

  st.site = &b_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    y = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      y += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[i1];
    }

    b_TB2P[i0] = y + TB2P[12 + i0];
  }

  st.site = &c_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      c_TP2B[i0] += TB2P[i0 + (i1 << 2)] * nInitCartesianB[3 + i1];
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
    for (i1 = 0; i1 < 3; i1++) {
      c_TB2P[i0] += TB2P[i0 + (i1 << 2)] * nGoalCartesianB[3 + i1];
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
  emxInit_real_T(sp, &b_pathC, 2, &b_emlrtRTEI, true);
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
    pathJ_idx_0 = pathJ->size[0];
    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = pathJ_idx_0;
    pathC->size[1] = 9;
    emxEnsureCapacity(&st, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    loop_ub = pathJ->size[0] * 9;
    for (i0 = 0; i0 < loop_ub; i0++) {
      pathC->data[i0] = 0.0;
    }

    dist2Go = 0.0;
    i = 0;
    while (i <= pathJ->size[0] - 1) {
      i0 = pathJ->size[0];
      i1 = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(i1, 1, i0, &g_emlrtBCI, &st);

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
      newState[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
        (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                      kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] *
        3] + kC->zeta)) - kC->l7) * muDoubleScalarCos(pathJ->data[i +
        pathJ->size[0]]);
      newState[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
        (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) +
                      kC->l5 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] *
        3] + kC->zeta)) - kC->l7) * muDoubleScalarSin(pathJ->data[i +
        pathJ->size[0]]);
      newState[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-pathJ->data[i +
        (pathJ->size[0] << 1)])) - kC->l4 * muDoubleScalarSin(kC->zeta)) -
                      kC->l5 * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] *
        3] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
      i0 = pathJ->size[0];
      i1 = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(i1, 1, i0, &f_emlrtBCI, &st);
      i0 = pathJ->size[0];
      i1 = i + 1;
      emlrtDynamicBoundsCheckFastR2012b(i1, 1, i0, &e_emlrtBCI, &st);

      /* sherpaTTFKVel.m */
      /* author: wreid */
      /* date: 20150122 */
      /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
      for (i0 = 0; i0 < 3; i0++) {
        y = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          y += TP2B[i0 + (i1 << 2)] * newState[i1];
        }

        uB[i0] = y + TP2B[12 + i0];
      }

      if (1 + i != 1) {
        i0 = pathC->size[0];
        pathJ_idx_0 = emlrtDynamicBoundsCheckFastR2012b(i, 1, i0, &d_emlrtBCI,
          &st);
        for (i0 = 0; i0 < 3; i0++) {
          c_TP2B[i0] = uB[i0] - pathC->data[(pathJ_idx_0 + pathC->size[0] * (2 +
            i0)) - 1];
        }

        dist2Go += norm(c_TP2B);
      }

      pathJ_idx_0 = pathC->size[0];
      i0 = 1 + i;
      emlrtDynamicBoundsCheckFastR2012b(i0, 1, pathJ_idx_0, &c_emlrtBCI, &st);
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
        for (i1 = 0; i1 < 3; i1++) {
          c_TP2B[i0] += TP2B[i0 + (i1 << 2)] * b_pathJ[i1];
        }
      }

      i0 = pathJ->size[0];
      i1 = 1 + i;
      pathC->data[i] = pathJ->data[emlrtDynamicBoundsCheckFastR2012b(i1, 1, i0,
        &k_emlrtBCI, &st) - 1];
      pathC->data[i + pathC->size[0]] = dist2Go;
      for (i0 = 0; i0 < 3; i0++) {
        pathC->data[i + pathC->size[0] * (i0 + 2)] = uB[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        pathC->data[i + pathC->size[0] * (i0 + 5)] = c_TP2B[i0];
      }

      pathC->data[i + (pathC->size[0] << 3)] = 0.0;
      i++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = pathC->size[0];
    b_pathC->size[1] = 9;
    emxEnsureCapacity(sp, (emxArray__common *)b_pathC, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    loop_ub = pathC->size[0] * pathC->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = pathC->data[i0];
    }
  } else {
    *success = false;
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = 0;
    b_pathC->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)b_pathC, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
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

  emxFree_real_T(&b_T);

  /* Linearly interpolate to the goal state from the final state. */
  i0 = b_pathC->size[0];
  i1 = b_pathC->size[0];
  emlrtDynamicBoundsCheckFastR2012b(i1, 1, i0, &i_emlrtBCI, sp);
  sGoalC[0] = 0.0;
  sGoalC[1] = 0.0;
  for (i0 = 0; i0 < 6; i0++) {
    sGoalC[i0 + 2] = nGoalCartesianB[i0];
  }

  sGoalC[8] = 1.0;
  st.site = &f_emlrtRSI;

  /* LININTERP Returns a Cartesian path that is linearly interpolated between */
  /* two points. */
  /*  */
  /* Inputs: */
  /* s1: The initial state that is represented by a 1x8 vector. The vector has */
  /* the form [t x y z xDot yDot zDot interpBoolean] */
  /* s2: The goal state that is represented by a 1x8 vector. The vector has */
  /* the form [t x y z xDot yDot zDot interpBoolean] */
  /* N: The number of waypoints to be placed in the path. */
  /*  */
  /* Outputs: */
  /* path: A 8xN matrix that returns a path from s2 to s1. */
  /*  */
  /* linInterp.m */
  /* author: wreid */
  /* date: 20150224 */
  i0 = b_pathC->size[1];
  emlrtDynamicBoundsCheckFastR2012b(1, 1, i0, &b_emlrtBCI, &st);
  i0 = b_pathC->size[1];
  emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &emlrtBCI, &st);
  pathJ_idx_0 = b_pathC->size[0];
  dist2Go = b_pathC->data[(pathJ_idx_0 + b_pathC->size[0]) - 1];
  i0 = b_pathC->size[1];
  loop_ub = b_pathC->size[1];
  pathJ_idx_0 = b_pathC->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    b_TP2B[i1] = b_pathC->data[(pathJ_idx_0 + b_pathC->size[0] * i1) - 1];
  }

  for (i1 = 0; i1 < 3; i1++) {
    pathJ_idx_0 = 3 + i1;
    qInit[i1] = b_TP2B[emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0, 1, i0,
      &j_emlrtBCI, &st) - 1];
  }

  for (i0 = 0; i0 < 3; i0++) {
    qGoal[i0] = qInit[i0];
  }

  for (i = 0; i < 10; i++) {
    y = (1.0 + (real_T)i) / 10.0;
    for (i0 = 0; i0 < 3; i0++) {
      b_newState = qInit[i0] + y * (sGoalC[2 + i0] - qInit[i0]);
      c_TP2B[i0] = b_newState - qGoal[i0];
      newState[i0] = b_newState;
    }

    dist2Go += norm(c_TP2B);
    pathJ_idx_0 = b_pathC->size[0];
    pathCorrection[i] = b_pathC->data[pathJ_idx_0 - 1] + (1.0 + (real_T)i) /
      10.0;
    pathCorrection[10 + i] = dist2Go;
    for (i0 = 0; i0 < 3; i0++) {
      pathCorrection[i + 10 * (i0 + 2)] = newState[i0];
      qGoal[i0] = newState[i0];
    }

    pathCorrection[50 + i] = 0.0;
    pathCorrection[60 + i] = 0.0;
    pathCorrection[70 + i] = 0.0;
    pathCorrection[80 + i] = 1.0;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  i0 = b_pathC->size[1];
  emlrtDimSizeEqCheckFastR2012b(9, i0, &emlrtECI, sp);
  i0 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = b_pathC->size[0] + 10;
  pathC->size[1] = b_pathC->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                    &emlrtRTEI);
  loop_ub = b_pathC->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    pathJ_idx_0 = b_pathC->size[0];
    for (i1 = 0; i1 < pathJ_idx_0; i1++) {
      pathC->data[i1 + pathC->size[0] * i0] = b_pathC->data[i1 + b_pathC->size[0]
        * i0];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    for (i1 = 0; i1 < 10; i1++) {
      pathC->data[(i1 + b_pathC->size[0]) + pathC->size[0] * i0] =
        pathCorrection[i1 + 10 * i0];
    }
  }

  emxFree_real_T(&b_pathC);
  emxInit_real_T(sp, &c_pathJ, 2, &emlrtRTEI, true);
  for (i = 0; i < 10; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      y = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        y += TB2P[i0 + (i1 << 2)] * pathCorrection[i + 10 * (2 + i1)];
      }

      b_TB2P[i0] = y + TB2P[12 + i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      c_TP2B[i0] = b_TB2P[i0];
    }

    st.site = &g_emlrtRSI;
    b_sherpaTTIK(&st, c_TP2B, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, qInit);
    i0 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = pathJ->size[0] + 1;
    c_pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)c_pathJ, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    for (i0 = 0; i0 < 11; i0++) {
      loop_ub = pathJ->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        c_pathJ->data[i1 + c_pathJ->size[0] * i0] = pathJ->data[i1 + pathJ->
          size[0] * i0];
      }
    }

    c_pathJ->data[pathJ->size[0]] = pathCorrection[i];
    for (i0 = 0; i0 < 3; i0++) {
      c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * (i0 + 1)] = qInit[i0];
    }

    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 2)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 5] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 6] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 7] = 0.0;
    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 3)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 9] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 10] = 0.0;
    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = c_pathJ->size[0];
    pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 11; i0++) {
      loop_ub = c_pathJ->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        pathJ->data[i1 + pathJ->size[0] * i0] = c_pathJ->data[i1 + c_pathJ->
          size[0] * i0];
      }
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&c_pathJ);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRTWrapper_init(void)
{
  int32_T i9;
  static const real_T dv13[4] = { -0.293, -1.1326, -0.671, -0.7546 };

  static const real_T dv14[3] = { 1.0, 0.0, 0.5 };

  for (i9 = 0; i9 < 4; i9++) {
    cartesianLimits[i9] = dv13[i9];
  }

  for (i9 = 0; i9 < 3; i9++) {
    HGAINS[i9] = dv14[i9];
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
