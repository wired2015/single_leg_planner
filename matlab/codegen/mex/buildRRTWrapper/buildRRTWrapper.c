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
#include "buildRRT.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static real_T cartesianLimits[4];
static boolean_T cartesianLimits_not_empty;
static emlrtRSInfo emlrtRSI = { 77, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo b_emlrtRSI = { 86, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo c_emlrtRSI = { 87, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo d_emlrtRSI = { 101, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo e_emlrtRSI = { 103, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo emlrtRTEI = { 27, 36, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtBCInfo emlrtBCI = { -1, -1, 134, 19, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 133, 19, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 134, 44, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 131, 60, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 131, 42, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 129, 37, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { 1, 4, 18, 17, "kC.legAngleOffset", "trP2B",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/trP2B.m",
  0 };

static emlrtDCInfo emlrtDCI = { 123, 19, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  1 };

static emlrtDCInfo b_emlrtDCI = { 123, 19, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  4 };

static emlrtDCInfo c_emlrtDCI = { 124, 19, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  1 };

static emlrtDCInfo d_emlrtDCI = { 124, 19, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  4 };

static emlrtBCInfo h_emlrtBCI = { -1, -1, 129, 39, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 131, 44, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 131, 62, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo k_emlrtBCI = { -1, -1, 134, 46, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo l_emlrtBCI = { -1, -1, 134, 61, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo m_emlrtBCI = { -1, -1, 134, 75, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

/* Function Definitions */
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
                     omegaInit, const real_T jointLimits[12], real_T bodyHeight,
                     const real_T U[10], real_T dt, real_T Dt, const struct0_T
                     *kC, real_T threshold, int32_T legNum, const real_T uBDot[6],
                     const real_T HGAINS[3], emxArray_real_T *T, emxArray_real_T
                     *pathC, emxArray_real_T *pathJ, boolean_T *success)
{
  int32_T i0;
  real_T TP2B[16];
  static const int8_T iv0[4] = { 0, 0, 0, 1 };

  real_T b_TP2B[9];
  int32_T md2;
  real_T c_TP2B[3];
  real_T TB2P[16];
  real_T b_TB2P[3];
  real_T d0;
  real_T qInit[3];
  real_T qGoal[3];
  real_T nInitJoint[6];
  real_T qDotInit[3];
  real_T nGoalJoint[6];
  emxArray_real_T *b_T;
  emxArray_real_T *b_pathC;
  emxArray_real_T *b_pathJ;
  emxArray_real_T *x;
  emxArray_real_T *b_x;
  real_T dv0[11];
  real_T dv1[11];
  real_T dv2[4];
  int32_T loop_ub;
  real_T xtmp;
  uint32_T count;
  real_T time;
  int32_T i;
  int32_T j;
  real_T uP[3];
  real_T betaDot;
  real_T gammaDot;
  real_T alphaDot[3];
  int32_T pathJ_idx_0;
  int32_T m;
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
  if (!cartesianLimits_not_empty) {
    for (i0 = 0; i0 < 4; i0++) {
      cartesianLimits[i0] = 0.0;
    }

    cartesianLimits_not_empty = true;

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
    cartesianLimits[0] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[2]))
      - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
      (jointLimits[4] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    cartesianLimits[1] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[3]))
      - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
      (jointLimits[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    cartesianLimits[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[2]))
      - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
      (jointLimits[5] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);

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
    cartesianLimits[3] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-jointLimits[3]))
      - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5 * muDoubleScalarSin
      (jointLimits[4] + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
  }

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
  emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &g_emlrtBCI, &st);

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
    for (md2 = 0; md2 < 3; md2++) {
      b_TP2B[md2 + 3 * i0] = -TP2B[i0 + (md2 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * md2] * TP2B[12 + md2];
    }

    for (md2 = 0; md2 < 3; md2++) {
      TB2P[md2 + (i0 << 2)] = TP2B[i0 + (md2 << 2)];
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
    for (md2 = 0; md2 < 3; md2++) {
      d0 += TB2P[i0 + (md2 << 2)] * nInitCartesianB[md2];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  st.site = &b_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      d0 += TB2P[i0 + (md2 << 2)] * nGoalCartesianB[md2];
    }

    b_TB2P[i0] = d0 + TB2P[12 + i0];
  }

  st.site = &c_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      c_TP2B[i0] += TB2P[i0 + (md2 << 2)] * nInitCartesianB[3 + md2];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nInitJoint[i0] = qInit[i0];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                qDotInit);
  for (i0 = 0; i0 < 3; i0++) {
    nInitJoint[i0 + 3] = qDotInit[i0];
    c_TP2B[i0] = 0.0;
    for (md2 = 0; md2 < 3; md2++) {
      c_TP2B[i0] += TB2P[i0 + (md2 << 2)] * nGoalCartesianB[3 + md2];
    }

    b_TB2P[i0] = c_TP2B[i0];
    nGoalJoint[i0] = qGoal[i0];
  }

  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                c_TP2B);
  for (i0 = 0; i0 < 3; i0++) {
    nGoalJoint[i0 + 3] = c_TP2B[i0];
  }

  /* Check that the initial and final positions are valid. If they are not */
  /* return failure and an empty path. */
  emxInit_real_T(sp, &b_T, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathC, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathJ, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &x, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &b_x, 2, &emlrtRTEI, true);
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    /* Run buildRRT. */
    dv0[0] = 0.0;
    dv0[1] = 0.0;
    dv0[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      dv0[i0 + 3] = nInitJoint[i0];
    }

    dv0[9] = 0.0;
    dv0[10] = 0.0;
    dv1[0] = 0.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      dv1[i0 + 3] = nGoalJoint[i0];
    }

    dv1[9] = 0.0;
    dv1[10] = 0.0;
    for (i0 = 0; i0 < 4; i0++) {
      dv2[i0] = cartesianLimits[i0];
    }

    st.site = &d_emlrtRSI;
    buildRRT(&st, dv0, dv1, jointLimits, dv2, -(bodyHeight + kC->B2PZOffset),
             HGAINS, U, dt, Dt, kC, uBDot, legNum, b_T, pathJ);
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

    /* Take the pathOld array and combine the general nodes and intermediate */
    /* states into a uniform path. The output path should be a npx6 array */
    /* that contains the n general nodes and the p intermediate nodes between */
    /* general nodes. Each row in the path matrix contains  */
    /* [t,x,y,z,xDot,yDot,zDot] state data. */
    xtmp = muDoubleScalarRound(Dt / dt);
    d0 = xtmp * (real_T)pathJ->size[0];
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &b_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d0, &emlrtDCI, &st);
    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = loop_ub;
    b_pathC->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)b_pathC, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    d0 = xtmp * (real_T)pathJ->size[0];
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &b_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d0, &emlrtDCI, &st) * 7;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = 0.0;
    }

    xtmp = muDoubleScalarRound(Dt / dt);
    d0 = xtmp * (real_T)pathJ->size[0];
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &d_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d0, &c_emlrtDCI, &st);
    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = loop_ub;
    b_pathJ->size[1] = 9;
    emxEnsureCapacity(&st, (emxArray__common *)b_pathJ, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    d0 = xtmp * (real_T)pathJ->size[0];
    d0 = emlrtNonNegativeCheckFastR2012b(d0, &d_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(d0, &c_emlrtDCI, &st) * 9;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathJ->data[i0] = 0.0;
    }

    count = 1U;
    time = muDoubleScalarRound(Dt / dt) * (real_T)pathJ->size[0] * dt;
    i = 0;
    while (i <= pathJ->size[0] - 1) {
      j = pathJ->size[1] - 5;
      while (j + 5 >= 18) {
        i0 = pathJ->size[0];
        md2 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &f_emlrtBCI, &st);
        for (i0 = 0; i0 < 3; i0++) {
          md2 = pathJ->size[1];
          loop_ub = i0 + j;
          emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, md2, &h_emlrtBCI, &st);
        }

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
        xtmp = pathJ->data[i + pathJ->size[0] * (-1 + j)];
        uP[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
          pathJ->size[0] * j])) + kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5
                  * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * (1 + j)]
                   + kC->zeta)) - kC->l7) * muDoubleScalarCos(xtmp);
        uP[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[i +
          pathJ->size[0] * j])) + kC->l4 * muDoubleScalarCos(kC->zeta)) + kC->l5
                  * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * (1 + j)]
                   + kC->zeta)) - kC->l7) * muDoubleScalarSin(xtmp);
        uP[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-pathJ->data[i +
          pathJ->size[0] * j])) - kC->l4 * muDoubleScalarSin(kC->zeta)) - kC->l5
                  * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * (1 + j)]
                   + kC->zeta)) - kC->l6) - (kC->l8 + kC->r);
        i0 = pathJ->size[0];
        md2 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &e_emlrtBCI, &st);
        for (i0 = 0; i0 < 3; i0++) {
          md2 = pathJ->size[1];
          loop_ub = (i0 + j) + 3;
          emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, md2, &i_emlrtBCI, &st);
        }

        i0 = pathJ->size[0];
        md2 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &d_emlrtBCI, &st);
        for (i0 = 0; i0 < 3; i0++) {
          md2 = pathJ->size[1];
          loop_ub = i0 + j;
          emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, md2, &j_emlrtBCI, &st);
        }

        /* sherpaTTFKVel.m */
        /* author: wreid */
        /* date: 20150122 */
        /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
        xtmp = pathJ->data[i + pathJ->size[0] * (2 + j)];
        betaDot = pathJ->data[i + pathJ->size[0] * (3 + j)];
        gammaDot = pathJ->data[i + pathJ->size[0] * (4 + j)];
        loop_ub = b_pathC->size[0];
        i0 = (int32_T)count;
        emlrtDynamicBoundsCheckFastR2012b(i0, 1, loop_ub, &b_emlrtBCI, &st);
        for (i0 = 0; i0 < 3; i0++) {
          d0 = 0.0;
          for (md2 = 0; md2 < 3; md2++) {
            d0 += TP2B[i0 + (md2 << 2)] * uP[md2];
          }

          c_TP2B[i0] = d0 + TP2B[12 + i0];
        }

        alphaDot[0] = (-xtmp * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] *
                        (-1 + j)]) * ((((kC->l2 - kC->l7) + kC->l5 *
          muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * (1 + j)] + kC->zeta))
          + kC->l3 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * j])) +
          kC->l4 * muDoubleScalarCos(kC->zeta)) - betaDot * kC->l3 *
                       muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * (-1 +
          j)]) * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * j])) -
          gammaDot * kC->l5 * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] *
          (1 + j)] + kC->zeta) * muDoubleScalarCos(pathJ->data[i + pathJ->size[0]
          * (-1 + j)]);
        alphaDot[1] = (xtmp * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] *
                        (-1 + j)]) * ((((kC->l2 - kC->l7) + kC->l5 *
          muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * (1 + j)] + kC->zeta))
          + kC->l3 * muDoubleScalarCos(pathJ->data[i + pathJ->size[0] * j])) +
          kC->l4 * muDoubleScalarCos(kC->zeta)) - gammaDot * kC->l5 *
                       muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * (1 + j)]
          + kC->zeta) * muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * (-1 +
          j)])) - betaDot * kC->l3 * muDoubleScalarSin(pathJ->data[i +
          pathJ->size[0] * (-1 + j)]) * muDoubleScalarSin(pathJ->data[i +
          pathJ->size[0] * j]);
        alphaDot[2] = -betaDot * kC->l3 * muDoubleScalarCos(pathJ->data[i +
          pathJ->size[0] * j]) - kC->l5 * gammaDot * muDoubleScalarCos(kC->zeta
          + pathJ->data[i + pathJ->size[0] * (1 + j)]);
        for (i0 = 0; i0 < 3; i0++) {
          b_TB2P[i0] = 0.0;
          for (md2 = 0; md2 < 3; md2++) {
            b_TB2P[i0] += TP2B[i0 + (md2 << 2)] * alphaDot[md2];
          }
        }

        b_pathC->data[(int32_T)count - 1] = time;
        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int32_T)count + b_pathC->size[0] * (i0 + 1)) - 1] =
            c_TP2B[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
          b_pathC->data[((int32_T)count + b_pathC->size[0] * (i0 + 4)) - 1] =
            b_TB2P[i0];
        }

        loop_ub = b_pathJ->size[0];
        i0 = (int32_T)count;
        emlrtDynamicBoundsCheckFastR2012b(i0, 1, loop_ub, &emlrtBCI, &st);
        i0 = pathJ->size[0];
        md2 = 1 + i;
        i0 = emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &c_emlrtBCI, &st);
        b_pathJ->data[(int32_T)count - 1] = time;
        for (md2 = 0; md2 < 6; md2++) {
          loop_ub = pathJ->size[1];
          pathJ_idx_0 = md2 + j;
          b_pathJ->data[((int32_T)count + b_pathJ->size[0] * (md2 + 1)) - 1] =
            pathJ->data[(i0 + pathJ->size[0] *
                         (emlrtDynamicBoundsCheckFastR2012b(pathJ_idx_0, 1,
            loop_ub, &k_emlrtBCI, &st) - 1)) - 1];
        }

        i0 = pathJ->size[0];
        md2 = 1 + i;
        b_pathJ->data[((int32_T)count + b_pathJ->size[0] * 7) - 1] = pathJ->
          data[(emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &l_emlrtBCI, &st)
                + pathJ->size[0] * 9) - 1];
        i0 = pathJ->size[0];
        md2 = 1 + i;
        b_pathJ->data[((int32_T)count + (b_pathJ->size[0] << 3)) - 1] =
          pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(md2, 1, i0, &m_emlrtBCI,
          &st) + pathJ->size[0] * 10) - 1];
        time -= dt;
        count++;
        j -= 6;
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
      }

      i++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    loop_ub = b_pathC->size[0];
    i0 = x->size[0] * x->size[1];
    x->size[0] = loop_ub;
    x->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)x, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 7; i0++) {
      for (md2 = 0; md2 < loop_ub; md2++) {
        x->data[md2 + x->size[0] * i0] = b_pathC->data[md2 + b_pathC->size[0] *
          i0];
      }
    }

    i0 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = x->size[0];
    b_pathC->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)b_pathC, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    loop_ub = x->size[0] * x->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathC->data[i0] = x->data[i0];
    }

    m = x->size[0];
    i0 = x->size[0];
    if (i0 >= 0) {
      md2 = (int32_T)((uint32_T)i0 >> 1);
    } else {
      md2 = (int32_T)~(~(uint32_T)i0 >> 1);
    }

    for (j = 0; j < 7; j++) {
      for (i = 1; i <= md2; i++) {
        loop_ub = b_pathC->size[0];
        xtmp = b_pathC->data[(i + loop_ub * j) - 1];
        loop_ub = b_pathC->size[0];
        pathJ_idx_0 = b_pathC->size[0];
        b_pathC->data[(i + loop_ub * j) - 1] = b_pathC->data[(m - i) +
          pathJ_idx_0 * j];
        loop_ub = b_pathC->size[0];
        b_pathC->data[(m - i) + loop_ub * j] = xtmp;
      }
    }

    loop_ub = b_pathJ->size[0];
    i0 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = loop_ub;
    b_x->size[1] = 9;
    emxEnsureCapacity(&st, (emxArray__common *)b_x, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 9; i0++) {
      for (md2 = 0; md2 < loop_ub; md2++) {
        b_x->data[md2 + b_x->size[0] * i0] = b_pathJ->data[md2 + b_pathJ->size[0]
          * i0];
      }
    }

    i0 = b_pathJ->size[0] * b_pathJ->size[1];
    b_pathJ->size[0] = b_x->size[0];
    b_pathJ->size[1] = 9;
    emxEnsureCapacity(&st, (emxArray__common *)b_pathJ, i0, (int32_T)sizeof
                      (real_T), &emlrtRTEI);
    loop_ub = b_x->size[0] * b_x->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_pathJ->data[i0] = b_x->data[i0];
    }

    m = b_x->size[0];
    i0 = b_x->size[0];
    if (i0 >= 0) {
      md2 = (int32_T)((uint32_T)i0 >> 1);
    } else {
      md2 = (int32_T)~(~(uint32_T)i0 >> 1);
    }

    for (j = 0; j < 9; j++) {
      for (i = 1; i <= md2; i++) {
        loop_ub = b_pathJ->size[0];
        xtmp = b_pathJ->data[(i + loop_ub * j) - 1];
        loop_ub = b_pathJ->size[0];
        pathJ_idx_0 = b_pathJ->size[0];
        b_pathJ->data[(i + loop_ub * j) - 1] = b_pathJ->data[(m - i) +
          pathJ_idx_0 * j];
        loop_ub = b_pathJ->size[0];
        b_pathJ->data[(m - i) + loop_ub * j] = xtmp;
      }
    }

    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 1 + b_pathC->size[0];
    pathC->size[1] = 7;
    emxEnsureCapacity(sp, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    pathC->data[0] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      pathC->data[pathC->size[0] * (i0 + 1)] = nInitCartesianB[i0];
    }

    for (i0 = 0; i0 < 7; i0++) {
      loop_ub = b_pathC->size[0];
      for (md2 = 0; md2 < loop_ub; md2++) {
        pathC->data[(md2 + pathC->size[0] * i0) + 1] = b_pathC->data[md2 +
          b_pathC->size[0] * i0];
      }
    }

    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 1 + b_pathJ->size[0];
    pathJ->size[1] = 9;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    pathJ->data[0] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      pathJ->data[pathJ->size[0] * (i0 + 1)] = qInit[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      pathJ->data[pathJ->size[0] * (i0 + 4)] = qDotInit[i0];
    }

    pathJ->data[pathJ->size[0] * 7] = phiInit;
    pathJ->data[pathJ->size[0] << 3] = omegaInit;
    for (i0 = 0; i0 < 9; i0++) {
      loop_ub = b_pathJ->size[0];
      for (md2 = 0; md2 < loop_ub; md2++) {
        pathJ->data[(md2 + pathJ->size[0] * i0) + 1] = b_pathJ->data[md2 +
          b_pathJ->size[0] * i0];
      }
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
    pathJ->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    i0 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)T, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
  }

  emxFree_real_T(&b_x);
  emxFree_real_T(&x);
  emxFree_real_T(&b_pathJ);
  emxFree_real_T(&b_pathC);
  emxFree_real_T(&b_T);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRTWrapper_init(void)
{
}

void cartesianLimits_not_empty_init(void)
{
  cartesianLimits_not_empty = false;
}

void exhaustive_not_empty_init(void)
{
}

void goalSeedFreq_not_empty_init(void)
{
}

/* End of code generation (buildRRTWrapper.c) */
