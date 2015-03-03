/*
 * buildRRTWrapper.c
 *
 * Code generation for function 'buildRRTWrapper'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_mex_emxutil.h"
#include "heuristicSingleLeg.h"
#include "selectInput.h"
#include "nearestNeighbour.h"
#include "randomState.h"
#include "norm.h"
#include "sherpaTTIK.h"
#include "validJointState.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo rb_emlrtRSI = { 44, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo sb_emlrtRSI = { 53, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo tb_emlrtRSI = { 54, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo ub_emlrtRSI = { 68, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo vb_emlrtRSI = { 70, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo wb_emlrtRSI = { 85, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo xb_emlrtRSI = { 90, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRSInfo yb_emlrtRSI = { 68, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo ac_emlrtRSI = { 43, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo bc_emlrtRSI = { 26, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo cc_emlrtRSI = { 75, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo dc_emlrtRSI = { 76, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRSInfo ec_emlrtRSI = { 77, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo k_emlrtRTEI = { 24, 36, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo l_emlrtRTEI = { 24, 13, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtRTEInfo m_emlrtRTEI = { 55, 9, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 68, 5, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo o_emlrtRTEI = { 5, 13, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo q_emlrtBCI = { -1, -1, 82, 25, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtECInfo e_emlrtECI = { 2, 86, 13, "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m"
};

static emlrtECInfo f_emlrtECI = { 1, 69, 12, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtECInfo g_emlrtECI = { -1, 64, 27, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo r_emlrtBCI = { 1, 1500, 61, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtDCInfo c_emlrtDCI = { 61, 18, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  1 };

static emlrtECInfo h_emlrtECI = { -1, 57, 47, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtRTEInfo v_emlrtRTEI = { 56, 9, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m"
};

static emlrtBCInfo s_emlrtBCI = { 1, 1500, 84, 7, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/normalRRT/buildRRT.m",
  0 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 101, 31, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo u_emlrtBCI = { -1, -1, 102, 37, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo v_emlrtBCI = { -1, -1, 102, 50, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo w_emlrtBCI = { -1, -1, 106, 48, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo x_emlrtBCI = { -1, -1, 108, 15, "pathC", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

static emlrtBCInfo y_emlrtBCI = { -1, -1, 21, 15, "s1", "linInterp",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/linearInterpolator/linInterp.m",
  0 };

static emlrtBCInfo ab_emlrtBCI = { -1, -1, 22, 10, "s1", "linInterp",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/linearInterpolator/linInterp.m",
  0 };

static emlrtBCInfo bb_emlrtBCI = { -1, -1, 108, 29, "pathJ", "buildRRTWrapper",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRTWrapper.m",
  0 };

/* Function Definitions */
void buildRRTWrapper(sherpaTTPlanner_mexStackData *SD, const emlrtStack *sp,
                     const real_T nInitCartesianB[6], const real_T
                     nGoalCartesianB[6], real_T phiInit, real_T omegaInit, const
                     real_T jointLimits[20], real_T bodyHeight, const struct0_T *
                     kC, int32_T legNum, const real_T uBDot[6], emxArray_real_T *
                     T, emxArray_real_T *pathC, emxArray_real_T *pathJ,
                     boolean_T *success)
{
  boolean_T b3;
  real_T panHeight;
  real_T TP2B[16];
  int32_T i17;
  static const int8_T iv11[4] = { 0, 0, 0, 1 };

  real_T b_TP2B[9];
  int32_T absb;
  real_T c_TP2B[3];
  real_T TB2P[16];
  real_T b_TB2P[3];
  real_T nodeIDCount;
  real_T qInit[3];
  real_T qGoal[3];
  real_T nInitJoint[10];
  real_T uB[3];
  real_T c_TB2P[3];
  real_T nGoalJoint[10];
  emxArray_real_T *b_pathC;
  emxArray_real_T *transitionPath;
  emxArray_real_T *t;
  emxArray_real_T *path;
  emxArray_real_T *y;
  emxArray_real_T *b_transitionPath;
  emxArray_real_T *c_transitionPath;
  int32_T cdiff;
  real_T xRand[13];
  real_T check;
  real_T transitionArrayNearest[80];
  real_T xNear[13];
  real_T xNew[13];
  real_T dv18[13];
  real_T next_data[93];
  int32_T transitionArray_size[2];
  real_T transitionArray_data[80];
  int32_T ndbl;
  int32_T iv12[2];
  int32_T apnd;
  int32_T next_size[2];
  int32_T tmp_size[2];
  int8_T tmp_data[80];
  real_T newState[3];
  real_T b_pathJ[3];
  real_T sFinalC_data[9];
  real_T sGoalC[9];
  real_T pathCorrection[90];
  emxArray_real_T *c_pathJ;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  b3 = false;

  /* BUILDRRTWRAPPER This function acts as a wrapper for the buildRRT function. */
  /* Code generation for the singleLegPlanner is performed using this function */
  /* as an entry point. */
  /*    */
  /* Inputs: */
  /* -nInitCartesianB: the */
  /* -nGoalCartesianB: */
  /* -jointLimits: */
  /* -bodyHeight: */
  /* -kC: */
  /* -legNum: */
  /* -uBDot: */
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
  /* GETPANHEIGHT Summary of this function goes here */
  /*    Detailed explanation goes here */
  panHeight = -(bodyHeight + kC->B2PZOffset);

  /* Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame */
  /* to the pan coordinate frame. */
  st.site = &rb_emlrtRSI;

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
  for (i17 = 0; i17 < 4; i17++) {
    TP2B[3 + (i17 << 2)] = iv11[i17];
  }

  for (i17 = 0; i17 < 3; i17++) {
    for (absb = 0; absb < 3; absb++) {
      b_TP2B[absb + 3 * i17] = -TP2B[i17 + (absb << 2)];
    }
  }

  for (i17 = 0; i17 < 3; i17++) {
    c_TP2B[i17] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TP2B[i17] += b_TP2B[i17 + 3 * absb] * TP2B[12 + absb];
    }

    for (absb = 0; absb < 3; absb++) {
      TB2P[absb + (i17 << 2)] = TP2B[i17 + (absb << 2)];
    }
  }

  for (i17 = 0; i17 < 3; i17++) {
    TB2P[12 + i17] = c_TP2B[i17];
  }

  for (i17 = 0; i17 < 4; i17++) {
    TB2P[3 + (i17 << 2)] = iv11[i17];
  }

  /* inv(TP2B);% */
  /* Transform the Cartesian goal and final positions in the pan coordinate */
  /* frame to the joint space. */
  for (i17 = 0; i17 < 3; i17++) {
    nodeIDCount = 0.0;
    for (absb = 0; absb < 3; absb++) {
      nodeIDCount += TB2P[i17 + (absb << 2)] * nInitCartesianB[absb];
    }

    b_TB2P[i17] = nodeIDCount + TB2P[12 + i17];
  }

  st.site = &sb_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qInit);
  for (i17 = 0; i17 < 3; i17++) {
    nodeIDCount = 0.0;
    for (absb = 0; absb < 3; absb++) {
      nodeIDCount += TB2P[i17 + (absb << 2)] * nGoalCartesianB[absb];
    }

    b_TB2P[i17] = nodeIDCount + TB2P[12 + i17];
  }

  st.site = &tb_emlrtRSI;
  sherpaTTIK(&st, b_TB2P, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
             kC->l8, kC->zeta, kC->r, jointLimits, qGoal);
  for (i17 = 0; i17 < 3; i17++) {
    c_TP2B[i17] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TP2B[i17] += TB2P[i17 + (absb << 2)] * nInitCartesianB[3 + absb];
    }

    b_TB2P[i17] = c_TP2B[i17];
    nInitJoint[i17] = qInit[i17];
  }

  sherpaTTIKVel(b_TB2P, qInit, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nInitJoint[3] = phiInit;
  nInitJoint[4] = 0.0;
  for (i17 = 0; i17 < 3; i17++) {
    nInitJoint[i17 + 5] = uB[i17];
    c_TB2P[i17] = 0.0;
    for (absb = 0; absb < 3; absb++) {
      c_TB2P[i17] += TB2P[i17 + (absb << 2)] * nGoalCartesianB[3 + absb];
    }

    b_TB2P[i17] = c_TB2P[i17];
    nGoalJoint[i17] = qGoal[i17];
  }

  nInitJoint[8] = 0.0;
  nInitJoint[9] = omegaInit;
  sherpaTTIKVel(b_TB2P, qGoal, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta,
                uB);
  nGoalJoint[3] = 0.0;
  nGoalJoint[4] = 0.0;
  for (i17 = 0; i17 < 3; i17++) {
    nGoalJoint[i17 + 5] = uB[i17];
  }

  nGoalJoint[8] = 0.0;
  nGoalJoint[9] = 0.0;

  /* Check that the initial and final positions are valid. If they are not */
  /* return failure and an empty path. */
  emxInit_real_T(sp, &b_pathC, 2, &l_emlrtRTEI, true);
  emxInit_real_T(sp, &transitionPath, 2, &m_emlrtRTEI, true);
  b_emxInit_real_T(sp, &t, 1, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &path, 2, &o_emlrtRTEI, true);
  emxInit_real_T(sp, &y, 2, &k_emlrtRTEI, true);
  emxInit_real_T(sp, &b_transitionPath, 2, &k_emlrtRTEI, true);
  emxInit_real_T(sp, &c_transitionPath, 2, &k_emlrtRTEI, true);
  if (validJointState(nInitJoint, jointLimits) && validJointState(nGoalJoint,
       jointLimits)) {
    *success = true;

    /* Run buildRRT. */
    st.site = &ub_emlrtRSI;

    /* buildRRT.m */
    /* author: wreid */
    /* date: 20150107 */
    /* buildRRT Icrementally builds a rapidly exploring random tree. */
    /*    An RRT is build by incrementally selecting a random state from the */
    /*    available state space as defined by the MIN and MAX vectors. The tree is */
    /*    started at xInit and is extended until the number of maximum nodes, K has */
    /*    been reached. A path is selected if the goal region as defined by xGoal */
    /*    has been reached by the RRT. */
    /* Constant Declaration                                                       */
    /* Variable Initialization */
    memset(&SD->u1.f1.T[0], 0, 139500U * sizeof(real_T));

    /* Define a zero array that will be used to  */
    /* store data from each tree node. */
    SD->u1.f1.T[0] = 1.0;
    SD->u1.f1.T[1500] = 0.0;
    SD->u1.f1.T[3000] = 0.0;
    for (i17 = 0; i17 < 10; i17++) {
      SD->u1.f1.T[1500 * (i17 + 3)] = nInitJoint[i17];
    }

    for (i17 = 0; i17 < 80; i17++) {
      SD->u1.f1.T[1500 * (i17 + 13)] = 0.0;
    }

    /* Initialize the tree with initial state. */
    nodeIDCount = 1.0;
    for (cdiff = 0; cdiff < 1499; cdiff++) {
      b_st.site = &bc_emlrtRSI;
      c_st.site = &cc_emlrtRSI;
      randomState(&c_st, jointLimits, panHeight, kC->l1, kC->l2, kC->l3, kC->l4,
                  kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r, xRand);
      c_st.site = &dc_emlrtRSI;
      nearestNeighbour(&c_st, xRand, SD->u1.f1.T, kC->l1, kC->l2, kC->l3, kC->l4,
                       kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r,
                       nodeIDCount, xNear, transitionArrayNearest, &check);
      c_st.site = &ec_emlrtRSI;
      selectInput(&c_st, xNear, xRand, kC, 0.087266462599716474, jointLimits,
                  uBDot, legNum, xNew, transitionArrayNearest);
      xNew[0] = nodeIDCount + 1.0;

      /* Node ID */
      xNew[1] = xNear[0];

      /* Parent ID */
      xNew[2] = xNear[2] + b_heuristicSingleLeg(xNew, xNear, kC->l1, kC->l2,
        kC->l3, kC->l4, kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r);

      /* Cost */
      i17 = (int32_T)(nodeIDCount + 1.0);
      emlrtDynamicBoundsCheckFastR2012b(i17, 1, 1500, &s_emlrtBCI, &b_st);
      for (i17 = 0; i17 < 13; i17++) {
        SD->u1.f1.T[((int32_T)(nodeIDCount + 1.0) + 1500 * i17) - 1] = xNew[i17];
      }

      for (i17 = 0; i17 < 80; i17++) {
        SD->u1.f1.T[((int32_T)(nodeIDCount + 1.0) + 1500 * (i17 + 13)) - 1] =
          transitionArrayNearest[i17];
      }

      /* Append the new node to the tree.     */
      /* if mod(nodeIDCount,100) == 0 */
      /* fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount); */
      /* end */
      nodeIDCount++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    /* Find the closest node in the tree to the goal node. */
    dv18[0] = 0.0;
    dv18[1] = 0.0;
    dv18[2] = 0.0;
    memcpy(&dv18[3], &nGoalJoint[0], 10U * sizeof(real_T));
    b_st.site = &ac_emlrtRSI;
    nearestNeighbour(&b_st, dv18, SD->u1.f1.T, kC->l1, kC->l2, kC->l3, kC->l4,
                     kC->l5, kC->l6, kC->l7, kC->l8, kC->zeta, kC->r,
                     nodeIDCount, xRand, transitionArrayNearest, &check);
    check = xRand[0];
    memcpy(&next_data[0], &xRand[0], 13U * sizeof(real_T));
    i17 = path->size[0] * path->size[1];
    path->size[0] = 0;
    path->size[1] = 10;
    emxEnsureCapacity(&st, (emxArray__common *)path, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
    transitionArray_size[0] = 1;
    transitionArray_size[1] = 80;
    memcpy(&transitionArray_data[0], &transitionArrayNearest[0], 80U * sizeof
           (real_T));

    /* Iterate over the tree until the initial state has been found. */
    while ((check != 0.0) && (next_data[1] != 0.0)) {
      i17 = transitionPath->size[0] * transitionPath->size[1];
      transitionPath->size[0] = 0;
      transitionPath->size[1] = 10;
      emxEnsureCapacity(&st, (emxArray__common *)transitionPath, i17, (int32_T)
                        sizeof(real_T), &k_emlrtRTEI);
      emlrtForLoopVectorCheckR2012b(1.0, 10.0, 80.0, mxDOUBLE_CLASS, 8,
        &v_emlrtRTEI, &st);
      for (cdiff = 0; cdiff < 8; cdiff++) {
        ndbl = cdiff * 10;
        if (!b3) {
          for (i17 = 0; i17 < 2; i17++) {
            iv12[i17] = 1 + 9 * i17;
          }

          b3 = true;
        }

        emlrtMatrixMatrixIndexCheckR2012b(transitionArray_size, 2, iv12, 2,
          &h_emlrtECI, &st);
        i17 = c_transitionPath->size[0] * c_transitionPath->size[1];
        c_transitionPath->size[0] = transitionPath->size[0] + 1;
        c_transitionPath->size[1] = 10;
        emxEnsureCapacity(&st, (emxArray__common *)c_transitionPath, i17,
                          (int32_T)sizeof(real_T), &k_emlrtRTEI);
        for (i17 = 0; i17 < 10; i17++) {
          apnd = transitionPath->size[0];
          for (absb = 0; absb < apnd; absb++) {
            c_transitionPath->data[absb + c_transitionPath->size[0] * i17] =
              transitionPath->data[absb + transitionPath->size[0] * i17];
          }
        }

        for (i17 = 0; i17 < 10; i17++) {
          c_transitionPath->data[transitionPath->size[0] +
            c_transitionPath->size[0] * i17] = transitionArray_data[i17 + ndbl];
        }

        i17 = transitionPath->size[0] * transitionPath->size[1];
        transitionPath->size[0] = c_transitionPath->size[0];
        transitionPath->size[1] = 10;
        emxEnsureCapacity(&st, (emxArray__common *)transitionPath, i17, (int32_T)
                          sizeof(real_T), &k_emlrtRTEI);
        for (i17 = 0; i17 < 10; i17++) {
          apnd = c_transitionPath->size[0];
          for (absb = 0; absb < apnd; absb++) {
            transitionPath->data[absb + transitionPath->size[0] * i17] =
              c_transitionPath->data[absb + c_transitionPath->size[0] * i17];
          }
        }

        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
      }

      i17 = b_transitionPath->size[0] * b_transitionPath->size[1];
      b_transitionPath->size[0] = transitionPath->size[0] + path->size[0];
      b_transitionPath->size[1] = 10;
      emxEnsureCapacity(&st, (emxArray__common *)b_transitionPath, i17, (int32_T)
                        sizeof(real_T), &k_emlrtRTEI);
      for (i17 = 0; i17 < 10; i17++) {
        apnd = transitionPath->size[0];
        for (absb = 0; absb < apnd; absb++) {
          b_transitionPath->data[absb + b_transitionPath->size[0] * i17] =
            transitionPath->data[absb + transitionPath->size[0] * i17];
        }
      }

      for (i17 = 0; i17 < 10; i17++) {
        apnd = path->size[0];
        for (absb = 0; absb < apnd; absb++) {
          b_transitionPath->data[(absb + transitionPath->size[0]) +
            b_transitionPath->size[0] * i17] = path->data[absb + path->size[0] *
            i17];
        }
      }

      i17 = path->size[0] * path->size[1];
      path->size[0] = b_transitionPath->size[0];
      path->size[1] = 10;
      emxEnsureCapacity(&st, (emxArray__common *)path, i17, (int32_T)sizeof
                        (real_T), &k_emlrtRTEI);
      for (i17 = 0; i17 < 10; i17++) {
        apnd = b_transitionPath->size[0];
        for (absb = 0; absb < apnd; absb++) {
          path->data[absb + path->size[0] * i17] = b_transitionPath->data[absb +
            b_transitionPath->size[0] * i17];
        }
      }

      check = next_data[1];
      i17 = (int32_T)emlrtIntegerCheckFastR2012b(next_data[1], &c_emlrtDCI, &st);
      emlrtDynamicBoundsCheckFastR2012b(i17, 1, 1500, &r_emlrtBCI, &st);
      next_size[0] = 1;
      next_size[1] = 93;
      for (i17 = 0; i17 < 93; i17++) {
        next_data[i17] = SD->u1.f1.T[((int32_T)check + 1500 * i17) - 1];
      }

      check = next_data[1];
      tmp_size[0] = 1;
      tmp_size[1] = 80;
      for (i17 = 0; i17 < 80; i17++) {
        tmp_data[i17] = (int8_T)(14 + i17);
      }

      emlrtMatrixMatrixIndexCheckR2012b(next_size, 2, tmp_size, 2, &g_emlrtECI,
        &st);
      transitionArray_size[0] = 1;
      transitionArray_size[1] = 80;
      for (i17 = 0; i17 < 80; i17++) {
        transitionArray_data[i17] = next_data[tmp_data[i17] - 1];
      }

      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    b_st.site = &yb_emlrtRSI;
    c_st.site = &ob_emlrtRSI;
    d_st.site = &pb_emlrtRSI;
    if (path->size[0] < 1) {
      absb = -1;
      apnd = 0;
    } else {
      ndbl = (int32_T)muDoubleScalarFloor(((real_T)path->size[0] - 1.0) + 0.5);
      apnd = ndbl + 1;
      cdiff = (ndbl - path->size[0]) + 1;
      absb = path->size[0];
      if (muDoubleScalarAbs(cdiff) < 4.4408920985006262E-16 * (real_T)absb) {
        ndbl++;
        apnd = path->size[0];
      } else if (cdiff > 0) {
        apnd = ndbl;
      } else {
        ndbl++;
      }

      absb = ndbl - 1;
    }

    i17 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = absb + 1;
    emxEnsureCapacity(&d_st, (emxArray__common *)y, i17, (int32_T)sizeof(real_T),
                      &c_emlrtRTEI);
    if (absb + 1 > 0) {
      y->data[0] = 1.0;
      if (absb + 1 > 1) {
        y->data[absb] = apnd;
        i17 = absb + (absb < 0);
        if (i17 >= 0) {
          ndbl = (int32_T)((uint32_T)i17 >> 1);
        } else {
          ndbl = (int32_T)~(~(uint32_T)i17 >> 1);
        }

        for (cdiff = 1; cdiff < ndbl; cdiff++) {
          y->data[cdiff] = 1.0 + (real_T)cdiff;
          y->data[absb - cdiff] = apnd - cdiff;
        }

        if (ndbl << 1 == absb) {
          y->data[ndbl] = (1.0 + (real_T)apnd) / 2.0;
        } else {
          y->data[ndbl] = 1.0 + (real_T)ndbl;
          y->data[ndbl + 1] = apnd - ndbl;
        }
      }
    }

    i17 = t->size[0];
    t->size[0] = y->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)t, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
    apnd = y->size[1];
    for (i17 = 0; i17 < apnd; i17++) {
      t->data[i17] = 0.1 * y->data[y->size[0] * i17];
    }

    ndbl = t->size[0];
    i17 = path->size[0];
    emlrtDimSizeEqCheckFastR2012b(ndbl, i17, &f_emlrtECI, &st);
    ndbl = t->size[0];
    i17 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = ndbl;
    pathJ->size[1] = 11;
    emxEnsureCapacity(&st, (emxArray__common *)pathJ, i17, (int32_T)sizeof
                      (real_T), &k_emlrtRTEI);
    for (i17 = 0; i17 < ndbl; i17++) {
      pathJ->data[i17] = t->data[i17];
    }

    for (i17 = 0; i17 < 10; i17++) {
      apnd = path->size[0];
      for (absb = 0; absb < apnd; absb++) {
        pathJ->data[absb + pathJ->size[0] * (i17 + 1)] = path->data[absb +
          path->size[0] * i17];
      }
    }

    i17 = T->size[0] * T->size[1];
    T->size[0] = 1500;
    T->size[1] = 93;
    emxEnsureCapacity(sp, (emxArray__common *)T, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
    for (i17 = 0; i17 < 139500; i17++) {
      T->data[i17] = SD->u1.f1.T[i17];
    }

    /* Transform path back to the Cartesian space. */
    st.site = &vb_emlrtRSI;
    ndbl = pathJ->size[0];
    i17 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = ndbl;
    pathC->size[1] = 9;
    emxEnsureCapacity(&st, (emxArray__common *)pathC, i17, (int32_T)sizeof
                      (real_T), &k_emlrtRTEI);
    apnd = pathJ->size[0] * 9;
    for (i17 = 0; i17 < apnd; i17++) {
      pathC->data[i17] = 0.0;
    }

    check = 0.0;
    cdiff = 0;
    while (cdiff <= pathJ->size[0] - 1) {
      i17 = pathJ->size[0];
      absb = cdiff + 1;
      emlrtDynamicBoundsCheckFastR2012b(absb, 1, i17, &t_emlrtBCI, &st);

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
      newState[0] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[cdiff +
                         (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos
                       (kC->zeta)) + kC->l5 * muDoubleScalarCos(pathJ->
        data[cdiff + pathJ->size[0] * 3] + kC->zeta)) - kC->l7) *
        muDoubleScalarCos(pathJ->data[cdiff + pathJ->size[0]]);
      newState[1] = ((((kC->l2 + kC->l3 * muDoubleScalarCos(-pathJ->data[cdiff +
                         (pathJ->size[0] << 1)])) + kC->l4 * muDoubleScalarCos
                       (kC->zeta)) + kC->l5 * muDoubleScalarCos(pathJ->
        data[cdiff + pathJ->size[0] * 3] + kC->zeta)) - kC->l7) *
        muDoubleScalarSin(pathJ->data[cdiff + pathJ->size[0]]);
      newState[2] = ((((kC->l1 + kC->l3 * muDoubleScalarSin(-pathJ->data[cdiff +
                         (pathJ->size[0] << 1)])) - kC->l4 * muDoubleScalarSin
                       (kC->zeta)) - kC->l5 * muDoubleScalarSin(pathJ->
        data[cdiff + pathJ->size[0] * 3] + kC->zeta)) - kC->l6) - (kC->l8 +
        kC->r);
      i17 = pathJ->size[0];
      absb = cdiff + 1;
      emlrtDynamicBoundsCheckFastR2012b(absb, 1, i17, &u_emlrtBCI, &st);
      i17 = pathJ->size[0];
      absb = cdiff + 1;
      emlrtDynamicBoundsCheckFastR2012b(absb, 1, i17, &v_emlrtBCI, &st);

      /* sherpaTTFKVel.m */
      /* author: wreid */
      /* date: 20150122 */
      /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
      for (i17 = 0; i17 < 3; i17++) {
        nodeIDCount = 0.0;
        for (absb = 0; absb < 3; absb++) {
          nodeIDCount += TP2B[i17 + (absb << 2)] * newState[absb];
        }

        uB[i17] = nodeIDCount + TP2B[12 + i17];
      }

      if (1 + cdiff != 1) {
        i17 = pathC->size[0];
        ndbl = emlrtDynamicBoundsCheckFastR2012b(cdiff, 1, i17, &w_emlrtBCI, &st);
        for (i17 = 0; i17 < 3; i17++) {
          c_TP2B[i17] = uB[i17] - pathC->data[(ndbl + pathC->size[0] * (2 + i17))
            - 1];
        }

        check += norm(c_TP2B);
      }

      ndbl = pathC->size[0];
      i17 = 1 + cdiff;
      emlrtDynamicBoundsCheckFastR2012b(i17, 1, ndbl, &x_emlrtBCI, &st);
      b_pathJ[0] = (-pathJ->data[cdiff + pathJ->size[0] * 6] * muDoubleScalarSin
                    (pathJ->data[cdiff + pathJ->size[0]]) * ((((kC->l2 - kC->l7)
        + kC->l5 * muDoubleScalarCos(pathJ->data[cdiff + pathJ->size[0] * 3] +
        kC->zeta)) + kC->l3 * muDoubleScalarCos(pathJ->data[cdiff + (pathJ->
        size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - pathJ->
                    data[cdiff + pathJ->size[0] * 7] * kC->l3 *
                    muDoubleScalarCos(pathJ->data[cdiff + pathJ->size[0]]) *
                    muDoubleScalarSin(pathJ->data[cdiff + (pathJ->size[0] << 1)]))
        - pathJ->data[cdiff + (pathJ->size[0] << 3)] * kC->l5 *
        muDoubleScalarSin(pathJ->data[cdiff + pathJ->size[0] * 3] + kC->zeta) *
        muDoubleScalarCos(pathJ->data[cdiff + pathJ->size[0]]);
      b_pathJ[1] = (pathJ->data[cdiff + pathJ->size[0] * 6] * muDoubleScalarCos
                    (pathJ->data[cdiff + pathJ->size[0]]) * ((((kC->l2 - kC->l7)
        + kC->l5 * muDoubleScalarCos(pathJ->data[cdiff + pathJ->size[0] * 3] +
        kC->zeta)) + kC->l3 * muDoubleScalarCos(pathJ->data[cdiff + (pathJ->
        size[0] << 1)])) + kC->l4 * muDoubleScalarCos(kC->zeta)) - pathJ->
                    data[cdiff + (pathJ->size[0] << 3)] * kC->l5 *
                    muDoubleScalarSin(pathJ->data[cdiff + pathJ->size[0] * 3] +
        kC->zeta) * muDoubleScalarSin(pathJ->data[cdiff + pathJ->size[0]])) -
        pathJ->data[cdiff + pathJ->size[0] * 7] * kC->l3 * muDoubleScalarSin
        (pathJ->data[cdiff + pathJ->size[0]]) * muDoubleScalarSin(pathJ->
        data[cdiff + (pathJ->size[0] << 1)]);
      b_pathJ[2] = -pathJ->data[cdiff + pathJ->size[0] * 7] * kC->l3 *
        muDoubleScalarCos(pathJ->data[cdiff + (pathJ->size[0] << 1)]) - kC->l5 *
        pathJ->data[cdiff + (pathJ->size[0] << 3)] * muDoubleScalarCos(kC->zeta
        + pathJ->data[cdiff + pathJ->size[0] * 3]);
      for (i17 = 0; i17 < 3; i17++) {
        c_TP2B[i17] = 0.0;
        for (absb = 0; absb < 3; absb++) {
          c_TP2B[i17] += TP2B[i17 + (absb << 2)] * b_pathJ[absb];
        }
      }

      i17 = pathJ->size[0];
      absb = 1 + cdiff;
      pathC->data[cdiff] = pathJ->data[emlrtDynamicBoundsCheckFastR2012b(absb, 1,
        i17, &bb_emlrtBCI, &st) - 1];
      pathC->data[cdiff + pathC->size[0]] = check;
      for (i17 = 0; i17 < 3; i17++) {
        pathC->data[cdiff + pathC->size[0] * (i17 + 2)] = uB[i17];
      }

      for (i17 = 0; i17 < 3; i17++) {
        pathC->data[cdiff + pathC->size[0] * (i17 + 5)] = c_TP2B[i17];
      }

      pathC->data[cdiff + (pathC->size[0] << 3)] = 0.0;
      cdiff++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    i17 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = pathC->size[0];
    b_pathC->size[1] = 9;
    emxEnsureCapacity(sp, (emxArray__common *)b_pathC, i17, (int32_T)sizeof
                      (real_T), &k_emlrtRTEI);
    apnd = pathC->size[0] * pathC->size[1];
    for (i17 = 0; i17 < apnd; i17++) {
      b_pathC->data[i17] = pathC->data[i17];
    }
  } else {
    *success = false;
    i17 = b_pathC->size[0] * b_pathC->size[1];
    b_pathC->size[0] = 0;
    b_pathC->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)b_pathC, i17, (int32_T)sizeof
                      (real_T), &k_emlrtRTEI);
    i17 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 0;
    pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
    i17 = T->size[0] * T->size[1];
    T->size[0] = 0;
    T->size[1] = 0;
    emxEnsureCapacity(sp, (emxArray__common *)T, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
  }

  emxFree_real_T(&c_transitionPath);
  emxFree_real_T(&b_transitionPath);
  emxFree_real_T(&y);
  emxFree_real_T(&path);
  emxFree_real_T(&t);
  emxFree_real_T(&transitionPath);

  /* Linearly interpolate to the goal state from the final state. */
  if ((b_pathC->size[0] == 0) || (b_pathC->size[1] == 0)) {
    b_TP2B[0] = 0.0;
    b_TP2B[1] = 0.0;
    for (i17 = 0; i17 < 6; i17++) {
      b_TP2B[i17 + 2] = nInitCartesianB[i17];
    }

    b_TP2B[8] = 1.0;
    apnd = 9;
    for (i17 = 0; i17 < 9; i17++) {
      sFinalC_data[i17] = b_TP2B[i17];
    }
  } else {
    apnd = b_pathC->size[1];
    i17 = b_pathC->size[0];
    absb = b_pathC->size[0];
    ndbl = emlrtDynamicBoundsCheckFastR2012b(absb, 1, i17, &q_emlrtBCI, sp);
    for (i17 = 0; i17 < apnd; i17++) {
      sFinalC_data[i17] = b_pathC->data[(ndbl + b_pathC->size[0] * i17) - 1];
    }
  }

  sGoalC[0] = 0.0;
  sGoalC[1] = 0.0;
  for (i17 = 0; i17 < 6; i17++) {
    sGoalC[i17 + 2] = nGoalCartesianB[i17];
  }

  sGoalC[8] = 1.0;
  st.site = &wb_emlrtRSI;

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
  emlrtDynamicBoundsCheckFastR2012b(2, 1, apnd, &y_emlrtBCI, &st);
  check = sFinalC_data[1];
  for (i17 = 0; i17 < 3; i17++) {
    absb = 3 + i17;
    qInit[i17] = sFinalC_data[emlrtDynamicBoundsCheckFastR2012b(absb, 1, apnd,
      &ab_emlrtBCI, &st) - 1];
  }

  for (i17 = 0; i17 < 3; i17++) {
    qGoal[i17] = qInit[i17];
  }

  for (cdiff = 0; cdiff < 10; cdiff++) {
    nodeIDCount = (1.0 + (real_T)cdiff) / 10.0;
    for (i17 = 0; i17 < 3; i17++) {
      panHeight = qInit[i17] + nodeIDCount * (sGoalC[2 + i17] - qInit[i17]);
      c_TP2B[i17] = panHeight - qGoal[i17];
      newState[i17] = panHeight;
    }

    check += norm(c_TP2B);
    pathCorrection[cdiff] = sFinalC_data[0] + (1.0 + (real_T)cdiff) / 10.0;
    pathCorrection[10 + cdiff] = check;
    for (i17 = 0; i17 < 3; i17++) {
      pathCorrection[cdiff + 10 * (i17 + 2)] = newState[i17];
      qGoal[i17] = newState[i17];
    }

    pathCorrection[50 + cdiff] = 0.0;
    pathCorrection[60 + cdiff] = 0.0;
    pathCorrection[70 + cdiff] = 0.0;
    pathCorrection[80 + cdiff] = 1.0;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  i17 = b_pathC->size[1];
  emlrtDimSizeEqCheckFastR2012b(9, i17, &e_emlrtECI, sp);
  i17 = pathC->size[0] * pathC->size[1];
  pathC->size[0] = b_pathC->size[0] + 10;
  pathC->size[1] = b_pathC->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)pathC, i17, (int32_T)sizeof(real_T),
                    &k_emlrtRTEI);
  apnd = b_pathC->size[1];
  for (i17 = 0; i17 < apnd; i17++) {
    ndbl = b_pathC->size[0];
    for (absb = 0; absb < ndbl; absb++) {
      pathC->data[absb + pathC->size[0] * i17] = b_pathC->data[absb +
        b_pathC->size[0] * i17];
    }
  }

  for (i17 = 0; i17 < 9; i17++) {
    for (absb = 0; absb < 10; absb++) {
      pathC->data[(absb + b_pathC->size[0]) + pathC->size[0] * i17] =
        pathCorrection[absb + 10 * i17];
    }
  }

  emxFree_real_T(&b_pathC);
  emxInit_real_T(sp, &c_pathJ, 2, &k_emlrtRTEI, true);
  for (cdiff = 0; cdiff < 10; cdiff++) {
    for (i17 = 0; i17 < 3; i17++) {
      nodeIDCount = 0.0;
      for (absb = 0; absb < 3; absb++) {
        nodeIDCount += TB2P[i17 + (absb << 2)] * pathCorrection[cdiff + 10 * (2
          + absb)];
      }

      b_TB2P[i17] = nodeIDCount + TB2P[12 + i17];
    }

    for (i17 = 0; i17 < 3; i17++) {
      c_TP2B[i17] = b_TB2P[i17];
    }

    st.site = &xb_emlrtRSI;
    b_sherpaTTIK(&st, c_TP2B, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6,
                 kC->l7, kC->l8, kC->zeta, kC->r, jointLimits, qInit);
    i17 = c_pathJ->size[0] * c_pathJ->size[1];
    c_pathJ->size[0] = pathJ->size[0] + 1;
    c_pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)c_pathJ, i17, (int32_T)sizeof
                      (real_T), &k_emlrtRTEI);
    for (i17 = 0; i17 < 11; i17++) {
      apnd = pathJ->size[0];
      for (absb = 0; absb < apnd; absb++) {
        c_pathJ->data[absb + c_pathJ->size[0] * i17] = pathJ->data[absb +
          pathJ->size[0] * i17];
      }
    }

    c_pathJ->data[pathJ->size[0]] = pathCorrection[cdiff];
    for (i17 = 0; i17 < 3; i17++) {
      c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * (i17 + 1)] = qInit[i17];
    }

    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 2)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 5] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 6] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 7] = 0.0;
    c_pathJ->data[pathJ->size[0] + (c_pathJ->size[0] << 3)] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 9] = 0.0;
    c_pathJ->data[pathJ->size[0] + c_pathJ->size[0] * 10] = 0.0;
    i17 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = c_pathJ->size[0];
    pathJ->size[1] = 11;
    emxEnsureCapacity(sp, (emxArray__common *)pathJ, i17, (int32_T)sizeof(real_T),
                      &k_emlrtRTEI);
    for (i17 = 0; i17 < 11; i17++) {
      apnd = c_pathJ->size[0];
      for (absb = 0; absb < apnd; absb++) {
        pathJ->data[absb + pathJ->size[0] * i17] = c_pathJ->data[absb +
          c_pathJ->size[0] * i17];
      }
    }

    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_real_T(&c_pathJ);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (buildRRTWrapper.c) */
