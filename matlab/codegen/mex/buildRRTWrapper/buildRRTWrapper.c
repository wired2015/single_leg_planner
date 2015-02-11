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
#include "extractKinematicConstants.h"
#include "nearestNeighbour.h"
#include "sherpaTTIKVel.h"
#include "sherpaTTIK.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static real_T HGAINS[3];
static emlrtRSInfo emlrtRSI = { 51, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m"
};

static emlrtRSInfo b_emlrtRSI = { 52, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m"
};

static emlrtRSInfo c_emlrtRSI = { 68, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m"
};

static emlrtRSInfo d_emlrtRSI = { 70, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m"
};

static emlrtRSInfo g_emlrtRSI = { 44, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRSInfo h_emlrtRSI = { 27, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtRTEInfo emlrtRTEI = { 6, 36, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m"
};

static emlrtRTEInfo b_emlrtRTEI = { 47, 5, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo emlrtBCI = { -1, -1, 113, 18, "path", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo b_emlrtBCI = { -1, -1, 109, 77, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo c_emlrtBCI = { -1, -1, 109, 75, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo d_emlrtBCI = { -1, -1, 109, 62, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo e_emlrtBCI = { -1, -1, 109, 60, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo f_emlrtBCI = { -1, -1, 109, 47, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo g_emlrtBCI = { -1, -1, 109, 45, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtECInfo emlrtECI = { -1, 20, 5, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo h_emlrtBCI = { -1, -1, 49, 25, "next", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtBCInfo i_emlrtBCI = { -1, -1, 50, 18, "next", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo emlrtDCI = { 50, 18, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtBCInfo j_emlrtBCI = { -1, -1, 50, 18, "T", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtECInfo b_emlrtECI = { 2, 51, 16, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m"
};

static emlrtBCInfo k_emlrtBCI = { -1, -1, 52, 17, "next", "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  0 };

static emlrtDCInfo b_emlrtDCI = { 18, 25, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo c_emlrtDCI = { 20, 29, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  1 };

static emlrtDCInfo d_emlrtDCI = { 20, 29, "buildRRT",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/buildRRT.m",
  4 };

static emlrtDCInfo e_emlrtDCI = { 104, 18, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  1 };

static emlrtDCInfo f_emlrtDCI = { 104, 18, "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  4 };

static emlrtBCInfo l_emlrtBCI = { -1, -1, 111, 43, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo m_emlrtBCI = { -1, -1, 111, 45, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo n_emlrtBCI = { -1, -1, 111, 58, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo o_emlrtBCI = { -1, -1, 111, 60, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo p_emlrtBCI = { -1, -1, 111, 73, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo q_emlrtBCI = { -1, -1, 111, 75, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo r_emlrtBCI = { -1, -1, 111, 89, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo s_emlrtBCI = { -1, -1, 111, 91, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo t_emlrtBCI = { -1, -1, 111, 104, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo u_emlrtBCI = { -1, -1, 111, 106, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo v_emlrtBCI = { -1, -1, 111, 119, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

static emlrtBCInfo w_emlrtBCI = { -1, -1, 111, 121, "pathOld", "buildRRTWrapper",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/buildRRTWrapper.m",
  0 };

/* Function Declarations */
static boolean_T validState(const real_T n[6], const real_T jointLimits[12]);

/* Function Definitions */
static boolean_T validState(const real_T n[6], const real_T jointLimits[12])
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
                     const real_T nGoalCartesianB[6], const real_T jointLimits
                     [12], real_T bodyHeight, const real_T U[10], real_T dt,
                     real_T Dt, const real_T kinematicConst[16], real_T
                     threshold, int32_T legNum, emxArray_real_T *T,
                     emxArray_real_T *pathC, emxArray_real_T *pathJ, boolean_T
                     *success)
{
  real_T unusedUe;
  real_T legAngleOffset;
  real_T unusedUc;
  real_T unusedUb;
  real_T B2PZOffset;
  real_T transitionArrayLength;
  real_T unusedU9;
  real_T unusedU8;
  real_T unusedU7;
  real_T unusedU6;
  real_T unusedU5;
  real_T unusedU4;
  real_T unusedU3;
  real_T unusedU2;
  real_T unusedU1;
  real_T unusedU0;
  real_T panHeight;
  real_T leg1AngleOffset;
  real_T TP2B[16];
  int32_T i0;
  static const int8_T iv0[4] = { 0, 0, 0, 1 };

  real_T b_TP2B[9];
  int32_T path_idx_0;
  real_T c_TP2B[3];
  real_T TB2P[16];
  real_T uInitP[3];
  real_T uGoalP[3];
  real_T alphaInit[3];
  real_T b_TB2P[3];
  real_T c_TB2P[3];
  real_T alphaGoal[3];
  real_T nInitJoint[6];
  real_T nGoalJoint[6];
  emxArray_real_T *b_T;
  emxArray_real_T *path;
  emxArray_real_T *next;
  emxArray_real_T *transitionArrayNearest;
  emxArray_int32_T *r0;
  emxArray_real_T *x;
  emxArray_real_T *b_pathJ;
  real_T nGoal[11];
  int32_T md2;
  int32_T iv1[2];
  int32_T i;
  real_T b_unusedU2;
  int32_T xNearest_size[2];
  real_T xNearest_data[11];
  boolean_T exitg1;
  int32_T b_path_idx_0;
  uint32_T count;
  real_T time;
  int32_T j;
  real_T c_pathJ[3];
  real_T d_pathJ[3];
  real_T zeta;
  real_T L7;
  real_T L5;
  real_T L4;
  real_T L3;
  real_T L2;
  real_T b_L2[3];
  real_T b_uInitP[3];
  int32_T m;
  emlrtStack st;
  emlrtStack b_st;
  (void)threshold;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /* buildRRTWrapper.m */
  /* author: wreid */
  /* date: 20150502 */
  /* GETPANHEIGHT Summary of this function goes here */
  /*    Detailed explanation goes here */
  extractKinematicConstants(kinematicConst, &unusedU0, &unusedU1, &unusedU2,
    &unusedU3, &unusedU4, &unusedU5, &unusedU6, &unusedU7, &unusedU8, &unusedU9,
    &transitionArrayLength, &B2PZOffset, &unusedUb, &unusedUc, &legAngleOffset,
    &unusedUe);
  panHeight = -(bodyHeight + B2PZOffset);

  /* Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame */
  /* to the pan coordinate frame. */
  /* TRP2B Calculates the homogeneous transformation matrix between the body */
  /* and pan coordinate frames. */
  extractKinematicConstants(kinematicConst, &unusedU0, &unusedU1, &unusedU2,
    &unusedU3, &unusedU4, &unusedU5, &unusedU6, &unusedU7, &unusedU8, &unusedU9,
    &transitionArrayLength, &B2PZOffset, &leg1AngleOffset, &unusedUb, &unusedUc,
    &legAngleOffset);
  switch (legNum) {
   case 1:
    legAngleOffset = leg1AngleOffset;
    break;

   case 2:
    legAngleOffset = unusedUb;
    break;

   case 3:
    legAngleOffset = unusedUc;
    break;
  }

  TP2B[0] = muDoubleScalarCos(legAngleOffset);
  TP2B[4] = -muDoubleScalarSin(legAngleOffset);
  TP2B[8] = muDoubleScalarSin(legAngleOffset) * 0.0;
  TP2B[12] = transitionArrayLength * muDoubleScalarCos(legAngleOffset);
  TP2B[1] = muDoubleScalarSin(legAngleOffset);
  TP2B[5] = muDoubleScalarCos(legAngleOffset);
  TP2B[9] = -muDoubleScalarCos(legAngleOffset) * 0.0;
  TP2B[13] = transitionArrayLength * muDoubleScalarSin(legAngleOffset);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = B2PZOffset;
  for (i0 = 0; i0 < 4; i0++) {
    TP2B[3 + (i0 << 2)] = iv0[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      b_TP2B[path_idx_0 + 3 * i0] = -TP2B[i0 + (path_idx_0 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    c_TP2B[i0] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      c_TP2B[i0] += b_TP2B[i0 + 3 * path_idx_0] * TP2B[12 + path_idx_0];
    }

    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      TB2P[path_idx_0 + (i0 << 2)] = TP2B[i0 + (path_idx_0 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    TB2P[12 + i0] = c_TP2B[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    TB2P[3 + (i0 << 2)] = iv0[i0];
  }

  /* inv(TP2B);% */
  for (i0 = 0; i0 < 3; i0++) {
    unusedUb = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      unusedUb += TB2P[i0 + (path_idx_0 << 2)] * nInitCartesianB[path_idx_0];
    }

    uInitP[i0] = unusedUb + TB2P[12 + i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    unusedUb = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      unusedUb += TB2P[i0 + (path_idx_0 << 2)] * nGoalCartesianB[path_idx_0];
    }

    uGoalP[i0] = unusedUb + TB2P[12 + i0];
  }

  /* Transform the Cartesian goal and final positions in the pan coordinate */
  /* frame to the joint space. */
  st.site = &emlrtRSI;
  sherpaTTIK(&st, uInitP[0], uInitP[1], uInitP[2], kinematicConst, jointLimits,
             &transitionArrayLength, &unusedUb, &unusedUc);
  st.site = &b_emlrtRSI;
  sherpaTTIK(&st, uGoalP[0], uGoalP[1], uGoalP[2], kinematicConst, jointLimits,
             &legAngleOffset, &unusedUe, &leg1AngleOffset);
  alphaInit[0] = transitionArrayLength;
  alphaInit[1] = unusedUb;
  alphaInit[2] = unusedUc;
  for (i0 = 0; i0 < 3; i0++) {
    uGoalP[i0] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      uGoalP[i0] += TB2P[i0 + (path_idx_0 << 2)] * nInitCartesianB[3 +
        path_idx_0];
    }

    c_TP2B[i0] = uGoalP[i0];
    c_TB2P[i0] = 0.0;
    for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
      c_TB2P[i0] += TB2P[i0 + (path_idx_0 << 2)] * nGoalCartesianB[3 +
        path_idx_0];
    }

    b_TB2P[i0] = c_TB2P[i0];
  }

  sherpaTTIKVel(c_TP2B, alphaInit, kinematicConst, uInitP);
  alphaGoal[0] = legAngleOffset;
  alphaGoal[1] = unusedUe;
  alphaGoal[2] = leg1AngleOffset;
  sherpaTTIKVel(b_TB2P, alphaGoal, kinematicConst, uGoalP);
  nInitJoint[0] = transitionArrayLength;
  nInitJoint[1] = unusedUb;
  nInitJoint[2] = unusedUc;
  nInitJoint[3] = uInitP[0];
  nInitJoint[4] = uInitP[1];
  nInitJoint[5] = uInitP[2];
  nGoalJoint[0] = legAngleOffset;
  nGoalJoint[1] = unusedUe;
  nGoalJoint[2] = leg1AngleOffset;
  nGoalJoint[3] = uGoalP[0];
  nGoalJoint[4] = uGoalP[1];
  nGoalJoint[5] = uGoalP[2];

  /* Check that the initial and final positions are valid. If they are not */
  /* return failure and an empty path. */
  emxInit_real_T(sp, &b_T, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &path, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &next, 2, &b_emlrtRTEI, true);
  emxInit_real_T(sp, &transitionArrayNearest, 2, &emlrtRTEI, true);
  emxInit_int32_T(sp, &r0, 1, &emlrtRTEI, true);
  emxInit_real_T(sp, &x, 2, &emlrtRTEI, true);
  emxInit_real_T(sp, &b_pathJ, 2, &emlrtRTEI, true);
  if (validState(nInitJoint, jointLimits) && validState(nGoalJoint, jointLimits))
  {
    *success = true;

    /* Run buildRRT. */
    nGoal[0] = 0.0;
    nGoal[1] = 0.0;
    nGoal[2] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      nGoal[i0 + 3] = nGoalJoint[i0];
    }

    nGoal[9] = 0.0;
    nGoal[10] = 0.0;
    st.site = &c_emlrtRSI;

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
    transitionArrayLength = (muDoubleScalarRound(Dt / dt) + 1.0) * 6.0;

    /* Variable Initialization */
    i0 = b_T->size[0] * b_T->size[1];
    b_T->size[0] = 1000;
    unusedUb = muDoubleScalarRound(11.0 + transitionArrayLength);
    if (unusedUb < 2.147483648E+9) {
      if (unusedUb >= -2.147483648E+9) {
        path_idx_0 = (int32_T)unusedUb;
      } else {
        path_idx_0 = MIN_int32_T;
      }
    } else if (unusedUb >= 2.147483648E+9) {
      path_idx_0 = MAX_int32_T;
    } else {
      path_idx_0 = 0;
    }

    b_T->size[1] = (int32_T)emlrtNonNegativeCheckFastR2012b(path_idx_0,
      &b_emlrtDCI, &st);
    emxEnsureCapacity(&st, (emxArray__common *)b_T, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    unusedUb = muDoubleScalarRound(11.0 + transitionArrayLength);
    if (unusedUb < 2.147483648E+9) {
      if (unusedUb >= -2.147483648E+9) {
        i0 = (int32_T)unusedUb;
      } else {
        i0 = MIN_int32_T;
      }
    } else if (unusedUb >= 2.147483648E+9) {
      i0 = MAX_int32_T;
    } else {
      i0 = 0;
    }

    md2 = 1000 * (int32_T)emlrtNonNegativeCheckFastR2012b(i0, &b_emlrtDCI, &st);
    for (i0 = 0; i0 < md2; i0++) {
      b_T->data[i0] = 0.0;
    }

    /* Define a zero array that will be used to  */
    /* store data from each tree node. */
    unusedUb = muDoubleScalarRound(11.0 + transitionArrayLength);
    if (unusedUb < 2.147483648E+9) {
      if (unusedUb >= -2.147483648E+9) {
        md2 = (int32_T)unusedUb;
      } else {
        md2 = MIN_int32_T;
      }
    } else if (unusedUb >= 2.147483648E+9) {
      md2 = MAX_int32_T;
    } else {
      md2 = 0;
    }

    i0 = r0->size[0];
    unusedUb = muDoubleScalarRound(11.0 + transitionArrayLength);
    if (unusedUb < 2.147483648E+9) {
      if (unusedUb >= -2.147483648E+9) {
        path_idx_0 = (int32_T)unusedUb;
      } else {
        path_idx_0 = MIN_int32_T;
      }
    } else if (unusedUb >= 2.147483648E+9) {
      path_idx_0 = MAX_int32_T;
    } else {
      path_idx_0 = 0;
    }

    r0->size[0] = path_idx_0;
    emxEnsureCapacity(&st, (emxArray__common *)r0, i0, (int32_T)sizeof(int32_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < md2; i0++) {
      r0->data[i0] = i0;
    }

    unusedUb = emlrtNonNegativeCheckFastR2012b(transitionArrayLength,
      &d_emlrtDCI, &st);
    emlrtIntegerCheckFastR2012b(unusedUb, &c_emlrtDCI, &st);
    i0 = transitionArrayNearest->size[0] * transitionArrayNearest->size[1];
    transitionArrayNearest->size[0] = 1;
    transitionArrayNearest->size[1] = 11 + (int32_T)transitionArrayLength;
    emxEnsureCapacity(&st, (emxArray__common *)transitionArrayNearest, i0,
                      (int32_T)sizeof(real_T), &emlrtRTEI);
    transitionArrayNearest->data[0] = 0.0;
    transitionArrayNearest->data[transitionArrayNearest->size[0]] = 0.0;
    transitionArrayNearest->data[transitionArrayNearest->size[0] << 1] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      transitionArrayNearest->data[transitionArrayNearest->size[0] * (i0 + 3)] =
        nInitJoint[i0];
    }

    transitionArrayNearest->data[transitionArrayNearest->size[0] * 9] = 0.0;
    transitionArrayNearest->data[transitionArrayNearest->size[0] * 10] = 0.0;
    md2 = (int32_T)transitionArrayLength;
    for (i0 = 0; i0 < md2; i0++) {
      transitionArrayNearest->data[transitionArrayNearest->size[0] * (i0 + 11)] =
        0.0;
    }

    iv1[0] = 1;
    iv1[1] = r0->size[0];
    emlrtSubAssignSizeCheckR2012b(iv1, 2, *(int32_T (*)[2])
      transitionArrayNearest->size, 2, &emlrtECI, &st);
    md2 = transitionArrayNearest->size[1];
    for (i0 = 0; i0 < md2; i0++) {
      b_T->data[b_T->size[0] * r0->data[i0]] = transitionArrayNearest->
        data[transitionArrayNearest->size[0] * i0];
    }

    /* Initialize the tree with initial state. */
    transitionArrayLength = 1.0;
    for (i0 = 0; i0 < 6; i0++) {
      nInitJoint[i0] = jointLimits[1 + (i0 << 1)] - jointLimits[i0 << 1];
    }

    for (i = 0; i < 999; i++) {
      b_st.site = &h_emlrtRSI;
      rrtLoop(&b_st, b_T, nInitJoint, jointLimits, kinematicConst, panHeight, U,
              Dt, dt, &transitionArrayLength, nGoal);
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    /* Find the closest node in the tree to the goal node. */
    b_st.site = &g_emlrtRSI;
    nearestNeighbour(&b_st, nGoal, b_T, jointLimits, kinematicConst,
                     transitionArrayLength, 11, xNearest_data, xNearest_size,
                     transitionArrayNearest, &b_unusedU2);
    transitionArrayLength = xNearest_data[0];
    i0 = next->size[0] * next->size[1];
    next->size[0] = 1;
    next->size[1] = 11;
    emxEnsureCapacity(&st, (emxArray__common *)next, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 11; i0++) {
      next->data[i0] = xNearest_data[i0];
    }

    i0 = pathJ->size[0] * pathJ->size[1];
    pathJ->size[0] = 1;
    pathJ->size[1] = 11 + transitionArrayNearest->size[1];
    emxEnsureCapacity(&st, (emxArray__common *)pathJ, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 11; i0++) {
      pathJ->data[pathJ->size[0] * i0] = xNearest_data[xNearest_size[0] * i0];
    }

    md2 = transitionArrayNearest->size[1];
    for (i0 = 0; i0 < md2; i0++) {
      pathJ->data[pathJ->size[0] * (i0 + 11)] = transitionArrayNearest->
        data[transitionArrayNearest->size[0] * i0];
    }

    exitg1 = false;
    while ((!exitg1) && (transitionArrayLength != 0.0)) {
      i0 = next->size[1];
      emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &h_emlrtBCI, &st);
      if (next->data[1] != 0.0) {
        i0 = next->size[1];
        emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &i_emlrtBCI, &st);
        transitionArrayLength = next->data[1];
        md2 = b_T->size[1];
        i0 = (int32_T)emlrtIntegerCheckFastR2012b(transitionArrayLength,
          &emlrtDCI, &st);
        emlrtDynamicBoundsCheckFastR2012b(i0, 1, 1000, &j_emlrtBCI, &st);
        i0 = next->size[0] * next->size[1];
        next->size[0] = 1;
        next->size[1] = md2;
        emxEnsureCapacity(&st, (emxArray__common *)next, i0, (int32_T)sizeof
                          (real_T), &emlrtRTEI);
        for (i0 = 0; i0 < md2; i0++) {
          next->data[next->size[0] * i0] = b_T->data[((int32_T)
            transitionArrayLength + b_T->size[0] * i0) - 1];
        }

        i0 = pathJ->size[1];
        path_idx_0 = next->size[1];
        emlrtDimSizeEqCheckFastR2012b(i0, path_idx_0, &b_emlrtECI, &st);
        i0 = b_pathJ->size[0] * b_pathJ->size[1];
        b_pathJ->size[0] = pathJ->size[0] + 1;
        b_pathJ->size[1] = pathJ->size[1];
        emxEnsureCapacity(&st, (emxArray__common *)b_pathJ, i0, (int32_T)sizeof
                          (real_T), &emlrtRTEI);
        md2 = pathJ->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          b_path_idx_0 = pathJ->size[0];
          for (path_idx_0 = 0; path_idx_0 < b_path_idx_0; path_idx_0++) {
            b_pathJ->data[path_idx_0 + b_pathJ->size[0] * i0] = pathJ->
              data[path_idx_0 + pathJ->size[0] * i0];
          }
        }

        md2 = next->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          b_pathJ->data[pathJ->size[0] + b_pathJ->size[0] * i0] = next->
            data[next->size[0] * i0];
        }

        i0 = pathJ->size[0] * pathJ->size[1];
        pathJ->size[0] = b_pathJ->size[0];
        pathJ->size[1] = b_pathJ->size[1];
        emxEnsureCapacity(&st, (emxArray__common *)pathJ, i0, (int32_T)sizeof
                          (real_T), &emlrtRTEI);
        md2 = b_pathJ->size[1];
        for (i0 = 0; i0 < md2; i0++) {
          b_path_idx_0 = b_pathJ->size[0];
          for (path_idx_0 = 0; path_idx_0 < b_path_idx_0; path_idx_0++) {
            pathJ->data[path_idx_0 + pathJ->size[0] * i0] = b_pathJ->
              data[path_idx_0 + b_pathJ->size[0] * i0];
          }
        }

        i0 = next->size[1];
        emlrtDynamicBoundsCheckFastR2012b(2, 1, i0, &k_emlrtBCI, &st);
        transitionArrayLength = next->data[1];
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
      } else {
        exitg1 = true;
      }
    }

    i0 = T->size[0] * T->size[1];
    T->size[0] = 1000;
    T->size[1] = b_T->size[1];
    emxEnsureCapacity(sp, (emxArray__common *)T, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    md2 = b_T->size[0] * b_T->size[1];
    for (i0 = 0; i0 < md2; i0++) {
      T->data[i0] = b_T->data[i0];
    }

    /* Transform path back to the Cartesian space. */
    st.site = &d_emlrtRSI;

    /* Take the pathOld array and combine the general nodes and intermediate */
    /* states into a uniform path. The output path should be a npx6 array */
    /* that contains the n general nodes and the p intermediate nodes between */
    /* general nodes. Each row in the path matrix contains  */
    /* [t,x,y,z,xDot,yDot,zDot] state data. */
    transitionArrayLength = muDoubleScalarRound(Dt / dt);
    unusedUb = transitionArrayLength * (real_T)pathJ->size[0];
    unusedUb = emlrtNonNegativeCheckFastR2012b(unusedUb, &f_emlrtDCI, &st);
    md2 = (int32_T)emlrtIntegerCheckFastR2012b(unusedUb, &e_emlrtDCI, &st);
    i0 = path->size[0] * path->size[1];
    path->size[0] = md2;
    path->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)path, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    unusedUb = transitionArrayLength * (real_T)pathJ->size[0];
    unusedUb = emlrtNonNegativeCheckFastR2012b(unusedUb, &f_emlrtDCI, &st);
    md2 = (int32_T)emlrtIntegerCheckFastR2012b(unusedUb, &e_emlrtDCI, &st) * 7;
    for (i0 = 0; i0 < md2; i0++) {
      path->data[i0] = 0.0;
    }

    count = 1U;
    time = muDoubleScalarRound(Dt / dt) * (real_T)pathJ->size[0] * dt;
    i = 0;
    while (i <= pathJ->size[0] - 1) {
      j = pathJ->size[1] - 4;
      while (j + 4 >= 18) {
        i0 = pathJ->size[0];
        path_idx_0 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(path_idx_0, 1, i0, &g_emlrtBCI, &st);
        i0 = pathJ->size[1];
        path_idx_0 = j - 1;
        emlrtDynamicBoundsCheckFastR2012b(path_idx_0, 1, i0, &f_emlrtBCI, &st);
        i0 = pathJ->size[0];
        path_idx_0 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(path_idx_0, 1, i0, &e_emlrtBCI, &st);
        i0 = pathJ->size[1];
        emlrtDynamicBoundsCheckFastR2012b(j, 1, i0, &d_emlrtBCI, &st);
        i0 = pathJ->size[0];
        path_idx_0 = i + 1;
        emlrtDynamicBoundsCheckFastR2012b(path_idx_0, 1, i0, &c_emlrtBCI, &st);
        i0 = pathJ->size[1];
        emlrtDynamicBoundsCheckFastR2012b(j + 1, 1, i0, &b_emlrtBCI, &st);

        /* sherpaTTFK.m */
        /* author: wreid */
        /* date: 20150122 */
        /* sherpaTTFK Sherpa_TT Forward Kinematics */
        /*    Calculates the x,y,z position of the contact point given the alpha, */
        /*    beta and gamma joint values. */
        extractKinematicConstants(kinematicConst, &transitionArrayLength,
          &unusedUb, &unusedUc, &unusedUe, &leg1AngleOffset, &legAngleOffset,
          &B2PZOffset, &panHeight, &b_unusedU2, &unusedU0, &unusedU1, &unusedU2,
          &unusedU3, &unusedU4, &unusedU5, &unusedU6);
        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        c_pathJ[0] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &l_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(j + 2, 1, md2, &m_emlrtBCI, &st) -
           1)) - 1];
        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        c_pathJ[1] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &n_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(j + 3, 1, md2, &o_emlrtBCI, &st) -
           1)) - 1];
        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        c_pathJ[2] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &p_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(j + 4, 1, md2, &q_emlrtBCI, &st) -
           1)) - 1];
        for (i0 = 0; i0 < 3; i0++) {
          uInitP[i0] = c_pathJ[i0];
        }

        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        b_path_idx_0 = j - 1;
        d_pathJ[0] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &r_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(b_path_idx_0, 1, md2, &s_emlrtBCI,
          &st) - 1)) - 1];
        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        d_pathJ[1] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &t_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(j, 1, md2, &u_emlrtBCI, &st) - 1))
          - 1];
        i0 = pathJ->size[0];
        path_idx_0 = 1 + i;
        md2 = pathJ->size[1];
        d_pathJ[2] = pathJ->data[(emlrtDynamicBoundsCheckFastR2012b(path_idx_0,
          1, i0, &v_emlrtBCI, &st) + pathJ->size[0] *
          (emlrtDynamicBoundsCheckFastR2012b(j + 1, 1, md2, &w_emlrtBCI, &st) -
           1)) - 1];
        for (i0 = 0; i0 < 3; i0++) {
          uGoalP[i0] = d_pathJ[i0];
        }

        /* sherpaTTFKVel.m */
        /* author: wreid */
        /* date: 20150122 */
        /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
        extractKinematicConstants(kinematicConst, &unusedU0, &L2, &L3, &L4, &L5,
          &unusedU1, &L7, &unusedU2, &zeta, &unusedU3, &unusedU4, &unusedU5,
          &unusedU6, &unusedU7, &unusedU8, &unusedU9);
        md2 = path->size[0];
        i0 = (int32_T)count;
        emlrtDynamicBoundsCheckFastR2012b(i0, 1, md2, &emlrtBCI, &st);
        b_L2[0] = ((((unusedUb + unusedUc * muDoubleScalarCos(-pathJ->data[i +
          pathJ->size[0] * (j - 1)])) + unusedUe * muDoubleScalarCos(b_unusedU2))
                    + leg1AngleOffset * muDoubleScalarCos(pathJ->data[i +
          pathJ->size[0] * j] + b_unusedU2)) - B2PZOffset) * muDoubleScalarCos
          (pathJ->data[i + pathJ->size[0] * (j - 2)]);
        b_L2[1] = ((((unusedUb + unusedUc * muDoubleScalarCos(-pathJ->data[i +
          pathJ->size[0] * (j - 1)])) + unusedUe * muDoubleScalarCos(b_unusedU2))
                    + leg1AngleOffset * muDoubleScalarCos(pathJ->data[i +
          pathJ->size[0] * j] + b_unusedU2)) - B2PZOffset) * muDoubleScalarSin
          (pathJ->data[i + pathJ->size[0] * (j - 2)]);
        b_L2[2] = ((((transitionArrayLength + unusedUc * muDoubleScalarSin
                      (-pathJ->data[i + pathJ->size[0] * (j - 1)])) - unusedUe *
                     muDoubleScalarSin(b_unusedU2)) - leg1AngleOffset *
                    muDoubleScalarSin(pathJ->data[i + pathJ->size[0] * j] +
          b_unusedU2)) - legAngleOffset) - panHeight;
        for (i0 = 0; i0 < 3; i0++) {
          unusedUb = 0.0;
          for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
            unusedUb += TP2B[i0 + (path_idx_0 << 2)] * b_L2[path_idx_0];
          }

          c_TP2B[i0] = unusedUb + TP2B[12 + i0];
        }

        b_uInitP[0] = (-uInitP[0] * muDoubleScalarSin(uGoalP[0]) * ((((L2 - L7)
          + L5 * muDoubleScalarCos(uGoalP[2] + zeta)) + L3 * muDoubleScalarCos
          (uGoalP[1])) + L4 * muDoubleScalarCos(zeta)) - uInitP[1] * L3 *
                       muDoubleScalarCos(uGoalP[0]) * muDoubleScalarSin(uGoalP[1]))
          - uInitP[2] * L5 * muDoubleScalarSin(uGoalP[2] + zeta) *
          muDoubleScalarCos(uGoalP[0]);
        b_uInitP[1] = (uInitP[0] * muDoubleScalarCos(uGoalP[0]) * ((((L2 - L7) +
          L5 * muDoubleScalarCos(uGoalP[2] + zeta)) + L3 * muDoubleScalarCos
          (uGoalP[1])) + L4 * muDoubleScalarCos(zeta)) - uInitP[2] * L5 *
                       muDoubleScalarSin(uGoalP[2] + zeta) * muDoubleScalarSin
                       (uGoalP[0])) - uInitP[1] * L3 * muDoubleScalarSin(uGoalP
          [0]) * muDoubleScalarSin(uGoalP[1]);
        b_uInitP[2] = -uInitP[1] * L3 * muDoubleScalarCos(uGoalP[1]) - L5 *
          uInitP[2] * muDoubleScalarCos(zeta + uGoalP[2]);
        for (i0 = 0; i0 < 3; i0++) {
          uGoalP[i0] = 0.0;
          for (path_idx_0 = 0; path_idx_0 < 3; path_idx_0++) {
            uGoalP[i0] += TP2B[i0 + (path_idx_0 << 2)] * b_uInitP[path_idx_0];
          }
        }

        path->data[(int32_T)count - 1] = time;
        for (i0 = 0; i0 < 3; i0++) {
          path->data[((int32_T)count + path->size[0] * (i0 + 1)) - 1] =
            c_TP2B[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
          path->data[((int32_T)count + path->size[0] * (i0 + 4)) - 1] =
            uGoalP[i0];
        }

        time -= dt;
        count++;
        j -= 6;
        emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
      }

      i++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    md2 = path->size[0];
    i0 = x->size[0] * x->size[1];
    x->size[0] = md2;
    x->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)x, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    for (i0 = 0; i0 < 7; i0++) {
      for (path_idx_0 = 0; path_idx_0 < md2; path_idx_0++) {
        x->data[path_idx_0 + x->size[0] * i0] = path->data[path_idx_0 +
          path->size[0] * i0];
      }
    }

    i0 = path->size[0] * path->size[1];
    path->size[0] = x->size[0];
    path->size[1] = 7;
    emxEnsureCapacity(&st, (emxArray__common *)path, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    md2 = x->size[0] * x->size[1];
    for (i0 = 0; i0 < md2; i0++) {
      path->data[i0] = x->data[i0];
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
        b_path_idx_0 = path->size[0];
        transitionArrayLength = path->data[(i + b_path_idx_0 * j) - 1];
        b_path_idx_0 = path->size[0];
        path_idx_0 = path->size[0];
        path->data[(i + b_path_idx_0 * j) - 1] = path->data[(m - i) + path_idx_0
          * j];
        b_path_idx_0 = path->size[0];
        path->data[(m - i) + b_path_idx_0 * j] = transitionArrayLength;
      }
    }

    i0 = pathC->size[0] * pathC->size[1];
    pathC->size[0] = 1 + path->size[0];
    pathC->size[1] = 7;
    emxEnsureCapacity(sp, (emxArray__common *)pathC, i0, (int32_T)sizeof(real_T),
                      &emlrtRTEI);
    pathC->data[0] = 0.0;
    for (i0 = 0; i0 < 6; i0++) {
      pathC->data[pathC->size[0] * (i0 + 1)] = nInitCartesianB[i0];
    }

    for (i0 = 0; i0 < 7; i0++) {
      md2 = path->size[0];
      for (path_idx_0 = 0; path_idx_0 < md2; path_idx_0++) {
        pathC->data[(path_idx_0 + pathC->size[0] * i0) + 1] = path->
          data[path_idx_0 + path->size[0] * i0];
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

  emxFree_real_T(&b_pathJ);
  emxFree_real_T(&x);
  emxFree_int32_T(&r0);
  emxFree_real_T(&transitionArrayNearest);
  emxFree_real_T(&next);
  emxFree_real_T(&path);
  emxFree_real_T(&b_T);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void buildRRTWrapper_init(void)
{
  int32_T i5;
  static const real_T dv0[3] = { 1.0, 0.0, 0.5 };

  for (i5 = 0; i5 < 3; i5++) {
    HGAINS[i5] = dv0[i5];
  }
}

void exhaustive_not_empty_init(void)
{
}

void goalSeedFreq_not_empty_init(void)
{
}

/* End of code generation (buildRRTWrapper.c) */
