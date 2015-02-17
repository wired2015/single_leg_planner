/*
 * selectInput.c
 *
 * Code generation for function 'selectInput'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "selectInput.h"
#include "buildRRTWrapper_emxutil.h"
#include "getConstrainedGammaDotDot.h"
#include "angDiff.h"
#include "trInv.h"
#include "heuristicSingleLeg.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo lb_emlrtRSI = { 32, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo mb_emlrtRSI = { 38, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo nb_emlrtRSI = { 48, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo ob_emlrtRSI = { 67, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRSInfo pb_emlrtRSI = { 6, "getPhiAndOmega",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getPhiAndOmega.m"
};

static emlrtRTEInfo e_emlrtRTEI = { 1, 35, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtRTEInfo f_emlrtRTEI = { 12, 5, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo c_emlrtECI = { -1, 32, 10, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo d_emlrtECI = { -1, 32, 26, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m"
};

static emlrtECInfo e_emlrtECI = { -1, 56, 9, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo cb_emlrtBCI = { -1, -1, 56, 9, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

static emlrtRTEInfo j_emlrtRTEI = { 16, 5, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo db_emlrtBCI = { 1, 4, 42, 17, "kC.legAngleOffset",
  "generateTrMatrices",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/generateTrMatrices.m",
  0 };

static emlrtDCInfo i_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  1 };

static emlrtDCInfo j_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  4 };

static emlrtDCInfo k_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  1 };

static emlrtDCInfo l_emlrtDCI = { 13, 31, "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  4 };

static emlrtBCInfo eb_emlrtBCI = { -1, -1, 14, 5, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

/* Function Definitions */
void selectInput(const emlrtStack *sp, const real_T xNear_data[], const int32_T
                 xNear_size[2], const real_T xRand_data[], const int32_T
                 xRand_size[2], const real_T U[10], real_T dt, real_T Dt, const
                 struct0_T *kC, const real_T jointLimits[12], const real_T
                 uBDot[6], int32_T legNum, real_T xNew_data[], int32_T
                 xNew_size[2], emxArray_real_T *transitionArray)
{
  boolean_T b2;
  real_T candStates_data[55];
  emxArray_real_T *candTransArrays;
  real_T scale;
  int32_T i8;
  real_T t;
  int32_T loop_ub;
  real_T UJoint_data[15];
  int32_T i;
  real_T b_U[2];
  emxArray_real_T *r1;
  emxArray_int32_T *r2;
  real_T distance_data[5];
  real_T vS[3];
  real_T numIterations;
  int32_T tmp_size[2];
  real_T tmp_data[16];
  real_T b_xNear_data[11];
  real_T xInit_data[11];
  int32_T ixstart;
  real_T AdB2S[6];
  real_T k1[6];
  real_T y;
  real_T k2[6];
  real_T k3[6];
  real_T AdO2S[6];
  int32_T itmp;
  real_T TP2S[16];
  int32_T iv6[2];
  int32_T iv7[2];
  emxArray_real_T b_xRand_data;
  real_T hDiff;
  real_T aGain;
  real_T q[4];
  real_T TO2S[16];
  real_T TI2S[16];
  real_T TO2J[16];
  real_T TQ2O[16];
  real_T TR2Q[16];
  static const int8_T iv8[4] = { 0, 0, 0, 1 };

  static const int8_T iv9[4] = { 0, 0, 1, 0 };

  static const real_T dv3[4] = { 0.0, -1.0, 6.123233995736766E-17, 0.0 };

  static const int8_T iv10[4] = { 1, 0, 0, 0 };

  real_T TS2R[16];
  static const int8_T iv11[4] = { 0, 1, 0, 0 };

  real_T dv4[16];
  real_T TB2S[16];
  real_T dv5[16];
  real_T dv6[16];
  real_T dv7[16];
  real_T dv8[16];
  real_T b_TO2S[16];
  real_T c_TO2S[16];
  real_T d_TO2S[16];
  real_T e_TO2S[16];
  real_T f_TO2S[16];
  real_T b_TI2S[16];
  real_T c_TI2S[16];
  real_T d_TI2S[16];
  real_T e_TI2S[16];
  real_T b_TQ2O[16];
  real_T c_TQ2O[16];
  real_T dv9[9];
  real_T dv10[9];
  real_T b_AdB2S[36];
  real_T dv11[9];
  real_T AdP2S[36];
  real_T dv12[9];
  real_T AdI2S[36];
  real_T dv13[9];
  real_T b_AdO2S[36];
  real_T uPDot[6];
  real_T uIDot[6];
  real_T c_AdB2S[6];
  real_T b_AdP2S[6];
  real_T uODot[6];
  real_T uSDot[6];
  int32_T i9;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  b2 = false;

  /* selectInput Selects the most appropriate control input. */
  /*    A control input is selected from a set of control inputs, U. An input */
  /*    is selected by applying each of the inputs to to state xNear, which */
  /*    results in p candidate states, where p is the size of the input set. */
  /*    The control input corresponding to candidate state that is closest to */
  /*    x1 is returned as u. */
  /* Initialize arrays to store the candidate new state data and the */
  /* distances between each candidate state and the xNear state. */
  memset(&candStates_data[0], 0, 55U * sizeof(real_T));
  emxInit_real_T(sp, &candTransArrays, 2, &f_emlrtRTEI, true);
  scale = muDoubleScalarRound(Dt / dt);
  i8 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  t = (scale + 1.0) * 6.0;
  t = emlrtNonNegativeCheckFastR2012b(t, &j_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(t, &i_emlrtDCI,
    sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i8, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  t = (scale + 1.0) * 6.0;
  t = emlrtNonNegativeCheckFastR2012b(t, &j_emlrtDCI, sp);
  loop_ub = 5 * (int32_T)emlrtIntegerCheckFastR2012b(t, &i_emlrtDCI, sp);
  for (i8 = 0; i8 < loop_ub; i8++) {
    candTransArrays->data[i8] = 0.0;
  }

  /* Transform the control inputs to joint space. */
  for (i = 0; i < 5; i++) {
    /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
    for (i8 = 0; i8 < 2; i8++) {
      b_U[i8] = U[i + 5 * i8];
    }

    t = getConstrainedGammaDotDot(kC->l3, kC->l5, kC->zeta, b_U, *(real_T (*)[3])
      &xNear_data[6], *(real_T (*)[3])&xNear_data[3]);
    for (i8 = 0; i8 < 2; i8++) {
      UJoint_data[i + 5 * i8] = U[i + 5 * i8];
    }

    UJoint_data[i + 10] = t;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  emxInit_real_T(sp, &r1, 2, &e_emlrtRTEI, true);
  emxInit_int32_T(sp, &r2, 1, &e_emlrtRTEI, true);
  for (i = 0; i < 5; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    st.site = &lb_emlrtRSI;
    for (i8 = 0; i8 < 3; i8++) {
      vS[i8] = UJoint_data[i + 5 * i8];
    }

    /* rk4.m */
    /* author: wreid */
    /* date: 20150107 */
    /* rk4 Summary of this function goes here */
    /*    Detailed explanation goes here */
    numIterations = muDoubleScalarRound(Dt / dt);
    tmp_size[1] = 11;
    memset(&tmp_data[0], 0, 11U * sizeof(real_T));
    for (i8 = 0; i8 < 11; i8++) {
      b_xNear_data[i8] = xNear_data[xNear_size[0] * i8];
    }

    for (i8 = 0; i8 < 6; i8++) {
      xInit_data[i8] = b_xNear_data[3 + i8];
    }

    /* xInitOrig = xInit; */
    i8 = r1->size[0] * r1->size[1];
    r1->size[0] = 1;
    t = (numIterations + 1.0) * 6.0;
    t = emlrtNonNegativeCheckFastR2012b(t, &l_emlrtDCI, &st);
    r1->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(t, &k_emlrtDCI, &st);
    emxEnsureCapacity(&st, (emxArray__common *)r1, i8, (int32_T)sizeof(real_T),
                      &e_emlrtRTEI);
    t = (numIterations + 1.0) * 6.0;
    t = emlrtNonNegativeCheckFastR2012b(t, &l_emlrtDCI, &st);
    loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(t, &k_emlrtDCI, &st);
    for (i8 = 0; i8 < loop_ub; i8++) {
      r1->data[i8] = 0.0;
    }

    ixstart = (int32_T)((numIterations + 1.0) * 6.0);
    for (i8 = 0; i8 < 6; i8++) {
      r1->data[emlrtDynamicBoundsCheckFastR2012b(i8 + 1, 1, ixstart,
        &eb_emlrtBCI, &st) - 1] = xInit_data[i8];
    }

    emlrtForLoopVectorCheckR2012b(1.0, 1.0, numIterations, mxDOUBLE_CLASS,
      (int32_T)numIterations, &j_emlrtRTEI, &st);
    ixstart = 0;
    while (ixstart <= (int32_T)numIterations - 1) {
      for (i8 = 0; i8 < 6; i8++) {
        AdB2S[i8] = xInit_data[i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k1[i8] = AdB2S[3 + i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k1[i8 + 3] = vS[i8];
      }

      y = dt / 2.0;
      for (i8 = 0; i8 < 6; i8++) {
        AdB2S[i8] = xInit_data[i8] + y * k1[i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k2[i8] = AdB2S[3 + i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k2[i8 + 3] = vS[i8];
      }

      y = dt / 2.0;
      for (i8 = 0; i8 < 6; i8++) {
        AdB2S[i8] = xInit_data[i8] + y * k2[i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k3[i8] = AdB2S[3 + i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        k3[i8 + 3] = vS[i8];
      }

      y = dt / 2.0;
      scale = dt / 6.0;
      for (i8 = 0; i8 < 6; i8++) {
        AdB2S[i8] = xInit_data[i8] + y * k3[i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        AdO2S[i8] = AdB2S[3 + i8];
      }

      for (i8 = 0; i8 < 3; i8++) {
        AdO2S[i8 + 3] = vS[i8];
      }

      tmp_size[1] = 6;
      for (i8 = 0; i8 < 6; i8++) {
        tmp_data[i8] = xInit_data[i8] + scale * (((k1[i8] + 2.0 * k2[i8]) + 2.0 *
          k3[i8]) + AdO2S[i8]);
      }

      /* Check pan angular position limits */
      if ((tmp_data[0] > jointLimits[1]) || (tmp_data[0] < jointLimits[0])) {
        tmp_data[0] = xInit_data[0];
        tmp_data[3] = 0.0;
        vS[0] = 0.0;
      }

      /* Check inner and outer leg angular position limits */
      if ((tmp_data[1] > jointLimits[3]) || (tmp_data[1] < jointLimits[2]) ||
          (tmp_data[2] > jointLimits[5]) || (tmp_data[2] < jointLimits[4])) {
        tmp_data[1] = xInit_data[1];
        tmp_data[2] = xInit_data[2];
        tmp_data[4] = 0.0;
        tmp_data[5] = 0.0;
        vS[1] = 0.0;
        vS[2] = 0.0;
      }

      /* Check pan angular velocity limits */
      if ((tmp_data[3] > jointLimits[7]) || (tmp_data[3] < jointLimits[6])) {
        tmp_data[3] = xInit_data[3];
        vS[0] = 0.0;
      }

      /* Check inner and outer leg angular velocity limits */
      if ((tmp_data[4] > jointLimits[9]) || (tmp_data[4] < jointLimits[8]) ||
          (tmp_data[5] > jointLimits[11]) || (tmp_data[5] < jointLimits[10])) {
        tmp_data[4] = xInit_data[4];
        tmp_data[5] = xInit_data[5];
        vS[1] = 0.0;
        vS[2] = 0.0;
      }

      for (i8 = 0; i8 < 6; i8++) {
        xInit_data[i8] = tmp_data[i8];
      }

      t = 6.0 * (1.0 + (real_T)ixstart) + 1.0;
      scale = 6.0 * ((1.0 + (real_T)ixstart) + 1.0);
      if (t > scale) {
        i8 = 0;
        loop_ub = 0;
      } else {
        i8 = r1->size[1];
        loop_ub = (int32_T)t;
        i8 = emlrtDynamicBoundsCheckFastR2012b(loop_ub, 1, i8, &cb_emlrtBCI, &st)
          - 1;
        loop_ub = r1->size[1];
        itmp = (int32_T)scale;
        loop_ub = emlrtDynamicBoundsCheckFastR2012b(itmp, 1, loop_ub,
          &cb_emlrtBCI, &st);
      }

      loop_ub -= i8;
      emlrtSizeEqCheck1DFastR2012b(loop_ub, 6, &e_emlrtECI, &st);
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        r1->data[i8 + loop_ub] = tmp_data[loop_ub];
      }

      ixstart++;
      emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
    }

    /* xInit = [zeros(1,3) xInitOrig 0 0]; */
    ixstart = 5 + tmp_size[1];
    for (i8 = 0; i8 < 3; i8++) {
      TP2S[i8] = 0.0;
    }

    loop_ub = tmp_size[1];
    for (i8 = 0; i8 < loop_ub; i8++) {
      TP2S[i8 + 3] = tmp_data[i8];
    }

    TP2S[3 + tmp_size[1]] = 0.0;
    TP2S[4 + tmp_size[1]] = 0.0;
    tmp_size[0] = 1;
    tmp_size[1] = ixstart;
    for (i8 = 0; i8 < ixstart; i8++) {
      tmp_data[i8] = TP2S[i8];
    }

    if (!b2) {
      for (i8 = 0; i8 < 2; i8++) {
        iv6[i8] = 1 + 10 * i8;
      }

      b2 = true;
    }

    emlrtSubAssignSizeCheckR2012b(iv6, 2, tmp_size, 2, &c_emlrtECI, sp);
    for (i8 = 0; i8 < ixstart; i8++) {
      candStates_data[i + 5 * i8] = tmp_data[i8];
    }

    loop_ub = candTransArrays->size[1];
    i8 = r2->size[0];
    r2->size[0] = loop_ub;
    emxEnsureCapacity(sp, (emxArray__common *)r2, i8, (int32_T)sizeof(int32_T),
                      &e_emlrtRTEI);
    for (i8 = 0; i8 < loop_ub; i8++) {
      r2->data[i8] = i8;
    }

    iv7[0] = 1;
    iv7[1] = r2->size[0];
    emlrtSubAssignSizeCheckR2012b(iv7, 2, *(int32_T (*)[2])r1->size, 2,
      &d_emlrtECI, sp);
    loop_ub = r1->size[1];
    for (i8 = 0; i8 < loop_ub; i8++) {
      candTransArrays->data[i + candTransArrays->size[0] * r2->data[i8]] =
        r1->data[r1->size[0] * i8];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    for (i8 = 0; i8 < 11; i8++) {
      b_xNear_data[i8] = candStates_data[i + 5 * i8];
    }

    b_xRand_data.data = (real_T *)xRand_data;
    b_xRand_data.size = (int32_T *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    st.site = &mb_emlrtRSI;
    hDiff = heuristicSingleLeg(&st, b_xNear_data, &b_xRand_data, jointLimits,
      kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);

    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    aGain = 0.5;
    if (xNear_data[0] == 1.0) {
      aGain = 0.0;
    }

    st.site = &nb_emlrtRSI;
    for (i8 = 0; i8 < 3; i8++) {
      q[i8] = candStates_data[i + 5 * (3 + i8)];
    }

    /* Homogeneous transformation matrices. */
    b_st.site = &pb_emlrtRSI;

    /* GENERATETRMATRICES Generates each of the homogeneous transformation */
    /* matrices that describe the kinematic chain between the Sherpa_TT rover's */
    /* body coordinate frame and its wheel contact frame. Denavit-Hartenburg */
    /* parameters are used to express the transformation between each coordinate */
    /* frame in the kinematic chain. */
    /*  */
    /* Inputs: */
    /* -uG: A 1x3 vector giving the [x y z] relationship between the body and */
    /* coordinate frame */
    /* -q: A 1x4 vector describing the leg's joint state. This vector includes  */
    /* [alpha beta gamma]. */
    /* -kC: A struct containing the kinematic parameters of the Sherpa_TT leg. */
    /* -legNum: The number of the leg that is being considered (1,2,3 or 4). */
    /*  */
    /* Outputs: */
    /* TB2G: Transformation from the body to the ground. */
    /* TP2B: Transformation from the pan joint to the body. */
    /* TI2P: Transformation from the inner leg joint to the pan joint. */
    /* TJ2I: Transformation from the inner leg knee joint to the inner leg joint. */
    /* TO2J: Transformation from the outer leg joint to the inner leg knee joint. */
    /* TQ2O: Transformation from the outer leg end joint to the outer leg joint. */
    /* TR2Q: Transformation from the steering base joint to the outer leg end */
    /* joint. */
    /* TS2R: Transformation from the steering joint to the steering base joint. */
    /* TW2S: Transformation from the wheel joint to the steering joint. */
    /* TC2W: Transformation from the wheel contact point to the wheel joint. */
    /*  */
    /* generateTrMatrices.m */
    /* author:    wreid */
    /* date:      20140214 */
    /* TODO: Use a 6-DOF relationship between the ground and body frames by */
    /* including the roll, pitch and yaw of the platform. */
    emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &db_emlrtBCI, &b_st);

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TO2S[0] = muDoubleScalarCos(q[0]);
    TO2S[4] = -muDoubleScalarSin(q[0]) * 6.123233995736766E-17;
    TO2S[8] = -muDoubleScalarSin(q[0]);
    TO2S[12] = kC->l2 * muDoubleScalarCos(q[0]);
    TO2S[1] = muDoubleScalarSin(q[0]);
    TO2S[5] = muDoubleScalarCos(q[0]) * 6.123233995736766E-17;
    TO2S[9] = -(-muDoubleScalarCos(q[0]));
    TO2S[13] = kC->l2 * muDoubleScalarSin(q[0]);
    TO2S[2] = 0.0;
    TO2S[6] = -1.0;
    TO2S[10] = 6.123233995736766E-17;
    TO2S[14] = kC->l1;

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TI2S[0] = muDoubleScalarCos(q[1]);
    TI2S[4] = -muDoubleScalarSin(q[1]);
    TI2S[8] = muDoubleScalarSin(q[1]) * 0.0;
    TI2S[12] = kC->l3 * muDoubleScalarCos(q[1]);
    TI2S[1] = muDoubleScalarSin(q[1]);
    TI2S[5] = muDoubleScalarCos(q[1]);
    TI2S[9] = -muDoubleScalarCos(q[1]) * 0.0;
    TI2S[13] = kC->l3 * muDoubleScalarSin(q[1]);
    scale = -q[1] + kC->zeta;

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TO2J[0] = muDoubleScalarCos(scale);
    TO2J[4] = -muDoubleScalarSin(scale);
    TO2J[8] = muDoubleScalarSin(scale) * 0.0;
    TO2J[12] = kC->l4 * muDoubleScalarCos(scale);
    TO2J[1] = muDoubleScalarSin(scale);
    TO2J[5] = muDoubleScalarCos(scale);
    TO2J[9] = -muDoubleScalarCos(scale) * 0.0;
    TO2J[13] = kC->l4 * muDoubleScalarSin(scale);

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TQ2O[0] = muDoubleScalarCos(q[2]);
    TQ2O[4] = -muDoubleScalarSin(q[2]);
    TQ2O[8] = muDoubleScalarSin(q[2]) * 0.0;
    TQ2O[12] = kC->l5 * muDoubleScalarCos(q[2]);
    TQ2O[1] = muDoubleScalarSin(q[2]);
    TQ2O[5] = muDoubleScalarCos(q[2]);
    TQ2O[9] = -muDoubleScalarCos(q[2]) * 0.0;
    TQ2O[13] = kC->l5 * muDoubleScalarSin(q[2]);
    scale = -q[2] - kC->zeta;

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TR2Q[0] = muDoubleScalarCos(scale);
    TR2Q[4] = -muDoubleScalarSin(scale) * 6.123233995736766E-17;
    TR2Q[8] = -muDoubleScalarSin(scale);
    TR2Q[12] = -kC->l7 * muDoubleScalarCos(scale);
    TR2Q[1] = muDoubleScalarSin(scale);
    TR2Q[5] = muDoubleScalarCos(scale) * 6.123233995736766E-17;
    TR2Q[9] = -(-muDoubleScalarCos(scale));
    TR2Q[13] = -kC->l7 * muDoubleScalarSin(scale);
    for (i8 = 0; i8 < 4; i8++) {
      TO2S[3 + (i8 << 2)] = iv8[i8];
      TI2S[2 + (i8 << 2)] = iv9[i8];
      TI2S[3 + (i8 << 2)] = iv8[i8];
      TO2J[2 + (i8 << 2)] = iv9[i8];
      TO2J[3 + (i8 << 2)] = iv8[i8];
      TQ2O[2 + (i8 << 2)] = iv9[i8];
      TQ2O[3 + (i8 << 2)] = iv8[i8];
      TR2Q[2 + (i8 << 2)] = dv3[i8];
      TR2Q[3 + (i8 << 2)] = iv8[i8];

      /* TRDH Generates the homogeneous transformation matrix A using the  */
      /* Denavit-Hartenberg parameters theta, d, a and alpha. */
      /*  */
      /* trDH.m */
      /* author:    wreid */
      /* date:      20150214 */
      TS2R[i8 << 2] = iv10[i8];
      TS2R[1 + (i8 << 2)] = iv11[i8];
    }

    TS2R[2] = 0.0;
    TS2R[6] = 0.0;
    TS2R[10] = 1.0;
    TS2R[14] = kC->l6;
    for (i8 = 0; i8 < 4; i8++) {
      TS2R[3 + (i8 << 2)] = iv8[i8];
    }

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TP2S[0] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2S[4] = -muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2S[8] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]) * 0.0;
    TP2S[12] = kC->B2PXOffset * muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2S[1] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2S[5] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
    TP2S[9] = -muDoubleScalarCos(kC->legAngleOffset[legNum - 1]) * 0.0;
    TP2S[13] = kC->B2PXOffset * muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
    TP2S[2] = 0.0;
    TP2S[6] = 0.0;
    TP2S[10] = 1.0;
    TP2S[14] = kC->B2PZOffset;
    for (i8 = 0; i8 < 4; i8++) {
      TP2S[3 + (i8 << 2)] = iv8[i8];
    }

    for (i8 = 0; i8 < 4; i8++) {
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        TB2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          TB2S[i8 + (loop_ub << 2)] += TP2S[i8 + (itmp << 2)] * TO2S[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv5[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          dv5[i8 + (loop_ub << 2)] += TB2S[i8 + (itmp << 2)] * TI2S[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv6[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          dv6[i8 + (loop_ub << 2)] += dv5[i8 + (itmp << 2)] * TO2J[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv7[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          dv7[i8 + (loop_ub << 2)] += dv6[i8 + (itmp << 2)] * TQ2O[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv8[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          dv8[i8 + (loop_ub << 2)] += dv7[i8 + (itmp << 2)] * TR2Q[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        dv4[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          dv4[i8 + (loop_ub << 2)] += dv8[i8 + (itmp << 2)] * TS2R[itmp +
            (loop_ub << 2)];
        }

        b_TO2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          b_TO2S[i8 + (loop_ub << 2)] += TO2S[i8 + (itmp << 2)] * TI2S[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        c_TO2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          c_TO2S[i8 + (loop_ub << 2)] += b_TO2S[i8 + (itmp << 2)] * TO2J[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        d_TO2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          d_TO2S[i8 + (loop_ub << 2)] += c_TO2S[i8 + (itmp << 2)] * TQ2O[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        e_TO2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          e_TO2S[i8 + (loop_ub << 2)] += d_TO2S[i8 + (itmp << 2)] * TR2Q[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        f_TO2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          f_TO2S[i8 + (loop_ub << 2)] += e_TO2S[i8 + (itmp << 2)] * TS2R[itmp +
            (loop_ub << 2)];
        }

        b_TI2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          b_TI2S[i8 + (loop_ub << 2)] += TI2S[i8 + (itmp << 2)] * TO2J[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        c_TI2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          c_TI2S[i8 + (loop_ub << 2)] += b_TI2S[i8 + (itmp << 2)] * TQ2O[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        d_TI2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          d_TI2S[i8 + (loop_ub << 2)] += c_TI2S[i8 + (itmp << 2)] * TR2Q[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        e_TI2S[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          e_TI2S[i8 + (loop_ub << 2)] += d_TI2S[i8 + (itmp << 2)] * TS2R[itmp +
            (loop_ub << 2)];
        }

        b_TQ2O[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          b_TQ2O[i8 + (loop_ub << 2)] += TQ2O[i8 + (itmp << 2)] * TR2Q[itmp +
            (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        c_TQ2O[i8 + (loop_ub << 2)] = 0.0;
        for (itmp = 0; itmp < 4; itmp++) {
          c_TQ2O[i8 + (loop_ub << 2)] += b_TQ2O[i8 + (itmp << 2)] * TS2R[itmp +
            (loop_ub << 2)];
        }
      }
    }

    trInv(dv4, TB2S);
    trInv(f_TO2S, TP2S);
    trInv(e_TI2S, TI2S);
    trInv(c_TQ2O, TO2S);

    /* Adjunct transformation matrices. */
    /* TR2ADJ Returns the adjunct matrix, A, based on the homogeneous */
    /* transformation matrix, T. The adjunct matrix serves to transform the */
    /* velocity from the one frame to another, as described by the homoegenous */
    /* transformation matrix. */
    /*  */
    /* Inputs: */
    /* -T: The 4x4 homogeneous transformation matrix representing the */
    /* transformation from one frame to another. */
    /* Outputs: */
    /* -A: The adjunct matrix that transforms velocity vectors from one frame to */
    /* another. */
    dv9[0] = 0.0;
    dv9[3] = -TB2S[14];
    dv9[6] = TB2S[13];
    dv9[1] = TB2S[14];
    dv9[4] = 0.0;
    dv9[7] = -TB2S[12];
    dv9[2] = -TB2S[13];
    dv9[5] = TB2S[12];
    dv9[8] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        dv10[i8 + 3 * loop_ub] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          dv10[i8 + 3 * loop_ub] += dv9[i8 + 3 * itmp] * TB2S[itmp + (loop_ub <<
            2)];
        }

        b_AdB2S[loop_ub + 6 * i8] = TB2S[loop_ub + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdB2S[loop_ub + 6 * (i8 + 3)] = dv10[loop_ub + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdB2S[(loop_ub + 6 * i8) + 3] = 0.0;
      }
    }

    /* TR2ADJ Returns the adjunct matrix, A, based on the homogeneous */
    /* transformation matrix, T. The adjunct matrix serves to transform the */
    /* velocity from the one frame to another, as described by the homoegenous */
    /* transformation matrix. */
    /*  */
    /* Inputs: */
    /* -T: The 4x4 homogeneous transformation matrix representing the */
    /* transformation from one frame to another. */
    /* Outputs: */
    /* -A: The adjunct matrix that transforms velocity vectors from one frame to */
    /* another. */
    dv11[0] = 0.0;
    dv11[3] = -TP2S[14];
    dv11[6] = TP2S[13];
    dv11[1] = TP2S[14];
    dv11[4] = 0.0;
    dv11[7] = -TP2S[12];
    dv11[2] = -TP2S[13];
    dv11[5] = TP2S[12];
    dv11[8] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdB2S[(loop_ub + 6 * (i8 + 3)) + 3] = TB2S[loop_ub + (i8 << 2)];
        dv10[i8 + 3 * loop_ub] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          dv10[i8 + 3 * loop_ub] += dv11[i8 + 3 * itmp] * TP2S[itmp + (loop_ub <<
            2)];
        }

        AdP2S[loop_ub + 6 * i8] = TP2S[loop_ub + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdP2S[loop_ub + 6 * (i8 + 3)] = dv10[loop_ub + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdP2S[(loop_ub + 6 * i8) + 3] = 0.0;
      }
    }

    /* TR2ADJ Returns the adjunct matrix, A, based on the homogeneous */
    /* transformation matrix, T. The adjunct matrix serves to transform the */
    /* velocity from the one frame to another, as described by the homoegenous */
    /* transformation matrix. */
    /*  */
    /* Inputs: */
    /* -T: The 4x4 homogeneous transformation matrix representing the */
    /* transformation from one frame to another. */
    /* Outputs: */
    /* -A: The adjunct matrix that transforms velocity vectors from one frame to */
    /* another. */
    dv12[0] = 0.0;
    dv12[3] = -TI2S[14];
    dv12[6] = TI2S[13];
    dv12[1] = TI2S[14];
    dv12[4] = 0.0;
    dv12[7] = -TI2S[12];
    dv12[2] = -TI2S[13];
    dv12[5] = TI2S[12];
    dv12[8] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdP2S[(loop_ub + 6 * (i8 + 3)) + 3] = TP2S[loop_ub + (i8 << 2)];
        dv10[i8 + 3 * loop_ub] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          dv10[i8 + 3 * loop_ub] += dv12[i8 + 3 * itmp] * TI2S[itmp + (loop_ub <<
            2)];
        }

        AdI2S[loop_ub + 6 * i8] = TI2S[loop_ub + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdI2S[loop_ub + 6 * (i8 + 3)] = dv10[loop_ub + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdI2S[(loop_ub + 6 * i8) + 3] = 0.0;
      }
    }

    /* TR2ADJ Returns the adjunct matrix, A, based on the homogeneous */
    /* transformation matrix, T. The adjunct matrix serves to transform the */
    /* velocity from the one frame to another, as described by the homoegenous */
    /* transformation matrix. */
    /*  */
    /* Inputs: */
    /* -T: The 4x4 homogeneous transformation matrix representing the */
    /* transformation from one frame to another. */
    /* Outputs: */
    /* -A: The adjunct matrix that transforms velocity vectors from one frame to */
    /* another. */
    dv13[0] = 0.0;
    dv13[3] = -TO2S[14];
    dv13[6] = TO2S[13];
    dv13[1] = TO2S[14];
    dv13[4] = 0.0;
    dv13[7] = -TO2S[12];
    dv13[2] = -TO2S[13];
    dv13[5] = TO2S[12];
    dv13[8] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        AdI2S[(loop_ub + 6 * (i8 + 3)) + 3] = TI2S[loop_ub + (i8 << 2)];
        dv10[i8 + 3 * loop_ub] = 0.0;
        for (itmp = 0; itmp < 3; itmp++) {
          dv10[i8 + 3 * loop_ub] += dv13[i8 + 3 * itmp] * TO2S[itmp + (loop_ub <<
            2)];
        }

        b_AdO2S[loop_ub + 6 * i8] = TO2S[loop_ub + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdO2S[loop_ub + 6 * (i8 + 3)] = dv10[loop_ub + 3 * i8];
      }
    }

    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdO2S[(loop_ub + 6 * i8) + 3] = 0.0;
      }
    }

    /* Pan joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i8 = 0; i8 < 3; i8++) {
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        b_AdO2S[(loop_ub + 6 * (i8 + 3)) + 3] = TO2S[loop_ub + (i8 << 2)];
      }

      AdB2S[i8] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 30];
    for (i8 = 0; i8 < 6; i8++) {
      uPDot[i8] = AdB2S[i8];
    }

    /* Beta joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i8 = 0; i8 < 3; i8++) {
      AdB2S[i8] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 35];
    for (i8 = 0; i8 < 6; i8++) {
      uIDot[i8] = AdB2S[i8];
    }

    /* Gamma joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i8 = 0; i8 < 3; i8++) {
      AdB2S[i8] = 0.0;
    }

    AdB2S[3] = 0.0;
    AdB2S[4] = 0.0;
    AdB2S[5] = candStates_data[i + 40];

    /* Velocity vector for the ankle frame. */
    for (i8 = 0; i8 < 6; i8++) {
      uODot[i8] = AdB2S[i8];
      c_AdB2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        c_AdB2S[i8] += b_AdB2S[i8 + 6 * loop_ub] * uBDot[loop_ub];
      }

      b_AdP2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        b_AdP2S[i8] += AdP2S[i8 + 6 * loop_ub] * uPDot[loop_ub];
      }
    }

    for (i8 = 0; i8 < 6; i8++) {
      t = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        t += AdI2S[i8 + 6 * loop_ub] * uIDot[loop_ub];
      }

      AdB2S[i8] = (c_AdB2S[i8] + b_AdP2S[i8]) + t;
    }

    for (i8 = 0; i8 < 6; i8++) {
      AdO2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        AdO2S[i8] += b_AdO2S[i8 + 6 * loop_ub] * uODot[loop_ub];
      }

      uSDot[i8] = AdB2S[i8] + AdO2S[i8];
      c_AdB2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        c_AdB2S[i8] += b_AdB2S[i8 + 6 * loop_ub] * uBDot[loop_ub];
      }

      b_AdP2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        b_AdP2S[i8] += AdP2S[i8 + 6 * loop_ub] * uPDot[loop_ub];
      }
    }

    for (i8 = 0; i8 < 6; i8++) {
      t = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        t += AdI2S[i8 + 6 * loop_ub] * uIDot[loop_ub];
      }

      AdB2S[i8] = (c_AdB2S[i8] + b_AdP2S[i8]) + t;
    }

    for (i8 = 0; i8 < 6; i8++) {
      AdO2S[i8] = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        AdO2S[i8] += b_AdO2S[i8 + 6 * loop_ub] * uODot[loop_ub];
      }

      c_AdB2S[i8] = AdB2S[i8] + AdO2S[i8];
    }

    for (i8 = 0; i8 < 3; i8++) {
      vS[i8] = c_AdB2S[i8];
    }

    /* [m/s] */
    /* [rad/s] */
    /* Calculate the required phi joint angle and the required wheel speed, */
    /* omega. */
    /* [rad] */
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (ixstart = 0; ixstart < 3; ixstart++) {
      numIterations = muDoubleScalarAbs(vS[ixstart]);
      if (numIterations > scale) {
        t = scale / numIterations;
        y = 1.0 + y * t * t;
        scale = numIterations;
      } else {
        t = numIterations / scale;
        y += t * t;
      }
    }

    y = scale * muDoubleScalarSqrt(y);

    /* [rad/s] */
    candStates_data[i + 45] = muDoubleScalarAtan2(uSDot[1], uSDot[0]);
    candStates_data[i + 50] = y / kC->r;

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    if (angDiff(xNear_data[10], candStates_data[i + 45]) > 0.39269908169872414)
    {
      i9 = 1;
    } else {
      i9 = 0;
    }

    distance_data[i] = (1.0 - aGain) * hDiff + aGain * (real_T)i9;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r2);
  emxFree_real_T(&r1);
  st.site = &ob_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  ixstart = 1;
  scale = distance_data[0];
  itmp = 0;
  if (muDoubleScalarIsNaN(distance_data[0])) {
    loop_ub = 1;
    exitg1 = false;
    while ((!exitg1) && (loop_ub + 1 <= 5)) {
      ixstart = loop_ub + 1;
      if (!muDoubleScalarIsNaN(distance_data[loop_ub])) {
        scale = distance_data[loop_ub];
        itmp = loop_ub;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < scale) {
        scale = distance_data[ixstart];
        itmp = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 11;
  for (i8 = 0; i8 < 11; i8++) {
    xNew_data[xNew_size[0] * i8] = candStates_data[itmp + 5 * i8];
  }

  loop_ub = candTransArrays->size[1];
  i8 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = loop_ub;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i8, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  for (i8 = 0; i8 < loop_ub; i8++) {
    transitionArray->data[transitionArray->size[0] * i8] = candTransArrays->
      data[itmp + candTransArrays->size[0] * i8];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
