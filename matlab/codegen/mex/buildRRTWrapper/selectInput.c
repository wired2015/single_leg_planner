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
#include "trInv.h"
#include "heuristicSingleLeg.h"
#include "rk4.h"
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

static emlrtRSInfo ob_emlrtRSI = { 66, "selectInput",
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

static emlrtBCInfo eb_emlrtBCI = { 1, 4, 42, 17, "kC.legAngleOffset",
  "generateTrMatrices",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/generateTrMatrices.m",
  0 };

static emlrtDCInfo i_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  1 };

static emlrtDCInfo j_emlrtDCI = { 12, 36, "selectInput",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/selectInput.m",
  4 };

/* Function Definitions */
void selectInput(const emlrtStack *sp, const real_T xNear_data[], const real_T
                 xRand_data[], const int32_T xRand_size[2], const real_T U[10],
                 real_T dt, real_T Dt, const real_T HGAINS[3], const struct0_T
                 *kC, const real_T jointLimits[12], const real_T uBDot[6],
                 int32_T legNum, real_T xNew_data[], int32_T xNew_size[2],
                 emxArray_real_T *transitionArray)
{
  boolean_T b2;
  real_T candStates_data[55];
  emxArray_real_T *candTransArrays;
  real_T aDiff;
  int32_T i6;
  real_T d1;
  int32_T ixstart;
  real_T UJoint_data[15];
  int32_T i;
  emxArray_real_T *r1;
  emxArray_int32_T *r2;
  real_T distance_data[5];
  real_T vS[3];
  real_T b_xNear_data[11];
  int32_T tmp_size[2];
  real_T tmp_data[16];
  int32_T iv6[2];
  int32_T iv7[2];
  real_T b_candStates_data[11];
  emxArray_real_T b_xRand_data;
  real_T hDiff;
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
  real_T TP2S[16];
  real_T TB2S[16];
  real_T dv5[16];
  real_T dv6[16];
  real_T dv7[16];
  real_T dv8[16];
  int32_T ix;
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
  real_T AdB2S[36];
  real_T dv11[9];
  real_T AdP2S[36];
  real_T dv12[9];
  real_T AdI2S[36];
  real_T dv13[9];
  real_T AdO2S[36];
  real_T b_AdB2S[6];
  real_T uPDot[6];
  real_T uIDot[6];
  real_T c_AdB2S[6];
  real_T b_AdP2S[6];
  real_T uODot[6];
  real_T b_AdO2S[6];
  real_T uSDot[6];
  real_T scale;
  real_T absxk;
  real_T t;
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
  aDiff = muDoubleScalarRound(Dt / dt);
  i6 = candTransArrays->size[0] * candTransArrays->size[1];
  candTransArrays->size[0] = 5;
  d1 = (aDiff + 1.0) * 6.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &j_emlrtDCI, sp);
  candTransArrays->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(d1,
    &i_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)candTransArrays, i6, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  d1 = (aDiff + 1.0) * 6.0;
  d1 = emlrtNonNegativeCheckFastR2012b(d1, &j_emlrtDCI, sp);
  ixstart = 5 * (int32_T)emlrtIntegerCheckFastR2012b(d1, &i_emlrtDCI, sp);
  for (i6 = 0; i6 < ixstart; i6++) {
    candTransArrays->data[i6] = 0.0;
  }

  /* Transform the control inputs to joint space. */
  for (i = 0; i < 5; i++) {
    /* gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma)); */
    /* getConstrainedGammaDotDot Summary of this function goes here */
    /*    Detailed explanation goes here */
    /* [~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); */
    for (i6 = 0; i6 < 2; i6++) {
      UJoint_data[i + 5 * i6] = U[i + 5 * i6];
    }

    UJoint_data[i + 10] = ((-U[5 + i] * kC->l3 * muDoubleScalarCos(xNear_data[4])
      + xNear_data[7] * xNear_data[7] * kC->l3 * muDoubleScalarSin(xNear_data[4]))
      + xNear_data[8] * xNear_data[8] * kC->l5 * muDoubleScalarSin(kC->zeta +
      xNear_data[5])) / (kC->l5 * muDoubleScalarCos(kC->zeta + xNear_data[5]));
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  /* Increment over the control vector. Generate a candidate state for each */
  /* possible control input. */
  emxInit_real_T(sp, &r1, 2, &e_emlrtRTEI, true);
  emxInit_int32_T(sp, &r2, 1, &e_emlrtRTEI, true);
  for (i = 0; i < 5; i++) {
    /* Generate a candidate state using a fourth order Runge-Kutta  */
    /* integration technique. */
    for (i6 = 0; i6 < 3; i6++) {
      vS[i6] = UJoint_data[i + 5 * i6];
    }

    memcpy(&b_xNear_data[0], &xNear_data[0], 11U * sizeof(real_T));
    st.site = &lb_emlrtRSI;
    rk4(&st, vS, dt, Dt, b_xNear_data, jointLimits, tmp_data, tmp_size, r1);
    if (!b2) {
      for (i6 = 0; i6 < 2; i6++) {
        iv6[i6] = 1 + 10 * i6;
      }

      b2 = true;
    }

    emlrtSubAssignSizeCheckR2012b(iv6, 2, tmp_size, 2, &c_emlrtECI, sp);
    ixstart = tmp_size[1];
    for (i6 = 0; i6 < ixstart; i6++) {
      candStates_data[i + 5 * i6] = tmp_data[tmp_size[0] * i6];
    }

    ixstart = candTransArrays->size[1];
    i6 = r2->size[0];
    r2->size[0] = ixstart;
    emxEnsureCapacity(sp, (emxArray__common *)r2, i6, (int32_T)sizeof(int32_T),
                      &e_emlrtRTEI);
    for (i6 = 0; i6 < ixstart; i6++) {
      r2->data[i6] = i6;
    }

    iv7[0] = 1;
    iv7[1] = r2->size[0];
    emlrtSubAssignSizeCheckR2012b(iv7, 2, *(int32_T (*)[2])r1->size, 2,
      &d_emlrtECI, sp);
    ixstart = r1->size[1];
    for (i6 = 0; i6 < ixstart; i6++) {
      candTransArrays->data[i + candTransArrays->size[0] * r2->data[i6]] =
        r1->data[r1->size[0] * i6];
    }

    /* U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst) */
    /* velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst); */
    /* Calculate the distance between the candidate state and the random */
    /* state. */
    for (i6 = 0; i6 < 11; i6++) {
      b_candStates_data[i6] = candStates_data[i + 5 * i6];
    }

    b_xRand_data.data = (real_T *)xRand_data;
    b_xRand_data.size = (int32_T *)xRand_size;
    b_xRand_data.allocatedSize = -1;
    b_xRand_data.numDimensions = 2;
    b_xRand_data.canFreeData = false;
    st.site = &mb_emlrtRSI;
    hDiff = heuristicSingleLeg(&st, b_candStates_data, &b_xRand_data,
      jointLimits, kC->l2, kC->l3, kC->l4, kC->l5, kC->l7, kC->zeta);

    /* Apply the ankle constraint to penalize any candidate state that */
    /* requires a change of ankle position greater than the allowed ankle */
    /* movement in a single time step. */
    /*          if xNear(1) == 1 */
    /*              aGain = 0; */
    /*          end */
    st.site = &nb_emlrtRSI;
    for (i6 = 0; i6 < 3; i6++) {
      q[i6] = candStates_data[i + 5 * (3 + i6)];
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
    emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &eb_emlrtBCI, &b_st);

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
    aDiff = -q[1] + kC->zeta;

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TO2J[0] = muDoubleScalarCos(aDiff);
    TO2J[4] = -muDoubleScalarSin(aDiff);
    TO2J[8] = muDoubleScalarSin(aDiff) * 0.0;
    TO2J[12] = kC->l4 * muDoubleScalarCos(aDiff);
    TO2J[1] = muDoubleScalarSin(aDiff);
    TO2J[5] = muDoubleScalarCos(aDiff);
    TO2J[9] = -muDoubleScalarCos(aDiff) * 0.0;
    TO2J[13] = kC->l4 * muDoubleScalarSin(aDiff);

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
    aDiff = -q[2] - kC->zeta;

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TR2Q[0] = muDoubleScalarCos(aDiff);
    TR2Q[4] = -muDoubleScalarSin(aDiff) * 6.123233995736766E-17;
    TR2Q[8] = -muDoubleScalarSin(aDiff);
    TR2Q[12] = -kC->l7 * muDoubleScalarCos(aDiff);
    TR2Q[1] = muDoubleScalarSin(aDiff);
    TR2Q[5] = muDoubleScalarCos(aDiff) * 6.123233995736766E-17;
    TR2Q[9] = -(-muDoubleScalarCos(aDiff));
    TR2Q[13] = -kC->l7 * muDoubleScalarSin(aDiff);
    for (i6 = 0; i6 < 4; i6++) {
      TO2S[3 + (i6 << 2)] = iv8[i6];
      TI2S[2 + (i6 << 2)] = iv9[i6];
      TI2S[3 + (i6 << 2)] = iv8[i6];
      TO2J[2 + (i6 << 2)] = iv9[i6];
      TO2J[3 + (i6 << 2)] = iv8[i6];
      TQ2O[2 + (i6 << 2)] = iv9[i6];
      TQ2O[3 + (i6 << 2)] = iv8[i6];
      TR2Q[2 + (i6 << 2)] = dv3[i6];
      TR2Q[3 + (i6 << 2)] = iv8[i6];

      /* TRDH Generates the homogeneous transformation matrix A using the  */
      /* Denavit-Hartenberg parameters theta, d, a and alpha. */
      /*  */
      /* trDH.m */
      /* author:    wreid */
      /* date:      20150214 */
      TS2R[i6 << 2] = iv10[i6];
      TS2R[1 + (i6 << 2)] = iv11[i6];
    }

    TS2R[2] = 0.0;
    TS2R[6] = 0.0;
    TS2R[10] = 1.0;
    TS2R[14] = kC->l6;
    for (i6 = 0; i6 < 4; i6++) {
      TS2R[3 + (i6 << 2)] = iv8[i6];
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
    for (i6 = 0; i6 < 4; i6++) {
      TP2S[3 + (i6 << 2)] = iv8[i6];
    }

    for (i6 = 0; i6 < 4; i6++) {
      for (ixstart = 0; ixstart < 4; ixstart++) {
        TB2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          TB2S[i6 + (ixstart << 2)] += TP2S[i6 + (ix << 2)] * TO2S[ix + (ixstart
            << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        dv5[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          dv5[i6 + (ixstart << 2)] += TB2S[i6 + (ix << 2)] * TI2S[ix + (ixstart <<
            2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        dv6[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          dv6[i6 + (ixstart << 2)] += dv5[i6 + (ix << 2)] * TO2J[ix + (ixstart <<
            2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        dv7[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          dv7[i6 + (ixstart << 2)] += dv6[i6 + (ix << 2)] * TQ2O[ix + (ixstart <<
            2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        dv8[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          dv8[i6 + (ixstart << 2)] += dv7[i6 + (ix << 2)] * TR2Q[ix + (ixstart <<
            2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        dv4[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          dv4[i6 + (ixstart << 2)] += dv8[i6 + (ix << 2)] * TS2R[ix + (ixstart <<
            2)];
        }

        b_TO2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          b_TO2S[i6 + (ixstart << 2)] += TO2S[i6 + (ix << 2)] * TI2S[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        c_TO2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          c_TO2S[i6 + (ixstart << 2)] += b_TO2S[i6 + (ix << 2)] * TO2J[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        d_TO2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          d_TO2S[i6 + (ixstart << 2)] += c_TO2S[i6 + (ix << 2)] * TQ2O[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        e_TO2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          e_TO2S[i6 + (ixstart << 2)] += d_TO2S[i6 + (ix << 2)] * TR2Q[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        f_TO2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          f_TO2S[i6 + (ixstart << 2)] += e_TO2S[i6 + (ix << 2)] * TS2R[ix +
            (ixstart << 2)];
        }

        b_TI2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          b_TI2S[i6 + (ixstart << 2)] += TI2S[i6 + (ix << 2)] * TO2J[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        c_TI2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          c_TI2S[i6 + (ixstart << 2)] += b_TI2S[i6 + (ix << 2)] * TQ2O[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        d_TI2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          d_TI2S[i6 + (ixstart << 2)] += c_TI2S[i6 + (ix << 2)] * TR2Q[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        e_TI2S[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          e_TI2S[i6 + (ixstart << 2)] += d_TI2S[i6 + (ix << 2)] * TS2R[ix +
            (ixstart << 2)];
        }

        b_TQ2O[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          b_TQ2O[i6 + (ixstart << 2)] += TQ2O[i6 + (ix << 2)] * TR2Q[ix +
            (ixstart << 2)];
        }
      }

      for (ixstart = 0; ixstart < 4; ixstart++) {
        c_TQ2O[i6 + (ixstart << 2)] = 0.0;
        for (ix = 0; ix < 4; ix++) {
          c_TQ2O[i6 + (ixstart << 2)] += b_TQ2O[i6 + (ix << 2)] * TS2R[ix +
            (ixstart << 2)];
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
    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        dv10[i6 + 3 * ixstart] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          dv10[i6 + 3 * ixstart] += dv9[i6 + 3 * ix] * TB2S[ix + (ixstart << 2)];
        }

        AdB2S[ixstart + 6 * i6] = TB2S[ixstart + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdB2S[ixstart + 6 * (i6 + 3)] = dv10[ixstart + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdB2S[(ixstart + 6 * i6) + 3] = 0.0;
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
    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdB2S[(ixstart + 6 * (i6 + 3)) + 3] = TB2S[ixstart + (i6 << 2)];
        dv10[i6 + 3 * ixstart] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          dv10[i6 + 3 * ixstart] += dv11[i6 + 3 * ix] * TP2S[ix + (ixstart << 2)];
        }

        AdP2S[ixstart + 6 * i6] = TP2S[ixstart + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdP2S[ixstart + 6 * (i6 + 3)] = dv10[ixstart + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdP2S[(ixstart + 6 * i6) + 3] = 0.0;
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
    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdP2S[(ixstart + 6 * (i6 + 3)) + 3] = TP2S[ixstart + (i6 << 2)];
        dv10[i6 + 3 * ixstart] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          dv10[i6 + 3 * ixstart] += dv12[i6 + 3 * ix] * TI2S[ix + (ixstart << 2)];
        }

        AdI2S[ixstart + 6 * i6] = TI2S[ixstart + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdI2S[ixstart + 6 * (i6 + 3)] = dv10[ixstart + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdI2S[(ixstart + 6 * i6) + 3] = 0.0;
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
    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdI2S[(ixstart + 6 * (i6 + 3)) + 3] = TI2S[ixstart + (i6 << 2)];
        dv10[i6 + 3 * ixstart] = 0.0;
        for (ix = 0; ix < 3; ix++) {
          dv10[i6 + 3 * ixstart] += dv13[i6 + 3 * ix] * TO2S[ix + (ixstart << 2)];
        }

        AdO2S[ixstart + 6 * i6] = TO2S[ixstart + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdO2S[ixstart + 6 * (i6 + 3)] = dv10[ixstart + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdO2S[(ixstart + 6 * i6) + 3] = 0.0;
      }
    }

    /* Pan joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i6 = 0; i6 < 3; i6++) {
      for (ixstart = 0; ixstart < 3; ixstart++) {
        AdO2S[(ixstart + 6 * (i6 + 3)) + 3] = TO2S[ixstart + (i6 << 2)];
      }

      b_AdB2S[i6] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = candStates_data[i + 30];
    for (i6 = 0; i6 < 6; i6++) {
      uPDot[i6] = b_AdB2S[i6];
    }

    /* Beta joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i6 = 0; i6 < 3; i6++) {
      b_AdB2S[i6] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = candStates_data[i + 35];
    for (i6 = 0; i6 < 6; i6++) {
      uIDot[i6] = b_AdB2S[i6];
    }

    /* Gamma joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i6 = 0; i6 < 3; i6++) {
      b_AdB2S[i6] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = candStates_data[i + 40];

    /* Velocity vector for the ankle frame. */
    for (i6 = 0; i6 < 6; i6++) {
      uODot[i6] = b_AdB2S[i6];
      c_AdB2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        c_AdB2S[i6] += AdB2S[i6 + 6 * ixstart] * uBDot[ixstart];
      }

      b_AdP2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        b_AdP2S[i6] += AdP2S[i6 + 6 * ixstart] * uPDot[ixstart];
      }
    }

    for (i6 = 0; i6 < 6; i6++) {
      d1 = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        d1 += AdI2S[i6 + 6 * ixstart] * uIDot[ixstart];
      }

      b_AdB2S[i6] = (c_AdB2S[i6] + b_AdP2S[i6]) + d1;
    }

    for (i6 = 0; i6 < 6; i6++) {
      b_AdO2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        b_AdO2S[i6] += AdO2S[i6 + 6 * ixstart] * uODot[ixstart];
      }

      uSDot[i6] = b_AdB2S[i6] + b_AdO2S[i6];
      c_AdB2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        c_AdB2S[i6] += AdB2S[i6 + 6 * ixstart] * uBDot[ixstart];
      }

      b_AdP2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        b_AdP2S[i6] += AdP2S[i6 + 6 * ixstart] * uPDot[ixstart];
      }
    }

    for (i6 = 0; i6 < 6; i6++) {
      d1 = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        d1 += AdI2S[i6 + 6 * ixstart] * uIDot[ixstart];
      }

      b_AdB2S[i6] = (c_AdB2S[i6] + b_AdP2S[i6]) + d1;
    }

    for (i6 = 0; i6 < 6; i6++) {
      b_AdO2S[i6] = 0.0;
      for (ixstart = 0; ixstart < 6; ixstart++) {
        b_AdO2S[i6] += AdO2S[i6 + 6 * ixstart] * uODot[ixstart];
      }

      c_AdB2S[i6] = b_AdB2S[i6] + b_AdO2S[i6];
    }

    for (i6 = 0; i6 < 3; i6++) {
      vS[i6] = c_AdB2S[i6];
    }

    /* [m/s] */
    /* [rad/s] */
    /* Calculate the required phi joint angle and the required wheel speed, */
    /* omega. */
    /* [rad] */
    aDiff = ((muDoubleScalarAtan2(uSDot[1], uSDot[0]) - 1.5707963267948966) +
             3.1415926535897931) / 6.2831853071795862;
    if (muDoubleScalarAbs(aDiff - muDoubleScalarRound(aDiff)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(aDiff)) {
      aDiff = 0.0;
    } else {
      aDiff = (aDiff - muDoubleScalarFloor(aDiff)) * 6.2831853071795862;
    }

    d1 = aDiff - 3.1415926535897931;
    ixstart = 1;
    if (aDiff - 3.1415926535897931 > 1.5707963267948966) {
      d1 = (aDiff - 3.1415926535897931) - 3.1415926535897931;
      ixstart = -1;
    } else {
      if (aDiff - 3.1415926535897931 <= -1.5707963267948966) {
        d1 = (aDiff - 3.1415926535897931) + 3.1415926535897931;
        ixstart = -1;
      }
    }

    aDiff = 0.0;
    scale = 2.2250738585072014E-308;
    for (ix = 0; ix < 3; ix++) {
      absxk = muDoubleScalarAbs(vS[ix]);
      if (absxk > scale) {
        t = scale / absxk;
        aDiff = 1.0 + aDiff * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        aDiff += t * t;
      }
    }

    aDiff = scale * muDoubleScalarSqrt(aDiff);

    /* [rad/s] */
    candStates_data[i + 45] = d1;
    candStates_data[i + 50] = (real_T)ixstart * aDiff / kC->r;

    /* angDiff Finds the angular difference between th1 and th2. */
    aDiff = ((xNear_data[10] - candStates_data[i + 45]) + 3.1415926535897931) /
      6.2831853071795862;
    if (muDoubleScalarAbs(aDiff - muDoubleScalarRound(aDiff)) <=
        2.2204460492503131E-16 * muDoubleScalarAbs(aDiff)) {
      aDiff = 0.0;
    } else {
      aDiff = (aDiff - muDoubleScalarFloor(aDiff)) * 6.2831853071795862;
    }

    aDiff = muDoubleScalarAbs(aDiff - 3.1415926535897931);
    if (aDiff > 0.39269908169872414) {
      aDiff = 1.0;
    } else {
      aDiff /= 1.5707963267948966;
    }

    /* Calculate a distance metric that includes the heurisitc distance */
    /* as well as any penalty due to ankle movements. */
    distance_data[i] = (1.0 - HGAINS[2]) * hDiff + HGAINS[2] * aDiff;

    /* distance(i) = hDiff; */
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  emxFree_int32_T(&r2);
  emxFree_real_T(&r1);
  st.site = &ob_emlrtRSI;
  b_st.site = &fb_emlrtRSI;
  ixstart = 1;
  aDiff = distance_data[0];
  i = 0;
  if (muDoubleScalarIsNaN(distance_data[0])) {
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix + 1 <= 5)) {
      ixstart = ix + 1;
      if (!muDoubleScalarIsNaN(distance_data[ix])) {
        aDiff = distance_data[ix];
        i = ix;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  if (ixstart < 5) {
    while (ixstart + 1 <= 5) {
      if (distance_data[ixstart] < aDiff) {
        aDiff = distance_data[ixstart];
        i = ixstart;
      }

      ixstart++;
    }
  }

  xNew_size[0] = 1;
  xNew_size[1] = 11;
  for (i6 = 0; i6 < 11; i6++) {
    xNew_data[xNew_size[0] * i6] = candStates_data[i + 5 * i6];
  }

  ixstart = candTransArrays->size[1];
  i6 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = ixstart;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i6, (int32_T)sizeof
                    (real_T), &e_emlrtRTEI);
  for (i6 = 0; i6 < ixstart; i6++) {
    transitionArray->data[transitionArray->size[0] * i6] = candTransArrays->
      data[i + candTransArrays->size[0] * i6];
  }

  emxFree_real_T(&candTransArrays);

  /* velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst) */
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (selectInput.c) */
