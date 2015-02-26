/*
 * rk4.c
 *
 * Code generation for function 'rk4'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "rk4.h"
#include "buildRRTWrapper_emxutil.h"
#include "trInv.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo sb_emlrtRSI = { 112, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtRSInfo tb_emlrtRSI = { 6, "getPhiAndOmega",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getPhiAndOmega.m"
};

static emlrtRTEInfo n_emlrtRTEI = { 5, 35, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo eb_emlrtBCI = { 1, 4, 42, 17, "kC.legAngleOffset",
  "generateTrMatrices",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/generateTrMatrices.m",
  0 };

static emlrtRTEInfo s_emlrtRTEI = { 18, 5, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtBCInfo fb_emlrtBCI = { -1, -1, 139, 9, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

static emlrtECInfo e_emlrtECI = { -1, 139, 9, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m"
};

static emlrtDCInfo g_emlrtDCI = { 15, 31, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  1 };

static emlrtDCInfo h_emlrtDCI = { 15, 31, "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  4 };

static emlrtBCInfo gb_emlrtBCI = { -1, -1, 16, 5, "transitionArray", "rk4",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/rk4.m",
  0 };

/* Function Declarations */
static void f(const real_T x[10], const real_T u[2], real_T kC_l3, real_T kC_l5,
              real_T kC_zeta, real_T xDot[10]);

/* Function Definitions */
static void f(const real_T x[10], const real_T u[2], real_T kC_l3, real_T kC_l5,
              real_T kC_zeta, real_T xDot[10])
{
  /* gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); */
  /* GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of */
  /* gamma given a pan height constraint and an independent beta angle. */
  xDot[0] = x[5];
  xDot[1] = x[6];
  xDot[2] = x[7];
  xDot[3] = 0.0;
  xDot[4] = 0.0;
  xDot[5] = u[0];
  xDot[6] = u[1];
  xDot[7] = ((-u[1] * kC_l3 * muDoubleScalarCos(x[1]) + x[6] * x[6] * kC_l3 *
              muDoubleScalarSin(x[1])) + x[7] * x[7] * kC_l5 * muDoubleScalarSin
             (kC_zeta + x[2])) / (kC_l5 * muDoubleScalarCos(kC_zeta + x[2]));
  xDot[8] = 0.0;
  xDot[9] = 0.0;
}

void rk4(const emlrtStack *sp, const real_T uIn[2], const real_T uBDot[6],
         real_T dt, real_T Dt, real_T xInit_data[], const real_T jointLimits[20],
         const struct0_T *kC, int32_T legNum, real_T xNew_data[], int32_T
         xNew_size[2], emxArray_real_T *transitionArray)
{
  real_T u[2];
  int32_T i6;
  real_T numIterations;
  real_T b_xInit_data[13];
  int32_T xInit_size_idx_1;
  real_T absxk;
  int32_T loop_ub;
  int32_T unnamed_idx_1;
  int32_T i;
  real_T c_xInit_data[10];
  real_T k2[10];
  real_T k3[10];
  real_T d_xInit_data[10];
  real_T e_xInit_data[10];
  real_T f_xInit_data[10];
  real_T g_xInit_data[10];
  real_T k1[10];
  real_T y;
  real_T scale;
  real_T alpha;
  real_T beta;
  real_T b_gamma;
  real_T alphaDot;
  real_T betaDot;
  real_T gammaDot;
  real_T alphaDotDot;
  real_T betaDotDot;
  real_T TO2S[16];
  real_T TI2S[16];
  real_T TO2J[16];
  real_T TQ2O[16];
  real_T TR2Q[16];
  static const int8_T iv8[4] = { 0, 0, 0, 1 };

  static const int8_T iv9[4] = { 0, 0, 1, 0 };

  static const real_T dv2[4] = { 0.0, -1.0, 6.123233995736766E-17, 0.0 };

  static const int8_T iv10[4] = { 1, 0, 0, 0 };

  real_T TS2R[16];
  static const int8_T iv11[4] = { 0, 1, 0, 0 };

  real_T dv3[16];
  real_T TP2S[16];
  real_T TB2S[16];
  real_T dv4[16];
  real_T dv5[16];
  real_T dv6[16];
  real_T dv7[16];
  int32_T i7;
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
  real_T dv8[9];
  real_T dv9[9];
  real_T AdB2S[36];
  real_T dv10[9];
  real_T AdP2S[36];
  real_T dv11[9];
  real_T AdI2S[36];
  real_T dv12[9];
  real_T AdO2S[36];
  real_T b_AdB2S[6];
  real_T uPDot[6];
  real_T uIDot[6];
  real_T c_AdB2S[6];
  real_T b_AdP2S[6];
  real_T uODot[6];
  real_T b_AdO2S[6];
  real_T uSDot[6];
  real_T vS[3];
  real_T r;
  real_T t;
  real_T b_alpha[10];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* rk4.m */
  /* author: wreid */
  /* date: 20150107 */
  /* rk4 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i6 = 0; i6 < 2; i6++) {
    u[i6] = uIn[i6];
  }

  numIterations = muDoubleScalarRound(Dt / dt);
  xNew_size[0] = 1;
  xNew_size[1] = 13;
  for (i6 = 0; i6 < 13; i6++) {
    xNew_data[i6] = 0.0;
  }

  for (i6 = 0; i6 < 10; i6++) {
    b_xInit_data[i6] = xInit_data[3 + i6];
  }

  xInit_size_idx_1 = 10;
  for (i6 = 0; i6 < 10; i6++) {
    xInit_data[i6] = b_xInit_data[i6];
  }

  i6 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  absxk = (numIterations + 1.0) * 10.0;
  absxk = emlrtNonNegativeCheckFastR2012b(absxk, &h_emlrtDCI, sp);
  transitionArray->size[1] = (int32_T)emlrtIntegerCheckFastR2012b(absxk,
    &g_emlrtDCI, sp);
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i6, (int32_T)sizeof
                    (real_T), &n_emlrtRTEI);
  absxk = (numIterations + 1.0) * 10.0;
  absxk = emlrtNonNegativeCheckFastR2012b(absxk, &h_emlrtDCI, sp);
  loop_ub = (int32_T)emlrtIntegerCheckFastR2012b(absxk, &g_emlrtDCI, sp);
  for (i6 = 0; i6 < loop_ub; i6++) {
    transitionArray->data[i6] = 0.0;
  }

  unnamed_idx_1 = (int32_T)((numIterations + 1.0) * 10.0);
  for (i6 = 0; i6 < 10; i6++) {
    transitionArray->data[emlrtDynamicBoundsCheckFastR2012b(i6 + 1, 1,
      unnamed_idx_1, &gb_emlrtBCI, sp) - 1] = xInit_data[i6];
  }

  emlrtForLoopVectorCheckR2012b(1.0, 1.0, numIterations, mxDOUBLE_CLASS,
    (int32_T)numIterations, &s_emlrtRTEI, sp);
  i = 0;
  while (i <= (int32_T)numIterations - 1) {
    /* gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma)); */
    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      c_xInit_data[i6] = xInit_data[i6];
    }

    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      k2[i6] = xInit_data[i6];
    }

    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      k3[i6] = xInit_data[i6];
    }

    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      d_xInit_data[i6] = xInit_data[i6];
    }

    /* GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of */
    /* gamma given a pan height constraint and an independent beta angle. */
    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      e_xInit_data[i6] = xInit_data[i6];
    }

    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      f_xInit_data[i6] = xInit_data[i6];
    }

    for (i6 = 0; i6 < xInit_size_idx_1; i6++) {
      g_xInit_data[i6] = xInit_data[i6];
    }

    k1[0] = e_xInit_data[5];
    k1[1] = f_xInit_data[6];
    k1[2] = g_xInit_data[7];
    k1[3] = 0.0;
    k1[4] = 0.0;
    k1[5] = u[0];
    k1[6] = u[1];
    k1[7] = ((-u[1] * kC->l3 * muDoubleScalarCos(k3[1]) + c_xInit_data[6] *
              c_xInit_data[6] * kC->l3 * muDoubleScalarSin(k3[1])) + k2[7] * k2
             [7] * kC->l5 * muDoubleScalarSin(kC->zeta + d_xInit_data[2])) /
      (kC->l5 * muDoubleScalarCos(kC->zeta + d_xInit_data[2]));
    k1[8] = 0.0;
    k1[9] = 0.0;
    y = dt / 2.0;
    for (i6 = 0; i6 < 10; i6++) {
      c_xInit_data[i6] = xInit_data[i6] + y * k1[i6];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k2);
    y = dt / 2.0;
    for (i6 = 0; i6 < 10; i6++) {
      c_xInit_data[i6] = xInit_data[i6] + y * k2[i6];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, k3);
    y = dt / 2.0;
    scale = dt / 6.0;
    for (i6 = 0; i6 < 10; i6++) {
      c_xInit_data[i6] = xInit_data[i6] + y * k3[i6];
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, d_xInit_data);
    for (i6 = 0; i6 < 10; i6++) {
      c_xInit_data[i6] = xInit_data[i6] + y * k3[i6];
      d_xInit_data[i6] = xInit_data[i6] + scale * (((k1[i6] + 2.0 * k2[i6]) +
        2.0 * k3[i6]) + d_xInit_data[i6]);
    }

    f(c_xInit_data, u, kC->l3, kC->l5, kC->zeta, e_xInit_data);
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i6 = 0; i6 < 10; i6++) {
      xNew_data[i6] = xInit_data[i6] + scale * (((k1[i6] + 2.0 * k2[i6]) + 2.0 *
        k3[i6]) + e_xInit_data[i6]);
    }

    alpha = d_xInit_data[0];
    beta = d_xInit_data[1];
    b_gamma = d_xInit_data[2];
    alphaDot = d_xInit_data[5];
    betaDot = d_xInit_data[6];
    gammaDot = d_xInit_data[7];
    alphaDotDot = u[0];
    betaDotDot = u[1];

    /* Check pan angular position limits */
    if ((xNew_data[0] > jointLimits[1]) || (xNew_data[0] < jointLimits[0])) {
      alpha = xInit_data[0];
      alphaDot = 0.0;
      alphaDotDot = 0.0;
    }

    /* Check inner and outer leg angular position limits */
    if ((xNew_data[1] > jointLimits[3]) || (xNew_data[1] < jointLimits[2]) ||
        (xNew_data[2] > jointLimits[5]) || (xNew_data[2] < jointLimits[4])) {
      beta = xInit_data[1];
      b_gamma = xInit_data[2];
      betaDot = 0.0;
      gammaDot = 0.0;
      betaDotDot = 0.0;
    }

    /* Check pan angular velocity limits */
    if ((alphaDot > jointLimits[11]) || (alphaDot < jointLimits[10])) {
      alphaDot = xInit_data[5];
      alphaDotDot = 0.0;

      /* else */
      /*     alphaDotDot = uIn(1); */
    }

    /* Check the inner leg velocity limit. */
    if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
      betaDot = xInit_data[6];
      betaDotDot = 0.0;

      /* GETCONSTRAINEDGAMMADOT This function calculates the velocity of */
      /* gamma given a pan height constraint and an independent beta angle. */
      /*  */
      /* getConstrainedGammaDot.m */
      /* author: wreid */
      /* date: 20150224 */
      gammaDot = -xInit_data[6] * kC->l3 * muDoubleScalarCos(beta) / (kC->l5 *
        muDoubleScalarCos(kC->zeta + b_gamma));
    }

    /* Check the outer leg velocity limit. */
    if ((gammaDot > jointLimits[15]) || (gammaDot < jointLimits[14])) {
      gammaDot = xInit_data[7];

      /* GETCONSTRAINEDBETAADOT This function calculates the velocity of */
      /* gamma given a pan height constraint and an independent beta angle. */
      /*  */
      /* getConstrainedBetaDot.m */
      /* author: wreid */
      /* date: 20150224 */
      betaDot = -xInit_data[7] * kC->l5 * muDoubleScalarCos(kC->zeta + b_gamma) /
        (kC->l3 * muDoubleScalarCos(beta));

      /* GETCONSTRAINEDBETADOTDOT This function calculates the acceleration of */
      /* beta given a pan height constraint and an indpendent gamma angle. */
      betaDotDot = ((-0.0 * kC->l5 * muDoubleScalarCos(kC->zeta + b_gamma) +
                     xInit_data[7] * xInit_data[7] * kC->l5 * muDoubleScalarSin
                     (kC->zeta + b_gamma)) - betaDot * betaDot * kC->l3 *
                    muDoubleScalarSin(beta)) / (kC->l3 * muDoubleScalarCos(beta));
      if ((betaDot > jointLimits[13]) || (betaDot < jointLimits[12])) {
        betaDot = xInit_data[6];
        betaDotDot = 0.0;
      }
    }

    /*          if betaDot > betaDotMax || betaDot < betaDotMin || gammaDot > gammaDotMax || gammaDot < gammaDotMin */
    /*              betaDot = xInit(7); */
    /*              gammaDot = xInit(8); */
    /*              betaDotDot = 0; */
    /*          else */
    /*             betaDotDot = uIn(2); */
    /*          end */
    /* Check the outer leg velocity limit. */
    st.site = &sb_emlrtRSI;

    /* Homogeneous transformation matrices. */
    b_st.site = &tb_emlrtRSI;

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
    TO2S[0] = muDoubleScalarCos(alpha);
    TO2S[4] = -muDoubleScalarSin(alpha) * 6.123233995736766E-17;
    TO2S[8] = -muDoubleScalarSin(alpha);
    TO2S[12] = kC->l2 * muDoubleScalarCos(alpha);
    TO2S[1] = muDoubleScalarSin(alpha);
    TO2S[5] = muDoubleScalarCos(alpha) * 6.123233995736766E-17;
    TO2S[9] = -(-muDoubleScalarCos(alpha));
    TO2S[13] = kC->l2 * muDoubleScalarSin(alpha);
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
    TI2S[0] = muDoubleScalarCos(beta);
    TI2S[4] = -muDoubleScalarSin(beta);
    TI2S[8] = muDoubleScalarSin(beta) * 0.0;
    TI2S[12] = kC->l3 * muDoubleScalarCos(beta);
    TI2S[1] = muDoubleScalarSin(beta);
    TI2S[5] = muDoubleScalarCos(beta);
    TI2S[9] = -muDoubleScalarCos(beta) * 0.0;
    TI2S[13] = kC->l3 * muDoubleScalarSin(beta);
    scale = -beta + kC->zeta;

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
    TQ2O[0] = muDoubleScalarCos(b_gamma);
    TQ2O[4] = -muDoubleScalarSin(b_gamma);
    TQ2O[8] = muDoubleScalarSin(b_gamma) * 0.0;
    TQ2O[12] = kC->l5 * muDoubleScalarCos(b_gamma);
    TQ2O[1] = muDoubleScalarSin(b_gamma);
    TQ2O[5] = muDoubleScalarCos(b_gamma);
    TQ2O[9] = -muDoubleScalarCos(b_gamma) * 0.0;
    TQ2O[13] = kC->l5 * muDoubleScalarSin(b_gamma);
    scale = -b_gamma - kC->zeta;

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
    for (i6 = 0; i6 < 4; i6++) {
      TO2S[3 + (i6 << 2)] = iv8[i6];
      TI2S[2 + (i6 << 2)] = iv9[i6];
      TI2S[3 + (i6 << 2)] = iv8[i6];
      TO2J[2 + (i6 << 2)] = iv9[i6];
      TO2J[3 + (i6 << 2)] = iv8[i6];
      TQ2O[2 + (i6 << 2)] = iv9[i6];
      TQ2O[3 + (i6 << 2)] = iv8[i6];
      TR2Q[2 + (i6 << 2)] = dv2[i6];
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
      for (i7 = 0; i7 < 4; i7++) {
        TB2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          TB2S[i6 + (i7 << 2)] += TP2S[i6 + (unnamed_idx_1 << 2)] *
            TO2S[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        dv4[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          dv4[i6 + (i7 << 2)] += TB2S[i6 + (unnamed_idx_1 << 2)] *
            TI2S[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        dv5[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          dv5[i6 + (i7 << 2)] += dv4[i6 + (unnamed_idx_1 << 2)] *
            TO2J[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        dv6[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          dv6[i6 + (i7 << 2)] += dv5[i6 + (unnamed_idx_1 << 2)] *
            TQ2O[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        dv7[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          dv7[i6 + (i7 << 2)] += dv6[i6 + (unnamed_idx_1 << 2)] *
            TR2Q[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        dv3[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          dv3[i6 + (i7 << 2)] += dv7[i6 + (unnamed_idx_1 << 2)] *
            TS2R[unnamed_idx_1 + (i7 << 2)];
        }

        b_TO2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          b_TO2S[i6 + (i7 << 2)] += TO2S[i6 + (unnamed_idx_1 << 2)] *
            TI2S[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        c_TO2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          c_TO2S[i6 + (i7 << 2)] += b_TO2S[i6 + (unnamed_idx_1 << 2)] *
            TO2J[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        d_TO2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          d_TO2S[i6 + (i7 << 2)] += c_TO2S[i6 + (unnamed_idx_1 << 2)] *
            TQ2O[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        e_TO2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          e_TO2S[i6 + (i7 << 2)] += d_TO2S[i6 + (unnamed_idx_1 << 2)] *
            TR2Q[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        f_TO2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          f_TO2S[i6 + (i7 << 2)] += e_TO2S[i6 + (unnamed_idx_1 << 2)] *
            TS2R[unnamed_idx_1 + (i7 << 2)];
        }

        b_TI2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          b_TI2S[i6 + (i7 << 2)] += TI2S[i6 + (unnamed_idx_1 << 2)] *
            TO2J[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        c_TI2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          c_TI2S[i6 + (i7 << 2)] += b_TI2S[i6 + (unnamed_idx_1 << 2)] *
            TQ2O[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        d_TI2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          d_TI2S[i6 + (i7 << 2)] += c_TI2S[i6 + (unnamed_idx_1 << 2)] *
            TR2Q[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        e_TI2S[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          e_TI2S[i6 + (i7 << 2)] += d_TI2S[i6 + (unnamed_idx_1 << 2)] *
            TS2R[unnamed_idx_1 + (i7 << 2)];
        }

        b_TQ2O[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          b_TQ2O[i6 + (i7 << 2)] += TQ2O[i6 + (unnamed_idx_1 << 2)] *
            TR2Q[unnamed_idx_1 + (i7 << 2)];
        }
      }

      for (i7 = 0; i7 < 4; i7++) {
        c_TQ2O[i6 + (i7 << 2)] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 4; unnamed_idx_1++) {
          c_TQ2O[i6 + (i7 << 2)] += b_TQ2O[i6 + (unnamed_idx_1 << 2)] *
            TS2R[unnamed_idx_1 + (i7 << 2)];
        }
      }
    }

    trInv(dv3, TB2S);
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
    dv8[0] = 0.0;
    dv8[3] = -TB2S[14];
    dv8[6] = TB2S[13];
    dv8[1] = TB2S[14];
    dv8[4] = 0.0;
    dv8[7] = -TB2S[12];
    dv8[2] = -TB2S[13];
    dv8[5] = TB2S[12];
    dv8[8] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        dv9[i6 + 3 * i7] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
          dv9[i6 + 3 * i7] += dv8[i6 + 3 * unnamed_idx_1] * TB2S[unnamed_idx_1 +
            (i7 << 2)];
        }

        AdB2S[i7 + 6 * i6] = TB2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdB2S[i7 + 6 * (i6 + 3)] = dv9[i7 + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdB2S[(i7 + 6 * i6) + 3] = 0.0;
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
    dv10[0] = 0.0;
    dv10[3] = -TP2S[14];
    dv10[6] = TP2S[13];
    dv10[1] = TP2S[14];
    dv10[4] = 0.0;
    dv10[7] = -TP2S[12];
    dv10[2] = -TP2S[13];
    dv10[5] = TP2S[12];
    dv10[8] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdB2S[(i7 + 6 * (i6 + 3)) + 3] = TB2S[i7 + (i6 << 2)];
        dv9[i6 + 3 * i7] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
          dv9[i6 + 3 * i7] += dv10[i6 + 3 * unnamed_idx_1] * TP2S[unnamed_idx_1
            + (i7 << 2)];
        }

        AdP2S[i7 + 6 * i6] = TP2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdP2S[i7 + 6 * (i6 + 3)] = dv9[i7 + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdP2S[(i7 + 6 * i6) + 3] = 0.0;
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
    dv11[3] = -TI2S[14];
    dv11[6] = TI2S[13];
    dv11[1] = TI2S[14];
    dv11[4] = 0.0;
    dv11[7] = -TI2S[12];
    dv11[2] = -TI2S[13];
    dv11[5] = TI2S[12];
    dv11[8] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdP2S[(i7 + 6 * (i6 + 3)) + 3] = TP2S[i7 + (i6 << 2)];
        dv9[i6 + 3 * i7] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
          dv9[i6 + 3 * i7] += dv11[i6 + 3 * unnamed_idx_1] * TI2S[unnamed_idx_1
            + (i7 << 2)];
        }

        AdI2S[i7 + 6 * i6] = TI2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdI2S[i7 + 6 * (i6 + 3)] = dv9[i7 + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdI2S[(i7 + 6 * i6) + 3] = 0.0;
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
    dv12[3] = -TO2S[14];
    dv12[6] = TO2S[13];
    dv12[1] = TO2S[14];
    dv12[4] = 0.0;
    dv12[7] = -TO2S[12];
    dv12[2] = -TO2S[13];
    dv12[5] = TO2S[12];
    dv12[8] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdI2S[(i7 + 6 * (i6 + 3)) + 3] = TI2S[i7 + (i6 << 2)];
        dv9[i6 + 3 * i7] = 0.0;
        for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
          dv9[i6 + 3 * i7] += dv12[i6 + 3 * unnamed_idx_1] * TO2S[unnamed_idx_1
            + (i7 << 2)];
        }

        AdO2S[i7 + 6 * i6] = TO2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdO2S[i7 + 6 * (i6 + 3)] = dv9[i7 + 3 * i6];
      }
    }

    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdO2S[(i7 + 6 * i6) + 3] = 0.0;
      }
    }

    /* Pan joint rate */
    /* [rad/s] */
    /* [m/s] */
    /* [rad/s] */
    for (i6 = 0; i6 < 3; i6++) {
      for (i7 = 0; i7 < 3; i7++) {
        AdO2S[(i7 + 6 * (i6 + 3)) + 3] = TO2S[i7 + (i6 << 2)];
      }

      b_AdB2S[i6] = 0.0;
    }

    b_AdB2S[3] = 0.0;
    b_AdB2S[4] = 0.0;
    b_AdB2S[5] = alphaDot;
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
    b_AdB2S[5] = betaDot;
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
    b_AdB2S[5] = gammaDot;

    /* Velocity vector for the ankle frame. */
    for (i6 = 0; i6 < 6; i6++) {
      uODot[i6] = b_AdB2S[i6];
      c_AdB2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        c_AdB2S[i6] += AdB2S[i6 + 6 * i7] * uBDot[i7];
      }

      b_AdP2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        b_AdP2S[i6] += AdP2S[i6 + 6 * i7] * uPDot[i7];
      }
    }

    for (i6 = 0; i6 < 6; i6++) {
      absxk = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        absxk += AdI2S[i6 + 6 * i7] * uIDot[i7];
      }

      b_AdB2S[i6] = (c_AdB2S[i6] + b_AdP2S[i6]) + absxk;
    }

    for (i6 = 0; i6 < 6; i6++) {
      b_AdO2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        b_AdO2S[i6] += AdO2S[i6 + 6 * i7] * uODot[i7];
      }

      uSDot[i6] = b_AdB2S[i6] + b_AdO2S[i6];
      c_AdB2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        c_AdB2S[i6] += AdB2S[i6 + 6 * i7] * uBDot[i7];
      }

      b_AdP2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        b_AdP2S[i6] += AdP2S[i6 + 6 * i7] * uPDot[i7];
      }
    }

    for (i6 = 0; i6 < 6; i6++) {
      absxk = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        absxk += AdI2S[i6 + 6 * i7] * uIDot[i7];
      }

      b_AdB2S[i6] = (c_AdB2S[i6] + b_AdP2S[i6]) + absxk;
    }

    for (i6 = 0; i6 < 6; i6++) {
      b_AdO2S[i6] = 0.0;
      for (i7 = 0; i7 < 6; i7++) {
        b_AdO2S[i6] += AdO2S[i6 + 6 * i7] * uODot[i7];
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
    r = ((muDoubleScalarAtan2(uSDot[1], uSDot[0]) - 1.5707963267948966) +
         3.1415926535897931) / 6.2831853071795862;
    if (muDoubleScalarAbs(r - muDoubleScalarRound(r)) <= 2.2204460492503131E-16 *
        muDoubleScalarAbs(r)) {
      r = 0.0;
    } else {
      r = (r - muDoubleScalarFloor(r)) * 6.2831853071795862;
    }

    /*      if phiDiff > pi/2 */
    /*          phi = phi - pi; */
    /*          phi = mod(phi+pi,2*pi)-pi; */
    /*          sign = -1; */
    /*      end */
    y = 0.0;
    scale = 2.2250738585072014E-308;
    for (unnamed_idx_1 = 0; unnamed_idx_1 < 3; unnamed_idx_1++) {
      absxk = muDoubleScalarAbs(vS[unnamed_idx_1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * muDoubleScalarSqrt(y);
    t = y / kC->r;

    /* [rad/s] */
    /*          %Check if phi is above threshold, if so then stop the leg and drive */
    /*          %the steering joint until it is in the correct orientation. */
    /*          if abs(phi-phiInit)/dt > phiDotMax */
    /*            if phi > phiInit */
    /*              phiDot = phiDotMax; */
    /*            else */
    /*              phiDot = phiDotMin;   */
    /*            end */
    /*            alphaDotDot = 0; */
    /*            betaDotDot = 0; */
    /*            alphaDot = 0; */
    /*            betaDot = 0; */
    /*            gammaDot = 0; */
    /*            omega = 0; */
    /*            alpha = xInitOrig(1); */
    /*            beta = xInitOrig(2); */
    /*            gamma = xInitOrig(3); */
    /*          end */
    u[0] = alphaDotDot;
    u[1] = betaDotDot;
    scale = xNew_data[4];
    absxk = xNew_data[8];
    b_alpha[0] = alpha;
    b_alpha[1] = beta;
    b_alpha[2] = b_gamma;
    b_alpha[3] = r - 3.1415926535897931;
    b_alpha[4] = scale + dt * t;
    b_alpha[5] = alphaDot;
    b_alpha[6] = betaDot;
    b_alpha[7] = gammaDot;
    b_alpha[8] = absxk;
    b_alpha[9] = t;
    xNew_size[0] = 1;
    xNew_size[1] = 10;
    for (i6 = 0; i6 < 10; i6++) {
      xNew_data[xNew_size[0] * i6] = b_alpha[i6];
    }

    xInit_size_idx_1 = xNew_size[1];
    loop_ub = xNew_size[0] * xNew_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      xInit_data[i6] = xNew_data[i6];
    }

    absxk = 10.0 * (1.0 + (real_T)i) + 1.0;
    scale = 10.0 * ((1.0 + (real_T)i) + 1.0);
    if (absxk > scale) {
      i6 = 1;
      i7 = 1;
    } else {
      i6 = transitionArray->size[1];
      i7 = (int32_T)absxk;
      i6 = emlrtDynamicBoundsCheckFastR2012b(i7, 1, i6, &fb_emlrtBCI, sp);
      i7 = transitionArray->size[1];
      unnamed_idx_1 = (int32_T)scale;
      i7 = emlrtDynamicBoundsCheckFastR2012b(unnamed_idx_1, 1, i7, &fb_emlrtBCI,
        sp) + 1;
    }

    i7 -= i6;
    emlrtSizeEqCheck1DFastR2012b(i7, 10, &e_emlrtECI, sp);
    loop_ub = xNew_size[1];
    for (i7 = 0; i7 < loop_ub; i7++) {
      transitionArray->data[(i6 + i7) - 1] = xNew_data[xNew_size[0] * i7];
    }

    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  unnamed_idx_1 = 3 + xNew_size[1];
  for (i6 = 0; i6 < 3; i6++) {
    TP2S[i6] = 0.0;
  }

  loop_ub = xNew_size[1];
  for (i6 = 0; i6 < loop_ub; i6++) {
    TP2S[i6 + 3] = xNew_data[xNew_size[0] * i6];
  }

  xNew_size[0] = 1;
  xNew_size[1] = unnamed_idx_1;
  for (i6 = 0; i6 < unnamed_idx_1; i6++) {
    xNew_data[xNew_size[0] * i6] = TP2S[i6];
  }
}

/* End of code generation (rk4.c) */
