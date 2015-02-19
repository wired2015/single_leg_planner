/*
 * generateTrMatrices.c
 *
 * Code generation for function 'generateTrMatrices'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "generateTrMatrices.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtBCInfo x_emlrtBCI = { 1, 4, 42, 17, "kC.legAngleOffset",
  "generateTrMatrices",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/generateTrMatrices.m",
  0 };

/* Function Definitions */
void generateTrMatrices(const emlrtStack *sp, const real_T q[4], real_T kC_l1,
  real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6, real_T
  kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, real_T kC_B2PXOffset, real_T
  kC_B2PZOffset, const real_T kC_legAngleOffset[4], int32_T legNum, real_T TB2G
  [16], real_T TP2B[16], real_T TI2P[16], real_T TJ2I[16], real_T TO2J[16],
  real_T TQ2O[16], real_T TR2Q[16], real_T TS2R[16], real_T TW2S[16], real_T
  TC2W[16])
{
  int8_T I[9];
  int32_T i10;
  int32_T k;
  static const int8_T iv6[4] = { 0, 0, 0, 1 };

  real_T theta;
  static const int8_T iv7[4] = { 0, 0, 1, 0 };

  static const real_T dv16[4] = { 0.0, -1.0, 6.123233995736766E-17, 0.0 };

  static const int8_T iv8[4] = { 1, 0, 0, 0 };

  static const int8_T iv9[4] = { 0, 1, 0, 0 };

  static const real_T dv17[4] = { 6.123233995736766E-17, -6.123233995736766E-17,
    1.0, 0.0 };

  static const real_T dv18[4] = { 1.0, 3.749399456654644E-33,
    -6.123233995736766E-17, 0.0 };

  real_T dv19[16];
  static const real_T dv20[4] = { 6.123233995736766E-17, -1.0, 0.0, 0.0 };

  static const real_T dv21[4] = { 1.0, 6.123233995736766E-17, -0.0, 0.0 };

  int32_T i11;
  static const real_T a[16] = { 1.0, 0.0, 0.0, 0.0, -0.0, 6.123233995736766E-17,
    1.0, 0.0, 0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0, 1.0 };

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
  for (i10 = 0; i10 < 9; i10++) {
    I[i10] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i10 = 0; i10 < 3; i10++) {
      TB2G[i10 + (k << 2)] = I[i10 + 3 * k];
    }
  }

  for (i10 = 0; i10 < 3; i10++) {
    TB2G[12 + i10] = 0.0;
  }

  for (i10 = 0; i10 < 4; i10++) {
    TB2G[3 + (i10 << 2)] = iv6[i10];
  }

  emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &x_emlrtBCI, sp);

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TP2B[0] = muDoubleScalarCos(kC_legAngleOffset[legNum - 1]);
  TP2B[4] = -muDoubleScalarSin(kC_legAngleOffset[legNum - 1]);
  TP2B[8] = muDoubleScalarSin(kC_legAngleOffset[legNum - 1]) * 0.0;
  TP2B[12] = kC_B2PXOffset * muDoubleScalarCos(kC_legAngleOffset[legNum - 1]);
  TP2B[1] = muDoubleScalarSin(kC_legAngleOffset[legNum - 1]);
  TP2B[5] = muDoubleScalarCos(kC_legAngleOffset[legNum - 1]);
  TP2B[9] = -muDoubleScalarCos(kC_legAngleOffset[legNum - 1]) * 0.0;
  TP2B[13] = kC_B2PXOffset * muDoubleScalarSin(kC_legAngleOffset[legNum - 1]);
  TP2B[2] = 0.0;
  TP2B[6] = 0.0;
  TP2B[10] = 1.0;
  TP2B[14] = kC_B2PZOffset;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TI2P[0] = muDoubleScalarCos(q[0]);
  TI2P[4] = -muDoubleScalarSin(q[0]) * 6.123233995736766E-17;
  TI2P[8] = -muDoubleScalarSin(q[0]);
  TI2P[12] = kC_l2 * muDoubleScalarCos(q[0]);
  TI2P[1] = muDoubleScalarSin(q[0]);
  TI2P[5] = muDoubleScalarCos(q[0]) * 6.123233995736766E-17;
  TI2P[9] = -(-muDoubleScalarCos(q[0]));
  TI2P[13] = kC_l2 * muDoubleScalarSin(q[0]);
  TI2P[2] = 0.0;
  TI2P[6] = -1.0;
  TI2P[10] = 6.123233995736766E-17;
  TI2P[14] = kC_l1;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TJ2I[0] = muDoubleScalarCos(q[1]);
  TJ2I[4] = -muDoubleScalarSin(q[1]);
  TJ2I[8] = muDoubleScalarSin(q[1]) * 0.0;
  TJ2I[12] = kC_l3 * muDoubleScalarCos(q[1]);
  TJ2I[1] = muDoubleScalarSin(q[1]);
  TJ2I[5] = muDoubleScalarCos(q[1]);
  TJ2I[9] = -muDoubleScalarCos(q[1]) * 0.0;
  TJ2I[13] = kC_l3 * muDoubleScalarSin(q[1]);
  theta = -q[1] + kC_zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TO2J[0] = muDoubleScalarCos(theta);
  TO2J[4] = -muDoubleScalarSin(theta);
  TO2J[8] = muDoubleScalarSin(theta) * 0.0;
  TO2J[12] = kC_l4 * muDoubleScalarCos(theta);
  TO2J[1] = muDoubleScalarSin(theta);
  TO2J[5] = muDoubleScalarCos(theta);
  TO2J[9] = -muDoubleScalarCos(theta) * 0.0;
  TO2J[13] = kC_l4 * muDoubleScalarSin(theta);

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TQ2O[0] = muDoubleScalarCos(q[2]);
  TQ2O[4] = -muDoubleScalarSin(q[2]);
  TQ2O[8] = muDoubleScalarSin(q[2]) * 0.0;
  TQ2O[12] = kC_l5 * muDoubleScalarCos(q[2]);
  TQ2O[1] = muDoubleScalarSin(q[2]);
  TQ2O[5] = muDoubleScalarCos(q[2]);
  TQ2O[9] = -muDoubleScalarCos(q[2]) * 0.0;
  TQ2O[13] = kC_l5 * muDoubleScalarSin(q[2]);
  theta = -q[2] - kC_zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TR2Q[0] = muDoubleScalarCos(theta);
  TR2Q[4] = -muDoubleScalarSin(theta) * 6.123233995736766E-17;
  TR2Q[8] = -muDoubleScalarSin(theta);
  TR2Q[12] = -kC_l7 * muDoubleScalarCos(theta);
  TR2Q[1] = muDoubleScalarSin(theta);
  TR2Q[5] = muDoubleScalarCos(theta) * 6.123233995736766E-17;
  TR2Q[9] = -(-muDoubleScalarCos(theta));
  TR2Q[13] = -kC_l7 * muDoubleScalarSin(theta);
  for (i10 = 0; i10 < 4; i10++) {
    TP2B[3 + (i10 << 2)] = iv6[i10];
    TI2P[3 + (i10 << 2)] = iv6[i10];
    TJ2I[2 + (i10 << 2)] = iv7[i10];
    TJ2I[3 + (i10 << 2)] = iv6[i10];
    TO2J[2 + (i10 << 2)] = iv7[i10];
    TO2J[3 + (i10 << 2)] = iv6[i10];
    TQ2O[2 + (i10 << 2)] = iv7[i10];
    TQ2O[3 + (i10 << 2)] = iv6[i10];
    TR2Q[2 + (i10 << 2)] = dv16[i10];
    TR2Q[3 + (i10 << 2)] = iv6[i10];

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TS2R[i10 << 2] = iv8[i10];
    TS2R[1 + (i10 << 2)] = iv9[i10];
  }

  TS2R[2] = 0.0;
  TS2R[6] = 0.0;
  TS2R[10] = 1.0;
  TS2R[14] = kC_l6;
  for (i10 = 0; i10 < 4; i10++) {
    TS2R[3 + (i10 << 2)] = iv6[i10];

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TW2S[i10 << 2] = dv17[i10];
    TW2S[1 + (i10 << 2)] = dv18[i10];
  }

  TW2S[2] = 0.0;
  TW2S[6] = 1.0;
  TW2S[10] = 6.123233995736766E-17;
  TW2S[14] = kC_l8;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  for (i10 = 0; i10 < 4; i10++) {
    TW2S[3 + (i10 << 2)] = iv6[i10];
    dv19[i10 << 2] = dv20[i10];
    dv19[1 + (i10 << 2)] = dv21[i10];
  }

  dv19[2] = 0.0;
  dv19[6] = 0.0;
  dv19[10] = 1.0;
  dv19[14] = -kC_r;
  for (i10 = 0; i10 < 4; i10++) {
    dv19[3 + (i10 << 2)] = iv6[i10];
  }

  for (i10 = 0; i10 < 4; i10++) {
    for (k = 0; k < 4; k++) {
      TC2W[i10 + (k << 2)] = 0.0;
      for (i11 = 0; i11 < 4; i11++) {
        TC2W[i10 + (k << 2)] += a[i10 + (i11 << 2)] * dv19[i11 + (k << 2)];
      }
    }
  }
}

/* End of code generation (generateTrMatrices.c) */
