/*
 * File: generateTrMatrices.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Feb-2015 09:54:49
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "generateTrMatrices.h"
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : const double q[4]
 *                double kC_l1
 *                double kC_l2
 *                double kC_l3
 *                double kC_l4
 *                double kC_l5
 *                double kC_l6
 *                double kC_l7
 *                double kC_l8
 *                double kC_zeta
 *                double kC_r
 *                double kC_B2PXOffset
 *                double kC_B2PZOffset
 *                const double kC_legAngleOffset[4]
 *                int legNum
 *                double TB2G[16]
 *                double TP2B[16]
 *                double TI2P[16]
 *                double TJ2I[16]
 *                double TO2J[16]
 *                double TQ2O[16]
 *                double TR2Q[16]
 *                double TS2R[16]
 *                double TW2S[16]
 *                double TC2W[16]
 * Return Type  : void
 */
void generateTrMatrices(const double q[4], double kC_l1, double kC_l2, double
  kC_l3, double kC_l4, double kC_l5, double kC_l6, double kC_l7, double kC_l8,
  double kC_zeta, double kC_r, double kC_B2PXOffset, double kC_B2PZOffset, const
  double kC_legAngleOffset[4], int legNum, double TB2G[16], double TP2B[16],
  double TI2P[16], double TJ2I[16], double TO2J[16], double TQ2O[16], double
  TR2Q[16], double TS2R[16], double TW2S[16], double TC2W[16])
{
  signed char I[9];
  int i8;
  int k;
  double theta;
  static const signed char iv2[4] = { 0, 0, 0, 1 };

  static const signed char iv3[4] = { 0, 0, 1, 0 };

  static const double dv8[4] = { 0.0, -1.0, 6.123233995736766E-17, 0.0 };

  static const signed char iv4[4] = { 1, 0, 0, 0 };

  static const signed char iv5[4] = { 0, 1, 0, 0 };

  static const double dv9[4] = { 6.123233995736766E-17, -6.123233995736766E-17,
    1.0, 0.0 };

  static const double dv10[4] = { 1.0, 3.749399456654644E-33,
    -6.123233995736766E-17, 0.0 };

  double dv11[16];
  static const double dv12[4] = { 6.123233995736766E-17, -1.0, 0.0, 0.0 };

  static const double dv13[4] = { 1.0, 6.123233995736766E-17, -0.0, 0.0 };

  int i9;
  static const double a[16] = { 1.0, 0.0, 0.0, 0.0, -0.0, 6.123233995736766E-17,
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
  for (i8 = 0; i8 < 9; i8++) {
    I[i8] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i8 = 0; i8 < 3; i8++) {
      TB2G[i8 + (k << 2)] = I[i8 + 3 * k];
    }
  }

  for (i8 = 0; i8 < 3; i8++) {
    TB2G[12 + i8] = 0.0;
  }

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TP2B[0] = cos(kC_legAngleOffset[legNum - 1]);
  TP2B[4] = -sin(kC_legAngleOffset[legNum - 1]);
  TP2B[8] = sin(kC_legAngleOffset[legNum - 1]) * 0.0;
  TP2B[12] = kC_B2PXOffset * cos(kC_legAngleOffset[legNum - 1]);
  TP2B[1] = sin(kC_legAngleOffset[legNum - 1]);
  TP2B[5] = cos(kC_legAngleOffset[legNum - 1]);
  TP2B[9] = -cos(kC_legAngleOffset[legNum - 1]) * 0.0;
  TP2B[13] = kC_B2PXOffset * sin(kC_legAngleOffset[legNum - 1]);
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
  TI2P[0] = cos(q[0]);
  TI2P[4] = -sin(q[0]) * 6.123233995736766E-17;
  TI2P[8] = -sin(q[0]);
  TI2P[12] = kC_l2 * cos(q[0]);
  TI2P[1] = sin(q[0]);
  TI2P[5] = cos(q[0]) * 6.123233995736766E-17;
  TI2P[9] = -(double)-cos(q[0]);
  TI2P[13] = kC_l2 * sin(q[0]);
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
  TJ2I[0] = cos(q[1]);
  TJ2I[4] = -sin(q[1]);
  TJ2I[8] = sin(q[1]) * 0.0;
  TJ2I[12] = kC_l3 * cos(q[1]);
  TJ2I[1] = sin(q[1]);
  TJ2I[5] = cos(q[1]);
  TJ2I[9] = -cos(q[1]) * 0.0;
  TJ2I[13] = kC_l3 * sin(q[1]);
  theta = -q[1] + kC_zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TO2J[0] = cos(theta);
  TO2J[4] = -sin(theta);
  TO2J[8] = sin(theta) * 0.0;
  TO2J[12] = kC_l4 * cos(theta);
  TO2J[1] = sin(theta);
  TO2J[5] = cos(theta);
  TO2J[9] = -cos(theta) * 0.0;
  TO2J[13] = kC_l4 * sin(theta);

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TQ2O[0] = cos(q[2]);
  TQ2O[4] = -sin(q[2]);
  TQ2O[8] = sin(q[2]) * 0.0;
  TQ2O[12] = kC_l5 * cos(q[2]);
  TQ2O[1] = sin(q[2]);
  TQ2O[5] = cos(q[2]);
  TQ2O[9] = -cos(q[2]) * 0.0;
  TQ2O[13] = kC_l5 * sin(q[2]);
  theta = -q[2] - kC_zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TR2Q[0] = cos(theta);
  TR2Q[4] = -sin(theta) * 6.123233995736766E-17;
  TR2Q[8] = -sin(theta);
  TR2Q[12] = -kC_l7 * cos(theta);
  TR2Q[1] = sin(theta);
  TR2Q[5] = cos(theta) * 6.123233995736766E-17;
  TR2Q[9] = -(double)-cos(theta);
  TR2Q[13] = -kC_l7 * sin(theta);
  for (i8 = 0; i8 < 4; i8++) {
    TB2G[3 + (i8 << 2)] = iv2[i8];
    TP2B[3 + (i8 << 2)] = iv2[i8];
    TI2P[3 + (i8 << 2)] = iv2[i8];
    TJ2I[2 + (i8 << 2)] = iv3[i8];
    TJ2I[3 + (i8 << 2)] = iv2[i8];
    TO2J[2 + (i8 << 2)] = iv3[i8];
    TO2J[3 + (i8 << 2)] = iv2[i8];
    TQ2O[2 + (i8 << 2)] = iv3[i8];
    TQ2O[3 + (i8 << 2)] = iv2[i8];
    TR2Q[2 + (i8 << 2)] = dv8[i8];
    TR2Q[3 + (i8 << 2)] = iv2[i8];

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TS2R[i8 << 2] = iv4[i8];
    TS2R[1 + (i8 << 2)] = iv5[i8];
  }

  TS2R[2] = 0.0;
  TS2R[6] = 0.0;
  TS2R[10] = 1.0;
  TS2R[14] = kC_l6;
  for (i8 = 0; i8 < 4; i8++) {
    TS2R[3 + (i8 << 2)] = iv2[i8];

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TW2S[i8 << 2] = dv9[i8];
    TW2S[1 + (i8 << 2)] = dv10[i8];
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
  for (i8 = 0; i8 < 4; i8++) {
    TW2S[3 + (i8 << 2)] = iv2[i8];
    dv11[i8 << 2] = dv12[i8];
    dv11[1 + (i8 << 2)] = dv13[i8];
  }

  dv11[2] = 0.0;
  dv11[6] = 0.0;
  dv11[10] = 1.0;
  dv11[14] = -kC_r;
  for (i8 = 0; i8 < 4; i8++) {
    dv11[3 + (i8 << 2)] = iv2[i8];
  }

  for (i8 = 0; i8 < 4; i8++) {
    for (k = 0; k < 4; k++) {
      TC2W[i8 + (k << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        TC2W[i8 + (k << 2)] += a[i8 + (i9 << 2)] * dv11[i9 + (k << 2)];
      }
    }
  }
}

/*
 * File trailer for generateTrMatrices.c
 *
 * [EOF]
 */
