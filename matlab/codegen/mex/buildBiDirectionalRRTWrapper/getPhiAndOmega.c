/*
 * getPhiAndOmega.c
 *
 * Code generation for function 'getPhiAndOmega'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "getPhiAndOmega.h"
#include "norm.h"
#include "trInv.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo lb_emlrtRSI = { 6, "getPhiAndOmega",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/getPhiAndOmega.m"
};

static emlrtBCInfo i_emlrtBCI = { 1, 4, 42, 17, "kC.legAngleOffset",
  "generateTrMatrices",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/generateTrMatrices.m",
  0 };

/* Function Definitions */
void getPhiAndOmega(const emlrtStack *sp, const real_T uBDot[6], const real_T
                    qDot[4], const real_T q[4], const struct0_T *kC, int32_T
                    legNum, real_T *phi, real_T *omega)
{
  real_T TO2S[16];
  real_T TI2S[16];
  real_T theta;
  real_T TP2S[16];
  real_T TB2S[16];
  real_T TR2Q[16];
  int32_T i5;
  static const int8_T iv4[4] = { 0, 0, 0, 1 };

  static const int8_T iv5[4] = { 0, 0, 1, 0 };

  static const real_T dv2[4] = { 0.0, -1.0, 6.123233995736766E-17, 0.0 };

  static const int8_T iv6[4] = { 1, 0, 0, 0 };

  real_T TS2R[16];
  static const int8_T iv7[4] = { 0, 1, 0, 0 };

  real_T dv3[16];
  real_T dv4[16];
  real_T dv5[16];
  real_T dv6[16];
  real_T dv7[16];
  real_T dv8[16];
  real_T dv9[16];
  int32_T i6;
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
  real_T TQ2O[16];
  real_T b_TQ2O[16];
  real_T dv10[9];
  real_T dv11[9];
  real_T b_TB2S[36];
  real_T dv12[9];
  real_T g_TO2S[6];
  real_T b_TP2S[36];
  real_T uSDot[6];
  real_T c_TP2S[6];
  real_T dv13[9];
  real_T f_TI2S[36];
  real_T dv14[9];
  real_T dv15[6];
  real_T h_TO2S[36];
  real_T c_TB2S[6];
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;

  /* Homogeneous transformation matrices. */
  st.site = &lb_emlrtRSI;

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
  emlrtDynamicBoundsCheckFastR2012b(legNum, 1, 4, &i_emlrtBCI, &st);

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
  theta = -q[1] + kC->zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TP2S[0] = muDoubleScalarCos(theta);
  TP2S[4] = -muDoubleScalarSin(theta);
  TP2S[8] = muDoubleScalarSin(theta) * 0.0;
  TP2S[12] = kC->l4 * muDoubleScalarCos(theta);
  TP2S[1] = muDoubleScalarSin(theta);
  TP2S[5] = muDoubleScalarCos(theta);
  TP2S[9] = -muDoubleScalarCos(theta) * 0.0;
  TP2S[13] = kC->l4 * muDoubleScalarSin(theta);

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TB2S[0] = muDoubleScalarCos(q[2]);
  TB2S[4] = -muDoubleScalarSin(q[2]);
  TB2S[8] = muDoubleScalarSin(q[2]) * 0.0;
  TB2S[12] = kC->l5 * muDoubleScalarCos(q[2]);
  TB2S[1] = muDoubleScalarSin(q[2]);
  TB2S[5] = muDoubleScalarCos(q[2]);
  TB2S[9] = -muDoubleScalarCos(q[2]) * 0.0;
  TB2S[13] = kC->l5 * muDoubleScalarSin(q[2]);
  theta = -q[2] - kC->zeta;

  /* TRDH Generates the homogeneous transformation matrix A using the  */
  /* Denavit-Hartenberg parameters theta, d, a and alpha. */
  /*  */
  /* trDH.m */
  /* author:    wreid */
  /* date:      20150214 */
  TR2Q[0] = muDoubleScalarCos(theta);
  TR2Q[4] = -muDoubleScalarSin(theta) * 6.123233995736766E-17;
  TR2Q[8] = -muDoubleScalarSin(theta);
  TR2Q[12] = -kC->l7 * muDoubleScalarCos(theta);
  TR2Q[1] = muDoubleScalarSin(theta);
  TR2Q[5] = muDoubleScalarCos(theta) * 6.123233995736766E-17;
  TR2Q[9] = -(-muDoubleScalarCos(theta));
  TR2Q[13] = -kC->l7 * muDoubleScalarSin(theta);
  for (i5 = 0; i5 < 4; i5++) {
    TO2S[3 + (i5 << 2)] = iv4[i5];
    TI2S[2 + (i5 << 2)] = iv5[i5];
    TI2S[3 + (i5 << 2)] = iv4[i5];
    TP2S[2 + (i5 << 2)] = iv5[i5];
    TP2S[3 + (i5 << 2)] = iv4[i5];
    TB2S[2 + (i5 << 2)] = iv5[i5];
    TB2S[3 + (i5 << 2)] = iv4[i5];
    TR2Q[2 + (i5 << 2)] = dv2[i5];
    TR2Q[3 + (i5 << 2)] = iv4[i5];

    /* TRDH Generates the homogeneous transformation matrix A using the  */
    /* Denavit-Hartenberg parameters theta, d, a and alpha. */
    /*  */
    /* trDH.m */
    /* author:    wreid */
    /* date:      20150214 */
    TS2R[i5 << 2] = iv6[i5];
    TS2R[1 + (i5 << 2)] = iv7[i5];
  }

  TS2R[2] = 0.0;
  TS2R[6] = 0.0;
  TS2R[10] = 1.0;
  TS2R[14] = kC->l6;
  for (i5 = 0; i5 < 4; i5++) {
    TS2R[3 + (i5 << 2)] = iv4[i5];
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
  dv4[0] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  dv4[4] = -muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  dv4[8] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]) * 0.0;
  dv4[12] = kC->B2PXOffset * muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  dv4[1] = muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  dv4[5] = muDoubleScalarCos(kC->legAngleOffset[legNum - 1]);
  dv4[9] = -muDoubleScalarCos(kC->legAngleOffset[legNum - 1]) * 0.0;
  dv4[13] = kC->B2PXOffset * muDoubleScalarSin(kC->legAngleOffset[legNum - 1]);
  dv4[2] = 0.0;
  dv4[6] = 0.0;
  dv4[10] = 1.0;
  dv4[14] = kC->B2PZOffset;
  for (i5 = 0; i5 < 4; i5++) {
    dv4[3 + (i5 << 2)] = iv4[i5];
  }

  for (i5 = 0; i5 < 4; i5++) {
    for (i6 = 0; i6 < 4; i6++) {
      dv5[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv5[i5 + (i6 << 2)] += dv4[i5 + (i7 << 2)] * TO2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      dv6[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv6[i5 + (i6 << 2)] += dv5[i5 + (i7 << 2)] * TI2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      dv7[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv7[i5 + (i6 << 2)] += dv6[i5 + (i7 << 2)] * TP2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      dv8[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv8[i5 + (i6 << 2)] += dv7[i5 + (i7 << 2)] * TB2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      dv9[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv9[i5 + (i6 << 2)] += dv8[i5 + (i7 << 2)] * TR2Q[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      dv3[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        dv3[i5 + (i6 << 2)] += dv9[i5 + (i7 << 2)] * TS2R[i7 + (i6 << 2)];
      }

      b_TO2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        b_TO2S[i5 + (i6 << 2)] += TO2S[i5 + (i7 << 2)] * TI2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      c_TO2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        c_TO2S[i5 + (i6 << 2)] += b_TO2S[i5 + (i7 << 2)] * TP2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      d_TO2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        d_TO2S[i5 + (i6 << 2)] += c_TO2S[i5 + (i7 << 2)] * TB2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      e_TO2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        e_TO2S[i5 + (i6 << 2)] += d_TO2S[i5 + (i7 << 2)] * TR2Q[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      f_TO2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        f_TO2S[i5 + (i6 << 2)] += e_TO2S[i5 + (i7 << 2)] * TS2R[i7 + (i6 << 2)];
      }

      b_TI2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        b_TI2S[i5 + (i6 << 2)] += TI2S[i5 + (i7 << 2)] * TP2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      c_TI2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        c_TI2S[i5 + (i6 << 2)] += b_TI2S[i5 + (i7 << 2)] * TB2S[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      d_TI2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        d_TI2S[i5 + (i6 << 2)] += c_TI2S[i5 + (i7 << 2)] * TR2Q[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      e_TI2S[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        e_TI2S[i5 + (i6 << 2)] += d_TI2S[i5 + (i7 << 2)] * TS2R[i7 + (i6 << 2)];
      }

      TQ2O[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        TQ2O[i5 + (i6 << 2)] += TB2S[i5 + (i7 << 2)] * TR2Q[i7 + (i6 << 2)];
      }
    }

    for (i6 = 0; i6 < 4; i6++) {
      b_TQ2O[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        b_TQ2O[i5 + (i6 << 2)] += TQ2O[i5 + (i7 << 2)] * TS2R[i7 + (i6 << 2)];
      }
    }
  }

  trInv(dv3, TB2S);
  trInv(f_TO2S, TP2S);
  trInv(e_TI2S, TI2S);
  trInv(b_TQ2O, TO2S);

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
  /* Pan joint rate */
  /* [rad/s] */
  /* [m/s] */
  /* [rad/s] */
  /* Beta joint rate */
  /* [rad/s] */
  /* [m/s] */
  /* [rad/s] */
  /* Gamma joint rate */
  /* [rad/s] */
  /* [m/s] */
  /* [rad/s] */
  /* Velocity vector for the ankle frame. */
  dv10[0] = 0.0;
  dv10[3] = -TB2S[14];
  dv10[6] = TB2S[13];
  dv10[1] = TB2S[14];
  dv10[4] = 0.0;
  dv10[7] = -TB2S[12];
  dv10[2] = -TB2S[13];
  dv10[5] = TB2S[12];
  dv10[8] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      dv11[i5 + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        dv11[i5 + 3 * i6] += dv10[i5 + 3 * i7] * TB2S[i7 + (i6 << 2)];
      }

      b_TB2S[i6 + 6 * i5] = TB2S[i6 + (i5 << 2)];
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      b_TB2S[i6 + 6 * (i5 + 3)] = dv11[i6 + 3 * i5];
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      b_TB2S[(i6 + 6 * i5) + 3] = 0.0;
    }
  }

  dv12[0] = 0.0;
  dv12[3] = -TP2S[14];
  dv12[6] = TP2S[13];
  dv12[1] = TP2S[14];
  dv12[4] = 0.0;
  dv12[7] = -TP2S[12];
  dv12[2] = -TP2S[13];
  dv12[5] = TP2S[12];
  dv12[8] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    g_TO2S[i5] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      b_TB2S[(i6 + 6 * (i5 + 3)) + 3] = TB2S[i6 + (i5 << 2)];
      dv10[i5 + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        dv10[i5 + 3 * i6] += dv12[i5 + 3 * i7] * TP2S[i7 + (i6 << 2)];
      }

      b_TP2S[i6 + 6 * i5] = TP2S[i6 + (i5 << 2)];
    }
  }

  g_TO2S[3] = 0.0;
  g_TO2S[4] = 0.0;
  g_TO2S[5] = qDot[0];
  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      b_TP2S[i6 + 6 * (i5 + 3)] = dv10[i6 + 3 * i5];
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      b_TP2S[(i6 + 6 * i5) + 3] = 0.0;
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      b_TP2S[(i6 + 6 * (i5 + 3)) + 3] = TP2S[i6 + (i5 << 2)];
    }
  }

  for (i5 = 0; i5 < 6; i5++) {
    uSDot[i5] = 0.0;
    for (i6 = 0; i6 < 6; i6++) {
      uSDot[i5] += b_TB2S[i5 + 6 * i6] * uBDot[i6];
    }

    c_TP2S[i5] = 0.0;
    for (i6 = 0; i6 < 6; i6++) {
      c_TP2S[i5] += b_TP2S[i5 + 6 * i6] * g_TO2S[i6];
    }
  }

  dv13[0] = 0.0;
  dv13[3] = -TI2S[14];
  dv13[6] = TI2S[13];
  dv13[1] = TI2S[14];
  dv13[4] = 0.0;
  dv13[7] = -TI2S[12];
  dv13[2] = -TI2S[13];
  dv13[5] = TI2S[12];
  dv13[8] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    g_TO2S[i5] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      dv10[i5 + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        dv10[i5 + 3 * i6] += dv13[i5 + 3 * i7] * TI2S[i7 + (i6 << 2)];
      }

      f_TI2S[i6 + 6 * i5] = TI2S[i6 + (i5 << 2)];
    }
  }

  g_TO2S[3] = 0.0;
  g_TO2S[4] = 0.0;
  g_TO2S[5] = qDot[1];
  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      f_TI2S[i6 + 6 * (i5 + 3)] = dv10[i6 + 3 * i5];
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      f_TI2S[(i6 + 6 * i5) + 3] = 0.0;
    }
  }

  dv14[0] = 0.0;
  dv14[3] = -TO2S[14];
  dv14[6] = TO2S[13];
  dv14[1] = TO2S[14];
  dv14[4] = 0.0;
  dv14[7] = -TO2S[12];
  dv14[2] = -TO2S[13];
  dv14[5] = TO2S[12];
  dv14[8] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    dv15[i5] = 0.0;
    for (i6 = 0; i6 < 3; i6++) {
      f_TI2S[(i6 + 6 * (i5 + 3)) + 3] = TI2S[i6 + (i5 << 2)];
      dv10[i5 + 3 * i6] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        dv10[i5 + 3 * i6] += dv14[i5 + 3 * i7] * TO2S[i7 + (i6 << 2)];
      }

      h_TO2S[i6 + 6 * i5] = TO2S[i6 + (i5 << 2)];
    }
  }

  dv15[3] = 0.0;
  dv15[4] = 0.0;
  dv15[5] = qDot[2];
  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      h_TO2S[i6 + 6 * (i5 + 3)] = dv10[i6 + 3 * i5];
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      h_TO2S[(i6 + 6 * i5) + 3] = 0.0;
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      h_TO2S[(i6 + 6 * (i5 + 3)) + 3] = TO2S[i6 + (i5 << 2)];
    }
  }

  for (i5 = 0; i5 < 6; i5++) {
    theta = 0.0;
    for (i6 = 0; i6 < 6; i6++) {
      theta += f_TI2S[i5 + 6 * i6] * g_TO2S[i6];
    }

    c_TB2S[i5] = (uSDot[i5] + c_TP2S[i5]) + theta;
  }

  for (i5 = 0; i5 < 6; i5++) {
    g_TO2S[i5] = 0.0;
    for (i6 = 0; i6 < 6; i6++) {
      g_TO2S[i5] += h_TO2S[i5 + 6 * i6] * dv15[i6];
    }

    uSDot[i5] = c_TB2S[i5] + g_TO2S[i5];
  }

  /* [m/s] */
  /* [rad/s] */
  /* Calculate the required phi joint angle and the required wheel speed, */
  /* omega. */
  /* [rad] */
  theta = ((muDoubleScalarAtan2(uSDot[1], uSDot[0]) - 1.5707963267948966) +
           3.1415926535897931) / 6.2831853071795862;
  if (muDoubleScalarAbs(theta - muDoubleScalarRound(theta)) <=
      2.2204460492503131E-16 * muDoubleScalarAbs(theta)) {
    theta = 0.0;
  } else {
    theta = (theta - muDoubleScalarFloor(theta)) * 6.2831853071795862;
  }

  *phi = theta - 3.1415926535897931;

  /*      if phiDiff > pi/2 */
  /*          phi = phi - pi; */
  /*          phi = mod(phi+pi,2*pi)-pi; */
  /*          sign = -1; */
  /*      end */
  *omega = b_norm(*(real_T (*)[3])&uSDot[0]) / kC->r;

  /* [rad/s] */
}

/* End of code generation (getPhiAndOmega.c) */
