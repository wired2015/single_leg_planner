//
// File: getPhiAndOmega.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 05-Mar-2015 10:13:51
//

// Include Files
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "getPhiAndOmega.h"
#include "norm.h"
#include "angDiff.h"
#include "trInv.h"
#include "generateTrMatrices.h"
#include "sherpaTTPlanner_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// Arguments    : const double uBDot[6]
//                const double qDot[4]
//                const double q[4]
//                const struct0_T *kC
//                int legNum
//                double *phi
//                double *omega
// Return Type  : void
//
void getPhiAndOmega(const double uBDot[6], const double qDot[4], const double q
                    [4], const struct0_T *kC, int legNum, double *phi, double
                    *omega)
{
  double TB2S[16];
  double TP2S[16];
  double TS2R[16];
  double TR2Q[16];
  double TQ2O[16];
  double TO2J[16];
  double TJ2I[16];
  double TI2P[16];
  double TI2S[16];
  double TO2S[16];
  double b_TI2S[16];
  double c_TI2S[16];
  double d_TI2S[16];
  int i7;
  int i8;
  int i9;
  double b_TI2P[16];
  double c_TI2P[16];
  double d_TI2P[16];
  double e_TI2P[16];
  double f_TI2P[16];
  double b_TJ2I[16];
  double c_TJ2I[16];
  double d_TJ2I[16];
  double e_TJ2I[16];
  double b_TQ2O[16];
  double c_TQ2O[16];
  double dv2[9];
  double dv3[9];
  double b_TB2S[36];
  double dv4[9];
  double b_TO2S[6];
  double b_TP2S[36];
  double uSDot[6];
  double c_TP2S[6];
  double dv5[9];
  double e_TI2S[36];
  double dv6[9];
  double dv7[6];
  double c_TO2S[36];
  double c_TB2S[6];
  double r;

  // Homogeneous transformation matrices.
  generateTrMatrices(q, kC->l1, kC->l2, kC->l3, kC->l4, kC->l5, kC->l6, kC->l7,
                     kC->l8, kC->zeta, kC->r, kC->B2PXOffset, kC->B2PZOffset,
                     kC->legAngleOffset, legNum, TO2S, TI2S, TI2P, TJ2I, TO2J,
                     TQ2O, TR2Q, TS2R, TP2S, TB2S);
  for (i7 = 0; i7 < 4; i7++) {
    for (i8 = 0; i8 < 4; i8++) {
      TO2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        TO2S[i7 + (i8 << 2)] += TI2S[i7 + (i9 << 2)] * TI2P[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      TP2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        TP2S[i7 + (i8 << 2)] += TO2S[i7 + (i9 << 2)] * TJ2I[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      TB2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        TB2S[i7 + (i8 << 2)] += TP2S[i7 + (i9 << 2)] * TO2J[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      c_TI2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        c_TI2S[i7 + (i8 << 2)] += TB2S[i7 + (i9 << 2)] * TQ2O[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      d_TI2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        d_TI2S[i7 + (i8 << 2)] += c_TI2S[i7 + (i9 << 2)] * TR2Q[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      b_TI2S[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        b_TI2S[i7 + (i8 << 2)] += d_TI2S[i7 + (i9 << 2)] * TS2R[i9 + (i8 << 2)];
      }

      b_TI2P[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        b_TI2P[i7 + (i8 << 2)] += TI2P[i7 + (i9 << 2)] * TJ2I[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      c_TI2P[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        c_TI2P[i7 + (i8 << 2)] += b_TI2P[i7 + (i9 << 2)] * TO2J[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      d_TI2P[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        d_TI2P[i7 + (i8 << 2)] += c_TI2P[i7 + (i9 << 2)] * TQ2O[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      e_TI2P[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        e_TI2P[i7 + (i8 << 2)] += d_TI2P[i7 + (i9 << 2)] * TR2Q[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      f_TI2P[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        f_TI2P[i7 + (i8 << 2)] += e_TI2P[i7 + (i9 << 2)] * TS2R[i9 + (i8 << 2)];
      }

      b_TJ2I[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        b_TJ2I[i7 + (i8 << 2)] += TJ2I[i7 + (i9 << 2)] * TO2J[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      c_TJ2I[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        c_TJ2I[i7 + (i8 << 2)] += b_TJ2I[i7 + (i9 << 2)] * TQ2O[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      d_TJ2I[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        d_TJ2I[i7 + (i8 << 2)] += c_TJ2I[i7 + (i9 << 2)] * TR2Q[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      e_TJ2I[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        e_TJ2I[i7 + (i8 << 2)] += d_TJ2I[i7 + (i9 << 2)] * TS2R[i9 + (i8 << 2)];
      }

      b_TQ2O[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        b_TQ2O[i7 + (i8 << 2)] += TQ2O[i7 + (i9 << 2)] * TR2Q[i9 + (i8 << 2)];
      }
    }

    for (i8 = 0; i8 < 4; i8++) {
      c_TQ2O[i7 + (i8 << 2)] = 0.0;
      for (i9 = 0; i9 < 4; i9++) {
        c_TQ2O[i7 + (i8 << 2)] += b_TQ2O[i7 + (i9 << 2)] * TS2R[i9 + (i8 << 2)];
      }
    }
  }

  trInv(b_TI2S, TB2S);
  trInv(f_TI2P, TP2S);
  trInv(e_TJ2I, TI2S);
  trInv(c_TQ2O, TO2S);

  // Adjunct transformation matrices.
  // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
  // transformation matrix, T. The adjunct matrix serves to transform the
  // velocity from the one frame to another, as described by the homoegenous
  // transformation matrix.
  //
  // Inputs:
  // -T: The 4x4 homogeneous transformation matrix representing the
  // transformation from one frame to another.
  // Outputs:
  // -A: The adjunct matrix that transforms velocity vectors from one frame to
  // another.
  // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
  // transformation matrix, T. The adjunct matrix serves to transform the
  // velocity from the one frame to another, as described by the homoegenous
  // transformation matrix.
  //
  // Inputs:
  // -T: The 4x4 homogeneous transformation matrix representing the
  // transformation from one frame to another.
  // Outputs:
  // -A: The adjunct matrix that transforms velocity vectors from one frame to
  // another.
  // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
  // transformation matrix, T. The adjunct matrix serves to transform the
  // velocity from the one frame to another, as described by the homoegenous
  // transformation matrix.
  //
  // Inputs:
  // -T: The 4x4 homogeneous transformation matrix representing the
  // transformation from one frame to another.
  // Outputs:
  // -A: The adjunct matrix that transforms velocity vectors from one frame to
  // another.
  // TR2ADJ Returns the adjunct matrix, A, based on the homogeneous
  // transformation matrix, T. The adjunct matrix serves to transform the
  // velocity from the one frame to another, as described by the homoegenous
  // transformation matrix.
  //
  // Inputs:
  // -T: The 4x4 homogeneous transformation matrix representing the
  // transformation from one frame to another.
  // Outputs:
  // -A: The adjunct matrix that transforms velocity vectors from one frame to
  // another.
  // Pan joint rate
  // [rad/s]
  // [m/s]
  // [rad/s]
  // Beta joint rate
  // [rad/s]
  // [m/s]
  // [rad/s]
  // Gamma joint rate
  // [rad/s]
  // [m/s]
  // [rad/s]
  // Velocity vector for the ankle frame.
  dv2[0] = 0.0;
  dv2[3] = -TB2S[14];
  dv2[6] = TB2S[13];
  dv2[1] = TB2S[14];
  dv2[4] = 0.0;
  dv2[7] = -TB2S[12];
  dv2[2] = -TB2S[13];
  dv2[5] = TB2S[12];
  dv2[8] = 0.0;
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      dv3[i7 + 3 * i8] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        dv3[i7 + 3 * i8] += dv2[i7 + 3 * i9] * TB2S[i9 + (i8 << 2)];
      }

      b_TB2S[i8 + 6 * i7] = TB2S[i8 + (i7 << 2)];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_TB2S[i8 + 6 * (i7 + 3)] = dv3[i8 + 3 * i7];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_TB2S[(i8 + 6 * i7) + 3] = 0.0;
    }
  }

  dv4[0] = 0.0;
  dv4[3] = -TP2S[14];
  dv4[6] = TP2S[13];
  dv4[1] = TP2S[14];
  dv4[4] = 0.0;
  dv4[7] = -TP2S[12];
  dv4[2] = -TP2S[13];
  dv4[5] = TP2S[12];
  dv4[8] = 0.0;
  for (i7 = 0; i7 < 3; i7++) {
    b_TO2S[i7] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      b_TB2S[(i8 + 6 * (i7 + 3)) + 3] = TB2S[i8 + (i7 << 2)];
      dv2[i7 + 3 * i8] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        dv2[i7 + 3 * i8] += dv4[i7 + 3 * i9] * TP2S[i9 + (i8 << 2)];
      }

      b_TP2S[i8 + 6 * i7] = TP2S[i8 + (i7 << 2)];
    }
  }

  b_TO2S[3] = 0.0;
  b_TO2S[4] = 0.0;
  b_TO2S[5] = qDot[0];
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_TP2S[i8 + 6 * (i7 + 3)] = dv2[i8 + 3 * i7];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_TP2S[(i8 + 6 * i7) + 3] = 0.0;
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      b_TP2S[(i8 + 6 * (i7 + 3)) + 3] = TP2S[i8 + (i7 << 2)];
    }
  }

  for (i7 = 0; i7 < 6; i7++) {
    uSDot[i7] = 0.0;
    for (i8 = 0; i8 < 6; i8++) {
      uSDot[i7] += b_TB2S[i7 + 6 * i8] * uBDot[i8];
    }

    c_TP2S[i7] = 0.0;
    for (i8 = 0; i8 < 6; i8++) {
      c_TP2S[i7] += b_TP2S[i7 + 6 * i8] * b_TO2S[i8];
    }
  }

  dv5[0] = 0.0;
  dv5[3] = -TI2S[14];
  dv5[6] = TI2S[13];
  dv5[1] = TI2S[14];
  dv5[4] = 0.0;
  dv5[7] = -TI2S[12];
  dv5[2] = -TI2S[13];
  dv5[5] = TI2S[12];
  dv5[8] = 0.0;
  for (i7 = 0; i7 < 3; i7++) {
    b_TO2S[i7] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      dv2[i7 + 3 * i8] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        dv2[i7 + 3 * i8] += dv5[i7 + 3 * i9] * TI2S[i9 + (i8 << 2)];
      }

      e_TI2S[i8 + 6 * i7] = TI2S[i8 + (i7 << 2)];
    }
  }

  b_TO2S[3] = 0.0;
  b_TO2S[4] = 0.0;
  b_TO2S[5] = qDot[1];
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      e_TI2S[i8 + 6 * (i7 + 3)] = dv2[i8 + 3 * i7];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      e_TI2S[(i8 + 6 * i7) + 3] = 0.0;
    }
  }

  dv6[0] = 0.0;
  dv6[3] = -TO2S[14];
  dv6[6] = TO2S[13];
  dv6[1] = TO2S[14];
  dv6[4] = 0.0;
  dv6[7] = -TO2S[12];
  dv6[2] = -TO2S[13];
  dv6[5] = TO2S[12];
  dv6[8] = 0.0;
  for (i7 = 0; i7 < 3; i7++) {
    dv7[i7] = 0.0;
    for (i8 = 0; i8 < 3; i8++) {
      e_TI2S[(i8 + 6 * (i7 + 3)) + 3] = TI2S[i8 + (i7 << 2)];
      dv2[i7 + 3 * i8] = 0.0;
      for (i9 = 0; i9 < 3; i9++) {
        dv2[i7 + 3 * i8] += dv6[i7 + 3 * i9] * TO2S[i9 + (i8 << 2)];
      }

      c_TO2S[i8 + 6 * i7] = TO2S[i8 + (i7 << 2)];
    }
  }

  dv7[3] = 0.0;
  dv7[4] = 0.0;
  dv7[5] = qDot[2];
  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_TO2S[i8 + 6 * (i7 + 3)] = dv2[i8 + 3 * i7];
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_TO2S[(i8 + 6 * i7) + 3] = 0.0;
    }
  }

  for (i7 = 0; i7 < 3; i7++) {
    for (i8 = 0; i8 < 3; i8++) {
      c_TO2S[(i8 + 6 * (i7 + 3)) + 3] = TO2S[i8 + (i7 << 2)];
    }
  }

  for (i7 = 0; i7 < 6; i7++) {
    r = 0.0;
    for (i8 = 0; i8 < 6; i8++) {
      r += e_TI2S[i7 + 6 * i8] * b_TO2S[i8];
    }

    c_TB2S[i7] = (uSDot[i7] + c_TP2S[i7]) + r;
  }

  for (i7 = 0; i7 < 6; i7++) {
    b_TO2S[i7] = 0.0;
    for (i8 = 0; i8 < 6; i8++) {
      b_TO2S[i7] += c_TO2S[i7 + 6 * i8] * dv7[i8];
    }

    uSDot[i7] = c_TB2S[i7] + b_TO2S[i7];
  }

  // [m/s]
  // [rad/s]
  // Calculate the required phi joint angle and the required wheel speed,
  // omega.
  // [rad]
  r = ((rt_atan2d_snf(uSDot[1], uSDot[0]) - 1.5707963267948966) +
       3.1415926535897931) / 6.2831853071795862;
  if (std::abs(r - rt_roundd_snf(r)) <= 2.2204460492503131E-16 * std::abs(r)) {
    r = 0.0;
  } else {
    r = (r - std::floor(r)) * 6.2831853071795862;
  }

  *phi = r - 3.1415926535897931;

  //      if phiDiff > pi/2
  //          phi = phi - pi;
  //          phi = mod(phi+pi,2*pi)-pi;
  //          sign = -1;
  //      end
  *omega = b_norm(*(double (*)[3])&uSDot[0]) / kC->r;

  // [rad/s]
}

//
// File trailer for getPhiAndOmega.cpp
//
// [EOF]
//
