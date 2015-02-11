/*
 * calcPhi.c
 *
 * Code generation for function 'calcPhi'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "calcPhi.h"

/* Function Definitions */
void calcPhi(const real_T qDot[3], const real_T q[3], const real_T
             kinematicConst[12], real_T *qS, real_T *qWDot)
{
  real_T v_LS[2];
  real_T scale;
  int32_T k;
  real_T absxk;
  real_T t;

  /* calcPhi.m */
  /* function [qS,qWDot] = calcPhi(qDot,q,kinematicConst,sS,qWDot,r) */
  /* v_XS = sin(conj(alpha))*conj(L6)*conj(alphaDot) - sin(conj(alpha))*conj(L2)*conj(alphaDot) - cos(conj(alpha))*conj(L7)*conj(gammaDot) - cos(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(beta))*sin(conj(alpha))*conj(L3)*conj(alphaDot) - cos(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - sin(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - cos(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) - cos(conj(gamma))*sin(conj(alpha))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - cos(conj(alpha))*cos(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - cos(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot) + sin(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot); */
  /* v_YS = cos(conj(alpha))*conj(L2)*conj(alphaDot) - sin(conj(alpha))*conj(L7)*conj(gammaDot) - sin(conj(alpha))*conj(L7)*conj(betaDot) - cos(conj(alpha))*conj(L6)*conj(alphaDot) + cos(conj(alpha))*cos(conj(beta))*conj(L3)*conj(alphaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(zeta))*conj(L4)*conj(alphaDot) - 1.0*sin(conj(alpha))*sin(conj(beta))*conj(L3)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(zeta))*conj(L4)*conj(gammaDot) + 0.99999999999999999999999999999999*cos(conj(alpha))*cos(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(alphaDot) - cos(conj(alpha))*sin(conj(gamma))*sin(conj(zeta))*conj(L5)*conj(alphaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(betaDot) - 1.0*cos(conj(gamma))*sin(conj(alpha))*sin(conj(zeta))*conj(L5)*conj(gammaDot) - 1.0*sin(conj(alpha))*sin(conj(gamma))*cos(conj(zeta))*conj(L5)*conj(gammaDot); */
  /* qA1 = atan(sS(2)/sS(1)); */
  /* v_WS = [qWDot*r*sign(sS(1))*cos(qA1); qWDot*r*sign(sS(1))*sin(qA1)]; */
  v_LS[0] = -kinematicConst[2] * qDot[1] * muDoubleScalarSin(q[1]) - qDot[2] *
    kinematicConst[4] * muDoubleScalarSin(kinematicConst[8] + q[2]);
  v_LS[1] = qDot[0] * (((kinematicConst[1] + kinematicConst[2] *
    muDoubleScalarCos(q[1])) + kinematicConst[3] * muDoubleScalarCos
                        (kinematicConst[8])) + kinematicConst[4] *
                       muDoubleScalarCos(kinematicConst[8] + q[2]));

  /*  + v_WS;     */
  *qWDot = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 2; k++) {
    absxk = muDoubleScalarAbs(v_LS[k]);
    if (absxk > scale) {
      t = scale / absxk;
      *qWDot = 1.0 + *qWDot * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      *qWDot += t * t;
    }
  }

  *qWDot = scale * muDoubleScalarSqrt(*qWDot);
  *qS = muDoubleScalarAtan(v_LS[0] / v_LS[1]);
}

/* End of code generation (calcPhi.c) */
