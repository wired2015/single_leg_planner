/*
 * getConstrainedGammaDotDot.c
 *
 * Code generation for function 'getConstrainedGammaDotDot'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "getConstrainedGammaDotDot.h"
#include <stdio.h>

/* Function Definitions */
real_T getConstrainedGammaDotDot(real_T kC_l3, real_T kC_l5, real_T kC_zeta,
  const real_T qDotDot[2], const real_T qDot[3], const real_T q[3])
{
  /* getConstrainedGammaDotDot Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* [~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst); */
  return ((-qDotDot[1] * kC_l3 * muDoubleScalarCos(q[1]) + qDot[1] * qDot[1] *
           kC_l3 * muDoubleScalarSin(q[1])) + qDot[2] * qDot[2] * kC_l5 *
          muDoubleScalarSin(kC_zeta + q[2])) / (kC_l5 * muDoubleScalarCos
    (kC_zeta + q[2]));
}

/* End of code generation (getConstrainedGammaDotDot.c) */
