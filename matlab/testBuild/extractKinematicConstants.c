/*
 * extractKinematicConstants.c
 *
 * Code generation for function 'extractKinematicConstants'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "extractKinematicConstants.h"
#include <stdio.h>

/* Function Definitions */
void extractKinematicConstants(const real_T kinematicConst[16], real_T *L1,
  real_T *L2, real_T *L3, real_T *L4, real_T *L5, real_T *L6, real_T *L7, real_T
  *L8, real_T *zeta, real_T *r, real_T *B2PXOffset, real_T *B2PZOffset, real_T
  *leg1AngleOffset, real_T *leg2AngleOffset, real_T *leg3AngleOffset, real_T
  *leg4AngleOffset)
{
  /* EXTRACTKINEMATICCONSTANTS Gets individual kinematic constants. */
  /*    Takes the kinematicConst array and returns each of the individual */
  /*    kinematic constants. */
  *L1 = kinematicConst[0];
  *L2 = kinematicConst[1];
  *L3 = kinematicConst[2];
  *L4 = kinematicConst[3];
  *L5 = kinematicConst[4];
  *L6 = kinematicConst[5];
  *L7 = kinematicConst[6];
  *L8 = kinematicConst[7];
  *zeta = kinematicConst[8];
  *r = kinematicConst[9];
  *B2PXOffset = kinematicConst[10];
  *B2PZOffset = kinematicConst[11];
  *leg1AngleOffset = kinematicConst[12];
  *leg2AngleOffset = kinematicConst[13];
  *leg3AngleOffset = kinematicConst[14];
  *leg4AngleOffset = kinematicConst[15];
}

/* End of code generation (extractKinematicConstants.c) */
