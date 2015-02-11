/*
 * sherpaTTIKVel.h
 *
 * Code generation for function 'sherpaTTIKVel'
 *
 */

#ifndef __SHERPATTIKVEL_H__
#define __SHERPATTIKVEL_H__

/* Include files */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "blas.h"
#include "rtwtypes.h"
#include "buildRRTWrapper_types.h"

/* Function Declarations */
extern void sherpaTTIKVel(const real_T uDot[3], const real_T q[3], const real_T
  kinematicConst[16], real_T qDot[3]);

#endif

/* End of code generation (sherpaTTIKVel.h) */
