/*
 * calcPhi.h
 *
 * Code generation for function 'calcPhi'
 *
 */

#ifndef __CALCPHI_H__
#define __CALCPHI_H__

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
#include "buildRRT_types.h"

/* Function Declarations */
extern void calcPhi(const real_T qDot[3], const real_T q[3], const real_T
                    kinematicConst[12], real_T *qS, real_T *qWDot);

#endif

/* End of code generation (calcPhi.h) */
