/*
 * heuristicSingleLeg.h
 *
 * Code generation for function 'heuristicSingleLeg'
 *
 */

#ifndef __HEURISTICSINGLELEG_H__
#define __HEURISTICSINGLELEG_H__

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
#include "sherpaTTPlanner_mex_types.h"

/* Function Declarations */
extern real_T b_heuristicSingleLeg(const real_T xA[13], const real_T xB[13],
  real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T
  kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r);

#ifdef __WATCOMC__

#pragma aux b_heuristicSingleLeg value [8087];

#endif

extern real_T c_heuristicSingleLeg(const real_T xA_data[], const real_T xB[93],
  real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T
  kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r);

#ifdef __WATCOMC__

#pragma aux c_heuristicSingleLeg value [8087];

#endif

extern real_T heuristicSingleLeg(const real_T xA[13], const real_T xB[93],
  real_T kC_l1, real_T kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T
  kC_l6, real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r);

#ifdef __WATCOMC__

#pragma aux heuristicSingleLeg value [8087];

#endif
#endif

/* End of code generation (heuristicSingleLeg.h) */
