/*
 * norm.h
 *
 * Code generation for function 'norm'
 *
 */

#ifndef __NORM_H__
#define __NORM_H__

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
extern real_T b_norm(const real_T x[3]);

#ifdef __WATCOMC__

#pragma aux b_norm value [8087];

#endif

extern real_T norm(const real_T x[3]);

#ifdef __WATCOMC__

#pragma aux norm value [8087];

#endif
#endif

/* End of code generation (norm.h) */
