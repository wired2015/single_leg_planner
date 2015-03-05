/*
 * angDiff.h
 *
 * Code generation for function 'angDiff'
 *
 */

#ifndef __ANGDIFF_H__
#define __ANGDIFF_H__

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
extern real_T angDiff(real_T th1, real_T th2);

#ifdef __WATCOMC__

#pragma aux angDiff value [8087];

#endif
#endif

/* End of code generation (angDiff.h) */
