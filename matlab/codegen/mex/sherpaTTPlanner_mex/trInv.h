/*
 * trInv.h
 *
 * Code generation for function 'trInv'
 *
 */

#ifndef __TRINV_H__
#define __TRINV_H__

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
extern void trInv(const real_T T[16], real_T TInv[16]);

#endif

/* End of code generation (trInv.h) */
