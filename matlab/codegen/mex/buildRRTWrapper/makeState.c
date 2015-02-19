/*
 * makeState.c
 *
 * Code generation for function 'makeState'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "makeState.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo k_emlrtRTEI = { 10, 14, "makeState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/makeState.m"
};

/* Function Definitions */
void makeState(const emlrtStack *sp, real_T alpha, real_T beta, real_T b_gamma,
               real_T alphaDot, real_T betaDot, real_T gammaDot, real_T phi,
               real_T omega, real_T dt, real_T Dt, struct1_T *s)
{
  emxArray_struct2_T *transitionArray;
  real_T y;
  int32_T i5;
  static const struct2_T r1 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  real_T d0;
  int32_T i;
  int32_T xs;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_struct2_T(sp, &transitionArray, 2, &l_emlrtRTEI, true);

  /* MAKESTATE Makes a structure containing state information. */
  /*  */
  /* Inputs: */
  /* Outputs: */
  /*  */
  /* makeState.m */
  /* author:wreid */
  /* date:20150218 */
  y = Dt / dt;
  i5 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = 1;
  emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i5, (int32_T)sizeof
                    (struct2_T), &k_emlrtRTEI);
  transitionArray->data[0] = r1;
  d0 = muDoubleScalarRound(y) - 1.0;
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, muDoubleScalarRound(y) - 1.0,
    mxDOUBLE_CLASS, (int32_T)d0, &y_emlrtRTEI, sp);
  i = 0;
  while (i <= (int32_T)d0 - 1) {
    xs = transitionArray->size[1];
    i5 = transitionArray->size[0] * transitionArray->size[1];
    transitionArray->size[1] = xs + 1;
    emxEnsureCapacity(sp, (emxArray__common *)transitionArray, i5, (int32_T)
                      sizeof(struct2_T), &k_emlrtRTEI);
    transitionArray->data[xs] = r1;
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, sp);
  }

  s->id = 1.0;
  s->parentID = 0.0;
  s->cost = 0.0;
  s->alpha = alpha;
  s->beta = beta;
  s->gamma = b_gamma;
  s->alphaDot = alphaDot;
  s->betaDot = betaDot;
  s->gammaDot = gammaDot;
  s->phi = phi;
  s->omega = omega;
  i5 = s->transitionArray->size[0] * s->transitionArray->size[1];
  s->transitionArray->size[0] = 1;
  s->transitionArray->size[1] = transitionArray->size[1];
  emxEnsureCapacity(sp, (emxArray__common *)s->transitionArray, i5, (int32_T)
                    sizeof(struct2_T), &k_emlrtRTEI);
  i = transitionArray->size[0] * transitionArray->size[1];
  for (i5 = 0; i5 < i; i5++) {
    s->transitionArray->data[i5] = transitionArray->data[i5];
  }

  emxFree_struct2_T(&transitionArray);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (makeState.c) */
