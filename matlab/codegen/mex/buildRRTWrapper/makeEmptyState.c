/*
 * makeEmptyState.c
 *
 * Code generation for function 'makeEmptyState'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "makeEmptyState.h"
#include "buildRRTWrapper_emxutil.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo p_emlrtRSI = { 2, "makeEmptyState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/makeEmptyState.m"
};

static emlrtRTEInfo m_emlrtRTEI = { 1, 14, "makeEmptyState",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/makeEmptyState.m"
};

/* Function Definitions */
void makeEmptyState(const emlrtStack *sp, real_T dt, real_T Dt, real_T *s_id,
                    real_T *s_parentID, real_T *s_cost, real_T *s_alpha, real_T *
                    s_beta, real_T *s_gamma, real_T *s_alphaDot, real_T
                    *s_betaDot, real_T *s_gammaDot, real_T *s_phi, real_T
                    *s_omega, emxArray_struct2_T *s_transitionArray)
{
  emxArray_struct2_T *transitionArray;
  real_T y;
  int32_T i6;
  static const struct2_T r2 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  real_T d1;
  int32_T i;
  int32_T xs;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_struct2_T(sp, &transitionArray, 2, &l_emlrtRTEI, true);
  st.site = &p_emlrtRSI;

  /* MAKESTATE Makes a structure containing state information. */
  /*  */
  /* Inputs: */
  /* Outputs: */
  /*  */
  /* makeState.m */
  /* author:wreid */
  /* date:20150218 */
  y = Dt / dt;
  i6 = transitionArray->size[0] * transitionArray->size[1];
  transitionArray->size[0] = 1;
  transitionArray->size[1] = 1;
  emxEnsureCapacity(&st, (emxArray__common *)transitionArray, i6, (int32_T)
                    sizeof(struct2_T), &m_emlrtRTEI);
  transitionArray->data[0] = r2;
  d1 = muDoubleScalarRound(y) - 1.0;
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, muDoubleScalarRound(y) - 1.0,
    mxDOUBLE_CLASS, (int32_T)d1, &y_emlrtRTEI, &st);
  i = 0;
  while (i <= (int32_T)d1 - 1) {
    xs = transitionArray->size[1];
    i6 = transitionArray->size[0] * transitionArray->size[1];
    transitionArray->size[1] = xs + 1;
    emxEnsureCapacity(&st, (emxArray__common *)transitionArray, i6, (int32_T)
                      sizeof(struct2_T), &m_emlrtRTEI);
    transitionArray->data[xs] = r2;
    i++;
    emlrtBreakCheckFastR2012b(emlrtBreakCheckR2012bFlagVar, &st);
  }

  *s_id = 0.0;
  *s_parentID = 0.0;
  *s_cost = 0.0;
  *s_alpha = 0.0;
  *s_beta = 0.0;
  *s_gamma = 0.0;
  *s_alphaDot = 0.0;
  *s_betaDot = 0.0;
  *s_gammaDot = 0.0;
  *s_phi = 0.0;
  *s_omega = 0.0;
  i6 = s_transitionArray->size[0] * s_transitionArray->size[1];
  s_transitionArray->size[0] = 1;
  s_transitionArray->size[1] = transitionArray->size[1];
  emxEnsureCapacity(&st, (emxArray__common *)s_transitionArray, i6, (int32_T)
                    sizeof(struct2_T), &m_emlrtRTEI);
  i = transitionArray->size[0] * transitionArray->size[1];
  for (i6 = 0; i6 < i; i6++) {
    s_transitionArray->data[i6] = transitionArray->data[i6];
  }

  emxFree_struct2_T(&transitionArray);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (makeEmptyState.c) */
