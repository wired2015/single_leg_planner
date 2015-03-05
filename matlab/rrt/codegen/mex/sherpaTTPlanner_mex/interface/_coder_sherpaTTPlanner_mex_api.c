/*
 * _coder_sherpaTTPlanner_mex_api.c
 *
 * Code generation for function '_coder_sherpaTTPlanner_mex_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "_coder_sherpaTTPlanner_mex_api.h"
#include "sherpaTTPlanner_mex_emxutil.h"
#include "fprintf.h"
#include "sherpaTTPlanner_mex_mexutil.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo q_emlrtRTEI = { 1, 1, "_coder_sherpaTTPlanner_mex_api", "" };

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *nInitCartesianB, const char_T *identifier))[6];
static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static const mxArray *d_emlrt_marshallOut(const boolean_T u);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *jointLimits, const char_T *identifier))[20];
static const mxArray *e_emlrt_marshallOut(const emxArray_real_T *u);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[20];
static const mxArray *f_emlrt_marshallOut(const emxArray_real_T *u);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y);
static const mxArray *g_emlrt_marshallOut(const emxArray_real_T *u);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static int32_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *legNum,
  const char_T *identifier);
static int32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *uBDot,
  const char_T *identifier))[6];
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[20];
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);
static int32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv21[2] = { 0, 0 };

  const mxArray *m8;
  y = NULL;
  m8 = emlrtCreateNumericArray(2, iv21, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m8, (void *)u->data);
  emlrtSetDimensions((mxArray *)m8, u->size, 2);
  emlrtAssign(&y, m8);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *nInitCartesianB, const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nInitCartesianB), &thisId);
  emlrtDestroyArray(&nInitCartesianB);
  return y;
}
  static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv22[2] = { 0, 0 };

  const mxArray *m9;
  y = NULL;
  m9 = emlrtCreateNumericArray(2, iv22, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m9, (void *)u->data);
  emlrtSetDimensions((mxArray *)m9, u->size, 2);
  emlrtAssign(&y, m9);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *d_emlrt_marshallOut(const boolean_T u)
{
  const mxArray *y;
  const mxArray *m10;
  y = NULL;
  m10 = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m10);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *jointLimits, const char_T *identifier))[20]
{
  real_T (*y)[20];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(jointLimits), &thisId);
  emlrtDestroyArray(&jointLimits);
  return y;
}
  static const mxArray *e_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv23[2] = { 0, 0 };

  const mxArray *m11;
  y = NULL;
  m11 = emlrtCreateNumericArray(2, iv23, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m11, (void *)u->data);
  emlrtSetDimensions((mxArray *)m11, u->size, 2);
  emlrtAssign(&y, m11);
  return y;
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv20[2] = { 0, 0 };

  const mxArray *m7;
  y = NULL;
  m7 = emlrtCreateNumericArray(2, iv20, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m7, (void *)u->data);
  emlrtSetDimensions((mxArray *)m7, u->size, 2);
  emlrtAssign(&y, m7);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[20]
{
  real_T (*y)[20];
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *f_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv24[2] = { 0, 0 };

  const mxArray *m12;
  y = NULL;
  m12 = emlrtCreateNumericArray(2, iv24, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m12, (void *)u->data);
  emlrtSetDimensions((mxArray *)m12, u->size, 2);
  emlrtAssign(&y, m12);
  return y;
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  h_emlrt_marshallIn(sp, emlrtAlias(kC), &thisId, y);
  emlrtDestroyArray(&kC);
}

static const mxArray *g_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv25[2] = { 0, 0 };

  const mxArray *m13;
  y = NULL;
  m13 = emlrtCreateNumericArray(2, iv25, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m13, (void *)u->data);
  emlrtSetDimensions((mxArray *)m13, u->size, 2);
  emlrtAssign(&y, m13);
  return y;
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[13] = { "l1", "l2", "l3", "l4", "l5", "l6",
    "l7", "l8", "zeta", "r", "B2PXOffset", "B2PZOffset", "legAngleOffset" };

  thisId.fParent = parentId;
  emlrtCheckStructR2012b(sp, parentId, u, 13, fieldNames, 0U, 0);
  thisId.fIdentifier = "l1";
  y->l1 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l1")),
    &thisId);
  thisId.fIdentifier = "l2";
  y->l2 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l2")),
    &thisId);
  thisId.fIdentifier = "l3";
  y->l3 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l3")),
    &thisId);
  thisId.fIdentifier = "l4";
  y->l4 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l4")),
    &thisId);
  thisId.fIdentifier = "l5";
  y->l5 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l5")),
    &thisId);
  thisId.fIdentifier = "l6";
  y->l6 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l6")),
    &thisId);
  thisId.fIdentifier = "l7";
  y->l7 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l7")),
    &thisId);
  thisId.fIdentifier = "l8";
  y->l8 = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "l8")),
    &thisId);
  thisId.fIdentifier = "zeta";
  y->zeta = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "zeta")), &thisId);
  thisId.fIdentifier = "r";
  y->r = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0, "r")),
    &thisId);
  thisId.fIdentifier = "B2PXOffset";
  y->B2PXOffset = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "B2PXOffset")), &thisId);
  thisId.fIdentifier = "B2PZOffset";
  y->B2PZOffset = b_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "B2PZOffset")), &thisId);
  thisId.fIdentifier = "legAngleOffset";
  i_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "legAngleOffset")), &thisId, y->legAngleOffset);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static int32_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *legNum,
  const char_T *identifier)
{
  int32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = k_emlrt_marshallIn(sp, emlrtAlias(legNum), &thisId);
  emlrtDestroyArray(&legNum);
  return y;
}

static int32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *uBDot,
  const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = m_emlrt_marshallIn(sp, emlrtAlias(uBDot), &thisId);
  emlrtDestroyArray(&uBDot);
  return y;
}
  static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  int32_T iv26[2];
  int32_T i21;
  for (i21 = 0; i21 < 2; i21++) {
    iv26[i21] = 1 + 5 * i21;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv26);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[20]
{
  real_T (*ret)[20];
  int32_T iv27[2];
  int32_T i22;
  for (i22 = 0; i22 < 2; i22++) {
    iv27[i22] = 2 + (i22 << 3);
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv27);
  ret = (real_T (*)[20])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  int32_T iv28[2];
  int32_T i23;
  for (i23 = 0; i23 < 2; i23++) {
    iv28[i23] = 1 + 3 * i23;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv28);
  for (i23 = 0; i23 < 4; i23++) {
    ret[i23] = (*(real_T (*)[4])mxGetData(src))[i23];
  }

  emlrtDestroyArray(&src);
}

static int32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  int32_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 0U, 0);
  ret = *(int32_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  int32_T iv29[1];
  iv29[0] = 6;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv29);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  void buildBiDirectionalRRTWrapper_api(sherpaTTPlanner_mexStackData *SD, const
  mxArray * const prhs[8], const mxArray *plhs[5])
{
  emxArray_real_T *T1;
  emxArray_real_T *T2;
  emxArray_real_T *pathC;
  emxArray_real_T *pathJ;
  real_T (*nInitCartesianB)[6];
  real_T (*nGoalCartesianB)[6];
  real_T phiInit;
  real_T omegaInit;
  real_T (*jointLimits)[20];
  struct0_T kC;
  int32_T legNum;
  real_T (*uBDot)[6];
  boolean_T success;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &T1, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &T2, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &pathC, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &pathJ, 2, &q_emlrtRTEI, true);

  /* Marshall function inputs */
  nInitCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]),
    "nInitCartesianB");
  nGoalCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]),
    "nGoalCartesianB");
  phiInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phiInit");
  omegaInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "omegaInit");
  jointLimits = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "jointLimits");
  g_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "kC", &kC);
  legNum = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "legNum");
  uBDot = l_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "uBDot");

  /* Invoke the target function */
  buildBiDirectionalRRTWrapper(SD, &st, *nInitCartesianB, *nGoalCartesianB,
    phiInit, omegaInit, *jointLimits, &kC, legNum, *uBDot, T1, T2, pathC, pathJ,
    &success);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(T1);
  plhs[1] = emlrt_marshallOut(T2);
  plhs[2] = b_emlrt_marshallOut(pathC);
  plhs[3] = c_emlrt_marshallOut(pathJ);
  plhs[4] = d_emlrt_marshallOut(success);
  pathJ->canFreeData = false;
  emxFree_real_T(&pathJ);
  pathC->canFreeData = false;
  emxFree_real_T(&pathC);
  T2->canFreeData = false;
  emxFree_real_T(&T2);
  T1->canFreeData = false;
  emxFree_real_T(&T1);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

void buildRRTWrapper_api(sherpaTTPlanner_mexStackData *SD, const mxArray * const
  prhs[8], const mxArray *plhs[4])
{
  emxArray_real_T *T;
  emxArray_real_T *pathC;
  emxArray_real_T *pathJ;
  real_T (*nInitCartesianB)[6];
  real_T (*nGoalCartesianB)[6];
  real_T phiInit;
  real_T omegaInit;
  real_T (*jointLimits)[20];
  struct0_T kC;
  int32_T legNum;
  real_T (*uBDot)[6];
  boolean_T success;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &T, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &pathC, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &pathJ, 2, &q_emlrtRTEI, true);

  /* Marshall function inputs */
  nInitCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]),
    "nInitCartesianB");
  nGoalCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]),
    "nGoalCartesianB");
  phiInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phiInit");
  omegaInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "omegaInit");
  jointLimits = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "jointLimits");
  g_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "kC", &kC);
  legNum = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "legNum");
  uBDot = l_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "uBDot");

  /* Invoke the target function */
  buildRRTWrapper(SD, &st, *nInitCartesianB, *nGoalCartesianB, phiInit,
                  omegaInit, *jointLimits, &kC, legNum, *uBDot, T, pathC, pathJ,
                  &success);

  /* Marshall function outputs */
  plhs[0] = e_emlrt_marshallOut(T);
  plhs[1] = b_emlrt_marshallOut(pathC);
  plhs[2] = f_emlrt_marshallOut(pathJ);
  plhs[3] = d_emlrt_marshallOut(success);
  pathJ->canFreeData = false;
  emxFree_real_T(&pathJ);
  pathC->canFreeData = false;
  emxFree_real_T(&pathC);
  T->canFreeData = false;
  emxFree_real_T(&T);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

void randomStateGenerator_api(const mxArray * const prhs[5], const mxArray *
  plhs[1])
{
  emxArray_real_T *states;
  int32_T N;
  real_T (*jointLimits)[20];
  struct0_T kC;
  real_T panHeight;
  int32_T legNum;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &states, 2, &q_emlrtRTEI, true);

  /* Marshall function inputs */
  N = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "N");
  jointLimits = e_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "jointLimits");
  g_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "kC", &kC);
  panHeight = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "panHeight");
  legNum = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "legNum");

  /* Invoke the target function */
  randomStateGenerator(&st, N, *jointLimits, &kC, panHeight, legNum, states);

  /* Marshall function outputs */
  plhs[0] = g_emlrt_marshallOut(states);
  states->canFreeData = false;
  emxFree_real_T(&states);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_sherpaTTPlanner_mex_api.c) */
