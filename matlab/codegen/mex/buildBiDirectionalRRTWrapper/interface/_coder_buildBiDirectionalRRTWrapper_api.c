/*
 * _coder_buildBiDirectionalRRTWrapper_api.c
 *
 * Code generation for function '_coder_buildBiDirectionalRRTWrapper_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "_coder_buildBiDirectionalRRTWrapper_api.h"
#include "buildBiDirectionalRRTWrapper_emxutil.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo j_emlrtRTEI = { 1, 1,
  "_coder_buildBiDirectionalRRTWrapper_api", "" };

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *nInitCartesianB, const char_T *identifier))[6];
static const mxArray *c_emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static const mxArray *d_emlrt_marshallOut(const boolean_T u);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *jointLimits, const char_T *identifier))[20];
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *feval, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[20];
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y);
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
static real_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
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
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv10[2] = { 0, 0 };

  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv10, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u->data);
  emlrtSetDimensions((mxArray *)m2, u->size, 2);
  emlrtAssign(&y, m2);
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
  static const int32_T iv11[2] = { 0, 0 };

  const mxArray *m3;
  y = NULL;
  m3 = emlrtCreateNumericArray(2, iv11, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)u->data);
  emlrtSetDimensions((mxArray *)m3, u->size, 2);
  emlrtAssign(&y, m3);
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
  const mxArray *m4;
  y = NULL;
  m4 = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m4);
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
  static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *feval,
  const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(feval), &thisId);
  emlrtDestroyArray(&feval);
  return y;
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv9[2] = { 0, 0 };

  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv9, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u->data);
  emlrtSetDimensions((mxArray *)m1, u->size, 2);
  emlrtAssign(&y, m1);
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
  static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  h_emlrt_marshallIn(sp, emlrtAlias(kC), &thisId, y);
  emlrtDestroyArray(&kC);
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

static real_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  int32_T iv12[2];
  int32_T i16;
  for (i16 = 0; i16 < 2; i16++) {
    iv12[i16] = 1 + 5 * i16;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv12);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[20]
{
  real_T (*ret)[20];
  int32_T iv13[2];
  int32_T i17;
  for (i17 = 0; i17 < 2; i17++) {
    iv13[i17] = 2 + (i17 << 3);
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv13);
  ret = (real_T (*)[20])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  int32_T iv14[2];
  int32_T i18;
  for (i18 = 0; i18 < 2; i18++) {
    iv14[i18] = 1 + 3 * i18;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv14);
  for (i18 = 0; i18 < 4; i18++) {
    ret[i18] = (*(real_T (*)[4])mxGetData(src))[i18];
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
  int32_T iv15[1];
  iv15[0] = 6;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv15);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  void buildBiDirectionalRRTWrapper_api(c_buildBiDirectionalRRTWrapperS *SD,
  const mxArray * const prhs[9], const mxArray *plhs[5])
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
  real_T bodyHeight;
  struct0_T kC;
  int32_T legNum;
  real_T (*uBDot)[6];
  boolean_T success;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &T1, 2, &j_emlrtRTEI, true);
  emxInit_real_T(&st, &T2, 2, &j_emlrtRTEI, true);
  emxInit_real_T(&st, &pathC, 2, &j_emlrtRTEI, true);
  emxInit_real_T(&st, &pathJ, 2, &j_emlrtRTEI, true);

  /* Marshall function inputs */
  nInitCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]),
    "nInitCartesianB");
  nGoalCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]),
    "nGoalCartesianB");
  phiInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phiInit");
  omegaInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "omegaInit");
  jointLimits = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "jointLimits");
  bodyHeight = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "bodyHeight");
  g_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "kC", &kC);
  legNum = j_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "legNum");
  uBDot = l_emlrt_marshallIn(&st, emlrtAlias(prhs[8]), "uBDot");

  /* Invoke the target function */
  buildBiDirectionalRRTWrapper(SD, &st, *nInitCartesianB, *nGoalCartesianB,
    phiInit, omegaInit, *jointLimits, bodyHeight, &kC, legNum, *uBDot, T1, T2,
    pathC, pathJ, &success);

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

/* End of code generation (_coder_buildBiDirectionalRRTWrapper_api.c) */
