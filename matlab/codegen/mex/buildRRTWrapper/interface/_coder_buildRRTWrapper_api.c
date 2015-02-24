/*
 * _coder_buildRRTWrapper_api.c
 *
 * Code generation for function '_coder_buildRRTWrapper_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "_coder_buildRRTWrapper_api.h"
#include "buildRRTWrapper_emxutil.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRTEInfo o_emlrtRTEI = { 1, 1, "_coder_buildRRTWrapper_api", "" };

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
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *U, const
  char_T *identifier))[10];
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[10];
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4]);
static int32_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *legNum,
  const char_T *identifier);
static int32_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *uBDot,
  const char_T *identifier))[6];
static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[20];
static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[10];
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4]);
static int32_T u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  static const int32_T iv13[2] = { 0, 0 };

  const mxArray *m3;
  y = NULL;
  m3 = emlrtCreateNumericArray(2, iv13, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)u->data);
  emlrtSetDimensions((mxArray *)m3, u->size, 2);
  emlrtAssign(&y, m3);
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
  static const int32_T iv14[2] = { 0, 0 };

  const mxArray *m4;
  y = NULL;
  m4 = emlrtCreateNumericArray(2, iv14, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m4, (void *)u->data);
  emlrtSetDimensions((mxArray *)m4, u->size, 2);
  emlrtAssign(&y, m4);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *d_emlrt_marshallOut(const boolean_T u)
{
  const mxArray *y;
  const mxArray *m5;
  y = NULL;
  m5 = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m5);
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
  static const int32_T iv12[2] = { 0, 0 };

  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv12, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u->data);
  emlrtSetDimensions((mxArray *)m2, u->size, 2);
  emlrtAssign(&y, m2);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[20]
{
  real_T (*y)[20];
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *U,
  const char_T *identifier))[10]
{
  real_T (*y)[10];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(U), &thisId);
  emlrtDestroyArray(&U);
  return y;
}

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[10]
{
  real_T (*y)[10];
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *kC, const
  char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  j_emlrt_marshallIn(sp, emlrtAlias(kC), &thisId, y);
  emlrtDestroyArray(&kC);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
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
  k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2013a(sp, u, 0,
    "legAngleOffset")), &thisId, y->legAngleOffset);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[4])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static int32_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *legNum,
  const char_T *identifier)
{
  int32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = m_emlrt_marshallIn(sp, emlrtAlias(legNum), &thisId);
  emlrtDestroyArray(&legNum);
  return y;
}

static int32_T m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = u_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *uBDot,
  const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = o_emlrt_marshallIn(sp, emlrtAlias(uBDot), &thisId);
  emlrtDestroyArray(&uBDot);
  return y;
}
  static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = v_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  int32_T iv15[2];
  int32_T i9;
  for (i9 = 0; i9 < 2; i9++) {
    iv15[i9] = 1 + 5 * i9;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv15);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[20]
{
  real_T (*ret)[20];
  int32_T iv16[2];
  int32_T i10;
  for (i10 = 0; i10 < 2; i10++) {
    iv16[i10] = 2 + (i10 << 3);
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv16);
  ret = (real_T (*)[20])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[10]
{
  real_T (*ret)[10];
  int32_T iv17[2];
  int32_T i11;
  for (i11 = 0; i11 < 2; i11++) {
    iv17[i11] = 5 + -3 * i11;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv17);
  ret = (real_T (*)[10])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[4])
{
  int32_T iv18[2];
  int32_T i12;
  for (i12 = 0; i12 < 2; i12++) {
    iv18[i12] = 1 + 3 * i12;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv18);
  for (i12 = 0; i12 < 4; i12++) {
    ret[i12] = (*(real_T (*)[4])mxGetData(src))[i12];
  }

  emlrtDestroyArray(&src);
}

static int32_T u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  int32_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "int32", false, 0U, 0);
  ret = *(int32_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  int32_T iv19[1];
  iv19[0] = 6;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv19);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  void buildRRTWrapper_api(const mxArray * const prhs[13], const mxArray *plhs[4])
{
  emxArray_real_T *T;
  emxArray_real_T *pathC;
  emxArray_real_T *pathJ;
  real_T (*nInitCartesianB)[6];
  real_T (*nGoalCartesianB)[6];
  real_T phiInit;
  real_T omegaInit;
  real_T (*jointLimits)[20];
  real_T bodyHeight;
  real_T (*U)[10];
  real_T dt;
  real_T Dt;
  struct0_T kC;
  real_T threshold;
  int32_T legNum;
  real_T (*uBDot)[6];
  boolean_T success;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &T, 2, &o_emlrtRTEI, true);
  emxInit_real_T(&st, &pathC, 2, &o_emlrtRTEI, true);
  emxInit_real_T(&st, &pathJ, 2, &o_emlrtRTEI, true);

  /* Marshall function inputs */
  nInitCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]),
    "nInitCartesianB");
  nGoalCartesianB = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]),
    "nGoalCartesianB");
  phiInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phiInit");
  omegaInit = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "omegaInit");
  jointLimits = e_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "jointLimits");
  bodyHeight = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "bodyHeight");
  U = g_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "U");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "dt");
  Dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "Dt");
  i_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "kC", &kC);
  threshold = emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "threshold");
  legNum = l_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "legNum");
  uBDot = n_emlrt_marshallIn(&st, emlrtAlias(prhs[12]), "uBDot");

  /* Invoke the target function */
  buildRRTWrapper(&st, *nInitCartesianB, *nGoalCartesianB, phiInit, omegaInit,
                  *jointLimits, bodyHeight, *U, dt, Dt, &kC, threshold, legNum, *
                  uBDot, T, pathC, pathJ, &success);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(T);
  plhs[1] = b_emlrt_marshallOut(pathC);
  plhs[2] = c_emlrt_marshallOut(pathJ);
  plhs[3] = d_emlrt_marshallOut(success);
  pathJ->canFreeData = false;
  emxFree_real_T(&pathJ);
  pathC->canFreeData = false;
  emxFree_real_T(&pathC);
  T->canFreeData = false;
  emxFree_real_T(&T);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_buildRRTWrapper_api.c) */
