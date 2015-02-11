/*
 * sherpaTTIK.c
 *
 * Code generation for function 'sherpaTTIK'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "sherpaTTIK.h"
#include "eml_error.h"
#include "asin.h"
#include "sin.h"
#include "log.h"
#include "exp.h"
#include "extractKinematicConstants.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo e_emlrtRSI = { 13, "sherpaTTIK",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/sherpaTTIK.m"
};

/* Function Definitions */
void sherpaTTIK(const emlrtStack *sp, real_T xC, real_T yC, real_T zC, const
                real_T kinematicConst[16], const real_T jointLimits[12], real_T *
                alpha, real_T *beta, real_T *b_gamma)
{
  real_T unusedU6;
  real_T unusedU5;
  real_T unusedU4;
  real_T unusedU3;
  real_T unusedU2;
  real_T unusedU1;
  real_T r;
  real_T zeta;
  real_T L8;
  real_T L7;
  real_T L6;
  real_T L5;
  real_T L4;
  real_T L3;
  real_T L2;
  real_T L1;
  real_T x;
  real_T a;
  real_T b_x;
  real_T c_x;
  real_T b_a;
  real_T d_x;
  real_T e_x;
  creal_T dc0;
  creal_T dc1;
  creal_T dc2;
  creal_T dc3;
  creal_T dc4;
  creal_T dc5;
  creal_T dc6;
  creal_T dc7;
  creal_T dc8;
  creal_T dc9;
  creal_T dc10;
  creal_T dc11;
  creal_T dc12;
  creal_T dc13;
  creal_T dc14;
  creal_T dc15;
  creal_T dc16;
  creal_T dc17;
  creal_T dc18;
  real_T L1_im;
  real_T L6_re;
  real_T L6_im;
  real_T L8_re;
  real_T L8_im;
  real_T re;
  real_T L4_re;
  real_T L4_im;
  real_T L2_re;
  real_T L2_im;
  creal_T dc19;
  creal_T dc20;
  creal_T dc21;
  creal_T dc22;
  creal_T f_x;
  creal_T g_x;
  creal_T h_x;
  creal_T i_x;
  creal_T j_x;
  creal_T k_x;
  creal_T dc23;
  creal_T dc24;
  real_T L3_re;
  real_T L3_im;
  real_T L5_re;
  real_T L5_im;
  real_T L7_re;
  real_T L7_im;
  real_T x_re;
  real_T x_im;
  real_T L1_re;
  real_T b_L1_im;
  real_T b_L1_re;
  real_T c_L1_im;
  real_T b_L2_re;
  real_T b_L2_im;
  real_T b_L6_re;
  real_T b_L6_im;
  real_T c_L2_re;
  real_T c_L2_im;
  real_T b_L7_re;
  real_T b_L7_im;
  real_T c_L1_re;
  real_T d_L1_im;
  real_T c_L6_re;
  real_T c_L6_im;
  real_T b_L8_re;
  real_T b_L8_im;
  real_T b_L4_re;
  real_T b_L4_im;
  real_T c_L4_re;
  real_T c_L4_im;
  real_T b_x_re;
  real_T b_x_im;
  real_T d_L2_re;
  real_T d_L2_im;
  real_T c_L7_re;
  real_T c_L7_im;
  real_T ar;
  creal_T y;
  creal_T gammaRaw[2];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* sherpaTTIK.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTIK Calculates the joint values for a g1iven contact point. */
  /*    Calculates the joint values for a g1iven contact point for the Sherpa TT */
  /*    leg. All coord1inates are in the pan joint coord1inate frame. */
  extractKinematicConstants(kinematicConst, &L1, &L2, &L3, &L4, &L5, &L6, &L7,
    &L8, &zeta, &r, &unusedU1, &unusedU2, &unusedU3, &unusedU4, &unusedU5,
    &unusedU6);
  st.site = &e_emlrtRSI;
  x = xC * xC + yC * yC;
  if (x < 0.0) {
    b_st.site = &f_emlrtRSI;
    eml_error(&b_st);
  }

  x = muDoubleScalarSqrt(x);
  a = ((((((((((((((((((((((((L1 * L1 - 2.0 * muDoubleScalarSin(zeta) * L1 * L4)
    - 2.0 * L1 * L6) - 2.0 * L1 * L8) - 2.0 * L1 * zC) + L2 * L2) + 2.0 *
    muDoubleScalarCos(zeta) * L2 * L4) - 2.0 * L2 * L7) - 2.0 * L2 * x) - L3 *
                      L3) + L4 * L4) + 2.0 * muDoubleScalarSin(zeta) * L4 * L6)
                   - 2.0 * muDoubleScalarCos(zeta) * L4 * L7) + 2.0 *
                  muDoubleScalarSin(zeta) * L4 * L8) - 2.0 * muDoubleScalarCos
                 (zeta) * L4 * x) + 2.0 * muDoubleScalarSin(zeta) * L4 * zC) +
               L5 * L5) + L6 * L6) + 2.0 * L6 * L8) + 2.0 * L6 * zC) + L7 * L7)
          + 2.0 * L7 * x) + L8 * L8) + 2.0 * L8 * zC) + x * x) + zC * zC;
  b_x = muDoubleScalarCos(zeta);
  c_x = muDoubleScalarSin(zeta);
  b_a = ((((((((((((((((((((((((L1 * L1 - 2.0 * muDoubleScalarSin(zeta) * L1 *
    L4) - 2.0 * L1 * L6) - 2.0 * L1 * L8) - 2.0 * L1 * zC) + L2 * L2) + 2.0 *
    muDoubleScalarCos(zeta) * L2 * L4) - 2.0 * L2 * L7) - 2.0 * L2 * x) - L3 *
                        L3) + L4 * L4) + 2.0 * muDoubleScalarSin(zeta) * L4 * L6)
                     - 2.0 * muDoubleScalarCos(zeta) * L4 * L7) + 2.0 *
                    muDoubleScalarSin(zeta) * L4 * L8) - 2.0 * muDoubleScalarCos
                   (zeta) * L4 * x) + 2.0 * muDoubleScalarSin(zeta) * L4 * zC) +
                 L5 * L5) + L6 * L6) + 2.0 * L6 * L8) + 2.0 * L6 * zC) + L7 * L7)
            + 2.0 * L7 * x) + L8 * L8) + 2.0 * L8 * zC) + x * x) + zC * zC;
  d_x = muDoubleScalarCos(zeta);
  e_x = muDoubleScalarSin(zeta);
  dc0.re = zeta * 0.0;
  dc0.im = zeta;
  b_exp(&dc0);
  dc1.re = zeta * 0.0;
  dc1.im = zeta;
  b_exp(&dc1);
  dc2.re = zeta * 0.0;
  dc2.im = zeta;
  b_exp(&dc2);
  dc3.re = zeta * 0.0;
  dc3.im = zeta;
  b_exp(&dc3);
  dc4.re = zeta * 0.0;
  dc4.im = zeta;
  b_exp(&dc4);
  dc5.re = zeta * 0.0;
  dc5.im = zeta;
  b_exp(&dc5);
  dc6.re = zeta * 0.0;
  dc6.im = zeta;
  b_exp(&dc6);
  dc7.re = zeta * 0.0;
  dc7.im = zeta;
  b_exp(&dc7);
  dc8.re = zeta * 0.0;
  dc8.im = zeta;
  b_exp(&dc8);
  dc9.re = zeta * 2.0 * 0.0;
  dc9.im = zeta * 2.0;
  b_exp(&dc9);
  dc10.re = zeta * 0.0;
  dc10.im = zeta;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  unusedU5 = zC * dc12.re;
  unusedU6 = zC * dc12.im;
  r = L1 * dc13.re;
  L1_im = L1 * dc13.im;
  L6_re = L6 * dc15.re;
  L6_im = L6 * dc15.im;
  L8_re = L8 * dc17.re;
  L8_im = L8 * dc17.im;
  re = 4.0 * (L5 * L5) * dc10.re;
  unusedU3 = 4.0 * (L5 * L5) * dc10.im;
  L4_re = ((((((-L4 + x * dc11.re) + (unusedU5 * 0.0 - unusedU6)) - (r * 0.0 -
    L1_im)) - L2 * dc14.re) + (L6_re * 0.0 - L6_im)) + L7 * dc16.re) + (L8_re *
    0.0 - L8_im);
  L4_im = (((((x * dc11.im + (unusedU5 + unusedU6 * 0.0)) - (r + L1_im * 0.0)) -
             L2 * dc14.im) + (L6_re + L6_im * 0.0)) + L7 * dc16.im) + (L8_re +
    L8_im * 0.0);
  unusedU2 = re * L4_re - unusedU3 * L4_im;
  unusedU3 = re * L4_im + unusedU3 * L4_re;
  L2_re = ((((((L2 - L1 * 0.0) + L6 * 0.0) - L7) + L8 * 0.0) - x) + zC * 0.0) +
    L4 * dc18.re;
  L2_im = ((((0.0 - L1) + L6) + L8) + zC) + L4 * dc18.im;
  dc9.re = dc9.re * (a * a) + (unusedU2 * L2_re - unusedU3 * L2_im);
  dc9.im = dc9.im * (a * a) + (unusedU2 * L2_im + unusedU3 * L2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = zeta * 2.0 * 0.0;
  dc10.im = zeta * 2.0;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  dc19.re = zeta * 0.0;
  dc19.im = zeta;
  b_exp(&dc19);
  dc20.re = zeta * 0.0;
  dc20.im = zeta;
  b_exp(&dc20);
  dc21.re = zeta * 0.0;
  dc21.im = zeta;
  b_exp(&dc21);
  dc22.re = zeta * 2.0 * 0.0;
  dc22.im = zeta * 2.0;
  b_exp(&dc22);
  f_x.re = zeta * 2.0 * 0.0;
  f_x.im = zeta * 2.0;
  b_exp(&f_x);
  g_x.re = zeta * 0.0;
  g_x.im = zeta;
  b_exp(&g_x);
  h_x.re = zeta * 0.0;
  h_x.im = zeta;
  b_exp(&h_x);
  i_x.re = zeta * 0.0;
  i_x.im = zeta;
  b_exp(&i_x);
  j_x.re = zeta * 0.0;
  j_x.im = zeta;
  b_exp(&j_x);
  k_x.re = zeta * 0.0;
  k_x.im = zeta;
  b_exp(&k_x);
  dc23.re = zeta * 0.0;
  dc23.im = zeta;
  b_exp(&dc23);
  dc24.re = zeta * 0.0;
  dc24.im = zeta;
  b_exp(&dc24);
  r = L1 * L1 * dc0.re;
  L1_im = L1 * L1 * dc0.im;
  L2_re = L2 * L2 * dc1.re;
  L2_im = L2 * L2 * dc1.im;
  L3_re = L3 * L3 * dc2.re;
  L3_im = L3 * L3 * dc2.im;
  L5_re = L5 * L5 * dc3.re;
  L5_im = L5 * L5 * dc3.im;
  L6_re = L6 * L6 * dc4.re;
  L6_im = L6 * L6 * dc4.im;
  L7_re = L7 * L7 * dc5.re;
  L7_im = L7 * L7 * dc5.im;
  L8_re = L8 * L8 * dc6.re;
  L8_im = L8 * L8 * dc6.im;
  x_re = x * x * dc7.re;
  x_im = x * x * dc7.im;
  unusedU5 = zC * zC * dc8.re;
  unusedU6 = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    re = dc10.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc10.re == 0.0) {
    re = 0.0;
    unusedU3 = dc10.im / 2.0;
  } else {
    re = dc10.re / 2.0;
    unusedU3 = dc10.im / 2.0;
  }

  L4_re = 2.0 * (L4 * x * (re + 0.5));
  L4_im = 2.0 * (L4 * x * unusedU3);
  L1_re = 2.0 * (L1 * L6 * dc11.re);
  b_L1_im = 2.0 * (L1 * L6 * dc11.im);
  b_L1_re = 2.0 * (L1 * L8 * dc12.re);
  c_L1_im = 2.0 * (L1 * L8 * dc12.im);
  b_L2_re = 2.0 * (L2 * L7 * dc13.re);
  b_L2_im = 2.0 * (L2 * L7 * dc13.im);
  b_L6_re = 2.0 * (L6 * L8 * dc14.re);
  b_L6_im = 2.0 * (L6 * L8 * dc14.im);
  c_L2_re = 2.0 * (L2 * x * dc15.re);
  c_L2_im = 2.0 * (L2 * x * dc15.im);
  b_L7_re = 2.0 * (L7 * x * dc16.re);
  b_L7_im = 2.0 * (L7 * x * dc16.im);
  c_L1_re = 2.0 * (L1 * zC * dc17.re);
  d_L1_im = 2.0 * (L1 * zC * dc17.im);
  c_L6_re = 2.0 * (L6 * zC * dc18.re);
  c_L6_im = 2.0 * (L6 * zC * dc18.im);
  b_L8_re = 2.0 * (L8 * zC * dc19.re);
  b_L8_im = 2.0 * (L8 * zC * dc19.im);
  b_L4_re = b_x * b_x * (L4 * L4 * dc20.re);
  b_L4_im = b_x * b_x * (L4 * L4 * dc20.im);
  c_L4_re = c_x * c_x * (L4 * L4 * dc21.re);
  c_L4_im = c_x * c_x * (L4 * L4 * dc21.im);
  if (dc22.im == 0.0) {
    re = dc22.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc22.re == 0.0) {
    re = 0.0;
    unusedU3 = dc22.im / 2.0;
  } else {
    re = dc22.re / 2.0;
    unusedU3 = dc22.im / 2.0;
  }

  unusedU4 = 2.0 * (L4 * L7 * (re + 0.5));
  unusedU3 = 2.0 * (L4 * L7 * unusedU3);
  b_x_re = x * g_x.re;
  b_x_im = x * g_x.im;
  d_L2_re = L2 * j_x.re;
  d_L2_im = L2 * j_x.im;
  c_L7_re = L7 * dc23.re;
  c_L7_im = L7 * dc23.im;
  ar = -((((((((((((((((((((((((((((r * 0.0 - L1_im) + (L2_re * 0.0 - L2_im)) -
    (L3_re * 0.0 - L3_im)) + (L5_re * 0.0 - L5_im)) + (L6_re * 0.0 - L6_im)) +
    (L7_re * 0.0 - L7_im)) + (L8_re * 0.0 - L8_im)) + (x_re * 0.0 - x_im)) +
    (unusedU5 * 0.0 - unusedU6)) - (dc9.re * 0.0 - dc9.im)) - (L4_re * 0.0 -
    L4_im)) - (L1_re * 0.0 - b_L1_im)) - (b_L1_re * 0.0 - c_L1_im)) - (b_L2_re *
    0.0 - b_L2_im)) + (b_L6_re * 0.0 - b_L6_im)) - L1 * L4 * (muDoubleScalarCos
    (2.0 * zeta) - 1.0)) + L4 * L6 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + L4
                   * L8 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) - (c_L2_re *
    0.0 - c_L2_im)) + (b_L7_re * 0.0 - b_L7_im)) - (c_L1_re * 0.0 - d_L1_im)) +
               (c_L6_re * 0.0 - c_L6_im)) + (b_L8_re * 0.0 - b_L8_im)) + L4 * zC
             * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + (b_L4_re * 0.0 - b_L4_im))
           + (c_L4_re * 0.0 - c_L4_im)) - (unusedU4 * 0.0 - unusedU3)) + L2 * L4
         * (f_x.re * 0.0 - f_x.im));
  unusedU6 = -((((((((((((((((((((((((((((r + L1_im * 0.0) + (L2_re + L2_im *
    0.0)) - (L3_re + L3_im * 0.0)) + (L5_re + L5_im * 0.0)) + (L6_re + L6_im *
    0.0)) + (L7_re + L7_im * 0.0)) + (L8_re + L8_im * 0.0)) + (x_re + x_im * 0.0))
    + (unusedU5 + unusedU6 * 0.0)) - (dc9.re + dc9.im * 0.0)) - (L4_re + L4_im *
    0.0)) - (L1_re + b_L1_im * 0.0)) - (b_L1_re + c_L1_im * 0.0)) - (b_L2_re +
    b_L2_im * 0.0)) + (b_L6_re + b_L6_im * 0.0)) - L1 * L4 * muDoubleScalarSin
    (2.0 * zeta)) + L4 * L6 * muDoubleScalarSin(2.0 * zeta)) + L4 * L8 *
    muDoubleScalarSin(2.0 * zeta)) - (c_L2_re + c_L2_im * 0.0)) + (b_L7_re +
    b_L7_im * 0.0)) - (c_L1_re + d_L1_im * 0.0)) + (c_L6_re + c_L6_im * 0.0)) +
                    (b_L8_re + b_L8_im * 0.0)) + L4 * zC * muDoubleScalarSin(2.0
    * zeta)) + (b_L4_re + b_L4_im * 0.0)) + (c_L4_re + c_L4_im * 0.0)) -
                (unusedU4 + unusedU3 * 0.0)) + L2 * L4 * ((f_x.re + f_x.im * 0.0)
    + 1.0));
  unusedU5 = 2.0 * L5 * (((((((L4 * 0.0 - (b_x_re * 0.0 - b_x_im)) + zC * h_x.re)
    - L1 * i_x.re) + (d_L2_re * 0.0 - d_L2_im)) + L6 * k_x.re) - (c_L7_re * 0.0
    - c_L7_im)) + L8 * dc24.re);
  unusedU3 = 2.0 * L5 * (((((((L4 - (b_x_re + b_x_im * 0.0)) + zC * h_x.im) - L1
    * i_x.im) + (d_L2_re + d_L2_im * 0.0)) + L6 * k_x.im) - (c_L7_re + c_L7_im *
    0.0)) + L8 * dc24.im);
  if (unusedU3 == 0.0) {
    if (unusedU6 == 0.0) {
      dc0.re = ar / unusedU5;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = unusedU6 / unusedU5;
    } else {
      dc0.re = ar / unusedU5;
      dc0.im = unusedU6 / unusedU5;
    }
  } else if (unusedU5 == 0.0) {
    if (ar == 0.0) {
      dc0.re = unusedU6 / unusedU3;
      dc0.im = 0.0;
    } else if (unusedU6 == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / unusedU3);
    } else {
      dc0.re = unusedU6 / unusedU3;
      dc0.im = -(ar / unusedU3);
    }
  } else {
    unusedU4 = muDoubleScalarAbs(unusedU5);
    unusedU1 = muDoubleScalarAbs(unusedU3);
    if (unusedU4 > unusedU1) {
      unusedU2 = unusedU3 / unusedU5;
      unusedU1 = unusedU5 + unusedU2 * unusedU3;
      dc0.re = (ar + unusedU2 * unusedU6) / unusedU1;
      dc0.im = (unusedU6 - unusedU2 * ar) / unusedU1;
    } else if (unusedU1 == unusedU4) {
      if (unusedU5 > 0.0) {
        unusedU2 = 0.5;
      } else {
        unusedU2 = -0.5;
      }

      if (unusedU3 > 0.0) {
        unusedU1 = 0.5;
      } else {
        unusedU1 = -0.5;
      }

      dc0.re = (ar * unusedU2 + unusedU6 * unusedU1) / unusedU4;
      dc0.im = (unusedU6 * unusedU2 - ar * unusedU1) / unusedU4;
    } else {
      unusedU2 = unusedU5 / unusedU3;
      unusedU1 = unusedU3 + unusedU2 * unusedU5;
      dc0.re = (unusedU2 * ar + unusedU6) / unusedU1;
      dc0.im = (unusedU2 * unusedU6 - ar) / unusedU1;
    }
  }

  b_log(&dc0);
  dc1.re = zeta * 0.0;
  dc1.im = zeta;
  b_exp(&dc1);
  dc2.re = zeta * 0.0;
  dc2.im = zeta;
  b_exp(&dc2);
  dc3.re = zeta * 0.0;
  dc3.im = zeta;
  b_exp(&dc3);
  dc4.re = zeta * 0.0;
  dc4.im = zeta;
  b_exp(&dc4);
  dc5.re = zeta * 0.0;
  dc5.im = zeta;
  b_exp(&dc5);
  dc6.re = zeta * 0.0;
  dc6.im = zeta;
  b_exp(&dc6);
  dc7.re = zeta * 0.0;
  dc7.im = zeta;
  b_exp(&dc7);
  dc8.re = zeta * 0.0;
  dc8.im = zeta;
  b_exp(&dc8);
  dc9.re = zeta * 0.0;
  dc9.im = zeta;
  b_exp(&dc9);
  dc10.re = zeta * 2.0 * 0.0;
  dc10.im = zeta * 2.0;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  dc19.re = zeta * 0.0;
  dc19.im = zeta;
  b_exp(&dc19);
  unusedU5 = zC * dc13.re;
  unusedU6 = zC * dc13.im;
  r = L1 * dc14.re;
  L1_im = L1 * dc14.im;
  L6_re = L6 * dc16.re;
  L6_im = L6 * dc16.im;
  L8_re = L8 * dc18.re;
  L8_im = L8 * dc18.im;
  re = 4.0 * (L5 * L5) * dc11.re;
  unusedU3 = 4.0 * (L5 * L5) * dc11.im;
  L4_re = ((((((-L4 + x * dc12.re) + (unusedU5 * 0.0 - unusedU6)) - (r * 0.0 -
    L1_im)) - L2 * dc15.re) + (L6_re * 0.0 - L6_im)) + L7 * dc17.re) + (L8_re *
    0.0 - L8_im);
  L4_im = (((((x * dc12.im + (unusedU5 + unusedU6 * 0.0)) - (r + L1_im * 0.0)) -
             L2 * dc15.im) + (L6_re + L6_im * 0.0)) + L7 * dc17.im) + (L8_re +
    L8_im * 0.0);
  unusedU2 = re * L4_re - unusedU3 * L4_im;
  unusedU3 = re * L4_im + unusedU3 * L4_re;
  L2_re = ((((((L2 - L1 * 0.0) + L6 * 0.0) - L7) + L8 * 0.0) - x) + zC * 0.0) +
    L4 * dc19.re;
  L2_im = ((((0.0 - L1) + L6) + L8) + zC) + L4 * dc19.im;
  dc10.re = dc10.re * (b_a * b_a) + (unusedU2 * L2_re - unusedU3 * L2_im);
  dc10.im = dc10.im * (b_a * b_a) + (unusedU2 * L2_im + unusedU3 * L2_re);
  eml_scalar_sqrt(&dc10);
  dc11.re = zeta * 2.0 * 0.0;
  dc11.im = zeta * 2.0;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  dc19.re = zeta * 0.0;
  dc19.im = zeta;
  b_exp(&dc19);
  dc20.re = zeta * 0.0;
  dc20.im = zeta;
  b_exp(&dc20);
  dc21.re = zeta * 0.0;
  dc21.im = zeta;
  b_exp(&dc21);
  dc22.re = zeta * 0.0;
  dc22.im = zeta;
  b_exp(&dc22);
  f_x.re = zeta * 2.0 * 0.0;
  f_x.im = zeta * 2.0;
  b_exp(&f_x);
  g_x.re = zeta * 2.0 * 0.0;
  g_x.im = zeta * 2.0;
  b_exp(&g_x);
  h_x.re = zeta * 0.0;
  h_x.im = zeta;
  b_exp(&h_x);
  i_x.re = zeta * 0.0;
  i_x.im = zeta;
  b_exp(&i_x);
  j_x.re = zeta * 0.0;
  j_x.im = zeta;
  b_exp(&j_x);
  k_x.re = zeta * 0.0;
  k_x.im = zeta;
  b_exp(&k_x);
  dc23.re = zeta * 0.0;
  dc23.im = zeta;
  b_exp(&dc23);
  dc24.re = zeta * 0.0;
  dc24.im = zeta;
  b_exp(&dc24);
  y.re = zeta * 0.0;
  y.im = zeta;
  b_exp(&y);
  r = L1 * L1 * dc1.re;
  L1_im = L1 * L1 * dc1.im;
  L2_re = L2 * L2 * dc2.re;
  L2_im = L2 * L2 * dc2.im;
  L3_re = L3 * L3 * dc3.re;
  L3_im = L3 * L3 * dc3.im;
  L5_re = L5 * L5 * dc4.re;
  L5_im = L5 * L5 * dc4.im;
  L6_re = L6 * L6 * dc5.re;
  L6_im = L6 * L6 * dc5.im;
  L7_re = L7 * L7 * dc6.re;
  L7_im = L7 * L7 * dc6.im;
  L8_re = L8 * L8 * dc7.re;
  L8_im = L8 * L8 * dc7.im;
  x_re = x * x * dc8.re;
  x_im = x * x * dc8.im;
  unusedU5 = zC * zC * dc9.re;
  unusedU6 = zC * zC * dc9.im;
  if (dc11.im == 0.0) {
    re = dc11.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc11.re == 0.0) {
    re = 0.0;
    unusedU3 = dc11.im / 2.0;
  } else {
    re = dc11.re / 2.0;
    unusedU3 = dc11.im / 2.0;
  }

  L4_re = 2.0 * (L4 * x * (re + 0.5));
  L4_im = 2.0 * (L4 * x * unusedU3);
  L1_re = 2.0 * (L1 * L6 * dc12.re);
  b_L1_im = 2.0 * (L1 * L6 * dc12.im);
  b_L1_re = 2.0 * (L1 * L8 * dc13.re);
  c_L1_im = 2.0 * (L1 * L8 * dc13.im);
  b_L2_re = 2.0 * (L2 * L7 * dc14.re);
  b_L2_im = 2.0 * (L2 * L7 * dc14.im);
  b_L6_re = 2.0 * (L6 * L8 * dc15.re);
  b_L6_im = 2.0 * (L6 * L8 * dc15.im);
  c_L2_re = 2.0 * (L2 * x * dc16.re);
  c_L2_im = 2.0 * (L2 * x * dc16.im);
  b_L7_re = 2.0 * (L7 * x * dc17.re);
  b_L7_im = 2.0 * (L7 * x * dc17.im);
  c_L1_re = 2.0 * (L1 * zC * dc18.re);
  d_L1_im = 2.0 * (L1 * zC * dc18.im);
  c_L6_re = 2.0 * (L6 * zC * dc19.re);
  c_L6_im = 2.0 * (L6 * zC * dc19.im);
  b_L8_re = 2.0 * (L8 * zC * dc20.re);
  b_L8_im = 2.0 * (L8 * zC * dc20.im);
  b_L4_re = d_x * d_x * (L4 * L4 * dc21.re);
  b_L4_im = d_x * d_x * (L4 * L4 * dc21.im);
  c_L4_re = e_x * e_x * (L4 * L4 * dc22.re);
  c_L4_im = e_x * e_x * (L4 * L4 * dc22.im);
  if (f_x.im == 0.0) {
    b_x_re = f_x.re / 2.0;
    b_x_im = 0.0;
  } else if (f_x.re == 0.0) {
    b_x_re = 0.0;
    b_x_im = f_x.im / 2.0;
  } else {
    b_x_re = f_x.re / 2.0;
    b_x_im = f_x.im / 2.0;
  }

  unusedU4 = 2.0 * (L4 * L7 * (b_x_re + 0.5));
  unusedU3 = 2.0 * (L4 * L7 * b_x_im);
  b_x_re = x * h_x.re;
  b_x_im = x * h_x.im;
  d_L2_re = L2 * k_x.re;
  d_L2_im = L2 * k_x.im;
  c_L7_re = L7 * dc24.re;
  c_L7_im = L7 * dc24.im;
  ar = -((((((((((((((((((((((((((((r * 0.0 - L1_im) + (L2_re * 0.0 - L2_im)) -
    (L3_re * 0.0 - L3_im)) + (L5_re * 0.0 - L5_im)) + (L6_re * 0.0 - L6_im)) +
    (L7_re * 0.0 - L7_im)) + (L8_re * 0.0 - L8_im)) + (x_re * 0.0 - x_im)) +
    (unusedU5 * 0.0 - unusedU6)) + (dc10.re * 0.0 - dc10.im)) - (L4_re * 0.0 -
    L4_im)) - (L1_re * 0.0 - b_L1_im)) - (b_L1_re * 0.0 - c_L1_im)) - (b_L2_re *
    0.0 - b_L2_im)) + (b_L6_re * 0.0 - b_L6_im)) - L1 * L4 * (muDoubleScalarCos
    (2.0 * zeta) - 1.0)) + L4 * L6 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + L4
                   * L8 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) - (c_L2_re *
    0.0 - c_L2_im)) + (b_L7_re * 0.0 - b_L7_im)) - (c_L1_re * 0.0 - d_L1_im)) +
               (c_L6_re * 0.0 - c_L6_im)) + (b_L8_re * 0.0 - b_L8_im)) + L4 * zC
             * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + (b_L4_re * 0.0 - b_L4_im))
           + (c_L4_re * 0.0 - c_L4_im)) - (unusedU4 * 0.0 - unusedU3)) + L2 * L4
         * (g_x.re * 0.0 - g_x.im));
  unusedU6 = -((((((((((((((((((((((((((((r + L1_im * 0.0) + (L2_re + L2_im *
    0.0)) - (L3_re + L3_im * 0.0)) + (L5_re + L5_im * 0.0)) + (L6_re + L6_im *
    0.0)) + (L7_re + L7_im * 0.0)) + (L8_re + L8_im * 0.0)) + (x_re + x_im * 0.0))
    + (unusedU5 + unusedU6 * 0.0)) + (dc10.re + dc10.im * 0.0)) - (L4_re + L4_im
    * 0.0)) - (L1_re + b_L1_im * 0.0)) - (b_L1_re + c_L1_im * 0.0)) - (b_L2_re +
    b_L2_im * 0.0)) + (b_L6_re + b_L6_im * 0.0)) - L1 * L4 * muDoubleScalarSin
    (2.0 * zeta)) + L4 * L6 * muDoubleScalarSin(2.0 * zeta)) + L4 * L8 *
    muDoubleScalarSin(2.0 * zeta)) - (c_L2_re + c_L2_im * 0.0)) + (b_L7_re +
    b_L7_im * 0.0)) - (c_L1_re + d_L1_im * 0.0)) + (c_L6_re + c_L6_im * 0.0)) +
                    (b_L8_re + b_L8_im * 0.0)) + L4 * zC * muDoubleScalarSin(2.0
    * zeta)) + (b_L4_re + b_L4_im * 0.0)) + (c_L4_re + c_L4_im * 0.0)) -
                (unusedU4 + unusedU3 * 0.0)) + L2 * L4 * ((g_x.re + g_x.im * 0.0)
    + 1.0));
  unusedU5 = 2.0 * L5 * (((((((L4 * 0.0 - (b_x_re * 0.0 - b_x_im)) + zC * i_x.re)
    - L1 * j_x.re) + (d_L2_re * 0.0 - d_L2_im)) + L6 * dc23.re) - (c_L7_re * 0.0
    - c_L7_im)) + L8 * y.re);
  unusedU3 = 2.0 * L5 * (((((((L4 - (b_x_re + b_x_im * 0.0)) + zC * i_x.im) - L1
    * j_x.im) + (d_L2_re + d_L2_im * 0.0)) + L6 * dc23.im) - (c_L7_re + c_L7_im *
    0.0)) + L8 * y.im);
  if (unusedU3 == 0.0) {
    if (unusedU6 == 0.0) {
      dc1.re = ar / unusedU5;
      dc1.im = 0.0;
    } else if (ar == 0.0) {
      dc1.re = 0.0;
      dc1.im = unusedU6 / unusedU5;
    } else {
      dc1.re = ar / unusedU5;
      dc1.im = unusedU6 / unusedU5;
    }
  } else if (unusedU5 == 0.0) {
    if (ar == 0.0) {
      dc1.re = unusedU6 / unusedU3;
      dc1.im = 0.0;
    } else if (unusedU6 == 0.0) {
      dc1.re = 0.0;
      dc1.im = -(ar / unusedU3);
    } else {
      dc1.re = unusedU6 / unusedU3;
      dc1.im = -(ar / unusedU3);
    }
  } else {
    unusedU4 = muDoubleScalarAbs(unusedU5);
    unusedU1 = muDoubleScalarAbs(unusedU3);
    if (unusedU4 > unusedU1) {
      unusedU2 = unusedU3 / unusedU5;
      unusedU1 = unusedU5 + unusedU2 * unusedU3;
      dc1.re = (ar + unusedU2 * unusedU6) / unusedU1;
      dc1.im = (unusedU6 - unusedU2 * ar) / unusedU1;
    } else if (unusedU1 == unusedU4) {
      if (unusedU5 > 0.0) {
        unusedU2 = 0.5;
      } else {
        unusedU2 = -0.5;
      }

      if (unusedU3 > 0.0) {
        unusedU1 = 0.5;
      } else {
        unusedU1 = -0.5;
      }

      dc1.re = (ar * unusedU2 + unusedU6 * unusedU1) / unusedU4;
      dc1.im = (unusedU6 * unusedU2 - ar * unusedU1) / unusedU4;
    } else {
      unusedU2 = unusedU5 / unusedU3;
      unusedU1 = unusedU3 + unusedU2 * unusedU5;
      dc1.re = (unusedU2 * ar + unusedU6) / unusedU1;
      dc1.im = (unusedU2 * unusedU6 - ar) / unusedU1;
    }
  }

  b_log(&dc1);
  gammaRaw[0].re = -zeta - (dc0.re * 0.0 - dc0.im);
  re = dc1.re * 0.0 - dc1.im;
  a = ((((((((((((((((((((((((L1 * L1 - 2.0 * muDoubleScalarSin(zeta) * L1 * L4)
    - 2.0 * L1 * L6) - 2.0 * L1 * L8) - 2.0 * L1 * zC) + L2 * L2) + 2.0 *
    muDoubleScalarCos(zeta) * L2 * L4) - 2.0 * L2 * L7) - 2.0 * L2 * x) - L3 *
                      L3) + L4 * L4) + 2.0 * muDoubleScalarSin(zeta) * L4 * L6)
                   - 2.0 * muDoubleScalarCos(zeta) * L4 * L7) + 2.0 *
                  muDoubleScalarSin(zeta) * L4 * L8) - 2.0 * muDoubleScalarCos
                 (zeta) * L4 * x) + 2.0 * muDoubleScalarSin(zeta) * L4 * zC) +
               L5 * L5) + L6 * L6) + 2.0 * L6 * L8) + 2.0 * L6 * zC) + L7 * L7)
          + 2.0 * L7 * x) + L8 * L8) + 2.0 * L8 * zC) + x * x) + zC * zC;
  b_x = muDoubleScalarCos(zeta);
  c_x = muDoubleScalarSin(zeta);
  b_a = ((((((((((((((((((((((((L1 * L1 - 2.0 * muDoubleScalarSin(zeta) * L1 *
    L4) - 2.0 * L1 * L6) - 2.0 * L1 * L8) - 2.0 * L1 * zC) + L2 * L2) + 2.0 *
    muDoubleScalarCos(zeta) * L2 * L4) - 2.0 * L2 * L7) - 2.0 * L2 * x) - L3 *
                        L3) + L4 * L4) + 2.0 * muDoubleScalarSin(zeta) * L4 * L6)
                     - 2.0 * muDoubleScalarCos(zeta) * L4 * L7) + 2.0 *
                    muDoubleScalarSin(zeta) * L4 * L8) - 2.0 * muDoubleScalarCos
                   (zeta) * L4 * x) + 2.0 * muDoubleScalarSin(zeta) * L4 * zC) +
                 L5 * L5) + L6 * L6) + 2.0 * L6 * L8) + 2.0 * L6 * zC) + L7 * L7)
            + 2.0 * L7 * x) + L8 * L8) + 2.0 * L8 * zC) + x * x) + zC * zC;
  d_x = muDoubleScalarCos(zeta);
  e_x = muDoubleScalarSin(zeta);
  y.re = zeta * 2.0 * 0.0;
  y.im = zeta * 2.0;
  r = muDoubleScalarExp(y.re / 2.0);
  x_re = r * (r * muDoubleScalarCos(y.im));
  x_im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  f_x.re = r * (r * muDoubleScalarCos(y.im));
  f_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  g_x.re = r * (r * muDoubleScalarCos(y.im));
  g_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  h_x.re = r * (r * muDoubleScalarCos(y.im));
  h_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  i_x.re = r * (r * muDoubleScalarCos(y.im));
  i_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  j_x.re = r * (r * muDoubleScalarCos(y.im));
  j_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  k_x.re = r * (r * muDoubleScalarCos(y.im));
  k_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = zeta * 0.0;
  y.im = zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  y.re = r * (r * muDoubleScalarCos(y.im));
  y.im = r * (r * muDoubleScalarSin(y.im));
  dc0.re = zeta * 0.0;
  dc0.im = zeta;
  b_exp(&dc0);
  dc1.re = zeta * 0.0;
  dc1.im = zeta;
  b_exp(&dc1);
  dc2.re = zeta * 0.0;
  dc2.im = zeta;
  b_exp(&dc2);
  dc3.re = zeta * 0.0;
  dc3.im = zeta;
  b_exp(&dc3);
  dc4.re = zeta * 0.0;
  dc4.im = zeta;
  b_exp(&dc4);
  dc5.re = zeta * 0.0;
  dc5.im = zeta;
  b_exp(&dc5);
  dc6.re = zeta * 0.0;
  dc6.im = zeta;
  b_exp(&dc6);
  dc7.re = zeta * 0.0;
  dc7.im = zeta;
  b_exp(&dc7);
  dc8.re = zeta * 0.0;
  dc8.im = zeta;
  b_exp(&dc8);
  dc9.re = zeta * 2.0 * 0.0;
  dc9.im = zeta * 2.0;
  b_exp(&dc9);
  dc10.re = zeta * 0.0;
  dc10.im = zeta;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  unusedU5 = zC * dc12.re;
  unusedU6 = zC * dc12.im;
  r = L1 * dc13.re;
  L1_im = L1 * dc13.im;
  L6_re = L6 * dc15.re;
  L6_im = L6 * dc15.im;
  L8_re = L8 * dc17.re;
  L8_im = L8 * dc17.im;
  unusedU2 = 4.0 * (L5 * L5) * dc10.re;
  unusedU3 = 4.0 * (L5 * L5) * dc10.im;
  L4_re = ((((((-L4 + x * dc11.re) + (unusedU5 * 0.0 - unusedU6)) - (r * 0.0 -
    L1_im)) - L2 * dc14.re) + (L6_re * 0.0 - L6_im)) + L7 * dc16.re) + (L8_re *
    0.0 - L8_im);
  L4_im = (((((x * dc11.im + (unusedU5 + unusedU6 * 0.0)) - (r + L1_im * 0.0)) -
             L2 * dc14.im) + (L6_re + L6_im * 0.0)) + L7 * dc16.im) + (L8_re +
    L8_im * 0.0);
  unusedU1 = unusedU2 * L4_re - unusedU3 * L4_im;
  unusedU3 = unusedU2 * L4_im + unusedU3 * L4_re;
  L2_re = ((((((L2 - L1 * 0.0) + L6 * 0.0) - L7) + L8 * 0.0) - x) + zC * 0.0) +
    L4 * dc18.re;
  L2_im = ((((0.0 - L1) + L6) + L8) + zC) + L4 * dc18.im;
  dc9.re = dc9.re * (b_a * b_a) + (unusedU1 * L2_re - unusedU3 * L2_im);
  dc9.im = dc9.im * (b_a * b_a) + (unusedU1 * L2_im + unusedU3 * L2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = zeta * 2.0 * 0.0;
  dc10.im = zeta * 2.0;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  dc19.re = zeta * 0.0;
  dc19.im = zeta;
  b_exp(&dc19);
  dc20.re = zeta * 0.0;
  dc20.im = zeta;
  b_exp(&dc20);
  dc21.re = zeta * 0.0;
  dc21.im = zeta;
  b_exp(&dc21);
  dc22.re = zeta * 2.0 * 0.0;
  dc22.im = zeta * 2.0;
  b_exp(&dc22);
  r = L1 * L1 * dc0.re;
  L1_im = L1 * L1 * dc0.im;
  L2_re = L2 * L2 * dc1.re;
  L2_im = L2 * L2 * dc1.im;
  L3_re = L3 * L3 * dc2.re;
  L3_im = L3 * L3 * dc2.im;
  L5_re = L5 * L5 * dc3.re;
  L5_im = L5 * L5 * dc3.im;
  L6_re = L6 * L6 * dc4.re;
  L6_im = L6 * L6 * dc4.im;
  L7_re = L7 * L7 * dc5.re;
  L7_im = L7 * L7 * dc5.im;
  L8_re = L8 * L8 * dc6.re;
  L8_im = L8 * L8 * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  unusedU5 = zC * zC * dc8.re;
  unusedU6 = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    unusedU2 = dc10.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc10.re == 0.0) {
    unusedU2 = 0.0;
    unusedU3 = dc10.im / 2.0;
  } else {
    unusedU2 = dc10.re / 2.0;
    unusedU3 = dc10.im / 2.0;
  }

  L4_re = 2.0 * (L4 * x * (unusedU2 + 0.5));
  L4_im = 2.0 * (L4 * x * unusedU3);
  L1_re = 2.0 * (L1 * L6 * dc11.re);
  b_L1_im = 2.0 * (L1 * L6 * dc11.im);
  b_L1_re = 2.0 * (L1 * L8 * dc12.re);
  c_L1_im = 2.0 * (L1 * L8 * dc12.im);
  b_L2_re = 2.0 * (L2 * L7 * dc13.re);
  b_L2_im = 2.0 * (L2 * L7 * dc13.im);
  b_L6_re = 2.0 * (L6 * L8 * dc14.re);
  b_L6_im = 2.0 * (L6 * L8 * dc14.im);
  c_L2_re = 2.0 * (L2 * x * dc15.re);
  c_L2_im = 2.0 * (L2 * x * dc15.im);
  b_L7_re = 2.0 * (L7 * x * dc16.re);
  b_L7_im = 2.0 * (L7 * x * dc16.im);
  c_L1_re = 2.0 * (L1 * zC * dc17.re);
  d_L1_im = 2.0 * (L1 * zC * dc17.im);
  c_L6_re = 2.0 * (L6 * zC * dc18.re);
  c_L6_im = 2.0 * (L6 * zC * dc18.im);
  b_L8_re = 2.0 * (L8 * zC * dc19.re);
  b_L8_im = 2.0 * (L8 * zC * dc19.im);
  b_L4_re = d_x * d_x * (L4 * L4 * dc20.re);
  b_L4_im = d_x * d_x * (L4 * L4 * dc20.im);
  c_L4_re = e_x * e_x * (L4 * L4 * dc21.re);
  c_L4_im = e_x * e_x * (L4 * L4 * dc21.im);
  if (dc22.im == 0.0) {
    unusedU2 = dc22.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc22.re == 0.0) {
    unusedU2 = 0.0;
    unusedU3 = dc22.im / 2.0;
  } else {
    unusedU2 = dc22.re / 2.0;
    unusedU3 = dc22.im / 2.0;
  }

  unusedU4 = 2.0 * (L4 * L7 * (unusedU2 + 0.5));
  unusedU3 = 2.0 * (L4 * L7 * unusedU3);
  unusedU1 = x * f_x.re;
  unusedU2 = x * f_x.im;
  d_L2_re = L2 * i_x.re;
  d_L2_im = L2 * i_x.im;
  c_L7_re = L7 * k_x.re;
  c_L7_im = L7 * k_x.im;
  ar = -((((((((((((((((((((((((((((r * 0.0 - L1_im) + (L2_re * 0.0 - L2_im)) -
    (L3_re * 0.0 - L3_im)) + (L5_re * 0.0 - L5_im)) + (L6_re * 0.0 - L6_im)) +
    (L7_re * 0.0 - L7_im)) + (L8_re * 0.0 - L8_im)) + (b_x_re * 0.0 - b_x_im)) +
    (unusedU5 * 0.0 - unusedU6)) + (dc9.re * 0.0 - dc9.im)) - (L4_re * 0.0 -
    L4_im)) - (L1_re * 0.0 - b_L1_im)) - (b_L1_re * 0.0 - c_L1_im)) - (b_L2_re *
    0.0 - b_L2_im)) + (b_L6_re * 0.0 - b_L6_im)) - L1 * L4 * (muDoubleScalarCos
    (2.0 * zeta) - 1.0)) + L4 * L6 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + L4
                   * L8 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) - (c_L2_re *
    0.0 - c_L2_im)) + (b_L7_re * 0.0 - b_L7_im)) - (c_L1_re * 0.0 - d_L1_im)) +
               (c_L6_re * 0.0 - c_L6_im)) + (b_L8_re * 0.0 - b_L8_im)) + L4 * zC
             * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + (b_L4_re * 0.0 - b_L4_im))
           + (c_L4_re * 0.0 - c_L4_im)) - (unusedU4 * 0.0 - unusedU3)) + L2 * L4
         * (x_re * 0.0 - x_im));
  unusedU6 = -((((((((((((((((((((((((((((r + L1_im * 0.0) + (L2_re + L2_im *
    0.0)) - (L3_re + L3_im * 0.0)) + (L5_re + L5_im * 0.0)) + (L6_re + L6_im *
    0.0)) + (L7_re + L7_im * 0.0)) + (L8_re + L8_im * 0.0)) + (b_x_re + b_x_im *
    0.0)) + (unusedU5 + unusedU6 * 0.0)) + (dc9.re + dc9.im * 0.0)) - (L4_re +
    L4_im * 0.0)) - (L1_re + b_L1_im * 0.0)) - (b_L1_re + c_L1_im * 0.0)) -
    (b_L2_re + b_L2_im * 0.0)) + (b_L6_re + b_L6_im * 0.0)) - L1 * L4 *
    muDoubleScalarSin(2.0 * zeta)) + L4 * L6 * muDoubleScalarSin(2.0 * zeta)) +
    L4 * L8 * muDoubleScalarSin(2.0 * zeta)) - (c_L2_re + c_L2_im * 0.0)) +
                       (b_L7_re + b_L7_im * 0.0)) - (c_L1_re + d_L1_im * 0.0)) +
                     (c_L6_re + c_L6_im * 0.0)) + (b_L8_re + b_L8_im * 0.0)) +
                   L4 * zC * muDoubleScalarSin(2.0 * zeta)) + (b_L4_re + b_L4_im
    * 0.0)) + (c_L4_re + c_L4_im * 0.0)) - (unusedU4 + unusedU3 * 0.0)) + L2 *
               L4 * ((x_re + x_im * 0.0) + 1.0));
  unusedU5 = 2.0 * L5 * (((((((L4 * 0.0 - (unusedU1 * 0.0 - unusedU2)) + zC *
    g_x.re) - L1 * h_x.re) + (d_L2_re * 0.0 - d_L2_im)) + L6 * j_x.re) -
    (c_L7_re * 0.0 - c_L7_im)) + L8 * y.re);
  unusedU3 = 2.0 * L5 * (((((((L4 - (unusedU1 + unusedU2 * 0.0)) + zC * g_x.im)
    - L1 * h_x.im) + (d_L2_re + d_L2_im * 0.0)) + L6 * j_x.im) - (c_L7_re +
    c_L7_im * 0.0)) + L8 * y.im);
  if (unusedU3 == 0.0) {
    if (unusedU6 == 0.0) {
      y.re = ar / unusedU5;
      y.im = 0.0;
    } else if (ar == 0.0) {
      y.re = 0.0;
      y.im = unusedU6 / unusedU5;
    } else {
      y.re = ar / unusedU5;
      y.im = unusedU6 / unusedU5;
    }
  } else if (unusedU5 == 0.0) {
    if (ar == 0.0) {
      y.re = unusedU6 / unusedU3;
      y.im = 0.0;
    } else if (unusedU6 == 0.0) {
      y.re = 0.0;
      y.im = -(ar / unusedU3);
    } else {
      y.re = unusedU6 / unusedU3;
      y.im = -(ar / unusedU3);
    }
  } else {
    unusedU4 = muDoubleScalarAbs(unusedU5);
    unusedU1 = muDoubleScalarAbs(unusedU3);
    if (unusedU4 > unusedU1) {
      unusedU2 = unusedU3 / unusedU5;
      unusedU1 = unusedU5 + unusedU2 * unusedU3;
      y.re = (ar + unusedU2 * unusedU6) / unusedU1;
      y.im = (unusedU6 - unusedU2 * ar) / unusedU1;
    } else if (unusedU1 == unusedU4) {
      if (unusedU5 > 0.0) {
        unusedU2 = 0.5;
      } else {
        unusedU2 = -0.5;
      }

      if (unusedU3 > 0.0) {
        unusedU1 = 0.5;
      } else {
        unusedU1 = -0.5;
      }

      y.re = (ar * unusedU2 + unusedU6 * unusedU1) / unusedU4;
      y.im = (unusedU6 * unusedU2 - ar * unusedU1) / unusedU4;
    } else {
      unusedU2 = unusedU5 / unusedU3;
      unusedU1 = unusedU3 + unusedU2 * unusedU5;
      y.re = (unusedU2 * ar + unusedU6) / unusedU1;
      y.im = (unusedU2 * unusedU6 - ar) / unusedU1;
    }
  }

  if ((y.im == 0.0) && muDoubleScalarIsNaN(y.re)) {
  } else if ((muDoubleScalarAbs(y.re) > 8.9884656743115785E+307) ||
             (muDoubleScalarAbs(y.im) > 8.9884656743115785E+307)) {
    r = y.re;
    y.re = muDoubleScalarLog(muDoubleScalarHypot(y.re / 2.0, y.im / 2.0)) +
      0.69314718055994529;
    y.im = muDoubleScalarAtan2(y.im, r);
  } else {
    r = y.re;
    y.re = muDoubleScalarLog(muDoubleScalarHypot(y.re, y.im));
    y.im = muDoubleScalarAtan2(y.im, r);
  }

  x_re = y.re * 0.0 - y.im;
  x_im = y.re + y.im * 0.0;
  if (x_im == 0.0) {
    x_re = muDoubleScalarSin(x_re);
    x_im = 0.0;
  } else {
    b_x_re = x_re;
    x_re = muDoubleScalarSin(x_re) * muDoubleScalarCosh(x_im);
    x_im = muDoubleScalarCos(b_x_re) * muDoubleScalarSinh(x_im);
  }

  dc0.re = zeta * 0.0;
  dc0.im = zeta;
  b_exp(&dc0);
  dc1.re = zeta * 0.0;
  dc1.im = zeta;
  b_exp(&dc1);
  dc2.re = zeta * 0.0;
  dc2.im = zeta;
  b_exp(&dc2);
  dc3.re = zeta * 0.0;
  dc3.im = zeta;
  b_exp(&dc3);
  dc4.re = zeta * 0.0;
  dc4.im = zeta;
  b_exp(&dc4);
  dc5.re = zeta * 0.0;
  dc5.im = zeta;
  b_exp(&dc5);
  dc6.re = zeta * 0.0;
  dc6.im = zeta;
  b_exp(&dc6);
  dc7.re = zeta * 0.0;
  dc7.im = zeta;
  b_exp(&dc7);
  dc8.re = zeta * 0.0;
  dc8.im = zeta;
  b_exp(&dc8);
  dc9.re = zeta * 2.0 * 0.0;
  dc9.im = zeta * 2.0;
  b_exp(&dc9);
  dc10.re = zeta * 0.0;
  dc10.im = zeta;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  unusedU5 = zC * dc12.re;
  unusedU6 = zC * dc12.im;
  r = L1 * dc13.re;
  L1_im = L1 * dc13.im;
  L6_re = L6 * dc15.re;
  L6_im = L6 * dc15.im;
  L8_re = L8 * dc17.re;
  L8_im = L8 * dc17.im;
  unusedU2 = 4.0 * (L5 * L5) * dc10.re;
  unusedU3 = 4.0 * (L5 * L5) * dc10.im;
  L4_re = ((((((-L4 + x * dc11.re) + (unusedU5 * 0.0 - unusedU6)) - (r * 0.0 -
    L1_im)) - L2 * dc14.re) + (L6_re * 0.0 - L6_im)) + L7 * dc16.re) + (L8_re *
    0.0 - L8_im);
  L4_im = (((((x * dc11.im + (unusedU5 + unusedU6 * 0.0)) - (r + L1_im * 0.0)) -
             L2 * dc14.im) + (L6_re + L6_im * 0.0)) + L7 * dc16.im) + (L8_re +
    L8_im * 0.0);
  unusedU1 = unusedU2 * L4_re - unusedU3 * L4_im;
  unusedU3 = unusedU2 * L4_im + unusedU3 * L4_re;
  L2_re = ((((((L2 - L1 * 0.0) + L6 * 0.0) - L7) + L8 * 0.0) - x) + zC * 0.0) +
    L4 * dc18.re;
  L2_im = ((((0.0 - L1) + L6) + L8) + zC) + L4 * dc18.im;
  dc9.re = dc9.re * (a * a) + (unusedU1 * L2_re - unusedU3 * L2_im);
  dc9.im = dc9.im * (a * a) + (unusedU1 * L2_im + unusedU3 * L2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = zeta * 2.0 * 0.0;
  dc10.im = zeta * 2.0;
  b_exp(&dc10);
  dc11.re = zeta * 0.0;
  dc11.im = zeta;
  b_exp(&dc11);
  dc12.re = zeta * 0.0;
  dc12.im = zeta;
  b_exp(&dc12);
  dc13.re = zeta * 0.0;
  dc13.im = zeta;
  b_exp(&dc13);
  dc14.re = zeta * 0.0;
  dc14.im = zeta;
  b_exp(&dc14);
  dc15.re = zeta * 0.0;
  dc15.im = zeta;
  b_exp(&dc15);
  dc16.re = zeta * 0.0;
  dc16.im = zeta;
  b_exp(&dc16);
  dc17.re = zeta * 0.0;
  dc17.im = zeta;
  b_exp(&dc17);
  dc18.re = zeta * 0.0;
  dc18.im = zeta;
  b_exp(&dc18);
  dc19.re = zeta * 0.0;
  dc19.im = zeta;
  b_exp(&dc19);
  dc20.re = zeta * 0.0;
  dc20.im = zeta;
  b_exp(&dc20);
  dc21.re = zeta * 0.0;
  dc21.im = zeta;
  b_exp(&dc21);
  dc22.re = zeta * 2.0 * 0.0;
  dc22.im = zeta * 2.0;
  b_exp(&dc22);
  f_x.re = zeta * 2.0 * 0.0;
  f_x.im = zeta * 2.0;
  b_exp(&f_x);
  g_x.re = zeta * 0.0;
  g_x.im = zeta;
  b_exp(&g_x);
  h_x.re = zeta * 0.0;
  h_x.im = zeta;
  b_exp(&h_x);
  i_x.re = zeta * 0.0;
  i_x.im = zeta;
  b_exp(&i_x);
  j_x.re = zeta * 0.0;
  j_x.im = zeta;
  b_exp(&j_x);
  k_x.re = zeta * 0.0;
  k_x.im = zeta;
  b_exp(&k_x);
  dc23.re = zeta * 0.0;
  dc23.im = zeta;
  b_exp(&dc23);
  dc24.re = zeta * 0.0;
  dc24.im = zeta;
  b_exp(&dc24);
  r = L1 * L1 * dc0.re;
  L1_im = L1 * L1 * dc0.im;
  L2_re = L2 * L2 * dc1.re;
  L2_im = L2 * L2 * dc1.im;
  L3_re = L3 * L3 * dc2.re;
  L3_im = L3 * L3 * dc2.im;
  L5_re = L5 * L5 * dc3.re;
  L5_im = L5 * L5 * dc3.im;
  L6_re = L6 * L6 * dc4.re;
  L6_im = L6 * L6 * dc4.im;
  L7_re = L7 * L7 * dc5.re;
  L7_im = L7 * L7 * dc5.im;
  L8_re = L8 * L8 * dc6.re;
  L8_im = L8 * L8 * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  unusedU5 = zC * zC * dc8.re;
  unusedU6 = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    unusedU2 = dc10.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc10.re == 0.0) {
    unusedU2 = 0.0;
    unusedU3 = dc10.im / 2.0;
  } else {
    unusedU2 = dc10.re / 2.0;
    unusedU3 = dc10.im / 2.0;
  }

  L4_re = 2.0 * (L4 * x * (unusedU2 + 0.5));
  L4_im = 2.0 * (L4 * x * unusedU3);
  L1_re = 2.0 * (L1 * L6 * dc11.re);
  b_L1_im = 2.0 * (L1 * L6 * dc11.im);
  b_L1_re = 2.0 * (L1 * L8 * dc12.re);
  c_L1_im = 2.0 * (L1 * L8 * dc12.im);
  b_L2_re = 2.0 * (L2 * L7 * dc13.re);
  b_L2_im = 2.0 * (L2 * L7 * dc13.im);
  b_L6_re = 2.0 * (L6 * L8 * dc14.re);
  b_L6_im = 2.0 * (L6 * L8 * dc14.im);
  c_L2_re = 2.0 * (L2 * x * dc15.re);
  c_L2_im = 2.0 * (L2 * x * dc15.im);
  b_L7_re = 2.0 * (L7 * x * dc16.re);
  b_L7_im = 2.0 * (L7 * x * dc16.im);
  c_L1_re = 2.0 * (L1 * zC * dc17.re);
  d_L1_im = 2.0 * (L1 * zC * dc17.im);
  c_L6_re = 2.0 * (L6 * zC * dc18.re);
  c_L6_im = 2.0 * (L6 * zC * dc18.im);
  b_L8_re = 2.0 * (L8 * zC * dc19.re);
  b_L8_im = 2.0 * (L8 * zC * dc19.im);
  b_L4_re = b_x * b_x * (L4 * L4 * dc20.re);
  b_L4_im = b_x * b_x * (L4 * L4 * dc20.im);
  c_L4_re = c_x * c_x * (L4 * L4 * dc21.re);
  c_L4_im = c_x * c_x * (L4 * L4 * dc21.im);
  if (dc22.im == 0.0) {
    unusedU2 = dc22.re / 2.0;
    unusedU3 = 0.0;
  } else if (dc22.re == 0.0) {
    unusedU2 = 0.0;
    unusedU3 = dc22.im / 2.0;
  } else {
    unusedU2 = dc22.re / 2.0;
    unusedU3 = dc22.im / 2.0;
  }

  unusedU4 = 2.0 * (L4 * L7 * (unusedU2 + 0.5));
  unusedU3 = 2.0 * (L4 * L7 * unusedU3);
  unusedU1 = x * g_x.re;
  unusedU2 = x * g_x.im;
  d_L2_re = L2 * j_x.re;
  d_L2_im = L2 * j_x.im;
  c_L7_re = L7 * dc23.re;
  c_L7_im = L7 * dc23.im;
  ar = -((((((((((((((((((((((((((((r * 0.0 - L1_im) + (L2_re * 0.0 - L2_im)) -
    (L3_re * 0.0 - L3_im)) + (L5_re * 0.0 - L5_im)) + (L6_re * 0.0 - L6_im)) +
    (L7_re * 0.0 - L7_im)) + (L8_re * 0.0 - L8_im)) + (b_x_re * 0.0 - b_x_im)) +
    (unusedU5 * 0.0 - unusedU6)) - (dc9.re * 0.0 - dc9.im)) - (L4_re * 0.0 -
    L4_im)) - (L1_re * 0.0 - b_L1_im)) - (b_L1_re * 0.0 - c_L1_im)) - (b_L2_re *
    0.0 - b_L2_im)) + (b_L6_re * 0.0 - b_L6_im)) - L1 * L4 * (muDoubleScalarCos
    (2.0 * zeta) - 1.0)) + L4 * L6 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + L4
                   * L8 * (muDoubleScalarCos(2.0 * zeta) - 1.0)) - (c_L2_re *
    0.0 - c_L2_im)) + (b_L7_re * 0.0 - b_L7_im)) - (c_L1_re * 0.0 - d_L1_im)) +
               (c_L6_re * 0.0 - c_L6_im)) + (b_L8_re * 0.0 - b_L8_im)) + L4 * zC
             * (muDoubleScalarCos(2.0 * zeta) - 1.0)) + (b_L4_re * 0.0 - b_L4_im))
           + (c_L4_re * 0.0 - c_L4_im)) - (unusedU4 * 0.0 - unusedU3)) + L2 * L4
         * (f_x.re * 0.0 - f_x.im));
  unusedU6 = -((((((((((((((((((((((((((((r + L1_im * 0.0) + (L2_re + L2_im *
    0.0)) - (L3_re + L3_im * 0.0)) + (L5_re + L5_im * 0.0)) + (L6_re + L6_im *
    0.0)) + (L7_re + L7_im * 0.0)) + (L8_re + L8_im * 0.0)) + (b_x_re + b_x_im *
    0.0)) + (unusedU5 + unusedU6 * 0.0)) - (dc9.re + dc9.im * 0.0)) - (L4_re +
    L4_im * 0.0)) - (L1_re + b_L1_im * 0.0)) - (b_L1_re + c_L1_im * 0.0)) -
    (b_L2_re + b_L2_im * 0.0)) + (b_L6_re + b_L6_im * 0.0)) - L1 * L4 *
    muDoubleScalarSin(2.0 * zeta)) + L4 * L6 * muDoubleScalarSin(2.0 * zeta)) +
    L4 * L8 * muDoubleScalarSin(2.0 * zeta)) - (c_L2_re + c_L2_im * 0.0)) +
                       (b_L7_re + b_L7_im * 0.0)) - (c_L1_re + d_L1_im * 0.0)) +
                     (c_L6_re + c_L6_im * 0.0)) + (b_L8_re + b_L8_im * 0.0)) +
                   L4 * zC * muDoubleScalarSin(2.0 * zeta)) + (b_L4_re + b_L4_im
    * 0.0)) + (c_L4_re + c_L4_im * 0.0)) - (unusedU4 + unusedU3 * 0.0)) + L2 *
               L4 * ((f_x.re + f_x.im * 0.0) + 1.0));
  unusedU5 = 2.0 * L5 * (((((((L4 * 0.0 - (unusedU1 * 0.0 - unusedU2)) + zC *
    h_x.re) - L1 * i_x.re) + (d_L2_re * 0.0 - d_L2_im)) + L6 * k_x.re) -
    (c_L7_re * 0.0 - c_L7_im)) + L8 * dc24.re);
  unusedU3 = 2.0 * L5 * (((((((L4 - (unusedU1 + unusedU2 * 0.0)) + zC * h_x.im)
    - L1 * i_x.im) + (d_L2_re + d_L2_im * 0.0)) + L6 * k_x.im) - (c_L7_re +
    c_L7_im * 0.0)) + L8 * dc24.im);
  if (unusedU3 == 0.0) {
    if (unusedU6 == 0.0) {
      dc0.re = ar / unusedU5;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = unusedU6 / unusedU5;
    } else {
      dc0.re = ar / unusedU5;
      dc0.im = unusedU6 / unusedU5;
    }
  } else if (unusedU5 == 0.0) {
    if (ar == 0.0) {
      dc0.re = unusedU6 / unusedU3;
      dc0.im = 0.0;
    } else if (unusedU6 == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / unusedU3);
    } else {
      dc0.re = unusedU6 / unusedU3;
      dc0.im = -(ar / unusedU3);
    }
  } else {
    unusedU4 = muDoubleScalarAbs(unusedU5);
    unusedU1 = muDoubleScalarAbs(unusedU3);
    if (unusedU4 > unusedU1) {
      unusedU2 = unusedU3 / unusedU5;
      unusedU1 = unusedU5 + unusedU2 * unusedU3;
      dc0.re = (ar + unusedU2 * unusedU6) / unusedU1;
      dc0.im = (unusedU6 - unusedU2 * ar) / unusedU1;
    } else if (unusedU1 == unusedU4) {
      if (unusedU5 > 0.0) {
        unusedU2 = 0.5;
      } else {
        unusedU2 = -0.5;
      }

      if (unusedU3 > 0.0) {
        unusedU1 = 0.5;
      } else {
        unusedU1 = -0.5;
      }

      dc0.re = (ar * unusedU2 + unusedU6 * unusedU1) / unusedU4;
      dc0.im = (unusedU6 * unusedU2 - ar * unusedU1) / unusedU4;
    } else {
      unusedU2 = unusedU5 / unusedU3;
      unusedU1 = unusedU3 + unusedU2 * unusedU5;
      dc0.re = (unusedU2 * ar + unusedU6) / unusedU1;
      dc0.im = (unusedU2 * unusedU6 - ar) / unusedU1;
    }
  }

  b_log(&dc0);
  unusedU2 = dc0.re;
  dc0.re = dc0.re * 0.0 - dc0.im;
  dc0.im = unusedU2 + dc0.im * 0.0;
  b_sin(&dc0);
  L6_re = ((((L6 - L1) + L8) + zC) + L4 * muDoubleScalarSin(zeta)) - L5 * dc0.re;
  L6_im = 0.0 - L5 * dc0.im;
  if (L6_im == 0.0) {
    dc0.re = L6_re / L3;
    dc0.im = 0.0;
  } else if (L6_re == 0.0) {
    dc0.re = 0.0;
    dc0.im = L6_im / L3;
  } else {
    dc0.re = L6_re / L3;
    dc0.im = L6_im / L3;
  }

  b_asin(&dc0);
  ar = ((((L6 - L1) + L8) + zC) + L4 * muDoubleScalarSin(zeta)) - L5 * x_re;
  unusedU6 = 0.0 - L5 * x_im;
  if (unusedU6 == 0.0) {
    dc1.re = ar / L3;
    dc1.im = 0.0;
  } else if (ar == 0.0) {
    dc1.re = 0.0;
    dc1.im = unusedU6 / L3;
  } else {
    dc1.re = ar / L3;
    dc1.im = unusedU6 / L3;
  }

  b_asin(&dc1);
  *alpha = muDoubleScalarAtan2(yC, xC);

  /* beta = betaRaw(1); */
  /* gamma = gammaRaw(1); */
  if ((*alpha >= jointLimits[0]) && (*alpha <= jointLimits[1]) && (-dc0.re >=
       jointLimits[2]) && (-dc0.re <= jointLimits[3]) && (gammaRaw[0].re >=
       jointLimits[4]) && (gammaRaw[0].re <= jointLimits[5])) {
    *beta = -dc0.re;
    *b_gamma = gammaRaw[0].re;

    /* elseif alpha >= alphaMin && alpha <= alphaMax && betaRaw(2) >= betaMin && betaRaw(2) <= betaMax && gammaRaw(2) >= gammaMin && gammaRaw(2) <= gammaMax */
  } else {
    *beta = -dc1.re;
    *b_gamma = -zeta - re;
  }
}

/* End of code generation (sherpaTTIK.c) */
