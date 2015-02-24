/*
 * sherpaTTIK.c
 *
 * Code generation for function 'sherpaTTIK'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "randomStateGenerator.h"
#include "sherpaTTIK.h"
#include "eml_error.h"
#include "asin.h"
#include "sin.h"
#include "log.h"
#include "exp.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo u_emlrtRSI = { 14, "sherpaTTIK",
  "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/sherpaTTIK.m"
};

static emlrtRSInfo v_emlrtRSI = { 14, "sqrt",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/sqrt.m" };

/* Function Definitions */
void sherpaTTIK(const emlrtStack *sp, const real_T u[3], real_T kC_l1, real_T
                kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6,
                real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const
                real_T jointLimits[20], real_T q[3])
{
  real_T r;
  real_T x;
  real_T a;
  real_T b_a;
  real_T b_x;
  real_T c_x;
  real_T c_a;
  real_T d_a;
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
  real_T u_re;
  real_T u_im;
  real_T kC_l1_re;
  real_T kC_l1_im;
  real_T kC_l6_re;
  real_T kC_l6_im;
  real_T kC_l8_re;
  real_T kC_l8_im;
  real_T re;
  real_T kC_l4_re;
  real_T kC_l4_im;
  real_T beta;
  real_T kC_l2_re;
  real_T kC_l2_im;
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
  real_T kC_l3_re;
  real_T kC_l3_im;
  real_T kC_l5_re;
  real_T kC_l5_im;
  real_T kC_l7_re;
  real_T kC_l7_im;
  real_T a_re;
  real_T a_im;
  real_T x_re;
  real_T x_im;
  real_T b_kC_l1_re;
  real_T b_kC_l1_im;
  real_T c_kC_l1_re;
  real_T c_kC_l1_im;
  real_T b_kC_l2_re;
  real_T b_kC_l2_im;
  real_T b_kC_l6_re;
  real_T b_kC_l6_im;
  real_T c_kC_l2_re;
  real_T c_kC_l2_im;
  real_T b_kC_l7_re;
  real_T b_kC_l7_im;
  real_T d_kC_l1_re;
  real_T d_kC_l1_im;
  real_T c_kC_l6_re;
  real_T c_kC_l6_im;
  real_T b_kC_l4_re;
  real_T b_kC_l4_im;
  real_T c_kC_l4_re;
  real_T c_kC_l4_im;
  real_T d_kC_l4_re;
  real_T b_x_re;
  real_T b_x_im;
  real_T d_kC_l2_re;
  real_T d_kC_l2_im;
  real_T c_kC_l7_re;
  real_T c_kC_l7_im;
  real_T ar;
  real_T alpha;
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
  r = u[0] * u[0];
  st.site = &u_emlrtRSI;
  if (r < 0.0) {
    b_st.site = &v_emlrtRSI;
    b_eml_error(&b_st);
  }

  x = muDoubleScalarSqrt(r);
  a = kC_l8 + kC_r;
  b_a = kC_l8 + kC_r;
  b_a = ((((((((((((((((((((((((kC_l1 * kC_l1 - 2.0 * muDoubleScalarSin(kC_zeta)
    * kC_l1 * kC_l4) - 2.0 * kC_l1 * kC_l6) - 2.0 * kC_l1 * (kC_l8 + kC_r)) -
    2.0 * kC_l1 * u[2]) + kC_l2 * kC_l2) + 2.0 * muDoubleScalarCos(kC_zeta) *
    kC_l2 * kC_l4) - 2.0 * kC_l2 * kC_l7) - 2.0 * kC_l2 * x) - kC_l3 * kC_l3) +
                       kC_l4 * kC_l4) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4
                      * kC_l6) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 *
                     kC_l7) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4 * (kC_l8
    + kC_r)) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 * x) + 2.0 *
                  muDoubleScalarSin(kC_zeta) * kC_l4 * u[2]) + kC_l5 * kC_l5) +
                kC_l6 * kC_l6) + 2.0 * kC_l6 * (kC_l8 + kC_r)) + 2.0 * kC_l6 *
              u[2]) + kC_l7 * kC_l7) + 2.0 * kC_l7 * x) + b_a * b_a) + 2.0 *
          (kC_l8 + kC_r) * u[2]) + x * x) + u[2] * u[2];
  b_x = muDoubleScalarCos(kC_zeta);
  c_x = muDoubleScalarSin(kC_zeta);
  c_a = kC_l8 + kC_r;
  d_a = kC_l8 + kC_r;
  d_a = ((((((((((((((((((((((((kC_l1 * kC_l1 - 2.0 * muDoubleScalarSin(kC_zeta)
    * kC_l1 * kC_l4) - 2.0 * kC_l1 * kC_l6) - 2.0 * kC_l1 * (kC_l8 + kC_r)) -
    2.0 * kC_l1 * u[2]) + kC_l2 * kC_l2) + 2.0 * muDoubleScalarCos(kC_zeta) *
    kC_l2 * kC_l4) - 2.0 * kC_l2 * kC_l7) - 2.0 * kC_l2 * x) - kC_l3 * kC_l3) +
                       kC_l4 * kC_l4) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4
                      * kC_l6) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 *
                     kC_l7) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4 * (kC_l8
    + kC_r)) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 * x) + 2.0 *
                  muDoubleScalarSin(kC_zeta) * kC_l4 * u[2]) + kC_l5 * kC_l5) +
                kC_l6 * kC_l6) + 2.0 * kC_l6 * (kC_l8 + kC_r)) + 2.0 * kC_l6 *
              u[2]) + kC_l7 * kC_l7) + 2.0 * kC_l7 * x) + d_a * d_a) + 2.0 *
          (kC_l8 + kC_r) * u[2]) + x * x) + u[2] * u[2];
  d_x = muDoubleScalarCos(kC_zeta);
  e_x = muDoubleScalarSin(kC_zeta);
  dc0.re = kC_zeta * 0.0;
  dc0.im = kC_zeta;
  b_exp(&dc0);
  dc1.re = kC_zeta * 0.0;
  dc1.im = kC_zeta;
  b_exp(&dc1);
  dc2.re = kC_zeta * 0.0;
  dc2.im = kC_zeta;
  b_exp(&dc2);
  dc3.re = kC_zeta * 0.0;
  dc3.im = kC_zeta;
  b_exp(&dc3);
  dc4.re = kC_zeta * 0.0;
  dc4.im = kC_zeta;
  b_exp(&dc4);
  dc5.re = kC_zeta * 0.0;
  dc5.im = kC_zeta;
  b_exp(&dc5);
  dc6.re = kC_zeta * 0.0;
  dc6.im = kC_zeta;
  b_exp(&dc6);
  dc7.re = kC_zeta * 0.0;
  dc7.im = kC_zeta;
  b_exp(&dc7);
  dc8.re = kC_zeta * 0.0;
  dc8.im = kC_zeta;
  b_exp(&dc8);
  dc9.re = kC_zeta * 2.0 * 0.0;
  dc9.im = kC_zeta * 2.0;
  b_exp(&dc9);
  dc10.re = kC_zeta * 0.0;
  dc10.im = kC_zeta;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  u_re = u[2] * dc12.re;
  u_im = u[2] * dc12.im;
  kC_l1_re = kC_l1 * dc13.re;
  kC_l1_im = kC_l1 * dc13.im;
  kC_l6_re = kC_l6 * dc15.re;
  kC_l6_im = kC_l6 * dc15.im;
  kC_l8_re = (kC_l8 + kC_r) * dc17.re;
  kC_l8_im = (kC_l8 + kC_r) * dc17.im;
  re = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = re * kC_l4_re - r * kC_l4_im;
  r = re * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc18.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc18.im;
  dc9.re = dc9.re * (b_a * b_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc9.im = dc9.im * (b_a * b_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kC_zeta * 2.0 * 0.0;
  dc10.im = kC_zeta * 2.0;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  dc19.re = kC_zeta * 0.0;
  dc19.im = kC_zeta;
  b_exp(&dc19);
  dc20.re = kC_zeta * 0.0;
  dc20.im = kC_zeta;
  b_exp(&dc20);
  dc21.re = kC_zeta * 0.0;
  dc21.im = kC_zeta;
  b_exp(&dc21);
  dc22.re = kC_zeta * 2.0 * 0.0;
  dc22.im = kC_zeta * 2.0;
  b_exp(&dc22);
  f_x.re = kC_zeta * 2.0 * 0.0;
  f_x.im = kC_zeta * 2.0;
  b_exp(&f_x);
  g_x.re = kC_zeta * 0.0;
  g_x.im = kC_zeta;
  b_exp(&g_x);
  h_x.re = kC_zeta * 0.0;
  h_x.im = kC_zeta;
  b_exp(&h_x);
  i_x.re = kC_zeta * 0.0;
  i_x.im = kC_zeta;
  b_exp(&i_x);
  j_x.re = kC_zeta * 0.0;
  j_x.im = kC_zeta;
  b_exp(&j_x);
  k_x.re = kC_zeta * 0.0;
  k_x.im = kC_zeta;
  b_exp(&k_x);
  dc23.re = kC_zeta * 0.0;
  dc23.im = kC_zeta;
  b_exp(&dc23);
  dc24.re = kC_zeta * 0.0;
  dc24.im = kC_zeta;
  b_exp(&dc24);
  kC_l1_re = kC_l1 * kC_l1 * dc0.re;
  kC_l1_im = kC_l1 * kC_l1 * dc0.im;
  kC_l2_re = kC_l2 * kC_l2 * dc1.re;
  kC_l2_im = kC_l2 * kC_l2 * dc1.im;
  kC_l3_re = kC_l3 * kC_l3 * dc2.re;
  kC_l3_im = kC_l3 * kC_l3 * dc2.im;
  kC_l5_re = kC_l5 * kC_l5 * dc3.re;
  kC_l5_im = kC_l5 * kC_l5 * dc3.im;
  kC_l6_re = kC_l6 * kC_l6 * dc4.re;
  kC_l6_im = kC_l6 * kC_l6 * dc4.im;
  kC_l7_re = kC_l7 * kC_l7 * dc5.re;
  kC_l7_im = kC_l7 * kC_l7 * dc5.im;
  a_re = a * a * dc6.re;
  a_im = a * a * dc6.im;
  x_re = x * x * dc7.re;
  x_im = x * x * dc7.im;
  u_re = u[2] * u[2] * dc8.re;
  u_im = u[2] * u[2] * dc8.im;
  if (dc10.im == 0.0) {
    re = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    re = 0.0;
    r = dc10.im / 2.0;
  } else {
    re = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (re + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc11.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc11.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc13.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc13.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc15.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc15.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc16.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc16.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc17.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc17.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc18.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc18.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.im);
  b_kC_l4_re = b_x * b_x * (kC_l4 * kC_l4 * dc20.re);
  b_kC_l4_im = b_x * b_x * (kC_l4 * kC_l4 * dc20.im);
  c_kC_l4_re = c_x * c_x * (kC_l4 * kC_l4 * dc21.re);
  c_kC_l4_im = c_x * c_x * (kC_l4 * kC_l4 * dc21.im);
  if (dc22.im == 0.0) {
    re = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    re = 0.0;
    r = dc22.im / 2.0;
  } else {
    re = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (re + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  b_x_re = x * g_x.re;
  b_x_im = x * g_x.im;
  d_kC_l2_re = kC_l2 * j_x.re;
  d_kC_l2_im = kC_l2 * j_x.im;
  c_kC_l7_re = kC_l7 * dc23.re;
  c_kC_l7_im = kC_l7 * dc23.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (x_re * 0.0 - x_im)) + (u_re * 0.0 - u_im)) - (dc9.re * 0.0 -
    dc9.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im)) -
                        (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
    b_kC_l2_im)) + (b_kC_l6_re * 0.0 - b_kC_l6_im)) - kC_l1 * kC_l4 *
                     (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * kC_l6 *
                    (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * (kC_l8 +
    kC_r) * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) - (c_kC_l2_re * 0.0 -
    c_kC_l2_im)) + (b_kC_l7_re * 0.0 - b_kC_l7_im)) - (d_kC_l1_re * 0.0 -
    d_kC_l1_im)) + (c_kC_l6_re * 0.0 - c_kC_l6_im)) + (kC_l8_re * 0.0 - kC_l8_im))
             + kC_l4 * u[2] * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) +
            (b_kC_l4_re * 0.0 - b_kC_l4_im)) + (c_kC_l4_re * 0.0 - c_kC_l4_im))
          - (d_kC_l4_re * 0.0 - r)) + kC_l2 * kC_l4 * (f_x.re * 0.0 - f_x.im));
  u_im = -((((((((((((((((((((((((((((kC_l1_re + kC_l1_im * 0.0) + (kC_l2_re +
    kC_l2_im * 0.0)) - (kC_l3_re + kC_l3_im * 0.0)) + (kC_l5_re + kC_l5_im * 0.0))
    + (kC_l6_re + kC_l6_im * 0.0)) + (kC_l7_re + kC_l7_im * 0.0)) + (a_re + a_im
    * 0.0)) + (x_re + x_im * 0.0)) + (u_re + u_im * 0.0)) - (dc9.re + dc9.im *
    0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im * 0.0)) -
    (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re + b_kC_l2_im * 0.0)) +
                        (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((f_x.re + f_x.im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (b_x_re * 0.0 - b_x_im)) + u[2] *
    h_x.re) - kC_l1 * i_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    k_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * dc24.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (b_x_re + b_x_im * 0.0)) + u[2] * h_x.im) -
    kC_l1 * i_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * k_x.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) *
                     dc24.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc0.re = ar / u_re;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = u_im / u_re;
    } else {
      dc0.re = ar / u_re;
      dc0.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc0.re = u_im / r;
      dc0.im = 0.0;
    } else if (u_im == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / r);
    } else {
      dc0.re = u_im / r;
      dc0.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    alpha = muDoubleScalarAbs(r);
    if (d_kC_l4_re > alpha) {
      beta = r / u_re;
      alpha = u_re + beta * r;
      dc0.re = (ar + beta * u_im) / alpha;
      dc0.im = (u_im - beta * ar) / alpha;
    } else if (alpha == d_kC_l4_re) {
      if (u_re > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      if (r > 0.0) {
        alpha = 0.5;
      } else {
        alpha = -0.5;
      }

      dc0.re = (ar * beta + u_im * alpha) / d_kC_l4_re;
      dc0.im = (u_im * beta - ar * alpha) / d_kC_l4_re;
    } else {
      beta = u_re / r;
      alpha = r + beta * u_re;
      dc0.re = (beta * ar + u_im) / alpha;
      dc0.im = (beta * u_im - ar) / alpha;
    }
  }

  b_log(&dc0);
  dc1.re = kC_zeta * 0.0;
  dc1.im = kC_zeta;
  b_exp(&dc1);
  dc2.re = kC_zeta * 0.0;
  dc2.im = kC_zeta;
  b_exp(&dc2);
  dc3.re = kC_zeta * 0.0;
  dc3.im = kC_zeta;
  b_exp(&dc3);
  dc4.re = kC_zeta * 0.0;
  dc4.im = kC_zeta;
  b_exp(&dc4);
  dc5.re = kC_zeta * 0.0;
  dc5.im = kC_zeta;
  b_exp(&dc5);
  dc6.re = kC_zeta * 0.0;
  dc6.im = kC_zeta;
  b_exp(&dc6);
  dc7.re = kC_zeta * 0.0;
  dc7.im = kC_zeta;
  b_exp(&dc7);
  dc8.re = kC_zeta * 0.0;
  dc8.im = kC_zeta;
  b_exp(&dc8);
  dc9.re = kC_zeta * 0.0;
  dc9.im = kC_zeta;
  b_exp(&dc9);
  dc10.re = kC_zeta * 2.0 * 0.0;
  dc10.im = kC_zeta * 2.0;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  dc19.re = kC_zeta * 0.0;
  dc19.im = kC_zeta;
  b_exp(&dc19);
  u_re = u[2] * dc13.re;
  u_im = u[2] * dc13.im;
  kC_l1_re = kC_l1 * dc14.re;
  kC_l1_im = kC_l1 * dc14.im;
  kC_l6_re = kC_l6 * dc16.re;
  kC_l6_im = kC_l6 * dc16.im;
  kC_l8_re = (kC_l8 + kC_r) * dc18.re;
  kC_l8_im = (kC_l8 + kC_r) * dc18.im;
  re = 4.0 * (kC_l5 * kC_l5) * dc11.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc11.im;
  kC_l4_re = ((((((-kC_l4 + x * dc12.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc15.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc17.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc12.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc15.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc17.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = re * kC_l4_re - r * kC_l4_im;
  r = re * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc19.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc19.im;
  dc10.re = dc10.re * (d_a * d_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc10.im = dc10.im * (d_a * d_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc10);
  dc11.re = kC_zeta * 2.0 * 0.0;
  dc11.im = kC_zeta * 2.0;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  dc19.re = kC_zeta * 0.0;
  dc19.im = kC_zeta;
  b_exp(&dc19);
  dc20.re = kC_zeta * 0.0;
  dc20.im = kC_zeta;
  b_exp(&dc20);
  dc21.re = kC_zeta * 0.0;
  dc21.im = kC_zeta;
  b_exp(&dc21);
  dc22.re = kC_zeta * 0.0;
  dc22.im = kC_zeta;
  b_exp(&dc22);
  f_x.re = kC_zeta * 2.0 * 0.0;
  f_x.im = kC_zeta * 2.0;
  b_exp(&f_x);
  g_x.re = kC_zeta * 2.0 * 0.0;
  g_x.im = kC_zeta * 2.0;
  b_exp(&g_x);
  h_x.re = kC_zeta * 0.0;
  h_x.im = kC_zeta;
  b_exp(&h_x);
  i_x.re = kC_zeta * 0.0;
  i_x.im = kC_zeta;
  b_exp(&i_x);
  j_x.re = kC_zeta * 0.0;
  j_x.im = kC_zeta;
  b_exp(&j_x);
  k_x.re = kC_zeta * 0.0;
  k_x.im = kC_zeta;
  b_exp(&k_x);
  dc23.re = kC_zeta * 0.0;
  dc23.im = kC_zeta;
  b_exp(&dc23);
  dc24.re = kC_zeta * 0.0;
  dc24.im = kC_zeta;
  b_exp(&dc24);
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  b_exp(&y);
  kC_l1_re = kC_l1 * kC_l1 * dc1.re;
  kC_l1_im = kC_l1 * kC_l1 * dc1.im;
  kC_l2_re = kC_l2 * kC_l2 * dc2.re;
  kC_l2_im = kC_l2 * kC_l2 * dc2.im;
  kC_l3_re = kC_l3 * kC_l3 * dc3.re;
  kC_l3_im = kC_l3 * kC_l3 * dc3.im;
  kC_l5_re = kC_l5 * kC_l5 * dc4.re;
  kC_l5_im = kC_l5 * kC_l5 * dc4.im;
  kC_l6_re = kC_l6 * kC_l6 * dc5.re;
  kC_l6_im = kC_l6 * kC_l6 * dc5.im;
  kC_l7_re = kC_l7 * kC_l7 * dc6.re;
  kC_l7_im = kC_l7 * kC_l7 * dc6.im;
  a_re = c_a * c_a * dc7.re;
  a_im = c_a * c_a * dc7.im;
  x_re = x * x * dc8.re;
  x_im = x * x * dc8.im;
  u_re = u[2] * u[2] * dc9.re;
  u_im = u[2] * u[2] * dc9.im;
  if (dc11.im == 0.0) {
    re = dc11.re / 2.0;
    r = 0.0;
  } else if (dc11.re == 0.0) {
    re = 0.0;
    r = dc11.im / 2.0;
  } else {
    re = dc11.re / 2.0;
    r = dc11.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (re + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc12.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc12.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc13.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc13.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc14.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc14.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc15.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc15.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc16.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc16.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc17.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc17.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc18.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc18.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc19.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc19.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc20.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc20.im);
  b_kC_l4_re = d_x * d_x * (kC_l4 * kC_l4 * dc21.re);
  b_kC_l4_im = d_x * d_x * (kC_l4 * kC_l4 * dc21.im);
  c_kC_l4_re = e_x * e_x * (kC_l4 * kC_l4 * dc22.re);
  c_kC_l4_im = e_x * e_x * (kC_l4 * kC_l4 * dc22.im);
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

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_x_re + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * b_x_im);
  b_x_re = x * h_x.re;
  b_x_im = x * h_x.im;
  d_kC_l2_re = kC_l2 * k_x.re;
  d_kC_l2_im = kC_l2 * k_x.im;
  c_kC_l7_re = kC_l7 * dc24.re;
  c_kC_l7_im = kC_l7 * dc24.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (x_re * 0.0 - x_im)) + (u_re * 0.0 - u_im)) + (dc10.re * 0.0 -
    dc10.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im))
                        - (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
    b_kC_l2_im)) + (b_kC_l6_re * 0.0 - b_kC_l6_im)) - kC_l1 * kC_l4 *
                     (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * kC_l6 *
                    (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * (kC_l8 +
    kC_r) * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) - (c_kC_l2_re * 0.0 -
    c_kC_l2_im)) + (b_kC_l7_re * 0.0 - b_kC_l7_im)) - (d_kC_l1_re * 0.0 -
    d_kC_l1_im)) + (c_kC_l6_re * 0.0 - c_kC_l6_im)) + (kC_l8_re * 0.0 - kC_l8_im))
             + kC_l4 * u[2] * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) +
            (b_kC_l4_re * 0.0 - b_kC_l4_im)) + (c_kC_l4_re * 0.0 - c_kC_l4_im))
          - (d_kC_l4_re * 0.0 - r)) + kC_l2 * kC_l4 * (g_x.re * 0.0 - g_x.im));
  u_im = -((((((((((((((((((((((((((((kC_l1_re + kC_l1_im * 0.0) + (kC_l2_re +
    kC_l2_im * 0.0)) - (kC_l3_re + kC_l3_im * 0.0)) + (kC_l5_re + kC_l5_im * 0.0))
    + (kC_l6_re + kC_l6_im * 0.0)) + (kC_l7_re + kC_l7_im * 0.0)) + (a_re + a_im
    * 0.0)) + (x_re + x_im * 0.0)) + (u_re + u_im * 0.0)) + (dc10.re + dc10.im *
    0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im * 0.0)) -
    (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re + b_kC_l2_im * 0.0)) +
                        (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((g_x.re + g_x.im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (b_x_re * 0.0 - b_x_im)) + u[2] *
    i_x.re) - kC_l1 * j_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    dc23.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * y.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (b_x_re + b_x_im * 0.0)) + u[2] * i_x.im) -
    kC_l1 * j_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * dc23.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) * y.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc1.re = ar / u_re;
      dc1.im = 0.0;
    } else if (ar == 0.0) {
      dc1.re = 0.0;
      dc1.im = u_im / u_re;
    } else {
      dc1.re = ar / u_re;
      dc1.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc1.re = u_im / r;
      dc1.im = 0.0;
    } else if (u_im == 0.0) {
      dc1.re = 0.0;
      dc1.im = -(ar / r);
    } else {
      dc1.re = u_im / r;
      dc1.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    alpha = muDoubleScalarAbs(r);
    if (d_kC_l4_re > alpha) {
      beta = r / u_re;
      alpha = u_re + beta * r;
      dc1.re = (ar + beta * u_im) / alpha;
      dc1.im = (u_im - beta * ar) / alpha;
    } else if (alpha == d_kC_l4_re) {
      if (u_re > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      if (r > 0.0) {
        alpha = 0.5;
      } else {
        alpha = -0.5;
      }

      dc1.re = (ar * beta + u_im * alpha) / d_kC_l4_re;
      dc1.im = (u_im * beta - ar * alpha) / d_kC_l4_re;
    } else {
      beta = u_re / r;
      alpha = r + beta * u_re;
      dc1.re = (beta * ar + u_im) / alpha;
      dc1.im = (beta * u_im - ar) / alpha;
    }
  }

  b_log(&dc1);
  gammaRaw[0].re = -kC_zeta - (dc0.re * 0.0 - dc0.im);
  re = dc1.re * 0.0 - dc1.im;
  a = kC_l8 + kC_r;
  b_a = kC_l8 + kC_r;
  b_a = ((((((((((((((((((((((((kC_l1 * kC_l1 - 2.0 * muDoubleScalarSin(kC_zeta)
    * kC_l1 * kC_l4) - 2.0 * kC_l1 * kC_l6) - 2.0 * kC_l1 * (kC_l8 + kC_r)) -
    2.0 * kC_l1 * u[2]) + kC_l2 * kC_l2) + 2.0 * muDoubleScalarCos(kC_zeta) *
    kC_l2 * kC_l4) - 2.0 * kC_l2 * kC_l7) - 2.0 * kC_l2 * x) - kC_l3 * kC_l3) +
                       kC_l4 * kC_l4) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4
                      * kC_l6) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 *
                     kC_l7) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4 * (kC_l8
    + kC_r)) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 * x) + 2.0 *
                  muDoubleScalarSin(kC_zeta) * kC_l4 * u[2]) + kC_l5 * kC_l5) +
                kC_l6 * kC_l6) + 2.0 * kC_l6 * (kC_l8 + kC_r)) + 2.0 * kC_l6 *
              u[2]) + kC_l7 * kC_l7) + 2.0 * kC_l7 * x) + b_a * b_a) + 2.0 *
          (kC_l8 + kC_r) * u[2]) + x * x) + u[2] * u[2];
  b_x = muDoubleScalarCos(kC_zeta);
  c_x = muDoubleScalarSin(kC_zeta);
  c_a = kC_l8 + kC_r;
  d_a = kC_l8 + kC_r;
  d_a = ((((((((((((((((((((((((kC_l1 * kC_l1 - 2.0 * muDoubleScalarSin(kC_zeta)
    * kC_l1 * kC_l4) - 2.0 * kC_l1 * kC_l6) - 2.0 * kC_l1 * (kC_l8 + kC_r)) -
    2.0 * kC_l1 * u[2]) + kC_l2 * kC_l2) + 2.0 * muDoubleScalarCos(kC_zeta) *
    kC_l2 * kC_l4) - 2.0 * kC_l2 * kC_l7) - 2.0 * kC_l2 * x) - kC_l3 * kC_l3) +
                       kC_l4 * kC_l4) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4
                      * kC_l6) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 *
                     kC_l7) + 2.0 * muDoubleScalarSin(kC_zeta) * kC_l4 * (kC_l8
    + kC_r)) - 2.0 * muDoubleScalarCos(kC_zeta) * kC_l4 * x) + 2.0 *
                  muDoubleScalarSin(kC_zeta) * kC_l4 * u[2]) + kC_l5 * kC_l5) +
                kC_l6 * kC_l6) + 2.0 * kC_l6 * (kC_l8 + kC_r)) + 2.0 * kC_l6 *
              u[2]) + kC_l7 * kC_l7) + 2.0 * kC_l7 * x) + d_a * d_a) + 2.0 *
          (kC_l8 + kC_r) * u[2]) + x * x) + u[2] * u[2];
  d_x = muDoubleScalarCos(kC_zeta);
  e_x = muDoubleScalarSin(kC_zeta);
  y.re = kC_zeta * 2.0 * 0.0;
  y.im = kC_zeta * 2.0;
  r = muDoubleScalarExp(y.re / 2.0);
  x_re = r * (r * muDoubleScalarCos(y.im));
  x_im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  f_x.re = r * (r * muDoubleScalarCos(y.im));
  f_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  g_x.re = r * (r * muDoubleScalarCos(y.im));
  g_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  h_x.re = r * (r * muDoubleScalarCos(y.im));
  h_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  i_x.re = r * (r * muDoubleScalarCos(y.im));
  i_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  j_x.re = r * (r * muDoubleScalarCos(y.im));
  j_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  k_x.re = r * (r * muDoubleScalarCos(y.im));
  k_x.im = r * (r * muDoubleScalarSin(y.im));
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  r = muDoubleScalarExp(y.re / 2.0);
  y.re = r * (r * muDoubleScalarCos(y.im));
  y.im = r * (r * muDoubleScalarSin(y.im));
  dc0.re = kC_zeta * 0.0;
  dc0.im = kC_zeta;
  b_exp(&dc0);
  dc1.re = kC_zeta * 0.0;
  dc1.im = kC_zeta;
  b_exp(&dc1);
  dc2.re = kC_zeta * 0.0;
  dc2.im = kC_zeta;
  b_exp(&dc2);
  dc3.re = kC_zeta * 0.0;
  dc3.im = kC_zeta;
  b_exp(&dc3);
  dc4.re = kC_zeta * 0.0;
  dc4.im = kC_zeta;
  b_exp(&dc4);
  dc5.re = kC_zeta * 0.0;
  dc5.im = kC_zeta;
  b_exp(&dc5);
  dc6.re = kC_zeta * 0.0;
  dc6.im = kC_zeta;
  b_exp(&dc6);
  dc7.re = kC_zeta * 0.0;
  dc7.im = kC_zeta;
  b_exp(&dc7);
  dc8.re = kC_zeta * 0.0;
  dc8.im = kC_zeta;
  b_exp(&dc8);
  dc9.re = kC_zeta * 2.0 * 0.0;
  dc9.im = kC_zeta * 2.0;
  b_exp(&dc9);
  dc10.re = kC_zeta * 0.0;
  dc10.im = kC_zeta;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  u_re = u[2] * dc12.re;
  u_im = u[2] * dc12.im;
  kC_l1_re = kC_l1 * dc13.re;
  kC_l1_im = kC_l1 * dc13.im;
  kC_l6_re = kC_l6 * dc15.re;
  kC_l6_im = kC_l6 * dc15.im;
  kC_l8_re = (kC_l8 + kC_r) * dc17.re;
  kC_l8_im = (kC_l8 + kC_r) * dc17.im;
  beta = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  alpha = beta * kC_l4_re - r * kC_l4_im;
  r = beta * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc18.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc18.im;
  dc9.re = dc9.re * (d_a * d_a) + (alpha * kC_l2_re - r * kC_l2_im);
  dc9.im = dc9.im * (d_a * d_a) + (alpha * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kC_zeta * 2.0 * 0.0;
  dc10.im = kC_zeta * 2.0;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  dc19.re = kC_zeta * 0.0;
  dc19.im = kC_zeta;
  b_exp(&dc19);
  dc20.re = kC_zeta * 0.0;
  dc20.im = kC_zeta;
  b_exp(&dc20);
  dc21.re = kC_zeta * 0.0;
  dc21.im = kC_zeta;
  b_exp(&dc21);
  dc22.re = kC_zeta * 2.0 * 0.0;
  dc22.im = kC_zeta * 2.0;
  b_exp(&dc22);
  kC_l1_re = kC_l1 * kC_l1 * dc0.re;
  kC_l1_im = kC_l1 * kC_l1 * dc0.im;
  kC_l2_re = kC_l2 * kC_l2 * dc1.re;
  kC_l2_im = kC_l2 * kC_l2 * dc1.im;
  kC_l3_re = kC_l3 * kC_l3 * dc2.re;
  kC_l3_im = kC_l3 * kC_l3 * dc2.im;
  kC_l5_re = kC_l5 * kC_l5 * dc3.re;
  kC_l5_im = kC_l5 * kC_l5 * dc3.im;
  kC_l6_re = kC_l6 * kC_l6 * dc4.re;
  kC_l6_im = kC_l6 * kC_l6 * dc4.im;
  kC_l7_re = kC_l7 * kC_l7 * dc5.re;
  kC_l7_im = kC_l7 * kC_l7 * dc5.im;
  a_re = c_a * c_a * dc6.re;
  a_im = c_a * c_a * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  u_re = u[2] * u[2] * dc8.re;
  u_im = u[2] * u[2] * dc8.im;
  if (dc10.im == 0.0) {
    beta = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    beta = 0.0;
    r = dc10.im / 2.0;
  } else {
    beta = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (beta + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc11.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc11.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc13.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc13.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc15.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc15.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc16.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc16.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc17.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc17.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc18.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc18.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.im);
  b_kC_l4_re = d_x * d_x * (kC_l4 * kC_l4 * dc20.re);
  b_kC_l4_im = d_x * d_x * (kC_l4 * kC_l4 * dc20.im);
  c_kC_l4_re = e_x * e_x * (kC_l4 * kC_l4 * dc21.re);
  c_kC_l4_im = e_x * e_x * (kC_l4 * kC_l4 * dc21.im);
  if (dc22.im == 0.0) {
    beta = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    beta = 0.0;
    r = dc22.im / 2.0;
  } else {
    beta = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (beta + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  alpha = x * f_x.re;
  beta = x * f_x.im;
  d_kC_l2_re = kC_l2 * i_x.re;
  d_kC_l2_im = kC_l2 * i_x.im;
  c_kC_l7_re = kC_l7 * k_x.re;
  c_kC_l7_im = kC_l7 * k_x.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (b_x_re * 0.0 - b_x_im)) + (u_re * 0.0 - u_im)) + (dc9.re * 0.0 -
    dc9.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im)) -
                        (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
    b_kC_l2_im)) + (b_kC_l6_re * 0.0 - b_kC_l6_im)) - kC_l1 * kC_l4 *
                     (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * kC_l6 *
                    (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * (kC_l8 +
    kC_r) * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) - (c_kC_l2_re * 0.0 -
    c_kC_l2_im)) + (b_kC_l7_re * 0.0 - b_kC_l7_im)) - (d_kC_l1_re * 0.0 -
    d_kC_l1_im)) + (c_kC_l6_re * 0.0 - c_kC_l6_im)) + (kC_l8_re * 0.0 - kC_l8_im))
             + kC_l4 * u[2] * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) +
            (b_kC_l4_re * 0.0 - b_kC_l4_im)) + (c_kC_l4_re * 0.0 - c_kC_l4_im))
          - (d_kC_l4_re * 0.0 - r)) + kC_l2 * kC_l4 * (x_re * 0.0 - x_im));
  u_im = -((((((((((((((((((((((((((((kC_l1_re + kC_l1_im * 0.0) + (kC_l2_re +
    kC_l2_im * 0.0)) - (kC_l3_re + kC_l3_im * 0.0)) + (kC_l5_re + kC_l5_im * 0.0))
    + (kC_l6_re + kC_l6_im * 0.0)) + (kC_l7_re + kC_l7_im * 0.0)) + (a_re + a_im
    * 0.0)) + (b_x_re + b_x_im * 0.0)) + (u_re + u_im * 0.0)) + (dc9.re + dc9.im
    * 0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im * 0.0)) -
                          (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re +
    b_kC_l2_im * 0.0)) + (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((x_re + x_im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (alpha * 0.0 - beta)) + u[2] *
    g_x.re) - kC_l1 * h_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    j_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * y.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (alpha + beta * 0.0)) + u[2] * g_x.im) -
    kC_l1 * h_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * j_x.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) * y.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      y.re = ar / u_re;
      y.im = 0.0;
    } else if (ar == 0.0) {
      y.re = 0.0;
      y.im = u_im / u_re;
    } else {
      y.re = ar / u_re;
      y.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      y.re = u_im / r;
      y.im = 0.0;
    } else if (u_im == 0.0) {
      y.re = 0.0;
      y.im = -(ar / r);
    } else {
      y.re = u_im / r;
      y.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    alpha = muDoubleScalarAbs(r);
    if (d_kC_l4_re > alpha) {
      beta = r / u_re;
      alpha = u_re + beta * r;
      y.re = (ar + beta * u_im) / alpha;
      y.im = (u_im - beta * ar) / alpha;
    } else if (alpha == d_kC_l4_re) {
      if (u_re > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      if (r > 0.0) {
        alpha = 0.5;
      } else {
        alpha = -0.5;
      }

      y.re = (ar * beta + u_im * alpha) / d_kC_l4_re;
      y.im = (u_im * beta - ar * alpha) / d_kC_l4_re;
    } else {
      beta = u_re / r;
      alpha = r + beta * u_re;
      y.re = (beta * ar + u_im) / alpha;
      y.im = (beta * u_im - ar) / alpha;
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

  dc0.re = kC_zeta * 0.0;
  dc0.im = kC_zeta;
  b_exp(&dc0);
  dc1.re = kC_zeta * 0.0;
  dc1.im = kC_zeta;
  b_exp(&dc1);
  dc2.re = kC_zeta * 0.0;
  dc2.im = kC_zeta;
  b_exp(&dc2);
  dc3.re = kC_zeta * 0.0;
  dc3.im = kC_zeta;
  b_exp(&dc3);
  dc4.re = kC_zeta * 0.0;
  dc4.im = kC_zeta;
  b_exp(&dc4);
  dc5.re = kC_zeta * 0.0;
  dc5.im = kC_zeta;
  b_exp(&dc5);
  dc6.re = kC_zeta * 0.0;
  dc6.im = kC_zeta;
  b_exp(&dc6);
  dc7.re = kC_zeta * 0.0;
  dc7.im = kC_zeta;
  b_exp(&dc7);
  dc8.re = kC_zeta * 0.0;
  dc8.im = kC_zeta;
  b_exp(&dc8);
  dc9.re = kC_zeta * 2.0 * 0.0;
  dc9.im = kC_zeta * 2.0;
  b_exp(&dc9);
  dc10.re = kC_zeta * 0.0;
  dc10.im = kC_zeta;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  u_re = u[2] * dc12.re;
  u_im = u[2] * dc12.im;
  kC_l1_re = kC_l1 * dc13.re;
  kC_l1_im = kC_l1 * dc13.im;
  kC_l6_re = kC_l6 * dc15.re;
  kC_l6_im = kC_l6 * dc15.im;
  kC_l8_re = (kC_l8 + kC_r) * dc17.re;
  kC_l8_im = (kC_l8 + kC_r) * dc17.im;
  beta = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  alpha = beta * kC_l4_re - r * kC_l4_im;
  r = beta * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc18.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc18.im;
  dc9.re = dc9.re * (b_a * b_a) + (alpha * kC_l2_re - r * kC_l2_im);
  dc9.im = dc9.im * (b_a * b_a) + (alpha * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kC_zeta * 2.0 * 0.0;
  dc10.im = kC_zeta * 2.0;
  b_exp(&dc10);
  dc11.re = kC_zeta * 0.0;
  dc11.im = kC_zeta;
  b_exp(&dc11);
  dc12.re = kC_zeta * 0.0;
  dc12.im = kC_zeta;
  b_exp(&dc12);
  dc13.re = kC_zeta * 0.0;
  dc13.im = kC_zeta;
  b_exp(&dc13);
  dc14.re = kC_zeta * 0.0;
  dc14.im = kC_zeta;
  b_exp(&dc14);
  dc15.re = kC_zeta * 0.0;
  dc15.im = kC_zeta;
  b_exp(&dc15);
  dc16.re = kC_zeta * 0.0;
  dc16.im = kC_zeta;
  b_exp(&dc16);
  dc17.re = kC_zeta * 0.0;
  dc17.im = kC_zeta;
  b_exp(&dc17);
  dc18.re = kC_zeta * 0.0;
  dc18.im = kC_zeta;
  b_exp(&dc18);
  dc19.re = kC_zeta * 0.0;
  dc19.im = kC_zeta;
  b_exp(&dc19);
  dc20.re = kC_zeta * 0.0;
  dc20.im = kC_zeta;
  b_exp(&dc20);
  dc21.re = kC_zeta * 0.0;
  dc21.im = kC_zeta;
  b_exp(&dc21);
  dc22.re = kC_zeta * 2.0 * 0.0;
  dc22.im = kC_zeta * 2.0;
  b_exp(&dc22);
  f_x.re = kC_zeta * 2.0 * 0.0;
  f_x.im = kC_zeta * 2.0;
  b_exp(&f_x);
  g_x.re = kC_zeta * 0.0;
  g_x.im = kC_zeta;
  b_exp(&g_x);
  h_x.re = kC_zeta * 0.0;
  h_x.im = kC_zeta;
  b_exp(&h_x);
  i_x.re = kC_zeta * 0.0;
  i_x.im = kC_zeta;
  b_exp(&i_x);
  j_x.re = kC_zeta * 0.0;
  j_x.im = kC_zeta;
  b_exp(&j_x);
  k_x.re = kC_zeta * 0.0;
  k_x.im = kC_zeta;
  b_exp(&k_x);
  dc23.re = kC_zeta * 0.0;
  dc23.im = kC_zeta;
  b_exp(&dc23);
  dc24.re = kC_zeta * 0.0;
  dc24.im = kC_zeta;
  b_exp(&dc24);
  kC_l1_re = kC_l1 * kC_l1 * dc0.re;
  kC_l1_im = kC_l1 * kC_l1 * dc0.im;
  kC_l2_re = kC_l2 * kC_l2 * dc1.re;
  kC_l2_im = kC_l2 * kC_l2 * dc1.im;
  kC_l3_re = kC_l3 * kC_l3 * dc2.re;
  kC_l3_im = kC_l3 * kC_l3 * dc2.im;
  kC_l5_re = kC_l5 * kC_l5 * dc3.re;
  kC_l5_im = kC_l5 * kC_l5 * dc3.im;
  kC_l6_re = kC_l6 * kC_l6 * dc4.re;
  kC_l6_im = kC_l6 * kC_l6 * dc4.im;
  kC_l7_re = kC_l7 * kC_l7 * dc5.re;
  kC_l7_im = kC_l7 * kC_l7 * dc5.im;
  a_re = a * a * dc6.re;
  a_im = a * a * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  u_re = u[2] * u[2] * dc8.re;
  u_im = u[2] * u[2] * dc8.im;
  if (dc10.im == 0.0) {
    beta = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    beta = 0.0;
    r = dc10.im / 2.0;
  } else {
    beta = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (beta + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc11.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc11.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc12.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc13.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc13.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc14.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc15.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc15.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc16.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc16.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc17.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc17.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc18.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc18.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc19.im);
  b_kC_l4_re = b_x * b_x * (kC_l4 * kC_l4 * dc20.re);
  b_kC_l4_im = b_x * b_x * (kC_l4 * kC_l4 * dc20.im);
  c_kC_l4_re = c_x * c_x * (kC_l4 * kC_l4 * dc21.re);
  c_kC_l4_im = c_x * c_x * (kC_l4 * kC_l4 * dc21.im);
  if (dc22.im == 0.0) {
    beta = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    beta = 0.0;
    r = dc22.im / 2.0;
  } else {
    beta = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (beta + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  alpha = x * g_x.re;
  beta = x * g_x.im;
  d_kC_l2_re = kC_l2 * j_x.re;
  d_kC_l2_im = kC_l2 * j_x.im;
  c_kC_l7_re = kC_l7 * dc23.re;
  c_kC_l7_im = kC_l7 * dc23.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (b_x_re * 0.0 - b_x_im)) + (u_re * 0.0 - u_im)) - (dc9.re * 0.0 -
    dc9.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im)) -
                        (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
    b_kC_l2_im)) + (b_kC_l6_re * 0.0 - b_kC_l6_im)) - kC_l1 * kC_l4 *
                     (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * kC_l6 *
                    (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) + kC_l4 * (kC_l8 +
    kC_r) * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) - (c_kC_l2_re * 0.0 -
    c_kC_l2_im)) + (b_kC_l7_re * 0.0 - b_kC_l7_im)) - (d_kC_l1_re * 0.0 -
    d_kC_l1_im)) + (c_kC_l6_re * 0.0 - c_kC_l6_im)) + (kC_l8_re * 0.0 - kC_l8_im))
             + kC_l4 * u[2] * (muDoubleScalarCos(2.0 * kC_zeta) - 1.0)) +
            (b_kC_l4_re * 0.0 - b_kC_l4_im)) + (c_kC_l4_re * 0.0 - c_kC_l4_im))
          - (d_kC_l4_re * 0.0 - r)) + kC_l2 * kC_l4 * (f_x.re * 0.0 - f_x.im));
  u_im = -((((((((((((((((((((((((((((kC_l1_re + kC_l1_im * 0.0) + (kC_l2_re +
    kC_l2_im * 0.0)) - (kC_l3_re + kC_l3_im * 0.0)) + (kC_l5_re + kC_l5_im * 0.0))
    + (kC_l6_re + kC_l6_im * 0.0)) + (kC_l7_re + kC_l7_im * 0.0)) + (a_re + a_im
    * 0.0)) + (b_x_re + b_x_im * 0.0)) + (u_re + u_im * 0.0)) - (dc9.re + dc9.im
    * 0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im * 0.0)) -
                          (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re +
    b_kC_l2_im * 0.0)) + (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((f_x.re + f_x.im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (alpha * 0.0 - beta)) + u[2] *
    h_x.re) - kC_l1 * i_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    k_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * dc24.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (alpha + beta * 0.0)) + u[2] * h_x.im) -
    kC_l1 * i_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * k_x.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) *
                     dc24.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc0.re = ar / u_re;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = u_im / u_re;
    } else {
      dc0.re = ar / u_re;
      dc0.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc0.re = u_im / r;
      dc0.im = 0.0;
    } else if (u_im == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / r);
    } else {
      dc0.re = u_im / r;
      dc0.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    alpha = muDoubleScalarAbs(r);
    if (d_kC_l4_re > alpha) {
      beta = r / u_re;
      alpha = u_re + beta * r;
      dc0.re = (ar + beta * u_im) / alpha;
      dc0.im = (u_im - beta * ar) / alpha;
    } else if (alpha == d_kC_l4_re) {
      if (u_re > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      if (r > 0.0) {
        alpha = 0.5;
      } else {
        alpha = -0.5;
      }

      dc0.re = (ar * beta + u_im * alpha) / d_kC_l4_re;
      dc0.im = (u_im * beta - ar * alpha) / d_kC_l4_re;
    } else {
      beta = u_re / r;
      alpha = r + beta * u_re;
      dc0.re = (beta * ar + u_im) / alpha;
      dc0.im = (beta * u_im - ar) / alpha;
    }
  }

  b_log(&dc0);
  beta = dc0.re;
  dc0.re = dc0.re * 0.0 - dc0.im;
  dc0.im = beta + dc0.im * 0.0;
  b_sin(&dc0);
  kC_l6_re = ((((kC_l6 - kC_l1) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
              muDoubleScalarSin(kC_zeta)) - kC_l5 * dc0.re;
  kC_l6_im = 0.0 - kC_l5 * dc0.im;
  if (kC_l6_im == 0.0) {
    dc0.re = kC_l6_re / kC_l3;
    dc0.im = 0.0;
  } else if (kC_l6_re == 0.0) {
    dc0.re = 0.0;
    dc0.im = kC_l6_im / kC_l3;
  } else {
    dc0.re = kC_l6_re / kC_l3;
    dc0.im = kC_l6_im / kC_l3;
  }

  b_asin(&dc0);
  ar = ((((kC_l6 - kC_l1) + (kC_l8 + kC_r)) + u[2]) + kC_l4 * muDoubleScalarSin
        (kC_zeta)) - kC_l5 * x_re;
  u_im = 0.0 - kC_l5 * x_im;
  if (u_im == 0.0) {
    dc1.re = ar / kC_l3;
    dc1.im = 0.0;
  } else if (ar == 0.0) {
    dc1.re = 0.0;
    dc1.im = u_im / kC_l3;
  } else {
    dc1.re = ar / kC_l3;
    dc1.im = u_im / kC_l3;
  }

  b_asin(&dc1);
  alpha = muDoubleScalarAtan2(0.0, u[0]);

  /* beta = betaRaw(1); */
  /* gamma = gammaRaw(1); */
  if ((alpha >= jointLimits[0]) && (alpha <= jointLimits[1]) && (-dc0.re >=
       jointLimits[2]) && (-dc0.re <= jointLimits[3]) && (gammaRaw[0].re >=
       jointLimits[4]) && (gammaRaw[0].re <= jointLimits[5])) {
    beta = -dc0.re;
    r = gammaRaw[0].re;

    /* elseif alpha >= alphaMin && alpha <= alphaMax && betaRaw(2) >= betaMin && betaRaw(2) <= betaMax && gammaRaw(2) >= gammaMin && gammaRaw(2) <= gammaMax */
  } else {
    beta = -dc1.re;
    r = -kC_zeta - re;
  }

  q[0] = alpha;
  q[1] = beta;
  q[2] = r;
}

/* End of code generation (sherpaTTIK.c) */
