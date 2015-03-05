/*
 * sherpaTTIK.c
 *
 * Code generation for function 'sherpaTTIK'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTIK.h"
#include "eml_error.h"
#include "fprintf.h"
#include "asin.h"
#include "sin.h"
#include "log.h"
#include "exp.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtRSInfo e_emlrtRSI = { 22, "sherpaTTIK",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/sherpaTTIK.m"
};

static emlrtRSInfo f_emlrtRSI = { 54, "sherpaTTIK",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/sherpaTTIK.m"
};

static emlrtRSInfo g_emlrtRSI = { 14, "sqrt",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/sqrt.m" };

/* Function Definitions */
void b_sherpaTTIK(const emlrtStack *sp, const real_T u[3], real_T kC_l1, real_T
                  kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6,
                  real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const
                  real_T jointLimits[20], real_T q[3])
{
  real_T x;
  real_T a;
  real_T b_a;
  real_T b_x;
  real_T c_x;
  real_T c_a;
  real_T d_a;
  real_T d_x;
  real_T e_x;
  creal_T dc25;
  creal_T dc26;
  creal_T dc27;
  creal_T dc28;
  creal_T dc29;
  creal_T dc30;
  creal_T dc31;
  creal_T dc32;
  creal_T dc33;
  creal_T dc34;
  creal_T dc35;
  creal_T dc36;
  creal_T dc37;
  creal_T dc38;
  creal_T dc39;
  creal_T dc40;
  creal_T dc41;
  creal_T dc42;
  creal_T dc43;
  real_T u_re;
  real_T u_im;
  real_T kC_l1_re;
  real_T kC_l1_im;
  real_T kC_l6_re;
  real_T kC_l6_im;
  real_T kC_l8_re;
  real_T kC_l8_im;
  real_T b_gamma;
  real_T r;
  real_T kC_l4_re;
  real_T kC_l4_im;
  real_T beta;
  real_T kC_l2_re;
  real_T kC_l2_im;
  creal_T dc44;
  creal_T dc45;
  creal_T dc46;
  creal_T dc47;
  creal_T f_x;
  creal_T g_x;
  creal_T h_x;
  creal_T i_x;
  creal_T j_x;
  creal_T k_x;
  creal_T dc48;
  creal_T dc49;
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
  creal_T y;
  creal_T gammaRaw[2];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* SHERPATTIK Calculates the joint values for a g1iven contact point. */
  /*    Calculates the joint values for a g1iven contact point for the Sherpa TT */
  /*    leg. All coord1inates are in the pan joint coord1inate frame. */
  /*  */
  /* Inputs: */
  /* -u: A 1x3 Cartesian vector in the pan frame containing [xP yP zP]. */
  /* -kC: A struct containing the kinematic constants of the Sherpa TT Rover. */
  /* -jointLimits: The joint limits of each of the rover's joints. */
  /* Outputs: */
  /* -q: A 1x3 joint vector containing [alpha beta gamma]. */
  /* sherpaTTIK.m */
  /* author: wreid */
  /* date: 20150122 */
  st.site = &e_emlrtRSI;
  x = u[0] * u[0] + u[1] * u[1];
  if (x < 0.0) {
    b_st.site = &g_emlrtRSI;
    eml_error(&b_st);
  }

  x = muDoubleScalarSqrt(x);
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
  dc25.re = kC_zeta * 0.0;
  dc25.im = kC_zeta;
  b_exp(&dc25);
  dc26.re = kC_zeta * 0.0;
  dc26.im = kC_zeta;
  b_exp(&dc26);
  dc27.re = kC_zeta * 0.0;
  dc27.im = kC_zeta;
  b_exp(&dc27);
  dc28.re = kC_zeta * 0.0;
  dc28.im = kC_zeta;
  b_exp(&dc28);
  dc29.re = kC_zeta * 0.0;
  dc29.im = kC_zeta;
  b_exp(&dc29);
  dc30.re = kC_zeta * 0.0;
  dc30.im = kC_zeta;
  b_exp(&dc30);
  dc31.re = kC_zeta * 0.0;
  dc31.im = kC_zeta;
  b_exp(&dc31);
  dc32.re = kC_zeta * 0.0;
  dc32.im = kC_zeta;
  b_exp(&dc32);
  dc33.re = kC_zeta * 0.0;
  dc33.im = kC_zeta;
  b_exp(&dc33);
  dc34.re = kC_zeta * 2.0 * 0.0;
  dc34.im = kC_zeta * 2.0;
  b_exp(&dc34);
  dc35.re = kC_zeta * 0.0;
  dc35.im = kC_zeta;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  u_re = u[2] * dc37.re;
  u_im = u[2] * dc37.im;
  kC_l1_re = kC_l1 * dc38.re;
  kC_l1_im = kC_l1 * dc38.im;
  kC_l6_re = kC_l6 * dc40.re;
  kC_l6_im = kC_l6 * dc40.im;
  kC_l8_re = (kC_l8 + kC_r) * dc42.re;
  kC_l8_im = (kC_l8 + kC_r) * dc42.im;
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc35.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc35.im;
  kC_l4_re = ((((((-kC_l4 + x * dc36.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc39.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc41.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc36.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc39.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc41.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc43.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc43.im;
  dc34.re = dc34.re * (b_a * b_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc34.im = dc34.im * (b_a * b_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc34);
  dc35.re = kC_zeta * 2.0 * 0.0;
  dc35.im = kC_zeta * 2.0;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  dc44.re = kC_zeta * 0.0;
  dc44.im = kC_zeta;
  b_exp(&dc44);
  dc45.re = kC_zeta * 0.0;
  dc45.im = kC_zeta;
  b_exp(&dc45);
  dc46.re = kC_zeta * 0.0;
  dc46.im = kC_zeta;
  b_exp(&dc46);
  dc47.re = kC_zeta * 2.0 * 0.0;
  dc47.im = kC_zeta * 2.0;
  b_exp(&dc47);
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
  dc48.re = kC_zeta * 0.0;
  dc48.im = kC_zeta;
  b_exp(&dc48);
  dc49.re = kC_zeta * 0.0;
  dc49.im = kC_zeta;
  b_exp(&dc49);
  kC_l1_re = kC_l1 * kC_l1 * dc25.re;
  kC_l1_im = kC_l1 * kC_l1 * dc25.im;
  kC_l2_re = kC_l2 * kC_l2 * dc26.re;
  kC_l2_im = kC_l2 * kC_l2 * dc26.im;
  kC_l3_re = kC_l3 * kC_l3 * dc27.re;
  kC_l3_im = kC_l3 * kC_l3 * dc27.im;
  kC_l5_re = kC_l5 * kC_l5 * dc28.re;
  kC_l5_im = kC_l5 * kC_l5 * dc28.im;
  kC_l6_re = kC_l6 * kC_l6 * dc29.re;
  kC_l6_im = kC_l6 * kC_l6 * dc29.im;
  kC_l7_re = kC_l7 * kC_l7 * dc30.re;
  kC_l7_im = kC_l7 * kC_l7 * dc30.im;
  a_re = a * a * dc31.re;
  a_im = a * a * dc31.im;
  x_re = x * x * dc32.re;
  x_im = x * x * dc32.im;
  u_re = u[2] * u[2] * dc33.re;
  u_im = u[2] * u[2] * dc33.im;
  if (dc35.im == 0.0) {
    b_gamma = dc35.re / 2.0;
    r = 0.0;
  } else if (dc35.re == 0.0) {
    b_gamma = 0.0;
    r = dc35.im / 2.0;
  } else {
    b_gamma = dc35.re / 2.0;
    r = dc35.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc36.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc36.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc38.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc38.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc40.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc40.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc41.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc41.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc42.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc42.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc43.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc43.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.im);
  b_kC_l4_re = b_x * b_x * (kC_l4 * kC_l4 * dc45.re);
  b_kC_l4_im = b_x * b_x * (kC_l4 * kC_l4 * dc45.im);
  c_kC_l4_re = c_x * c_x * (kC_l4 * kC_l4 * dc46.re);
  c_kC_l4_im = c_x * c_x * (kC_l4 * kC_l4 * dc46.im);
  if (dc47.im == 0.0) {
    b_gamma = dc47.re / 2.0;
    r = 0.0;
  } else if (dc47.re == 0.0) {
    b_gamma = 0.0;
    r = dc47.im / 2.0;
  } else {
    b_gamma = dc47.re / 2.0;
    r = dc47.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  b_x_re = x * g_x.re;
  b_x_im = x * g_x.im;
  d_kC_l2_re = kC_l2 * j_x.re;
  d_kC_l2_im = kC_l2 * j_x.im;
  c_kC_l7_re = kC_l7 * dc48.re;
  c_kC_l7_im = kC_l7 * dc48.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (x_re * 0.0 - x_im)) + (u_re * 0.0 - u_im)) - (dc34.re * 0.0 -
    dc34.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im))
                        - (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
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
    * 0.0)) + (x_re + x_im * 0.0)) + (u_re + u_im * 0.0)) - (dc34.re + dc34.im *
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
    k_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * dc49.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (b_x_re + b_x_im * 0.0)) + u[2] * h_x.im) -
    kC_l1 * i_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * k_x.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) *
                     dc49.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc25.re = ar / u_re;
      dc25.im = 0.0;
    } else if (ar == 0.0) {
      dc25.re = 0.0;
      dc25.im = u_im / u_re;
    } else {
      dc25.re = ar / u_re;
      dc25.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc25.re = u_im / r;
      dc25.im = 0.0;
    } else if (u_im == 0.0) {
      dc25.re = 0.0;
      dc25.im = -(ar / r);
    } else {
      dc25.re = u_im / r;
      dc25.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc25.re = (ar + b_gamma * u_im) / beta;
      dc25.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc25.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc25.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc25.re = (b_gamma * ar + u_im) / beta;
      dc25.im = (b_gamma * u_im - ar) / beta;
    }
  }

  b_log(&dc25);
  dc26.re = kC_zeta * 0.0;
  dc26.im = kC_zeta;
  b_exp(&dc26);
  dc27.re = kC_zeta * 0.0;
  dc27.im = kC_zeta;
  b_exp(&dc27);
  dc28.re = kC_zeta * 0.0;
  dc28.im = kC_zeta;
  b_exp(&dc28);
  dc29.re = kC_zeta * 0.0;
  dc29.im = kC_zeta;
  b_exp(&dc29);
  dc30.re = kC_zeta * 0.0;
  dc30.im = kC_zeta;
  b_exp(&dc30);
  dc31.re = kC_zeta * 0.0;
  dc31.im = kC_zeta;
  b_exp(&dc31);
  dc32.re = kC_zeta * 0.0;
  dc32.im = kC_zeta;
  b_exp(&dc32);
  dc33.re = kC_zeta * 0.0;
  dc33.im = kC_zeta;
  b_exp(&dc33);
  dc34.re = kC_zeta * 0.0;
  dc34.im = kC_zeta;
  b_exp(&dc34);
  dc35.re = kC_zeta * 2.0 * 0.0;
  dc35.im = kC_zeta * 2.0;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  dc44.re = kC_zeta * 0.0;
  dc44.im = kC_zeta;
  b_exp(&dc44);
  u_re = u[2] * dc38.re;
  u_im = u[2] * dc38.im;
  kC_l1_re = kC_l1 * dc39.re;
  kC_l1_im = kC_l1 * dc39.im;
  kC_l6_re = kC_l6 * dc41.re;
  kC_l6_im = kC_l6 * dc41.im;
  kC_l8_re = (kC_l8 + kC_r) * dc43.re;
  kC_l8_im = (kC_l8 + kC_r) * dc43.im;
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc36.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc36.im;
  kC_l4_re = ((((((-kC_l4 + x * dc37.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc40.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc42.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc37.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc40.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc42.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc44.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc44.im;
  dc35.re = dc35.re * (d_a * d_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc35.im = dc35.im * (d_a * d_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc35);
  dc36.re = kC_zeta * 2.0 * 0.0;
  dc36.im = kC_zeta * 2.0;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  dc44.re = kC_zeta * 0.0;
  dc44.im = kC_zeta;
  b_exp(&dc44);
  dc45.re = kC_zeta * 0.0;
  dc45.im = kC_zeta;
  b_exp(&dc45);
  dc46.re = kC_zeta * 0.0;
  dc46.im = kC_zeta;
  b_exp(&dc46);
  dc47.re = kC_zeta * 0.0;
  dc47.im = kC_zeta;
  b_exp(&dc47);
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
  dc48.re = kC_zeta * 0.0;
  dc48.im = kC_zeta;
  b_exp(&dc48);
  dc49.re = kC_zeta * 0.0;
  dc49.im = kC_zeta;
  b_exp(&dc49);
  y.re = kC_zeta * 0.0;
  y.im = kC_zeta;
  b_exp(&y);
  kC_l1_re = kC_l1 * kC_l1 * dc26.re;
  kC_l1_im = kC_l1 * kC_l1 * dc26.im;
  kC_l2_re = kC_l2 * kC_l2 * dc27.re;
  kC_l2_im = kC_l2 * kC_l2 * dc27.im;
  kC_l3_re = kC_l3 * kC_l3 * dc28.re;
  kC_l3_im = kC_l3 * kC_l3 * dc28.im;
  kC_l5_re = kC_l5 * kC_l5 * dc29.re;
  kC_l5_im = kC_l5 * kC_l5 * dc29.im;
  kC_l6_re = kC_l6 * kC_l6 * dc30.re;
  kC_l6_im = kC_l6 * kC_l6 * dc30.im;
  kC_l7_re = kC_l7 * kC_l7 * dc31.re;
  kC_l7_im = kC_l7 * kC_l7 * dc31.im;
  a_re = c_a * c_a * dc32.re;
  a_im = c_a * c_a * dc32.im;
  x_re = x * x * dc33.re;
  x_im = x * x * dc33.im;
  u_re = u[2] * u[2] * dc34.re;
  u_im = u[2] * u[2] * dc34.im;
  if (dc36.im == 0.0) {
    b_gamma = dc36.re / 2.0;
    r = 0.0;
  } else if (dc36.re == 0.0) {
    b_gamma = 0.0;
    r = dc36.im / 2.0;
  } else {
    b_gamma = dc36.re / 2.0;
    r = dc36.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc37.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc37.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc38.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc38.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc39.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc39.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc40.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc40.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc41.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc41.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc42.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc42.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc43.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc43.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc44.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc44.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc45.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc45.im);
  b_kC_l4_re = d_x * d_x * (kC_l4 * kC_l4 * dc46.re);
  b_kC_l4_im = d_x * d_x * (kC_l4 * kC_l4 * dc46.im);
  c_kC_l4_re = e_x * e_x * (kC_l4 * kC_l4 * dc47.re);
  c_kC_l4_im = e_x * e_x * (kC_l4 * kC_l4 * dc47.im);
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
  c_kC_l7_re = kC_l7 * dc49.re;
  c_kC_l7_im = kC_l7 * dc49.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (x_re * 0.0 - x_im)) + (u_re * 0.0 - u_im)) + (dc35.re * 0.0 -
    dc35.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im))
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
    * 0.0)) + (x_re + x_im * 0.0)) + (u_re + u_im * 0.0)) + (dc35.re + dc35.im *
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
    dc48.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * y.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (b_x_re + b_x_im * 0.0)) + u[2] * i_x.im) -
    kC_l1 * j_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * dc48.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) * y.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc26.re = ar / u_re;
      dc26.im = 0.0;
    } else if (ar == 0.0) {
      dc26.re = 0.0;
      dc26.im = u_im / u_re;
    } else {
      dc26.re = ar / u_re;
      dc26.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc26.re = u_im / r;
      dc26.im = 0.0;
    } else if (u_im == 0.0) {
      dc26.re = 0.0;
      dc26.im = -(ar / r);
    } else {
      dc26.re = u_im / r;
      dc26.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc26.re = (ar + b_gamma * u_im) / beta;
      dc26.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc26.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc26.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc26.re = (b_gamma * ar + u_im) / beta;
      dc26.im = (b_gamma * u_im - ar) / beta;
    }
  }

  b_log(&dc26);
  gammaRaw[0].re = -kC_zeta - (dc25.re * 0.0 - dc25.im);
  gammaRaw[1].re = -kC_zeta - (dc26.re * 0.0 - dc26.im);
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
  dc25.re = kC_zeta * 0.0;
  dc25.im = kC_zeta;
  b_exp(&dc25);
  dc26.re = kC_zeta * 0.0;
  dc26.im = kC_zeta;
  b_exp(&dc26);
  dc27.re = kC_zeta * 0.0;
  dc27.im = kC_zeta;
  b_exp(&dc27);
  dc28.re = kC_zeta * 0.0;
  dc28.im = kC_zeta;
  b_exp(&dc28);
  dc29.re = kC_zeta * 0.0;
  dc29.im = kC_zeta;
  b_exp(&dc29);
  dc30.re = kC_zeta * 0.0;
  dc30.im = kC_zeta;
  b_exp(&dc30);
  dc31.re = kC_zeta * 0.0;
  dc31.im = kC_zeta;
  b_exp(&dc31);
  dc32.re = kC_zeta * 0.0;
  dc32.im = kC_zeta;
  b_exp(&dc32);
  dc33.re = kC_zeta * 0.0;
  dc33.im = kC_zeta;
  b_exp(&dc33);
  dc34.re = kC_zeta * 2.0 * 0.0;
  dc34.im = kC_zeta * 2.0;
  b_exp(&dc34);
  dc35.re = kC_zeta * 0.0;
  dc35.im = kC_zeta;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  u_re = u[2] * dc37.re;
  u_im = u[2] * dc37.im;
  kC_l1_re = kC_l1 * dc38.re;
  kC_l1_im = kC_l1 * dc38.im;
  kC_l6_re = kC_l6 * dc40.re;
  kC_l6_im = kC_l6 * dc40.im;
  kC_l8_re = (kC_l8 + kC_r) * dc42.re;
  kC_l8_im = (kC_l8 + kC_r) * dc42.im;
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc35.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc35.im;
  kC_l4_re = ((((((-kC_l4 + x * dc36.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc39.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc41.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc36.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc39.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc41.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc43.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc43.im;
  dc34.re = dc34.re * (d_a * d_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc34.im = dc34.im * (d_a * d_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc34);
  dc35.re = kC_zeta * 2.0 * 0.0;
  dc35.im = kC_zeta * 2.0;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  dc44.re = kC_zeta * 0.0;
  dc44.im = kC_zeta;
  b_exp(&dc44);
  dc45.re = kC_zeta * 0.0;
  dc45.im = kC_zeta;
  b_exp(&dc45);
  dc46.re = kC_zeta * 0.0;
  dc46.im = kC_zeta;
  b_exp(&dc46);
  dc47.re = kC_zeta * 2.0 * 0.0;
  dc47.im = kC_zeta * 2.0;
  b_exp(&dc47);
  kC_l1_re = kC_l1 * kC_l1 * dc25.re;
  kC_l1_im = kC_l1 * kC_l1 * dc25.im;
  kC_l2_re = kC_l2 * kC_l2 * dc26.re;
  kC_l2_im = kC_l2 * kC_l2 * dc26.im;
  kC_l3_re = kC_l3 * kC_l3 * dc27.re;
  kC_l3_im = kC_l3 * kC_l3 * dc27.im;
  kC_l5_re = kC_l5 * kC_l5 * dc28.re;
  kC_l5_im = kC_l5 * kC_l5 * dc28.im;
  kC_l6_re = kC_l6 * kC_l6 * dc29.re;
  kC_l6_im = kC_l6 * kC_l6 * dc29.im;
  kC_l7_re = kC_l7 * kC_l7 * dc30.re;
  kC_l7_im = kC_l7 * kC_l7 * dc30.im;
  a_re = c_a * c_a * dc31.re;
  a_im = c_a * c_a * dc31.im;
  b_x_re = x * x * dc32.re;
  b_x_im = x * x * dc32.im;
  u_re = u[2] * u[2] * dc33.re;
  u_im = u[2] * u[2] * dc33.im;
  if (dc35.im == 0.0) {
    b_gamma = dc35.re / 2.0;
    r = 0.0;
  } else if (dc35.re == 0.0) {
    b_gamma = 0.0;
    r = dc35.im / 2.0;
  } else {
    b_gamma = dc35.re / 2.0;
    r = dc35.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc36.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc36.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc38.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc38.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc40.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc40.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc41.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc41.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc42.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc42.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc43.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc43.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.im);
  b_kC_l4_re = d_x * d_x * (kC_l4 * kC_l4 * dc45.re);
  b_kC_l4_im = d_x * d_x * (kC_l4 * kC_l4 * dc45.im);
  c_kC_l4_re = e_x * e_x * (kC_l4 * kC_l4 * dc46.re);
  c_kC_l4_im = e_x * e_x * (kC_l4 * kC_l4 * dc46.im);
  if (dc47.im == 0.0) {
    b_gamma = dc47.re / 2.0;
    r = 0.0;
  } else if (dc47.re == 0.0) {
    b_gamma = 0.0;
    r = dc47.im / 2.0;
  } else {
    b_gamma = dc47.re / 2.0;
    r = dc47.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  beta = x * f_x.re;
  b_gamma = x * f_x.im;
  d_kC_l2_re = kC_l2 * i_x.re;
  d_kC_l2_im = kC_l2 * i_x.im;
  c_kC_l7_re = kC_l7 * k_x.re;
  c_kC_l7_im = kC_l7 * k_x.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (b_x_re * 0.0 - b_x_im)) + (u_re * 0.0 - u_im)) + (dc34.re * 0.0 -
    dc34.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im))
                        - (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
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
    * 0.0)) + (b_x_re + b_x_im * 0.0)) + (u_re + u_im * 0.0)) + (dc34.re +
    dc34.im * 0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im *
    0.0)) - (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re + b_kC_l2_im * 0.0))
                        + (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((x_re + x_im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (beta * 0.0 - b_gamma)) + u[2] *
    g_x.re) - kC_l1 * h_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    j_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * y.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (beta + b_gamma * 0.0)) + u[2] * g_x.im) -
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
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      y.re = (ar + b_gamma * u_im) / beta;
      y.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      y.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      y.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      y.re = (b_gamma * ar + u_im) / beta;
      y.im = (b_gamma * u_im - ar) / beta;
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

  dc25.re = kC_zeta * 0.0;
  dc25.im = kC_zeta;
  b_exp(&dc25);
  dc26.re = kC_zeta * 0.0;
  dc26.im = kC_zeta;
  b_exp(&dc26);
  dc27.re = kC_zeta * 0.0;
  dc27.im = kC_zeta;
  b_exp(&dc27);
  dc28.re = kC_zeta * 0.0;
  dc28.im = kC_zeta;
  b_exp(&dc28);
  dc29.re = kC_zeta * 0.0;
  dc29.im = kC_zeta;
  b_exp(&dc29);
  dc30.re = kC_zeta * 0.0;
  dc30.im = kC_zeta;
  b_exp(&dc30);
  dc31.re = kC_zeta * 0.0;
  dc31.im = kC_zeta;
  b_exp(&dc31);
  dc32.re = kC_zeta * 0.0;
  dc32.im = kC_zeta;
  b_exp(&dc32);
  dc33.re = kC_zeta * 0.0;
  dc33.im = kC_zeta;
  b_exp(&dc33);
  dc34.re = kC_zeta * 2.0 * 0.0;
  dc34.im = kC_zeta * 2.0;
  b_exp(&dc34);
  dc35.re = kC_zeta * 0.0;
  dc35.im = kC_zeta;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  u_re = u[2] * dc37.re;
  u_im = u[2] * dc37.im;
  kC_l1_re = kC_l1 * dc38.re;
  kC_l1_im = kC_l1 * dc38.im;
  kC_l6_re = kC_l6 * dc40.re;
  kC_l6_im = kC_l6 * dc40.im;
  kC_l8_re = (kC_l8 + kC_r) * dc42.re;
  kC_l8_im = (kC_l8 + kC_r) * dc42.im;
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc35.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc35.im;
  kC_l4_re = ((((((-kC_l4 + x * dc36.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc39.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc41.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc36.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc39.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc41.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc43.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc43.im;
  dc34.re = dc34.re * (b_a * b_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc34.im = dc34.im * (b_a * b_a) + (beta * kC_l2_im + r * kC_l2_re);
  eml_scalar_sqrt(&dc34);
  dc35.re = kC_zeta * 2.0 * 0.0;
  dc35.im = kC_zeta * 2.0;
  b_exp(&dc35);
  dc36.re = kC_zeta * 0.0;
  dc36.im = kC_zeta;
  b_exp(&dc36);
  dc37.re = kC_zeta * 0.0;
  dc37.im = kC_zeta;
  b_exp(&dc37);
  dc38.re = kC_zeta * 0.0;
  dc38.im = kC_zeta;
  b_exp(&dc38);
  dc39.re = kC_zeta * 0.0;
  dc39.im = kC_zeta;
  b_exp(&dc39);
  dc40.re = kC_zeta * 0.0;
  dc40.im = kC_zeta;
  b_exp(&dc40);
  dc41.re = kC_zeta * 0.0;
  dc41.im = kC_zeta;
  b_exp(&dc41);
  dc42.re = kC_zeta * 0.0;
  dc42.im = kC_zeta;
  b_exp(&dc42);
  dc43.re = kC_zeta * 0.0;
  dc43.im = kC_zeta;
  b_exp(&dc43);
  dc44.re = kC_zeta * 0.0;
  dc44.im = kC_zeta;
  b_exp(&dc44);
  dc45.re = kC_zeta * 0.0;
  dc45.im = kC_zeta;
  b_exp(&dc45);
  dc46.re = kC_zeta * 0.0;
  dc46.im = kC_zeta;
  b_exp(&dc46);
  dc47.re = kC_zeta * 2.0 * 0.0;
  dc47.im = kC_zeta * 2.0;
  b_exp(&dc47);
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
  dc48.re = kC_zeta * 0.0;
  dc48.im = kC_zeta;
  b_exp(&dc48);
  dc49.re = kC_zeta * 0.0;
  dc49.im = kC_zeta;
  b_exp(&dc49);
  kC_l1_re = kC_l1 * kC_l1 * dc25.re;
  kC_l1_im = kC_l1 * kC_l1 * dc25.im;
  kC_l2_re = kC_l2 * kC_l2 * dc26.re;
  kC_l2_im = kC_l2 * kC_l2 * dc26.im;
  kC_l3_re = kC_l3 * kC_l3 * dc27.re;
  kC_l3_im = kC_l3 * kC_l3 * dc27.im;
  kC_l5_re = kC_l5 * kC_l5 * dc28.re;
  kC_l5_im = kC_l5 * kC_l5 * dc28.im;
  kC_l6_re = kC_l6 * kC_l6 * dc29.re;
  kC_l6_im = kC_l6 * kC_l6 * dc29.im;
  kC_l7_re = kC_l7 * kC_l7 * dc30.re;
  kC_l7_im = kC_l7 * kC_l7 * dc30.im;
  a_re = a * a * dc31.re;
  a_im = a * a * dc31.im;
  b_x_re = x * x * dc32.re;
  b_x_im = x * x * dc32.im;
  u_re = u[2] * u[2] * dc33.re;
  u_im = u[2] * u[2] * dc33.im;
  if (dc35.im == 0.0) {
    b_gamma = dc35.re / 2.0;
    r = 0.0;
  } else if (dc35.re == 0.0) {
    b_gamma = 0.0;
    r = dc35.im / 2.0;
  } else {
    b_gamma = dc35.re / 2.0;
    r = dc35.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
  kC_l4_im = 2.0 * (kC_l4 * x * r);
  b_kC_l1_re = 2.0 * (kC_l1 * kC_l6 * dc36.re);
  b_kC_l1_im = 2.0 * (kC_l1 * kC_l6 * dc36.im);
  c_kC_l1_re = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.re);
  c_kC_l1_im = 2.0 * (kC_l1 * (kC_l8 + kC_r) * dc37.im);
  b_kC_l2_re = 2.0 * (kC_l2 * kC_l7 * dc38.re);
  b_kC_l2_im = 2.0 * (kC_l2 * kC_l7 * dc38.im);
  b_kC_l6_re = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.re);
  b_kC_l6_im = 2.0 * (kC_l6 * (kC_l8 + kC_r) * dc39.im);
  c_kC_l2_re = 2.0 * (kC_l2 * x * dc40.re);
  c_kC_l2_im = 2.0 * (kC_l2 * x * dc40.im);
  b_kC_l7_re = 2.0 * (kC_l7 * x * dc41.re);
  b_kC_l7_im = 2.0 * (kC_l7 * x * dc41.im);
  d_kC_l1_re = 2.0 * (kC_l1 * u[2] * dc42.re);
  d_kC_l1_im = 2.0 * (kC_l1 * u[2] * dc42.im);
  c_kC_l6_re = 2.0 * (kC_l6 * u[2] * dc43.re);
  c_kC_l6_im = 2.0 * (kC_l6 * u[2] * dc43.im);
  kC_l8_re = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.re);
  kC_l8_im = 2.0 * ((kC_l8 + kC_r) * u[2] * dc44.im);
  b_kC_l4_re = b_x * b_x * (kC_l4 * kC_l4 * dc45.re);
  b_kC_l4_im = b_x * b_x * (kC_l4 * kC_l4 * dc45.im);
  c_kC_l4_re = c_x * c_x * (kC_l4 * kC_l4 * dc46.re);
  c_kC_l4_im = c_x * c_x * (kC_l4 * kC_l4 * dc46.im);
  if (dc47.im == 0.0) {
    b_gamma = dc47.re / 2.0;
    r = 0.0;
  } else if (dc47.re == 0.0) {
    b_gamma = 0.0;
    r = dc47.im / 2.0;
  } else {
    b_gamma = dc47.re / 2.0;
    r = dc47.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  beta = x * g_x.re;
  b_gamma = x * g_x.im;
  d_kC_l2_re = kC_l2 * j_x.re;
  d_kC_l2_im = kC_l2 * j_x.im;
  c_kC_l7_re = kC_l7 * dc48.re;
  c_kC_l7_im = kC_l7 * dc48.im;
  ar = -((((((((((((((((((((((((((((kC_l1_re * 0.0 - kC_l1_im) + (kC_l2_re * 0.0
    - kC_l2_im)) - (kC_l3_re * 0.0 - kC_l3_im)) + (kC_l5_re * 0.0 - kC_l5_im)) +
    (kC_l6_re * 0.0 - kC_l6_im)) + (kC_l7_re * 0.0 - kC_l7_im)) + (a_re * 0.0 -
    a_im)) + (b_x_re * 0.0 - b_x_im)) + (u_re * 0.0 - u_im)) - (dc34.re * 0.0 -
    dc34.im)) - (kC_l4_re * 0.0 - kC_l4_im)) - (b_kC_l1_re * 0.0 - b_kC_l1_im))
                        - (c_kC_l1_re * 0.0 - c_kC_l1_im)) - (b_kC_l2_re * 0.0 -
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
    * 0.0)) + (b_x_re + b_x_im * 0.0)) + (u_re + u_im * 0.0)) - (dc34.re +
    dc34.im * 0.0)) - (kC_l4_re + kC_l4_im * 0.0)) - (b_kC_l1_re + b_kC_l1_im *
    0.0)) - (c_kC_l1_re + c_kC_l1_im * 0.0)) - (b_kC_l2_re + b_kC_l2_im * 0.0))
                        + (b_kC_l6_re + b_kC_l6_im * 0.0)) - kC_l1 * kC_l4 *
                       muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * kC_l6 *
                      muDoubleScalarSin(2.0 * kC_zeta)) + kC_l4 * (kC_l8 + kC_r)
                     * muDoubleScalarSin(2.0 * kC_zeta)) - (c_kC_l2_re +
    c_kC_l2_im * 0.0)) + (b_kC_l7_re + b_kC_l7_im * 0.0)) - (d_kC_l1_re +
    d_kC_l1_im * 0.0)) + (c_kC_l6_re + c_kC_l6_im * 0.0)) + (kC_l8_re + kC_l8_im
    * 0.0)) + kC_l4 * u[2] * muDoubleScalarSin(2.0 * kC_zeta)) + (b_kC_l4_re +
    b_kC_l4_im * 0.0)) + (c_kC_l4_re + c_kC_l4_im * 0.0)) - (d_kC_l4_re + r *
             0.0)) + kC_l2 * kC_l4 * ((f_x.re + f_x.im * 0.0) + 1.0));
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (beta * 0.0 - b_gamma)) + u[2] *
    h_x.re) - kC_l1 * i_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    k_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * dc49.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (beta + b_gamma * 0.0)) + u[2] * h_x.im) -
    kC_l1 * i_x.im) + (d_kC_l2_re + d_kC_l2_im * 0.0)) + kC_l6 * k_x.im) -
                      (c_kC_l7_re + c_kC_l7_im * 0.0)) + (kC_l8 + kC_r) *
                     dc49.im);
  if (r == 0.0) {
    if (u_im == 0.0) {
      dc25.re = ar / u_re;
      dc25.im = 0.0;
    } else if (ar == 0.0) {
      dc25.re = 0.0;
      dc25.im = u_im / u_re;
    } else {
      dc25.re = ar / u_re;
      dc25.im = u_im / u_re;
    }
  } else if (u_re == 0.0) {
    if (ar == 0.0) {
      dc25.re = u_im / r;
      dc25.im = 0.0;
    } else if (u_im == 0.0) {
      dc25.re = 0.0;
      dc25.im = -(ar / r);
    } else {
      dc25.re = u_im / r;
      dc25.im = -(ar / r);
    }
  } else {
    d_kC_l4_re = muDoubleScalarAbs(u_re);
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc25.re = (ar + b_gamma * u_im) / beta;
      dc25.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc25.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc25.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc25.re = (b_gamma * ar + u_im) / beta;
      dc25.im = (b_gamma * u_im - ar) / beta;
    }
  }

  b_log(&dc25);
  b_gamma = dc25.re;
  dc25.re = dc25.re * 0.0 - dc25.im;
  dc25.im = b_gamma + dc25.im * 0.0;
  b_sin(&dc25);
  kC_l6_re = ((((kC_l6 - kC_l1) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
              muDoubleScalarSin(kC_zeta)) - kC_l5 * dc25.re;
  kC_l6_im = 0.0 - kC_l5 * dc25.im;
  if (kC_l6_im == 0.0) {
    dc25.re = kC_l6_re / kC_l3;
    dc25.im = 0.0;
  } else if (kC_l6_re == 0.0) {
    dc25.re = 0.0;
    dc25.im = kC_l6_im / kC_l3;
  } else {
    dc25.re = kC_l6_re / kC_l3;
    dc25.im = kC_l6_im / kC_l3;
  }

  b_asin(&dc25);
  ar = ((((kC_l6 - kC_l1) + (kC_l8 + kC_r)) + u[2]) + kC_l4 * muDoubleScalarSin
        (kC_zeta)) - kC_l5 * x_re;
  u_im = 0.0 - kC_l5 * x_im;
  if (u_im == 0.0) {
    dc26.re = ar / kC_l3;
    dc26.im = 0.0;
  } else if (ar == 0.0) {
    dc26.re = 0.0;
    dc26.im = u_im / kC_l3;
  } else {
    dc26.re = ar / kC_l3;
    dc26.im = u_im / kC_l3;
  }

  b_asin(&dc26);
  r = muDoubleScalarAtan2(u[1], u[0]);

  /* beta = betaRaw(1); */
  /* gamma = gammaRaw(1); */
  if ((r >= jointLimits[0]) && (r <= jointLimits[1]) && (-dc25.re >=
       jointLimits[2]) && (-dc25.re <= jointLimits[3]) && (gammaRaw[0].re >=
       jointLimits[4]) && (gammaRaw[0].re <= jointLimits[5])) {
    beta = -dc25.re;
    b_gamma = gammaRaw[0].re;
  } else if ((r >= jointLimits[0]) && (r <= jointLimits[1]) && (-dc26.re >=
              jointLimits[2]) && (-dc26.re <= jointLimits[3]) && (gammaRaw[1].re
              >= jointLimits[4]) && (gammaRaw[1].re <= jointLimits[5])) {
    beta = -dc26.re;
    b_gamma = gammaRaw[1].re;
  } else {
    st.site = &f_emlrtRSI;
    b_fprintf(&st);
    beta = -dc25.re;
    b_gamma = gammaRaw[0].re;
  }

  q[0] = r;
  q[1] = beta;
  q[2] = b_gamma;
}

void sherpaTTIK(const emlrtStack *sp, const real_T u[3], real_T kC_l1, real_T
                kC_l2, real_T kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l6,
                real_T kC_l7, real_T kC_l8, real_T kC_zeta, real_T kC_r, const
                real_T jointLimits[20], real_T q[3])
{
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
  real_T b_gamma;
  real_T r;
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
  creal_T y;
  creal_T gammaRaw[2];
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;

  /* SHERPATTIK Calculates the joint values for a g1iven contact point. */
  /*    Calculates the joint values for a g1iven contact point for the Sherpa TT */
  /*    leg. All coord1inates are in the pan joint coord1inate frame. */
  /*  */
  /* Inputs: */
  /* -u: A 1x3 Cartesian vector in the pan frame containing [xP yP zP]. */
  /* -kC: A struct containing the kinematic constants of the Sherpa TT Rover. */
  /* -jointLimits: The joint limits of each of the rover's joints. */
  /* Outputs: */
  /* -q: A 1x3 joint vector containing [alpha beta gamma]. */
  /* sherpaTTIK.m */
  /* author: wreid */
  /* date: 20150122 */
  st.site = &e_emlrtRSI;
  x = u[0] * u[0] + u[1] * u[1];
  if (x < 0.0) {
    b_st.site = &g_emlrtRSI;
    eml_error(&b_st);
  }

  x = muDoubleScalarSqrt(x);
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
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
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
    b_gamma = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    b_gamma = 0.0;
    r = dc10.im / 2.0;
  } else {
    b_gamma = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
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
    b_gamma = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    b_gamma = 0.0;
    r = dc22.im / 2.0;
  } else {
    b_gamma = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
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
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc0.re = (ar + b_gamma * u_im) / beta;
      dc0.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc0.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc0.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc0.re = (b_gamma * ar + u_im) / beta;
      dc0.im = (b_gamma * u_im - ar) / beta;
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
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc11.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc11.im;
  kC_l4_re = ((((((-kC_l4 + x * dc12.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc15.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc17.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc12.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc15.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc17.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
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
    b_gamma = dc11.re / 2.0;
    r = 0.0;
  } else if (dc11.re == 0.0) {
    b_gamma = 0.0;
    r = dc11.im / 2.0;
  } else {
    b_gamma = dc11.re / 2.0;
    r = dc11.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
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
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc1.re = (ar + b_gamma * u_im) / beta;
      dc1.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc1.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc1.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc1.re = (b_gamma * ar + u_im) / beta;
      dc1.im = (b_gamma * u_im - ar) / beta;
    }
  }

  b_log(&dc1);
  gammaRaw[0].re = -kC_zeta - (dc0.re * 0.0 - dc0.im);
  gammaRaw[1].re = -kC_zeta - (dc1.re * 0.0 - dc1.im);
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
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
  kC_l2_re = ((((((kC_l2 - kC_l1 * 0.0) + kC_l6 * 0.0) - kC_l7) + (kC_l8 + kC_r)
                * 0.0) - x) + u[2] * 0.0) + kC_l4 * dc18.re;
  kC_l2_im = ((((0.0 - kC_l1) + kC_l6) + (kC_l8 + kC_r)) + u[2]) + kC_l4 *
    dc18.im;
  dc9.re = dc9.re * (d_a * d_a) + (beta * kC_l2_re - r * kC_l2_im);
  dc9.im = dc9.im * (d_a * d_a) + (beta * kC_l2_im + r * kC_l2_re);
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
    b_gamma = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    b_gamma = 0.0;
    r = dc10.im / 2.0;
  } else {
    b_gamma = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
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
    b_gamma = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    b_gamma = 0.0;
    r = dc22.im / 2.0;
  } else {
    b_gamma = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  beta = x * f_x.re;
  b_gamma = x * f_x.im;
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
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (beta * 0.0 - b_gamma)) + u[2] *
    g_x.re) - kC_l1 * h_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    j_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * y.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (beta + b_gamma * 0.0)) + u[2] * g_x.im) -
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
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      y.re = (ar + b_gamma * u_im) / beta;
      y.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      y.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      y.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      y.re = (b_gamma * ar + u_im) / beta;
      y.im = (b_gamma * u_im - ar) / beta;
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
  b_gamma = 4.0 * (kC_l5 * kC_l5) * dc10.re;
  r = 4.0 * (kC_l5 * kC_l5) * dc10.im;
  kC_l4_re = ((((((-kC_l4 + x * dc11.re) + (u_re * 0.0 - u_im)) - (kC_l1_re *
    0.0 - kC_l1_im)) - kC_l2 * dc14.re) + (kC_l6_re * 0.0 - kC_l6_im)) + kC_l7 *
              dc16.re) + (kC_l8_re * 0.0 - kC_l8_im);
  kC_l4_im = (((((x * dc11.im + (u_re + u_im * 0.0)) - (kC_l1_re + kC_l1_im *
    0.0)) - kC_l2 * dc14.im) + (kC_l6_re + kC_l6_im * 0.0)) + kC_l7 * dc16.im) +
    (kC_l8_re + kC_l8_im * 0.0);
  beta = b_gamma * kC_l4_re - r * kC_l4_im;
  r = b_gamma * kC_l4_im + r * kC_l4_re;
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
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  u_re = u[2] * u[2] * dc8.re;
  u_im = u[2] * u[2] * dc8.im;
  if (dc10.im == 0.0) {
    b_gamma = dc10.re / 2.0;
    r = 0.0;
  } else if (dc10.re == 0.0) {
    b_gamma = 0.0;
    r = dc10.im / 2.0;
  } else {
    b_gamma = dc10.re / 2.0;
    r = dc10.im / 2.0;
  }

  kC_l4_re = 2.0 * (kC_l4 * x * (b_gamma + 0.5));
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
    b_gamma = dc22.re / 2.0;
    r = 0.0;
  } else if (dc22.re == 0.0) {
    b_gamma = 0.0;
    r = dc22.im / 2.0;
  } else {
    b_gamma = dc22.re / 2.0;
    r = dc22.im / 2.0;
  }

  d_kC_l4_re = 2.0 * (kC_l4 * kC_l7 * (b_gamma + 0.5));
  r = 2.0 * (kC_l4 * kC_l7 * r);
  beta = x * g_x.re;
  b_gamma = x * g_x.im;
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
  u_re = 2.0 * kC_l5 * (((((((kC_l4 * 0.0 - (beta * 0.0 - b_gamma)) + u[2] *
    h_x.re) - kC_l1 * i_x.re) + (d_kC_l2_re * 0.0 - d_kC_l2_im)) + kC_l6 *
    k_x.re) - (c_kC_l7_re * 0.0 - c_kC_l7_im)) + (kC_l8 + kC_r) * dc24.re);
  r = 2.0 * kC_l5 * (((((((kC_l4 - (beta + b_gamma * 0.0)) + u[2] * h_x.im) -
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
    beta = muDoubleScalarAbs(r);
    if (d_kC_l4_re > beta) {
      b_gamma = r / u_re;
      beta = u_re + b_gamma * r;
      dc0.re = (ar + b_gamma * u_im) / beta;
      dc0.im = (u_im - b_gamma * ar) / beta;
    } else if (beta == d_kC_l4_re) {
      if (u_re > 0.0) {
        b_gamma = 0.5;
      } else {
        b_gamma = -0.5;
      }

      if (r > 0.0) {
        beta = 0.5;
      } else {
        beta = -0.5;
      }

      dc0.re = (ar * b_gamma + u_im * beta) / d_kC_l4_re;
      dc0.im = (u_im * b_gamma - ar * beta) / d_kC_l4_re;
    } else {
      b_gamma = u_re / r;
      beta = r + b_gamma * u_re;
      dc0.re = (b_gamma * ar + u_im) / beta;
      dc0.im = (b_gamma * u_im - ar) / beta;
    }
  }

  b_log(&dc0);
  b_gamma = dc0.re;
  dc0.re = dc0.re * 0.0 - dc0.im;
  dc0.im = b_gamma + dc0.im * 0.0;
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
  r = muDoubleScalarAtan2(u[1], u[0]);

  /* beta = betaRaw(1); */
  /* gamma = gammaRaw(1); */
  if ((r >= jointLimits[0]) && (r <= jointLimits[1]) && (-dc0.re >= jointLimits
       [2]) && (-dc0.re <= jointLimits[3]) && (gammaRaw[0].re >= jointLimits[4])
      && (gammaRaw[0].re <= jointLimits[5])) {
    beta = -dc0.re;
    b_gamma = gammaRaw[0].re;
  } else if ((r >= jointLimits[0]) && (r <= jointLimits[1]) && (-dc1.re >=
              jointLimits[2]) && (-dc1.re <= jointLimits[3]) && (gammaRaw[1].re >=
              jointLimits[4]) && (gammaRaw[1].re <= jointLimits[5])) {
    beta = -dc1.re;
    b_gamma = gammaRaw[1].re;
  } else {
    st.site = &f_emlrtRSI;
    b_fprintf(&st);
    beta = -dc0.re;
    b_gamma = gammaRaw[0].re;
  }

  q[0] = r;
  q[1] = beta;
  q[2] = b_gamma;
}

/* End of code generation (sherpaTTIK.c) */
