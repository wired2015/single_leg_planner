//
// File: sherpaTTIK.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 09-Feb-2015 13:36:11
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "sherpaTTIK.h"
#include "asin.h"
#include "sin.h"
#include "log.h"
#include "exp.h"
#include "buildRRTWrapper_rtwutil.h"
#include <stdio.h>

// Function Definitions

//
// sherpaTTIK Calculates the joint values for a g1iven contact point.
//    Calculates the joint values for a g1iven contact point for the Sherpa TT
//    leg. All coord1inates are in the pan joint coord1inate frame.
// Arguments    : double xC
//                double yC
//                double zC
//                const double kinematicConst[15]
//                const double jointLimits[12]
//                double *alpha
//                double *beta
//                double *b_gamma
// Return Type  : void
//
void sherpaTTIK(double xC, double yC, double zC, const double kinematicConst[15],
                const double jointLimits[12], double *alpha, double *beta,
                double *b_gamma)
{
  double x;
  double a;
  double b_x;
  double c_x;
  double b_a;
  double d_x;
  double e_x;
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
  double zC_re;
  double zC_im;
  double kinematicConst_re;
  double kinematicConst_im;
  double b_kinematicConst_re;
  double b_kinematicConst_im;
  double c_kinematicConst_re;
  double c_kinematicConst_im;
  double re;
  double im;
  double d_kinematicConst_re;
  double b_re;
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
  double d_kinematicConst_im;
  double e_kinematicConst_re;
  double e_kinematicConst_im;
  double f_kinematicConst_re;
  double f_kinematicConst_im;
  double g_kinematicConst_re;
  double g_kinematicConst_im;
  double x_re;
  double x_im;
  double h_kinematicConst_re;
  double h_kinematicConst_im;
  double i_kinematicConst_re;
  double i_kinematicConst_im;
  double j_kinematicConst_re;
  double j_kinematicConst_im;
  double k_kinematicConst_re;
  double k_kinematicConst_im;
  double l_kinematicConst_re;
  double l_kinematicConst_im;
  double m_kinematicConst_re;
  double m_kinematicConst_im;
  double n_kinematicConst_re;
  double n_kinematicConst_im;
  double o_kinematicConst_re;
  double o_kinematicConst_im;
  double p_kinematicConst_re;
  double p_kinematicConst_im;
  double q_kinematicConst_re;
  double q_kinematicConst_im;
  double r_kinematicConst_re;
  double r_kinematicConst_im;
  double s_kinematicConst_re;
  double s_kinematicConst_im;
  double t_kinematicConst_re;
  double t_kinematicConst_im;
  double b_x_re;
  double b_x_im;
  double u_kinematicConst_re;
  double u_kinematicConst_im;
  double ar;
  double brm;
  double r;
  creal_T y;
  creal_T gammaRaw[2];

  // sherpaTTIK.m
  // author: wreid
  // date: 20150122
  x = sqrt(xC * xC + yC * yC);
  a = ((((((((((((((((((((((((kinematicConst[0] * kinematicConst[0] - 2.0 * sin
    (kinematicConst[8]) * kinematicConst[0] * kinematicConst[3]) - 2.0 *
    kinematicConst[0] * kinematicConst[5]) - 2.0 * kinematicConst[0] *
    kinematicConst[7]) - 2.0 * kinematicConst[0] * zC) + kinematicConst[1] *
    kinematicConst[1]) + 2.0 * cos(kinematicConst[8]) * kinematicConst[1] *
    kinematicConst[3]) - 2.0 * kinematicConst[1] * kinematicConst[6]) - 2.0 *
                       kinematicConst[1] * x) - kinematicConst[2] *
                      kinematicConst[2]) + kinematicConst[3] * kinematicConst[3])
                    + 2.0 * sin(kinematicConst[8]) * kinematicConst[3] *
                    kinematicConst[5]) - 2.0 * cos(kinematicConst[8]) *
                   kinematicConst[3] * kinematicConst[6]) + 2.0 * sin
                  (kinematicConst[8]) * kinematicConst[3] * kinematicConst[7]) -
                 2.0 * cos(kinematicConst[8]) * kinematicConst[3] * x) + 2.0 *
                sin(kinematicConst[8]) * kinematicConst[3] * zC) +
               kinematicConst[4] * kinematicConst[4]) + kinematicConst[5] *
              kinematicConst[5]) + 2.0 * kinematicConst[5] * kinematicConst[7])
            + 2.0 * kinematicConst[5] * zC) + kinematicConst[6] *
           kinematicConst[6]) + 2.0 * kinematicConst[6] * x) + kinematicConst[7]
         * kinematicConst[7]) + 2.0 * kinematicConst[7] * zC) + x * x) + zC * zC;
  b_x = cos(kinematicConst[8]);
  c_x = sin(kinematicConst[8]);
  b_a = ((((((((((((((((((((((((kinematicConst[0] * kinematicConst[0] - 2.0 *
    sin(kinematicConst[8]) * kinematicConst[0] * kinematicConst[3]) - 2.0 *
    kinematicConst[0] * kinematicConst[5]) - 2.0 * kinematicConst[0] *
    kinematicConst[7]) - 2.0 * kinematicConst[0] * zC) + kinematicConst[1] *
    kinematicConst[1]) + 2.0 * cos(kinematicConst[8]) * kinematicConst[1] *
    kinematicConst[3]) - 2.0 * kinematicConst[1] * kinematicConst[6]) - 2.0 *
    kinematicConst[1] * x) - kinematicConst[2] * kinematicConst[2]) +
                       kinematicConst[3] * kinematicConst[3]) + 2.0 * sin
                      (kinematicConst[8]) * kinematicConst[3] * kinematicConst[5])
                     - 2.0 * cos(kinematicConst[8]) * kinematicConst[3] *
                     kinematicConst[6]) + 2.0 * sin(kinematicConst[8]) *
                    kinematicConst[3] * kinematicConst[7]) - 2.0 * cos
                   (kinematicConst[8]) * kinematicConst[3] * x) + 2.0 * sin
                  (kinematicConst[8]) * kinematicConst[3] * zC) +
                 kinematicConst[4] * kinematicConst[4]) + kinematicConst[5] *
                kinematicConst[5]) + 2.0 * kinematicConst[5] * kinematicConst[7])
              + 2.0 * kinematicConst[5] * zC) + kinematicConst[6] *
             kinematicConst[6]) + 2.0 * kinematicConst[6] * x) + kinematicConst
           [7] * kinematicConst[7]) + 2.0 * kinematicConst[7] * zC) + x * x) +
    zC * zC;
  d_x = cos(kinematicConst[8]);
  e_x = sin(kinematicConst[8]);
  dc0.re = kinematicConst[8] * 0.0;
  dc0.im = kinematicConst[8];
  b_exp(&dc0);
  dc1.re = kinematicConst[8] * 0.0;
  dc1.im = kinematicConst[8];
  b_exp(&dc1);
  dc2.re = kinematicConst[8] * 0.0;
  dc2.im = kinematicConst[8];
  b_exp(&dc2);
  dc3.re = kinematicConst[8] * 0.0;
  dc3.im = kinematicConst[8];
  b_exp(&dc3);
  dc4.re = kinematicConst[8] * 0.0;
  dc4.im = kinematicConst[8];
  b_exp(&dc4);
  dc5.re = kinematicConst[8] * 0.0;
  dc5.im = kinematicConst[8];
  b_exp(&dc5);
  dc6.re = kinematicConst[8] * 0.0;
  dc6.im = kinematicConst[8];
  b_exp(&dc6);
  dc7.re = kinematicConst[8] * 0.0;
  dc7.im = kinematicConst[8];
  b_exp(&dc7);
  dc8.re = kinematicConst[8] * 0.0;
  dc8.im = kinematicConst[8];
  b_exp(&dc8);
  dc9.re = kinematicConst[8] * 2.0 * 0.0;
  dc9.im = kinematicConst[8] * 2.0;
  b_exp(&dc9);
  dc10.re = kinematicConst[8] * 0.0;
  dc10.im = kinematicConst[8];
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  zC_re = zC * dc12.re;
  zC_im = zC * dc12.im;
  kinematicConst_re = kinematicConst[0] * dc13.re;
  kinematicConst_im = kinematicConst[0] * dc13.im;
  b_kinematicConst_re = kinematicConst[5] * dc15.re;
  b_kinematicConst_im = kinematicConst[5] * dc15.im;
  c_kinematicConst_re = kinematicConst[7] * dc17.re;
  c_kinematicConst_im = kinematicConst[7] * dc17.im;
  re = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.re;
  im = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.im;
  d_kinematicConst_re = ((((((-kinematicConst[3] + x * dc11.re) + (zC_re * 0.0 -
    zC_im)) - (kinematicConst_re * 0.0 - kinematicConst_im)) - kinematicConst[1]
    * dc14.re) + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) +
    kinematicConst[6] * dc16.re) + (c_kinematicConst_re * 0.0 -
    c_kinematicConst_im);
  kinematicConst_im = (((((x * dc11.im + (zC_re + zC_im * 0.0)) -
    (kinematicConst_re + kinematicConst_im * 0.0)) - kinematicConst[1] * dc14.im)
                        + (b_kinematicConst_re + b_kinematicConst_im * 0.0)) +
                       kinematicConst[6] * dc16.im) + (c_kinematicConst_re +
    c_kinematicConst_im * 0.0);
  b_re = re * d_kinematicConst_re - im * kinematicConst_im;
  im = re * kinematicConst_im + im * d_kinematicConst_re;
  kinematicConst_re = ((((((kinematicConst[1] - kinematicConst[0] * 0.0) +
    kinematicConst[5] * 0.0) - kinematicConst[6]) + kinematicConst[7] * 0.0) - x)
                       + zC * 0.0) + kinematicConst[3] * dc18.re;
  kinematicConst_im = ((((0.0 - kinematicConst[0]) + kinematicConst[5]) +
                        kinematicConst[7]) + zC) + kinematicConst[3] * dc18.im;
  dc9.re = dc9.re * (a * a) + (b_re * kinematicConst_re - im * kinematicConst_im);
  dc9.im = dc9.im * (a * a) + (b_re * kinematicConst_im + im * kinematicConst_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kinematicConst[8] * 2.0 * 0.0;
  dc10.im = kinematicConst[8] * 2.0;
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  dc19.re = kinematicConst[8] * 0.0;
  dc19.im = kinematicConst[8];
  b_exp(&dc19);
  dc20.re = kinematicConst[8] * 0.0;
  dc20.im = kinematicConst[8];
  b_exp(&dc20);
  dc21.re = kinematicConst[8] * 0.0;
  dc21.im = kinematicConst[8];
  b_exp(&dc21);
  dc22.re = kinematicConst[8] * 2.0 * 0.0;
  dc22.im = kinematicConst[8] * 2.0;
  b_exp(&dc22);
  f_x.re = kinematicConst[8] * 2.0 * 0.0;
  f_x.im = kinematicConst[8] * 2.0;
  b_exp(&f_x);
  g_x.re = kinematicConst[8] * 0.0;
  g_x.im = kinematicConst[8];
  b_exp(&g_x);
  h_x.re = kinematicConst[8] * 0.0;
  h_x.im = kinematicConst[8];
  b_exp(&h_x);
  i_x.re = kinematicConst[8] * 0.0;
  i_x.im = kinematicConst[8];
  b_exp(&i_x);
  j_x.re = kinematicConst[8] * 0.0;
  j_x.im = kinematicConst[8];
  b_exp(&j_x);
  k_x.re = kinematicConst[8] * 0.0;
  k_x.im = kinematicConst[8];
  b_exp(&k_x);
  dc23.re = kinematicConst[8] * 0.0;
  dc23.im = kinematicConst[8];
  b_exp(&dc23);
  dc24.re = kinematicConst[8] * 0.0;
  dc24.im = kinematicConst[8];
  b_exp(&dc24);
  kinematicConst_re = kinematicConst[0] * kinematicConst[0] * dc0.re;
  kinematicConst_im = kinematicConst[0] * kinematicConst[0] * dc0.im;
  b_kinematicConst_re = kinematicConst[1] * kinematicConst[1] * dc1.re;
  b_kinematicConst_im = kinematicConst[1] * kinematicConst[1] * dc1.im;
  c_kinematicConst_re = kinematicConst[2] * kinematicConst[2] * dc2.re;
  c_kinematicConst_im = kinematicConst[2] * kinematicConst[2] * dc2.im;
  d_kinematicConst_re = kinematicConst[4] * kinematicConst[4] * dc3.re;
  d_kinematicConst_im = kinematicConst[4] * kinematicConst[4] * dc3.im;
  e_kinematicConst_re = kinematicConst[5] * kinematicConst[5] * dc4.re;
  e_kinematicConst_im = kinematicConst[5] * kinematicConst[5] * dc4.im;
  f_kinematicConst_re = kinematicConst[6] * kinematicConst[6] * dc5.re;
  f_kinematicConst_im = kinematicConst[6] * kinematicConst[6] * dc5.im;
  g_kinematicConst_re = kinematicConst[7] * kinematicConst[7] * dc6.re;
  g_kinematicConst_im = kinematicConst[7] * kinematicConst[7] * dc6.im;
  x_re = x * x * dc7.re;
  x_im = x * x * dc7.im;
  zC_re = zC * zC * dc8.re;
  zC_im = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    re = dc10.re / 2.0;
    im = 0.0;
  } else if (dc10.re == 0.0) {
    re = 0.0;
    im = dc10.im / 2.0;
  } else {
    re = dc10.re / 2.0;
    im = dc10.im / 2.0;
  }

  h_kinematicConst_re = 2.0 * (kinematicConst[3] * x * (re + 0.5));
  h_kinematicConst_im = 2.0 * (kinematicConst[3] * x * im);
  i_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.re);
  i_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.im);
  j_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.re);
  j_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.im);
  k_kinematicConst_re = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.re);
  k_kinematicConst_im = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.im);
  l_kinematicConst_re = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.re);
  l_kinematicConst_im = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.im);
  m_kinematicConst_re = 2.0 * (kinematicConst[1] * x * dc15.re);
  m_kinematicConst_im = 2.0 * (kinematicConst[1] * x * dc15.im);
  n_kinematicConst_re = 2.0 * (kinematicConst[6] * x * dc16.re);
  n_kinematicConst_im = 2.0 * (kinematicConst[6] * x * dc16.im);
  o_kinematicConst_re = 2.0 * (kinematicConst[0] * zC * dc17.re);
  o_kinematicConst_im = 2.0 * (kinematicConst[0] * zC * dc17.im);
  p_kinematicConst_re = 2.0 * (kinematicConst[5] * zC * dc18.re);
  p_kinematicConst_im = 2.0 * (kinematicConst[5] * zC * dc18.im);
  q_kinematicConst_re = 2.0 * (kinematicConst[7] * zC * dc19.re);
  q_kinematicConst_im = 2.0 * (kinematicConst[7] * zC * dc19.im);
  r_kinematicConst_re = b_x * b_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.re);
  r_kinematicConst_im = b_x * b_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.im);
  s_kinematicConst_re = c_x * c_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.re);
  s_kinematicConst_im = c_x * c_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.im);
  if (dc22.im == 0.0) {
    re = dc22.re / 2.0;
    im = 0.0;
  } else if (dc22.re == 0.0) {
    re = 0.0;
    im = dc22.im / 2.0;
  } else {
    re = dc22.re / 2.0;
    im = dc22.im / 2.0;
  }

  t_kinematicConst_re = 2.0 * (kinematicConst[3] * kinematicConst[6] * (re + 0.5));
  t_kinematicConst_im = 2.0 * (kinematicConst[3] * kinematicConst[6] * im);
  b_x_re = x * g_x.re;
  b_x_im = x * g_x.im;
  b_re = kinematicConst[1] * j_x.re;
  im = kinematicConst[1] * j_x.im;
  u_kinematicConst_re = kinematicConst[6] * dc23.re;
  u_kinematicConst_im = kinematicConst[6] * dc23.im;
  ar = -((((((((((((((((((((((((((((kinematicConst_re * 0.0 - kinematicConst_im)
    + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) - (c_kinematicConst_re *
    0.0 - c_kinematicConst_im)) + (d_kinematicConst_re * 0.0 -
    d_kinematicConst_im)) + (e_kinematicConst_re * 0.0 - e_kinematicConst_im)) +
    (f_kinematicConst_re * 0.0 - f_kinematicConst_im)) + (g_kinematicConst_re *
    0.0 - g_kinematicConst_im)) + (x_re * 0.0 - x_im)) + (zC_re * 0.0 - zC_im))
    - (dc9.re * 0.0 - dc9.im)) - (h_kinematicConst_re * 0.0 -
    h_kinematicConst_im)) - (i_kinematicConst_re * 0.0 - i_kinematicConst_im)) -
                        (j_kinematicConst_re * 0.0 - j_kinematicConst_im)) -
                       (k_kinematicConst_re * 0.0 - k_kinematicConst_im)) +
                      (l_kinematicConst_re * 0.0 - l_kinematicConst_im)) -
                     kinematicConst[0] * kinematicConst[3] * (cos(2.0 *
    kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[5] * (cos
    (2.0 * kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[7] *
                   (cos(2.0 * kinematicConst[8]) - 1.0)) - (m_kinematicConst_re *
    0.0 - m_kinematicConst_im)) + (n_kinematicConst_re * 0.0 -
    n_kinematicConst_im)) - (o_kinematicConst_re * 0.0 - o_kinematicConst_im)) +
               (p_kinematicConst_re * 0.0 - p_kinematicConst_im)) +
              (q_kinematicConst_re * 0.0 - q_kinematicConst_im)) +
             kinematicConst[3] * zC * (cos(2.0 * kinematicConst[8]) - 1.0)) +
            (r_kinematicConst_re * 0.0 - r_kinematicConst_im)) +
           (s_kinematicConst_re * 0.0 - s_kinematicConst_im)) -
          (t_kinematicConst_re * 0.0 - t_kinematicConst_im)) + kinematicConst[1]
         * kinematicConst[3] * (f_x.re * 0.0 - f_x.im));
  t_kinematicConst_re = -((((((((((((((((((((((((((((kinematicConst_re +
    kinematicConst_im * 0.0) + (b_kinematicConst_re + b_kinematicConst_im * 0.0))
    - (c_kinematicConst_re + c_kinematicConst_im * 0.0)) + (d_kinematicConst_re
    + d_kinematicConst_im * 0.0)) + (e_kinematicConst_re + e_kinematicConst_im *
    0.0)) + (f_kinematicConst_re + f_kinematicConst_im * 0.0)) +
    (g_kinematicConst_re + g_kinematicConst_im * 0.0)) + (x_re + x_im * 0.0)) +
    (zC_re + zC_im * 0.0)) - (dc9.re + dc9.im * 0.0)) - (h_kinematicConst_re +
    h_kinematicConst_im * 0.0)) - (i_kinematicConst_re + i_kinematicConst_im *
    0.0)) - (j_kinematicConst_re + j_kinematicConst_im * 0.0)) -
    (k_kinematicConst_re + k_kinematicConst_im * 0.0)) + (l_kinematicConst_re +
    l_kinematicConst_im * 0.0)) - kinematicConst[0] * kinematicConst[3] * sin
    (2.0 * kinematicConst[8])) + kinematicConst[3] * kinematicConst[5] * sin(2.0
    * kinematicConst[8])) + kinematicConst[3] * kinematicConst[7] * sin(2.0 *
    kinematicConst[8])) - (m_kinematicConst_re + m_kinematicConst_im * 0.0)) +
    (n_kinematicConst_re + n_kinematicConst_im * 0.0)) - (o_kinematicConst_re +
    o_kinematicConst_im * 0.0)) + (p_kinematicConst_re + p_kinematicConst_im *
    0.0)) + (q_kinematicConst_re + q_kinematicConst_im * 0.0)) + kinematicConst
    [3] * zC * sin(2.0 * kinematicConst[8])) + (r_kinematicConst_re +
    r_kinematicConst_im * 0.0)) + (s_kinematicConst_re + s_kinematicConst_im *
    0.0)) - (t_kinematicConst_re + t_kinematicConst_im * 0.0)) + kinematicConst
    [1] * kinematicConst[3] * ((f_x.re + f_x.im * 0.0) + 1.0));
  t_kinematicConst_im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] * 0.0
    - (b_x_re * 0.0 - b_x_im)) + zC * h_x.re) - kinematicConst[0] * i_x.re) +
    (b_re * 0.0 - im)) + kinematicConst[5] * k_x.re) - (u_kinematicConst_re *
    0.0 - u_kinematicConst_im)) + kinematicConst[7] * dc24.re);
  im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] - (b_x_re + b_x_im *
    0.0)) + zC * h_x.im) - kinematicConst[0] * i_x.im) + (b_re + im * 0.0)) +
    kinematicConst[5] * k_x.im) - (u_kinematicConst_re + u_kinematicConst_im *
    0.0)) + kinematicConst[7] * dc24.im);
  if (im == 0.0) {
    if (t_kinematicConst_re == 0.0) {
      dc0.re = ar / t_kinematicConst_im;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = t_kinematicConst_re / t_kinematicConst_im;
    } else {
      dc0.re = ar / t_kinematicConst_im;
      dc0.im = t_kinematicConst_re / t_kinematicConst_im;
    }
  } else if (t_kinematicConst_im == 0.0) {
    if (ar == 0.0) {
      dc0.re = t_kinematicConst_re / im;
      dc0.im = 0.0;
    } else if (t_kinematicConst_re == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / im);
    } else {
      dc0.re = t_kinematicConst_re / im;
      dc0.im = -(ar / im);
    }
  } else {
    brm = fabs(t_kinematicConst_im);
    r = fabs(im);
    if (brm > r) {
      b_re = im / t_kinematicConst_im;
      r = t_kinematicConst_im + b_re * im;
      dc0.re = (ar + b_re * t_kinematicConst_re) / r;
      dc0.im = (t_kinematicConst_re - b_re * ar) / r;
    } else if (r == brm) {
      if (t_kinematicConst_im > 0.0) {
        b_re = 0.5;
      } else {
        b_re = -0.5;
      }

      if (im > 0.0) {
        r = 0.5;
      } else {
        r = -0.5;
      }

      dc0.re = (ar * b_re + t_kinematicConst_re * r) / brm;
      dc0.im = (t_kinematicConst_re * b_re - ar * r) / brm;
    } else {
      b_re = t_kinematicConst_im / im;
      r = im + b_re * t_kinematicConst_im;
      dc0.re = (b_re * ar + t_kinematicConst_re) / r;
      dc0.im = (b_re * t_kinematicConst_re - ar) / r;
    }
  }

  b_log(&dc0);
  dc1.re = kinematicConst[8] * 0.0;
  dc1.im = kinematicConst[8];
  b_exp(&dc1);
  dc2.re = kinematicConst[8] * 0.0;
  dc2.im = kinematicConst[8];
  b_exp(&dc2);
  dc3.re = kinematicConst[8] * 0.0;
  dc3.im = kinematicConst[8];
  b_exp(&dc3);
  dc4.re = kinematicConst[8] * 0.0;
  dc4.im = kinematicConst[8];
  b_exp(&dc4);
  dc5.re = kinematicConst[8] * 0.0;
  dc5.im = kinematicConst[8];
  b_exp(&dc5);
  dc6.re = kinematicConst[8] * 0.0;
  dc6.im = kinematicConst[8];
  b_exp(&dc6);
  dc7.re = kinematicConst[8] * 0.0;
  dc7.im = kinematicConst[8];
  b_exp(&dc7);
  dc8.re = kinematicConst[8] * 0.0;
  dc8.im = kinematicConst[8];
  b_exp(&dc8);
  dc9.re = kinematicConst[8] * 0.0;
  dc9.im = kinematicConst[8];
  b_exp(&dc9);
  dc10.re = kinematicConst[8] * 2.0 * 0.0;
  dc10.im = kinematicConst[8] * 2.0;
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  dc19.re = kinematicConst[8] * 0.0;
  dc19.im = kinematicConst[8];
  b_exp(&dc19);
  zC_re = zC * dc13.re;
  zC_im = zC * dc13.im;
  kinematicConst_re = kinematicConst[0] * dc14.re;
  kinematicConst_im = kinematicConst[0] * dc14.im;
  b_kinematicConst_re = kinematicConst[5] * dc16.re;
  b_kinematicConst_im = kinematicConst[5] * dc16.im;
  c_kinematicConst_re = kinematicConst[7] * dc18.re;
  c_kinematicConst_im = kinematicConst[7] * dc18.im;
  re = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc11.re;
  im = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc11.im;
  d_kinematicConst_re = ((((((-kinematicConst[3] + x * dc12.re) + (zC_re * 0.0 -
    zC_im)) - (kinematicConst_re * 0.0 - kinematicConst_im)) - kinematicConst[1]
    * dc15.re) + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) +
    kinematicConst[6] * dc17.re) + (c_kinematicConst_re * 0.0 -
    c_kinematicConst_im);
  kinematicConst_im = (((((x * dc12.im + (zC_re + zC_im * 0.0)) -
    (kinematicConst_re + kinematicConst_im * 0.0)) - kinematicConst[1] * dc15.im)
                        + (b_kinematicConst_re + b_kinematicConst_im * 0.0)) +
                       kinematicConst[6] * dc17.im) + (c_kinematicConst_re +
    c_kinematicConst_im * 0.0);
  b_re = re * d_kinematicConst_re - im * kinematicConst_im;
  im = re * kinematicConst_im + im * d_kinematicConst_re;
  kinematicConst_re = ((((((kinematicConst[1] - kinematicConst[0] * 0.0) +
    kinematicConst[5] * 0.0) - kinematicConst[6]) + kinematicConst[7] * 0.0) - x)
                       + zC * 0.0) + kinematicConst[3] * dc19.re;
  kinematicConst_im = ((((0.0 - kinematicConst[0]) + kinematicConst[5]) +
                        kinematicConst[7]) + zC) + kinematicConst[3] * dc19.im;
  dc10.re = dc10.re * (b_a * b_a) + (b_re * kinematicConst_re - im *
    kinematicConst_im);
  dc10.im = dc10.im * (b_a * b_a) + (b_re * kinematicConst_im + im *
    kinematicConst_re);
  eml_scalar_sqrt(&dc10);
  dc11.re = kinematicConst[8] * 2.0 * 0.0;
  dc11.im = kinematicConst[8] * 2.0;
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  dc19.re = kinematicConst[8] * 0.0;
  dc19.im = kinematicConst[8];
  b_exp(&dc19);
  dc20.re = kinematicConst[8] * 0.0;
  dc20.im = kinematicConst[8];
  b_exp(&dc20);
  dc21.re = kinematicConst[8] * 0.0;
  dc21.im = kinematicConst[8];
  b_exp(&dc21);
  dc22.re = kinematicConst[8] * 0.0;
  dc22.im = kinematicConst[8];
  b_exp(&dc22);
  f_x.re = kinematicConst[8] * 2.0 * 0.0;
  f_x.im = kinematicConst[8] * 2.0;
  b_exp(&f_x);
  g_x.re = kinematicConst[8] * 2.0 * 0.0;
  g_x.im = kinematicConst[8] * 2.0;
  b_exp(&g_x);
  h_x.re = kinematicConst[8] * 0.0;
  h_x.im = kinematicConst[8];
  b_exp(&h_x);
  i_x.re = kinematicConst[8] * 0.0;
  i_x.im = kinematicConst[8];
  b_exp(&i_x);
  j_x.re = kinematicConst[8] * 0.0;
  j_x.im = kinematicConst[8];
  b_exp(&j_x);
  k_x.re = kinematicConst[8] * 0.0;
  k_x.im = kinematicConst[8];
  b_exp(&k_x);
  dc23.re = kinematicConst[8] * 0.0;
  dc23.im = kinematicConst[8];
  b_exp(&dc23);
  dc24.re = kinematicConst[8] * 0.0;
  dc24.im = kinematicConst[8];
  b_exp(&dc24);
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  b_exp(&y);
  kinematicConst_re = kinematicConst[0] * kinematicConst[0] * dc1.re;
  kinematicConst_im = kinematicConst[0] * kinematicConst[0] * dc1.im;
  b_kinematicConst_re = kinematicConst[1] * kinematicConst[1] * dc2.re;
  b_kinematicConst_im = kinematicConst[1] * kinematicConst[1] * dc2.im;
  c_kinematicConst_re = kinematicConst[2] * kinematicConst[2] * dc3.re;
  c_kinematicConst_im = kinematicConst[2] * kinematicConst[2] * dc3.im;
  d_kinematicConst_re = kinematicConst[4] * kinematicConst[4] * dc4.re;
  d_kinematicConst_im = kinematicConst[4] * kinematicConst[4] * dc4.im;
  e_kinematicConst_re = kinematicConst[5] * kinematicConst[5] * dc5.re;
  e_kinematicConst_im = kinematicConst[5] * kinematicConst[5] * dc5.im;
  f_kinematicConst_re = kinematicConst[6] * kinematicConst[6] * dc6.re;
  f_kinematicConst_im = kinematicConst[6] * kinematicConst[6] * dc6.im;
  g_kinematicConst_re = kinematicConst[7] * kinematicConst[7] * dc7.re;
  g_kinematicConst_im = kinematicConst[7] * kinematicConst[7] * dc7.im;
  x_re = x * x * dc8.re;
  x_im = x * x * dc8.im;
  zC_re = zC * zC * dc9.re;
  zC_im = zC * zC * dc9.im;
  if (dc11.im == 0.0) {
    re = dc11.re / 2.0;
    im = 0.0;
  } else if (dc11.re == 0.0) {
    re = 0.0;
    im = dc11.im / 2.0;
  } else {
    re = dc11.re / 2.0;
    im = dc11.im / 2.0;
  }

  h_kinematicConst_re = 2.0 * (kinematicConst[3] * x * (re + 0.5));
  h_kinematicConst_im = 2.0 * (kinematicConst[3] * x * im);
  i_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc12.re);
  i_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc12.im);
  j_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc13.re);
  j_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc13.im);
  k_kinematicConst_re = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc14.re);
  k_kinematicConst_im = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc14.im);
  l_kinematicConst_re = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc15.re);
  l_kinematicConst_im = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc15.im);
  m_kinematicConst_re = 2.0 * (kinematicConst[1] * x * dc16.re);
  m_kinematicConst_im = 2.0 * (kinematicConst[1] * x * dc16.im);
  n_kinematicConst_re = 2.0 * (kinematicConst[6] * x * dc17.re);
  n_kinematicConst_im = 2.0 * (kinematicConst[6] * x * dc17.im);
  o_kinematicConst_re = 2.0 * (kinematicConst[0] * zC * dc18.re);
  o_kinematicConst_im = 2.0 * (kinematicConst[0] * zC * dc18.im);
  p_kinematicConst_re = 2.0 * (kinematicConst[5] * zC * dc19.re);
  p_kinematicConst_im = 2.0 * (kinematicConst[5] * zC * dc19.im);
  q_kinematicConst_re = 2.0 * (kinematicConst[7] * zC * dc20.re);
  q_kinematicConst_im = 2.0 * (kinematicConst[7] * zC * dc20.im);
  r_kinematicConst_re = d_x * d_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.re);
  r_kinematicConst_im = d_x * d_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.im);
  s_kinematicConst_re = e_x * e_x * (kinematicConst[3] * kinematicConst[3] *
    dc22.re);
  s_kinematicConst_im = e_x * e_x * (kinematicConst[3] * kinematicConst[3] *
    dc22.im);
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

  t_kinematicConst_re = 2.0 * (kinematicConst[3] * kinematicConst[6] * (b_x_re +
    0.5));
  t_kinematicConst_im = 2.0 * (kinematicConst[3] * kinematicConst[6] * b_x_im);
  b_x_re = x * h_x.re;
  b_x_im = x * h_x.im;
  b_re = kinematicConst[1] * k_x.re;
  im = kinematicConst[1] * k_x.im;
  u_kinematicConst_re = kinematicConst[6] * dc24.re;
  u_kinematicConst_im = kinematicConst[6] * dc24.im;
  ar = -((((((((((((((((((((((((((((kinematicConst_re * 0.0 - kinematicConst_im)
    + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) - (c_kinematicConst_re *
    0.0 - c_kinematicConst_im)) + (d_kinematicConst_re * 0.0 -
    d_kinematicConst_im)) + (e_kinematicConst_re * 0.0 - e_kinematicConst_im)) +
    (f_kinematicConst_re * 0.0 - f_kinematicConst_im)) + (g_kinematicConst_re *
    0.0 - g_kinematicConst_im)) + (x_re * 0.0 - x_im)) + (zC_re * 0.0 - zC_im))
    + (dc10.re * 0.0 - dc10.im)) - (h_kinematicConst_re * 0.0 -
    h_kinematicConst_im)) - (i_kinematicConst_re * 0.0 - i_kinematicConst_im)) -
                        (j_kinematicConst_re * 0.0 - j_kinematicConst_im)) -
                       (k_kinematicConst_re * 0.0 - k_kinematicConst_im)) +
                      (l_kinematicConst_re * 0.0 - l_kinematicConst_im)) -
                     kinematicConst[0] * kinematicConst[3] * (cos(2.0 *
    kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[5] * (cos
    (2.0 * kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[7] *
                   (cos(2.0 * kinematicConst[8]) - 1.0)) - (m_kinematicConst_re *
    0.0 - m_kinematicConst_im)) + (n_kinematicConst_re * 0.0 -
    n_kinematicConst_im)) - (o_kinematicConst_re * 0.0 - o_kinematicConst_im)) +
               (p_kinematicConst_re * 0.0 - p_kinematicConst_im)) +
              (q_kinematicConst_re * 0.0 - q_kinematicConst_im)) +
             kinematicConst[3] * zC * (cos(2.0 * kinematicConst[8]) - 1.0)) +
            (r_kinematicConst_re * 0.0 - r_kinematicConst_im)) +
           (s_kinematicConst_re * 0.0 - s_kinematicConst_im)) -
          (t_kinematicConst_re * 0.0 - t_kinematicConst_im)) + kinematicConst[1]
         * kinematicConst[3] * (g_x.re * 0.0 - g_x.im));
  t_kinematicConst_re = -((((((((((((((((((((((((((((kinematicConst_re +
    kinematicConst_im * 0.0) + (b_kinematicConst_re + b_kinematicConst_im * 0.0))
    - (c_kinematicConst_re + c_kinematicConst_im * 0.0)) + (d_kinematicConst_re
    + d_kinematicConst_im * 0.0)) + (e_kinematicConst_re + e_kinematicConst_im *
    0.0)) + (f_kinematicConst_re + f_kinematicConst_im * 0.0)) +
    (g_kinematicConst_re + g_kinematicConst_im * 0.0)) + (x_re + x_im * 0.0)) +
    (zC_re + zC_im * 0.0)) + (dc10.re + dc10.im * 0.0)) - (h_kinematicConst_re +
    h_kinematicConst_im * 0.0)) - (i_kinematicConst_re + i_kinematicConst_im *
    0.0)) - (j_kinematicConst_re + j_kinematicConst_im * 0.0)) -
    (k_kinematicConst_re + k_kinematicConst_im * 0.0)) + (l_kinematicConst_re +
    l_kinematicConst_im * 0.0)) - kinematicConst[0] * kinematicConst[3] * sin
    (2.0 * kinematicConst[8])) + kinematicConst[3] * kinematicConst[5] * sin(2.0
    * kinematicConst[8])) + kinematicConst[3] * kinematicConst[7] * sin(2.0 *
    kinematicConst[8])) - (m_kinematicConst_re + m_kinematicConst_im * 0.0)) +
    (n_kinematicConst_re + n_kinematicConst_im * 0.0)) - (o_kinematicConst_re +
    o_kinematicConst_im * 0.0)) + (p_kinematicConst_re + p_kinematicConst_im *
    0.0)) + (q_kinematicConst_re + q_kinematicConst_im * 0.0)) + kinematicConst
    [3] * zC * sin(2.0 * kinematicConst[8])) + (r_kinematicConst_re +
    r_kinematicConst_im * 0.0)) + (s_kinematicConst_re + s_kinematicConst_im *
    0.0)) - (t_kinematicConst_re + t_kinematicConst_im * 0.0)) + kinematicConst
    [1] * kinematicConst[3] * ((g_x.re + g_x.im * 0.0) + 1.0));
  t_kinematicConst_im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] * 0.0
    - (b_x_re * 0.0 - b_x_im)) + zC * i_x.re) - kinematicConst[0] * j_x.re) +
    (b_re * 0.0 - im)) + kinematicConst[5] * dc23.re) - (u_kinematicConst_re *
    0.0 - u_kinematicConst_im)) + kinematicConst[7] * y.re);
  im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] - (b_x_re + b_x_im *
    0.0)) + zC * i_x.im) - kinematicConst[0] * j_x.im) + (b_re + im * 0.0)) +
    kinematicConst[5] * dc23.im) - (u_kinematicConst_re + u_kinematicConst_im *
    0.0)) + kinematicConst[7] * y.im);
  if (im == 0.0) {
    if (t_kinematicConst_re == 0.0) {
      dc1.re = ar / t_kinematicConst_im;
      dc1.im = 0.0;
    } else if (ar == 0.0) {
      dc1.re = 0.0;
      dc1.im = t_kinematicConst_re / t_kinematicConst_im;
    } else {
      dc1.re = ar / t_kinematicConst_im;
      dc1.im = t_kinematicConst_re / t_kinematicConst_im;
    }
  } else if (t_kinematicConst_im == 0.0) {
    if (ar == 0.0) {
      dc1.re = t_kinematicConst_re / im;
      dc1.im = 0.0;
    } else if (t_kinematicConst_re == 0.0) {
      dc1.re = 0.0;
      dc1.im = -(ar / im);
    } else {
      dc1.re = t_kinematicConst_re / im;
      dc1.im = -(ar / im);
    }
  } else {
    brm = fabs(t_kinematicConst_im);
    r = fabs(im);
    if (brm > r) {
      b_re = im / t_kinematicConst_im;
      r = t_kinematicConst_im + b_re * im;
      dc1.re = (ar + b_re * t_kinematicConst_re) / r;
      dc1.im = (t_kinematicConst_re - b_re * ar) / r;
    } else if (r == brm) {
      if (t_kinematicConst_im > 0.0) {
        b_re = 0.5;
      } else {
        b_re = -0.5;
      }

      if (im > 0.0) {
        r = 0.5;
      } else {
        r = -0.5;
      }

      dc1.re = (ar * b_re + t_kinematicConst_re * r) / brm;
      dc1.im = (t_kinematicConst_re * b_re - ar * r) / brm;
    } else {
      b_re = t_kinematicConst_im / im;
      r = im + b_re * t_kinematicConst_im;
      dc1.re = (b_re * ar + t_kinematicConst_re) / r;
      dc1.im = (b_re * t_kinematicConst_re - ar) / r;
    }
  }

  b_log(&dc1);
  gammaRaw[0].re = -kinematicConst[8] - (dc0.re * 0.0 - dc0.im);
  re = dc1.re * 0.0 - dc1.im;
  a = ((((((((((((((((((((((((kinematicConst[0] * kinematicConst[0] - 2.0 * sin
    (kinematicConst[8]) * kinematicConst[0] * kinematicConst[3]) - 2.0 *
    kinematicConst[0] * kinematicConst[5]) - 2.0 * kinematicConst[0] *
    kinematicConst[7]) - 2.0 * kinematicConst[0] * zC) + kinematicConst[1] *
    kinematicConst[1]) + 2.0 * cos(kinematicConst[8]) * kinematicConst[1] *
    kinematicConst[3]) - 2.0 * kinematicConst[1] * kinematicConst[6]) - 2.0 *
                       kinematicConst[1] * x) - kinematicConst[2] *
                      kinematicConst[2]) + kinematicConst[3] * kinematicConst[3])
                    + 2.0 * sin(kinematicConst[8]) * kinematicConst[3] *
                    kinematicConst[5]) - 2.0 * cos(kinematicConst[8]) *
                   kinematicConst[3] * kinematicConst[6]) + 2.0 * sin
                  (kinematicConst[8]) * kinematicConst[3] * kinematicConst[7]) -
                 2.0 * cos(kinematicConst[8]) * kinematicConst[3] * x) + 2.0 *
                sin(kinematicConst[8]) * kinematicConst[3] * zC) +
               kinematicConst[4] * kinematicConst[4]) + kinematicConst[5] *
              kinematicConst[5]) + 2.0 * kinematicConst[5] * kinematicConst[7])
            + 2.0 * kinematicConst[5] * zC) + kinematicConst[6] *
           kinematicConst[6]) + 2.0 * kinematicConst[6] * x) + kinematicConst[7]
         * kinematicConst[7]) + 2.0 * kinematicConst[7] * zC) + x * x) + zC * zC;
  b_x = cos(kinematicConst[8]);
  c_x = sin(kinematicConst[8]);
  b_a = ((((((((((((((((((((((((kinematicConst[0] * kinematicConst[0] - 2.0 *
    sin(kinematicConst[8]) * kinematicConst[0] * kinematicConst[3]) - 2.0 *
    kinematicConst[0] * kinematicConst[5]) - 2.0 * kinematicConst[0] *
    kinematicConst[7]) - 2.0 * kinematicConst[0] * zC) + kinematicConst[1] *
    kinematicConst[1]) + 2.0 * cos(kinematicConst[8]) * kinematicConst[1] *
    kinematicConst[3]) - 2.0 * kinematicConst[1] * kinematicConst[6]) - 2.0 *
    kinematicConst[1] * x) - kinematicConst[2] * kinematicConst[2]) +
                       kinematicConst[3] * kinematicConst[3]) + 2.0 * sin
                      (kinematicConst[8]) * kinematicConst[3] * kinematicConst[5])
                     - 2.0 * cos(kinematicConst[8]) * kinematicConst[3] *
                     kinematicConst[6]) + 2.0 * sin(kinematicConst[8]) *
                    kinematicConst[3] * kinematicConst[7]) - 2.0 * cos
                   (kinematicConst[8]) * kinematicConst[3] * x) + 2.0 * sin
                  (kinematicConst[8]) * kinematicConst[3] * zC) +
                 kinematicConst[4] * kinematicConst[4]) + kinematicConst[5] *
                kinematicConst[5]) + 2.0 * kinematicConst[5] * kinematicConst[7])
              + 2.0 * kinematicConst[5] * zC) + kinematicConst[6] *
             kinematicConst[6]) + 2.0 * kinematicConst[6] * x) + kinematicConst
           [7] * kinematicConst[7]) + 2.0 * kinematicConst[7] * zC) + x * x) +
    zC * zC;
  d_x = cos(kinematicConst[8]);
  e_x = sin(kinematicConst[8]);
  y.re = kinematicConst[8] * 2.0 * 0.0;
  y.im = kinematicConst[8] * 2.0;
  r = exp(y.re / 2.0);
  x_re = r * (r * cos(y.im));
  x_im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  f_x.re = r * (r * cos(y.im));
  f_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  g_x.re = r * (r * cos(y.im));
  g_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  h_x.re = r * (r * cos(y.im));
  h_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  i_x.re = r * (r * cos(y.im));
  i_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  j_x.re = r * (r * cos(y.im));
  j_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  k_x.re = r * (r * cos(y.im));
  k_x.im = r * (r * sin(y.im));
  y.re = kinematicConst[8] * 0.0;
  y.im = kinematicConst[8];
  r = exp(y.re / 2.0);
  y.re = r * (r * cos(y.im));
  y.im = r * (r * sin(y.im));
  dc0.re = kinematicConst[8] * 0.0;
  dc0.im = kinematicConst[8];
  b_exp(&dc0);
  dc1.re = kinematicConst[8] * 0.0;
  dc1.im = kinematicConst[8];
  b_exp(&dc1);
  dc2.re = kinematicConst[8] * 0.0;
  dc2.im = kinematicConst[8];
  b_exp(&dc2);
  dc3.re = kinematicConst[8] * 0.0;
  dc3.im = kinematicConst[8];
  b_exp(&dc3);
  dc4.re = kinematicConst[8] * 0.0;
  dc4.im = kinematicConst[8];
  b_exp(&dc4);
  dc5.re = kinematicConst[8] * 0.0;
  dc5.im = kinematicConst[8];
  b_exp(&dc5);
  dc6.re = kinematicConst[8] * 0.0;
  dc6.im = kinematicConst[8];
  b_exp(&dc6);
  dc7.re = kinematicConst[8] * 0.0;
  dc7.im = kinematicConst[8];
  b_exp(&dc7);
  dc8.re = kinematicConst[8] * 0.0;
  dc8.im = kinematicConst[8];
  b_exp(&dc8);
  dc9.re = kinematicConst[8] * 2.0 * 0.0;
  dc9.im = kinematicConst[8] * 2.0;
  b_exp(&dc9);
  dc10.re = kinematicConst[8] * 0.0;
  dc10.im = kinematicConst[8];
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  zC_re = zC * dc12.re;
  zC_im = zC * dc12.im;
  kinematicConst_re = kinematicConst[0] * dc13.re;
  kinematicConst_im = kinematicConst[0] * dc13.im;
  b_kinematicConst_re = kinematicConst[5] * dc15.re;
  b_kinematicConst_im = kinematicConst[5] * dc15.im;
  c_kinematicConst_re = kinematicConst[7] * dc17.re;
  c_kinematicConst_im = kinematicConst[7] * dc17.im;
  b_re = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.re;
  im = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.im;
  d_kinematicConst_re = ((((((-kinematicConst[3] + x * dc11.re) + (zC_re * 0.0 -
    zC_im)) - (kinematicConst_re * 0.0 - kinematicConst_im)) - kinematicConst[1]
    * dc14.re) + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) +
    kinematicConst[6] * dc16.re) + (c_kinematicConst_re * 0.0 -
    c_kinematicConst_im);
  kinematicConst_im = (((((x * dc11.im + (zC_re + zC_im * 0.0)) -
    (kinematicConst_re + kinematicConst_im * 0.0)) - kinematicConst[1] * dc14.im)
                        + (b_kinematicConst_re + b_kinematicConst_im * 0.0)) +
                       kinematicConst[6] * dc16.im) + (c_kinematicConst_re +
    c_kinematicConst_im * 0.0);
  r = b_re * d_kinematicConst_re - im * kinematicConst_im;
  im = b_re * kinematicConst_im + im * d_kinematicConst_re;
  kinematicConst_re = ((((((kinematicConst[1] - kinematicConst[0] * 0.0) +
    kinematicConst[5] * 0.0) - kinematicConst[6]) + kinematicConst[7] * 0.0) - x)
                       + zC * 0.0) + kinematicConst[3] * dc18.re;
  kinematicConst_im = ((((0.0 - kinematicConst[0]) + kinematicConst[5]) +
                        kinematicConst[7]) + zC) + kinematicConst[3] * dc18.im;
  dc9.re = dc9.re * (b_a * b_a) + (r * kinematicConst_re - im *
    kinematicConst_im);
  dc9.im = dc9.im * (b_a * b_a) + (r * kinematicConst_im + im *
    kinematicConst_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kinematicConst[8] * 2.0 * 0.0;
  dc10.im = kinematicConst[8] * 2.0;
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  dc19.re = kinematicConst[8] * 0.0;
  dc19.im = kinematicConst[8];
  b_exp(&dc19);
  dc20.re = kinematicConst[8] * 0.0;
  dc20.im = kinematicConst[8];
  b_exp(&dc20);
  dc21.re = kinematicConst[8] * 0.0;
  dc21.im = kinematicConst[8];
  b_exp(&dc21);
  dc22.re = kinematicConst[8] * 2.0 * 0.0;
  dc22.im = kinematicConst[8] * 2.0;
  b_exp(&dc22);
  kinematicConst_re = kinematicConst[0] * kinematicConst[0] * dc0.re;
  kinematicConst_im = kinematicConst[0] * kinematicConst[0] * dc0.im;
  b_kinematicConst_re = kinematicConst[1] * kinematicConst[1] * dc1.re;
  b_kinematicConst_im = kinematicConst[1] * kinematicConst[1] * dc1.im;
  c_kinematicConst_re = kinematicConst[2] * kinematicConst[2] * dc2.re;
  c_kinematicConst_im = kinematicConst[2] * kinematicConst[2] * dc2.im;
  d_kinematicConst_re = kinematicConst[4] * kinematicConst[4] * dc3.re;
  d_kinematicConst_im = kinematicConst[4] * kinematicConst[4] * dc3.im;
  e_kinematicConst_re = kinematicConst[5] * kinematicConst[5] * dc4.re;
  e_kinematicConst_im = kinematicConst[5] * kinematicConst[5] * dc4.im;
  f_kinematicConst_re = kinematicConst[6] * kinematicConst[6] * dc5.re;
  f_kinematicConst_im = kinematicConst[6] * kinematicConst[6] * dc5.im;
  g_kinematicConst_re = kinematicConst[7] * kinematicConst[7] * dc6.re;
  g_kinematicConst_im = kinematicConst[7] * kinematicConst[7] * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  zC_re = zC * zC * dc8.re;
  zC_im = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    b_re = dc10.re / 2.0;
    im = 0.0;
  } else if (dc10.re == 0.0) {
    b_re = 0.0;
    im = dc10.im / 2.0;
  } else {
    b_re = dc10.re / 2.0;
    im = dc10.im / 2.0;
  }

  h_kinematicConst_re = 2.0 * (kinematicConst[3] * x * (b_re + 0.5));
  h_kinematicConst_im = 2.0 * (kinematicConst[3] * x * im);
  i_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.re);
  i_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.im);
  j_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.re);
  j_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.im);
  k_kinematicConst_re = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.re);
  k_kinematicConst_im = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.im);
  l_kinematicConst_re = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.re);
  l_kinematicConst_im = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.im);
  m_kinematicConst_re = 2.0 * (kinematicConst[1] * x * dc15.re);
  m_kinematicConst_im = 2.0 * (kinematicConst[1] * x * dc15.im);
  n_kinematicConst_re = 2.0 * (kinematicConst[6] * x * dc16.re);
  n_kinematicConst_im = 2.0 * (kinematicConst[6] * x * dc16.im);
  o_kinematicConst_re = 2.0 * (kinematicConst[0] * zC * dc17.re);
  o_kinematicConst_im = 2.0 * (kinematicConst[0] * zC * dc17.im);
  p_kinematicConst_re = 2.0 * (kinematicConst[5] * zC * dc18.re);
  p_kinematicConst_im = 2.0 * (kinematicConst[5] * zC * dc18.im);
  q_kinematicConst_re = 2.0 * (kinematicConst[7] * zC * dc19.re);
  q_kinematicConst_im = 2.0 * (kinematicConst[7] * zC * dc19.im);
  r_kinematicConst_re = d_x * d_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.re);
  r_kinematicConst_im = d_x * d_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.im);
  s_kinematicConst_re = e_x * e_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.re);
  s_kinematicConst_im = e_x * e_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.im);
  if (dc22.im == 0.0) {
    b_re = dc22.re / 2.0;
    im = 0.0;
  } else if (dc22.re == 0.0) {
    b_re = 0.0;
    im = dc22.im / 2.0;
  } else {
    b_re = dc22.re / 2.0;
    im = dc22.im / 2.0;
  }

  t_kinematicConst_re = 2.0 * (kinematicConst[3] * kinematicConst[6] * (b_re +
    0.5));
  t_kinematicConst_im = 2.0 * (kinematicConst[3] * kinematicConst[6] * im);
  r = x * f_x.re;
  brm = x * f_x.im;
  b_re = kinematicConst[1] * i_x.re;
  im = kinematicConst[1] * i_x.im;
  u_kinematicConst_re = kinematicConst[6] * k_x.re;
  u_kinematicConst_im = kinematicConst[6] * k_x.im;
  ar = -((((((((((((((((((((((((((((kinematicConst_re * 0.0 - kinematicConst_im)
    + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) - (c_kinematicConst_re *
    0.0 - c_kinematicConst_im)) + (d_kinematicConst_re * 0.0 -
    d_kinematicConst_im)) + (e_kinematicConst_re * 0.0 - e_kinematicConst_im)) +
    (f_kinematicConst_re * 0.0 - f_kinematicConst_im)) + (g_kinematicConst_re *
    0.0 - g_kinematicConst_im)) + (b_x_re * 0.0 - b_x_im)) + (zC_re * 0.0 -
    zC_im)) + (dc9.re * 0.0 - dc9.im)) - (h_kinematicConst_re * 0.0 -
    h_kinematicConst_im)) - (i_kinematicConst_re * 0.0 - i_kinematicConst_im)) -
                        (j_kinematicConst_re * 0.0 - j_kinematicConst_im)) -
                       (k_kinematicConst_re * 0.0 - k_kinematicConst_im)) +
                      (l_kinematicConst_re * 0.0 - l_kinematicConst_im)) -
                     kinematicConst[0] * kinematicConst[3] * (cos(2.0 *
    kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[5] * (cos
    (2.0 * kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[7] *
                   (cos(2.0 * kinematicConst[8]) - 1.0)) - (m_kinematicConst_re *
    0.0 - m_kinematicConst_im)) + (n_kinematicConst_re * 0.0 -
    n_kinematicConst_im)) - (o_kinematicConst_re * 0.0 - o_kinematicConst_im)) +
               (p_kinematicConst_re * 0.0 - p_kinematicConst_im)) +
              (q_kinematicConst_re * 0.0 - q_kinematicConst_im)) +
             kinematicConst[3] * zC * (cos(2.0 * kinematicConst[8]) - 1.0)) +
            (r_kinematicConst_re * 0.0 - r_kinematicConst_im)) +
           (s_kinematicConst_re * 0.0 - s_kinematicConst_im)) -
          (t_kinematicConst_re * 0.0 - t_kinematicConst_im)) + kinematicConst[1]
         * kinematicConst[3] * (x_re * 0.0 - x_im));
  t_kinematicConst_re = -((((((((((((((((((((((((((((kinematicConst_re +
    kinematicConst_im * 0.0) + (b_kinematicConst_re + b_kinematicConst_im * 0.0))
    - (c_kinematicConst_re + c_kinematicConst_im * 0.0)) + (d_kinematicConst_re
    + d_kinematicConst_im * 0.0)) + (e_kinematicConst_re + e_kinematicConst_im *
    0.0)) + (f_kinematicConst_re + f_kinematicConst_im * 0.0)) +
    (g_kinematicConst_re + g_kinematicConst_im * 0.0)) + (b_x_re + b_x_im * 0.0))
    + (zC_re + zC_im * 0.0)) + (dc9.re + dc9.im * 0.0)) - (h_kinematicConst_re +
    h_kinematicConst_im * 0.0)) - (i_kinematicConst_re + i_kinematicConst_im *
    0.0)) - (j_kinematicConst_re + j_kinematicConst_im * 0.0)) -
    (k_kinematicConst_re + k_kinematicConst_im * 0.0)) + (l_kinematicConst_re +
    l_kinematicConst_im * 0.0)) - kinematicConst[0] * kinematicConst[3] * sin
    (2.0 * kinematicConst[8])) + kinematicConst[3] * kinematicConst[5] * sin(2.0
    * kinematicConst[8])) + kinematicConst[3] * kinematicConst[7] * sin(2.0 *
    kinematicConst[8])) - (m_kinematicConst_re + m_kinematicConst_im * 0.0)) +
    (n_kinematicConst_re + n_kinematicConst_im * 0.0)) - (o_kinematicConst_re +
    o_kinematicConst_im * 0.0)) + (p_kinematicConst_re + p_kinematicConst_im *
    0.0)) + (q_kinematicConst_re + q_kinematicConst_im * 0.0)) + kinematicConst
    [3] * zC * sin(2.0 * kinematicConst[8])) + (r_kinematicConst_re +
    r_kinematicConst_im * 0.0)) + (s_kinematicConst_re + s_kinematicConst_im *
    0.0)) - (t_kinematicConst_re + t_kinematicConst_im * 0.0)) + kinematicConst
    [1] * kinematicConst[3] * ((x_re + x_im * 0.0) + 1.0));
  t_kinematicConst_im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] * 0.0
    - (r * 0.0 - brm)) + zC * g_x.re) - kinematicConst[0] * h_x.re) + (b_re *
    0.0 - im)) + kinematicConst[5] * j_x.re) - (u_kinematicConst_re * 0.0 -
    u_kinematicConst_im)) + kinematicConst[7] * y.re);
  im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] - (r + brm * 0.0)) +
    zC * g_x.im) - kinematicConst[0] * h_x.im) + (b_re + im * 0.0)) +
    kinematicConst[5] * j_x.im) - (u_kinematicConst_re + u_kinematicConst_im *
    0.0)) + kinematicConst[7] * y.im);
  if (im == 0.0) {
    if (t_kinematicConst_re == 0.0) {
      y.re = ar / t_kinematicConst_im;
      y.im = 0.0;
    } else if (ar == 0.0) {
      y.re = 0.0;
      y.im = t_kinematicConst_re / t_kinematicConst_im;
    } else {
      y.re = ar / t_kinematicConst_im;
      y.im = t_kinematicConst_re / t_kinematicConst_im;
    }
  } else if (t_kinematicConst_im == 0.0) {
    if (ar == 0.0) {
      y.re = t_kinematicConst_re / im;
      y.im = 0.0;
    } else if (t_kinematicConst_re == 0.0) {
      y.re = 0.0;
      y.im = -(ar / im);
    } else {
      y.re = t_kinematicConst_re / im;
      y.im = -(ar / im);
    }
  } else {
    brm = fabs(t_kinematicConst_im);
    r = fabs(im);
    if (brm > r) {
      b_re = im / t_kinematicConst_im;
      r = t_kinematicConst_im + b_re * im;
      y.re = (ar + b_re * t_kinematicConst_re) / r;
      y.im = (t_kinematicConst_re - b_re * ar) / r;
    } else if (r == brm) {
      if (t_kinematicConst_im > 0.0) {
        b_re = 0.5;
      } else {
        b_re = -0.5;
      }

      if (im > 0.0) {
        r = 0.5;
      } else {
        r = -0.5;
      }

      y.re = (ar * b_re + t_kinematicConst_re * r) / brm;
      y.im = (t_kinematicConst_re * b_re - ar * r) / brm;
    } else {
      b_re = t_kinematicConst_im / im;
      r = im + b_re * t_kinematicConst_im;
      y.re = (b_re * ar + t_kinematicConst_re) / r;
      y.im = (b_re * t_kinematicConst_re - ar) / r;
    }
  }

  if ((y.im == 0.0) && rtIsNaN(y.re)) {
  } else if ((fabs(y.re) > 8.9884656743115785E+307) || (fabs(y.im) >
              8.9884656743115785E+307)) {
    r = y.re;
    y.re = log(rt_hypotd_snf(y.re / 2.0, y.im / 2.0)) + 0.69314718055994529;
    y.im = rt_atan2d_snf(y.im, r);
  } else {
    r = y.re;
    y.re = log(rt_hypotd_snf(y.re, y.im));
    y.im = rt_atan2d_snf(y.im, r);
  }

  x_re = y.re * 0.0 - y.im;
  x_im = y.re + y.im * 0.0;
  if (x_im == 0.0) {
    x_re = sin(x_re);
    x_im = 0.0;
  } else {
    b_x_re = x_re;
    x_re = sin(x_re) * cosh(x_im);
    x_im = cos(b_x_re) * sinh(x_im);
  }

  dc0.re = kinematicConst[8] * 0.0;
  dc0.im = kinematicConst[8];
  b_exp(&dc0);
  dc1.re = kinematicConst[8] * 0.0;
  dc1.im = kinematicConst[8];
  b_exp(&dc1);
  dc2.re = kinematicConst[8] * 0.0;
  dc2.im = kinematicConst[8];
  b_exp(&dc2);
  dc3.re = kinematicConst[8] * 0.0;
  dc3.im = kinematicConst[8];
  b_exp(&dc3);
  dc4.re = kinematicConst[8] * 0.0;
  dc4.im = kinematicConst[8];
  b_exp(&dc4);
  dc5.re = kinematicConst[8] * 0.0;
  dc5.im = kinematicConst[8];
  b_exp(&dc5);
  dc6.re = kinematicConst[8] * 0.0;
  dc6.im = kinematicConst[8];
  b_exp(&dc6);
  dc7.re = kinematicConst[8] * 0.0;
  dc7.im = kinematicConst[8];
  b_exp(&dc7);
  dc8.re = kinematicConst[8] * 0.0;
  dc8.im = kinematicConst[8];
  b_exp(&dc8);
  dc9.re = kinematicConst[8] * 2.0 * 0.0;
  dc9.im = kinematicConst[8] * 2.0;
  b_exp(&dc9);
  dc10.re = kinematicConst[8] * 0.0;
  dc10.im = kinematicConst[8];
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  zC_re = zC * dc12.re;
  zC_im = zC * dc12.im;
  kinematicConst_re = kinematicConst[0] * dc13.re;
  kinematicConst_im = kinematicConst[0] * dc13.im;
  b_kinematicConst_re = kinematicConst[5] * dc15.re;
  b_kinematicConst_im = kinematicConst[5] * dc15.im;
  c_kinematicConst_re = kinematicConst[7] * dc17.re;
  c_kinematicConst_im = kinematicConst[7] * dc17.im;
  b_re = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.re;
  im = 4.0 * (kinematicConst[4] * kinematicConst[4]) * dc10.im;
  d_kinematicConst_re = ((((((-kinematicConst[3] + x * dc11.re) + (zC_re * 0.0 -
    zC_im)) - (kinematicConst_re * 0.0 - kinematicConst_im)) - kinematicConst[1]
    * dc14.re) + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) +
    kinematicConst[6] * dc16.re) + (c_kinematicConst_re * 0.0 -
    c_kinematicConst_im);
  kinematicConst_im = (((((x * dc11.im + (zC_re + zC_im * 0.0)) -
    (kinematicConst_re + kinematicConst_im * 0.0)) - kinematicConst[1] * dc14.im)
                        + (b_kinematicConst_re + b_kinematicConst_im * 0.0)) +
                       kinematicConst[6] * dc16.im) + (c_kinematicConst_re +
    c_kinematicConst_im * 0.0);
  r = b_re * d_kinematicConst_re - im * kinematicConst_im;
  im = b_re * kinematicConst_im + im * d_kinematicConst_re;
  kinematicConst_re = ((((((kinematicConst[1] - kinematicConst[0] * 0.0) +
    kinematicConst[5] * 0.0) - kinematicConst[6]) + kinematicConst[7] * 0.0) - x)
                       + zC * 0.0) + kinematicConst[3] * dc18.re;
  kinematicConst_im = ((((0.0 - kinematicConst[0]) + kinematicConst[5]) +
                        kinematicConst[7]) + zC) + kinematicConst[3] * dc18.im;
  dc9.re = dc9.re * (a * a) + (r * kinematicConst_re - im * kinematicConst_im);
  dc9.im = dc9.im * (a * a) + (r * kinematicConst_im + im * kinematicConst_re);
  eml_scalar_sqrt(&dc9);
  dc10.re = kinematicConst[8] * 2.0 * 0.0;
  dc10.im = kinematicConst[8] * 2.0;
  b_exp(&dc10);
  dc11.re = kinematicConst[8] * 0.0;
  dc11.im = kinematicConst[8];
  b_exp(&dc11);
  dc12.re = kinematicConst[8] * 0.0;
  dc12.im = kinematicConst[8];
  b_exp(&dc12);
  dc13.re = kinematicConst[8] * 0.0;
  dc13.im = kinematicConst[8];
  b_exp(&dc13);
  dc14.re = kinematicConst[8] * 0.0;
  dc14.im = kinematicConst[8];
  b_exp(&dc14);
  dc15.re = kinematicConst[8] * 0.0;
  dc15.im = kinematicConst[8];
  b_exp(&dc15);
  dc16.re = kinematicConst[8] * 0.0;
  dc16.im = kinematicConst[8];
  b_exp(&dc16);
  dc17.re = kinematicConst[8] * 0.0;
  dc17.im = kinematicConst[8];
  b_exp(&dc17);
  dc18.re = kinematicConst[8] * 0.0;
  dc18.im = kinematicConst[8];
  b_exp(&dc18);
  dc19.re = kinematicConst[8] * 0.0;
  dc19.im = kinematicConst[8];
  b_exp(&dc19);
  dc20.re = kinematicConst[8] * 0.0;
  dc20.im = kinematicConst[8];
  b_exp(&dc20);
  dc21.re = kinematicConst[8] * 0.0;
  dc21.im = kinematicConst[8];
  b_exp(&dc21);
  dc22.re = kinematicConst[8] * 2.0 * 0.0;
  dc22.im = kinematicConst[8] * 2.0;
  b_exp(&dc22);
  f_x.re = kinematicConst[8] * 2.0 * 0.0;
  f_x.im = kinematicConst[8] * 2.0;
  b_exp(&f_x);
  g_x.re = kinematicConst[8] * 0.0;
  g_x.im = kinematicConst[8];
  b_exp(&g_x);
  h_x.re = kinematicConst[8] * 0.0;
  h_x.im = kinematicConst[8];
  b_exp(&h_x);
  i_x.re = kinematicConst[8] * 0.0;
  i_x.im = kinematicConst[8];
  b_exp(&i_x);
  j_x.re = kinematicConst[8] * 0.0;
  j_x.im = kinematicConst[8];
  b_exp(&j_x);
  k_x.re = kinematicConst[8] * 0.0;
  k_x.im = kinematicConst[8];
  b_exp(&k_x);
  dc23.re = kinematicConst[8] * 0.0;
  dc23.im = kinematicConst[8];
  b_exp(&dc23);
  dc24.re = kinematicConst[8] * 0.0;
  dc24.im = kinematicConst[8];
  b_exp(&dc24);
  kinematicConst_re = kinematicConst[0] * kinematicConst[0] * dc0.re;
  kinematicConst_im = kinematicConst[0] * kinematicConst[0] * dc0.im;
  b_kinematicConst_re = kinematicConst[1] * kinematicConst[1] * dc1.re;
  b_kinematicConst_im = kinematicConst[1] * kinematicConst[1] * dc1.im;
  c_kinematicConst_re = kinematicConst[2] * kinematicConst[2] * dc2.re;
  c_kinematicConst_im = kinematicConst[2] * kinematicConst[2] * dc2.im;
  d_kinematicConst_re = kinematicConst[4] * kinematicConst[4] * dc3.re;
  d_kinematicConst_im = kinematicConst[4] * kinematicConst[4] * dc3.im;
  e_kinematicConst_re = kinematicConst[5] * kinematicConst[5] * dc4.re;
  e_kinematicConst_im = kinematicConst[5] * kinematicConst[5] * dc4.im;
  f_kinematicConst_re = kinematicConst[6] * kinematicConst[6] * dc5.re;
  f_kinematicConst_im = kinematicConst[6] * kinematicConst[6] * dc5.im;
  g_kinematicConst_re = kinematicConst[7] * kinematicConst[7] * dc6.re;
  g_kinematicConst_im = kinematicConst[7] * kinematicConst[7] * dc6.im;
  b_x_re = x * x * dc7.re;
  b_x_im = x * x * dc7.im;
  zC_re = zC * zC * dc8.re;
  zC_im = zC * zC * dc8.im;
  if (dc10.im == 0.0) {
    b_re = dc10.re / 2.0;
    im = 0.0;
  } else if (dc10.re == 0.0) {
    b_re = 0.0;
    im = dc10.im / 2.0;
  } else {
    b_re = dc10.re / 2.0;
    im = dc10.im / 2.0;
  }

  h_kinematicConst_re = 2.0 * (kinematicConst[3] * x * (b_re + 0.5));
  h_kinematicConst_im = 2.0 * (kinematicConst[3] * x * im);
  i_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.re);
  i_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[5] * dc11.im);
  j_kinematicConst_re = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.re);
  j_kinematicConst_im = 2.0 * (kinematicConst[0] * kinematicConst[7] * dc12.im);
  k_kinematicConst_re = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.re);
  k_kinematicConst_im = 2.0 * (kinematicConst[1] * kinematicConst[6] * dc13.im);
  l_kinematicConst_re = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.re);
  l_kinematicConst_im = 2.0 * (kinematicConst[5] * kinematicConst[7] * dc14.im);
  m_kinematicConst_re = 2.0 * (kinematicConst[1] * x * dc15.re);
  m_kinematicConst_im = 2.0 * (kinematicConst[1] * x * dc15.im);
  n_kinematicConst_re = 2.0 * (kinematicConst[6] * x * dc16.re);
  n_kinematicConst_im = 2.0 * (kinematicConst[6] * x * dc16.im);
  o_kinematicConst_re = 2.0 * (kinematicConst[0] * zC * dc17.re);
  o_kinematicConst_im = 2.0 * (kinematicConst[0] * zC * dc17.im);
  p_kinematicConst_re = 2.0 * (kinematicConst[5] * zC * dc18.re);
  p_kinematicConst_im = 2.0 * (kinematicConst[5] * zC * dc18.im);
  q_kinematicConst_re = 2.0 * (kinematicConst[7] * zC * dc19.re);
  q_kinematicConst_im = 2.0 * (kinematicConst[7] * zC * dc19.im);
  r_kinematicConst_re = b_x * b_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.re);
  r_kinematicConst_im = b_x * b_x * (kinematicConst[3] * kinematicConst[3] *
    dc20.im);
  s_kinematicConst_re = c_x * c_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.re);
  s_kinematicConst_im = c_x * c_x * (kinematicConst[3] * kinematicConst[3] *
    dc21.im);
  if (dc22.im == 0.0) {
    b_re = dc22.re / 2.0;
    im = 0.0;
  } else if (dc22.re == 0.0) {
    b_re = 0.0;
    im = dc22.im / 2.0;
  } else {
    b_re = dc22.re / 2.0;
    im = dc22.im / 2.0;
  }

  t_kinematicConst_re = 2.0 * (kinematicConst[3] * kinematicConst[6] * (b_re +
    0.5));
  t_kinematicConst_im = 2.0 * (kinematicConst[3] * kinematicConst[6] * im);
  r = x * g_x.re;
  brm = x * g_x.im;
  b_re = kinematicConst[1] * j_x.re;
  im = kinematicConst[1] * j_x.im;
  u_kinematicConst_re = kinematicConst[6] * dc23.re;
  u_kinematicConst_im = kinematicConst[6] * dc23.im;
  ar = -((((((((((((((((((((((((((((kinematicConst_re * 0.0 - kinematicConst_im)
    + (b_kinematicConst_re * 0.0 - b_kinematicConst_im)) - (c_kinematicConst_re *
    0.0 - c_kinematicConst_im)) + (d_kinematicConst_re * 0.0 -
    d_kinematicConst_im)) + (e_kinematicConst_re * 0.0 - e_kinematicConst_im)) +
    (f_kinematicConst_re * 0.0 - f_kinematicConst_im)) + (g_kinematicConst_re *
    0.0 - g_kinematicConst_im)) + (b_x_re * 0.0 - b_x_im)) + (zC_re * 0.0 -
    zC_im)) - (dc9.re * 0.0 - dc9.im)) - (h_kinematicConst_re * 0.0 -
    h_kinematicConst_im)) - (i_kinematicConst_re * 0.0 - i_kinematicConst_im)) -
                        (j_kinematicConst_re * 0.0 - j_kinematicConst_im)) -
                       (k_kinematicConst_re * 0.0 - k_kinematicConst_im)) +
                      (l_kinematicConst_re * 0.0 - l_kinematicConst_im)) -
                     kinematicConst[0] * kinematicConst[3] * (cos(2.0 *
    kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[5] * (cos
    (2.0 * kinematicConst[8]) - 1.0)) + kinematicConst[3] * kinematicConst[7] *
                   (cos(2.0 * kinematicConst[8]) - 1.0)) - (m_kinematicConst_re *
    0.0 - m_kinematicConst_im)) + (n_kinematicConst_re * 0.0 -
    n_kinematicConst_im)) - (o_kinematicConst_re * 0.0 - o_kinematicConst_im)) +
               (p_kinematicConst_re * 0.0 - p_kinematicConst_im)) +
              (q_kinematicConst_re * 0.0 - q_kinematicConst_im)) +
             kinematicConst[3] * zC * (cos(2.0 * kinematicConst[8]) - 1.0)) +
            (r_kinematicConst_re * 0.0 - r_kinematicConst_im)) +
           (s_kinematicConst_re * 0.0 - s_kinematicConst_im)) -
          (t_kinematicConst_re * 0.0 - t_kinematicConst_im)) + kinematicConst[1]
         * kinematicConst[3] * (f_x.re * 0.0 - f_x.im));
  t_kinematicConst_re = -((((((((((((((((((((((((((((kinematicConst_re +
    kinematicConst_im * 0.0) + (b_kinematicConst_re + b_kinematicConst_im * 0.0))
    - (c_kinematicConst_re + c_kinematicConst_im * 0.0)) + (d_kinematicConst_re
    + d_kinematicConst_im * 0.0)) + (e_kinematicConst_re + e_kinematicConst_im *
    0.0)) + (f_kinematicConst_re + f_kinematicConst_im * 0.0)) +
    (g_kinematicConst_re + g_kinematicConst_im * 0.0)) + (b_x_re + b_x_im * 0.0))
    + (zC_re + zC_im * 0.0)) - (dc9.re + dc9.im * 0.0)) - (h_kinematicConst_re +
    h_kinematicConst_im * 0.0)) - (i_kinematicConst_re + i_kinematicConst_im *
    0.0)) - (j_kinematicConst_re + j_kinematicConst_im * 0.0)) -
    (k_kinematicConst_re + k_kinematicConst_im * 0.0)) + (l_kinematicConst_re +
    l_kinematicConst_im * 0.0)) - kinematicConst[0] * kinematicConst[3] * sin
    (2.0 * kinematicConst[8])) + kinematicConst[3] * kinematicConst[5] * sin(2.0
    * kinematicConst[8])) + kinematicConst[3] * kinematicConst[7] * sin(2.0 *
    kinematicConst[8])) - (m_kinematicConst_re + m_kinematicConst_im * 0.0)) +
    (n_kinematicConst_re + n_kinematicConst_im * 0.0)) - (o_kinematicConst_re +
    o_kinematicConst_im * 0.0)) + (p_kinematicConst_re + p_kinematicConst_im *
    0.0)) + (q_kinematicConst_re + q_kinematicConst_im * 0.0)) + kinematicConst
    [3] * zC * sin(2.0 * kinematicConst[8])) + (r_kinematicConst_re +
    r_kinematicConst_im * 0.0)) + (s_kinematicConst_re + s_kinematicConst_im *
    0.0)) - (t_kinematicConst_re + t_kinematicConst_im * 0.0)) + kinematicConst
    [1] * kinematicConst[3] * ((f_x.re + f_x.im * 0.0) + 1.0));
  t_kinematicConst_im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] * 0.0
    - (r * 0.0 - brm)) + zC * h_x.re) - kinematicConst[0] * i_x.re) + (b_re *
    0.0 - im)) + kinematicConst[5] * k_x.re) - (u_kinematicConst_re * 0.0 -
    u_kinematicConst_im)) + kinematicConst[7] * dc24.re);
  im = 2.0 * kinematicConst[4] * (((((((kinematicConst[3] - (r + brm * 0.0)) +
    zC * h_x.im) - kinematicConst[0] * i_x.im) + (b_re + im * 0.0)) +
    kinematicConst[5] * k_x.im) - (u_kinematicConst_re + u_kinematicConst_im *
    0.0)) + kinematicConst[7] * dc24.im);
  if (im == 0.0) {
    if (t_kinematicConst_re == 0.0) {
      dc0.re = ar / t_kinematicConst_im;
      dc0.im = 0.0;
    } else if (ar == 0.0) {
      dc0.re = 0.0;
      dc0.im = t_kinematicConst_re / t_kinematicConst_im;
    } else {
      dc0.re = ar / t_kinematicConst_im;
      dc0.im = t_kinematicConst_re / t_kinematicConst_im;
    }
  } else if (t_kinematicConst_im == 0.0) {
    if (ar == 0.0) {
      dc0.re = t_kinematicConst_re / im;
      dc0.im = 0.0;
    } else if (t_kinematicConst_re == 0.0) {
      dc0.re = 0.0;
      dc0.im = -(ar / im);
    } else {
      dc0.re = t_kinematicConst_re / im;
      dc0.im = -(ar / im);
    }
  } else {
    brm = fabs(t_kinematicConst_im);
    r = fabs(im);
    if (brm > r) {
      b_re = im / t_kinematicConst_im;
      r = t_kinematicConst_im + b_re * im;
      dc0.re = (ar + b_re * t_kinematicConst_re) / r;
      dc0.im = (t_kinematicConst_re - b_re * ar) / r;
    } else if (r == brm) {
      if (t_kinematicConst_im > 0.0) {
        b_re = 0.5;
      } else {
        b_re = -0.5;
      }

      if (im > 0.0) {
        r = 0.5;
      } else {
        r = -0.5;
      }

      dc0.re = (ar * b_re + t_kinematicConst_re * r) / brm;
      dc0.im = (t_kinematicConst_re * b_re - ar * r) / brm;
    } else {
      b_re = t_kinematicConst_im / im;
      r = im + b_re * t_kinematicConst_im;
      dc0.re = (b_re * ar + t_kinematicConst_re) / r;
      dc0.im = (b_re * t_kinematicConst_re - ar) / r;
    }
  }

  b_log(&dc0);
  b_re = dc0.re;
  dc0.re = dc0.re * 0.0 - dc0.im;
  dc0.im = b_re + dc0.im * 0.0;
  b_sin(&dc0);
  kinematicConst_re = ((((kinematicConst[5] - kinematicConst[0]) +
    kinematicConst[7]) + zC) + kinematicConst[3] * sin(kinematicConst[8])) -
    kinematicConst[4] * dc0.re;
  kinematicConst_im = 0.0 - kinematicConst[4] * dc0.im;
  if (kinematicConst_im == 0.0) {
    dc0.re = kinematicConst_re / kinematicConst[2];
    dc0.im = 0.0;
  } else if (kinematicConst_re == 0.0) {
    dc0.re = 0.0;
    dc0.im = kinematicConst_im / kinematicConst[2];
  } else {
    dc0.re = kinematicConst_re / kinematicConst[2];
    dc0.im = kinematicConst_im / kinematicConst[2];
  }

  b_asin(&dc0);
  ar = ((((kinematicConst[5] - kinematicConst[0]) + kinematicConst[7]) + zC) +
        kinematicConst[3] * sin(kinematicConst[8])) - kinematicConst[4] * x_re;
  t_kinematicConst_re = 0.0 - kinematicConst[4] * x_im;
  if (t_kinematicConst_re == 0.0) {
    dc1.re = ar / kinematicConst[2];
    dc1.im = 0.0;
  } else if (ar == 0.0) {
    dc1.re = 0.0;
    dc1.im = t_kinematicConst_re / kinematicConst[2];
  } else {
    dc1.re = ar / kinematicConst[2];
    dc1.im = t_kinematicConst_re / kinematicConst[2];
  }

  b_asin(&dc1);
  *alpha = rt_atan2d_snf(yC, xC);

  // beta = betaRaw(1);
  // gamma = gammaRaw(1);
  if ((*alpha >= jointLimits[0]) && (*alpha <= jointLimits[1]) && (-dc0.re >=
       jointLimits[2]) && (-dc0.re <= jointLimits[3]) && (gammaRaw[0].re >=
       jointLimits[4]) && (gammaRaw[0].re <= jointLimits[5])) {
    *beta = -dc0.re;
    *b_gamma = gammaRaw[0].re;

    // elseif alpha >= alphaMin && alpha <= alphaMax && betaRaw(2) >= betaMin && betaRaw(2) <= betaMax && gammaRaw(2) >= gammaMin && gammaRaw(2) <= gammaMax 
  } else {
    *beta = -dc1.re;
    *b_gamma = -kinematicConst[8] - re;
  }
}

//
// File trailer for sherpaTTIK.cpp
//
// [EOF]
//
