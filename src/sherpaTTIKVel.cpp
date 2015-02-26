//
// File: sherpaTTIKVel.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 26-Feb-2015 11:03:31
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTIKVel.h"
#include <stdio.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d1;
  double d2;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d1 = fabs(u0);
    d2 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d1 == 1.0) {
        y = rtNaN;
      } else if (d1 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d2 == 0.0) {
      y = 1.0;
    } else if (d2 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.
// Arguments    : const double uDot[3]
//                const double q[3]
//                double kC_l2
//                double kC_l3
//                double kC_l4
//                double kC_l5
//                double kC_l7
//                double kC_zeta
//                double qDot[3]
// Return Type  : void
//
void sherpaTTIKVel(const double uDot[3], const double q[3], double kC_l2, double
                   kC_l3, double kC_l4, double kC_l5, double kC_l7, double
                   kC_zeta, double qDot[3])
{
  double x;
  double b_x;
  double c_x;
  double d_x;
  double e_x;
  double f_x;
  double g_x;
  double h_x;
  double i_x;
  double j_x;
  double k_x;
  double l_x;
  double m_x;
  double n_x;
  double o_x;
  double p_x;
  double q_x;
  double r_x;
  double s_x;
  double t_x;
  double u_x;
  double v_x;
  double w_x;
  double x_x;
  double y_x;
  double ab_x;
  double bb_x;
  double cb_x;
  double db_x;
  double eb_x;
  double fb_x;
  double gb_x;
  double hb_x;
  double ib_x;
  double jb_x;
  double kb_x;
  double lb_x;
  double mb_x;
  double nb_x;
  double ob_x;
  double pb_x;
  double qb_x;
  double rb_x;
  double sb_x;
  double tb_x;
  double ub_x;
  double vb_x;
  double wb_x;
  double xb_x;
  double yb_x;
  double ac_x;
  double bc_x;
  double cc_x;
  double dc_x;
  double ec_x;
  double fc_x;
  double gc_x;
  double hc_x;
  double ic_x;
  double jc_x;
  double kc_x;
  double lc_x;
  double mc_x;
  double nc_x;
  double oc_x;
  double pc_x;
  double qc_x;
  double rc_x;
  double sc_x;
  double tc_x;
  double uc_x;
  double vc_x;
  double wc_x;
  double xc_x;
  double yc_x;
  double ad_x;
  double bd_x;
  double cd_x;
  double dd_x;
  double ed_x;
  double fd_x;
  double gd_x;
  double hd_x;
  double id_x;
  double jd_x;
  double kd_x;
  double ld_x;
  double md_x;
  double nd_x;
  double od_x;
  double pd_x;
  double qd_x;
  double rd_x;
  double sd_x;
  double td_x;
  double ud_x;
  double vd_x;
  double wd_x;
  double xd_x;
  double yd_x;
  double ae_x;
  double be_x;

  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  x = cos(q[1]);
  b_x = sin(q[0]);
  c_x = cos(q[2] + kC_zeta);
  d_x = sin(q[0]);
  e_x = sin(q[0]);
  f_x = sin(q[0]);
  g_x = cos(q[2] + kC_zeta);
  h_x = cos(q[0]);
  i_x = cos(q[0]);
  j_x = cos(q[1]);
  k_x = sin(q[0]);
  l_x = sin(q[0]);
  m_x = sin(q[0]);
  n_x = cos(q[0]);
  o_x = sin(q[0]);
  p_x = cos(q[0]);
  q_x = sin(q[0]);
  r_x = sin(q[0]);
  s_x = cos(q[0]);
  t_x = cos(q[0]);
  u_x = cos(q[0]);
  v_x = cos(q[0]);
  w_x = cos(q[0]);
  x_x = cos(q[0]);
  y_x = sin(q[2] + kC_zeta);
  ab_x = sin(q[1]);
  bb_x = cos(q[1]);
  cb_x = sin(q[0]);
  db_x = cos(q[2] + kC_zeta);
  eb_x = sin(q[0]);
  fb_x = sin(q[0]);
  gb_x = sin(q[0]);
  hb_x = cos(q[2] + kC_zeta);
  ib_x = cos(q[0]);
  jb_x = cos(q[0]);
  kb_x = cos(q[1]);
  lb_x = sin(q[0]);
  mb_x = sin(q[0]);
  nb_x = sin(q[0]);
  ob_x = cos(q[0]);
  pb_x = sin(q[0]);
  qb_x = cos(q[0]);
  rb_x = sin(q[0]);
  sb_x = sin(q[0]);
  tb_x = cos(q[0]);
  ub_x = cos(q[0]);
  vb_x = cos(q[0]);
  wb_x = cos(q[0]);
  xb_x = cos(q[0]);
  yb_x = cos(q[0]);
  ac_x = cos(q[1]);
  bc_x = sin(q[0]);
  cc_x = cos(q[2] + kC_zeta);
  dc_x = sin(q[0]);
  ec_x = sin(q[0]);
  fc_x = sin(q[0]);
  gc_x = cos(q[2] + kC_zeta);
  hc_x = cos(q[0]);
  ic_x = cos(q[0]);
  jc_x = cos(q[1]);
  kc_x = sin(q[0]);
  lc_x = sin(q[0]);
  mc_x = sin(q[0]);
  nc_x = cos(q[0]);
  oc_x = sin(q[0]);
  pc_x = cos(q[0]);
  qc_x = sin(q[0]);
  rc_x = sin(q[0]);
  sc_x = cos(q[0]);
  tc_x = cos(q[0]);
  uc_x = cos(q[0]);
  vc_x = cos(q[0]);
  wc_x = cos(q[0]);
  xc_x = cos(q[0]);
  yc_x = cos(q[0]);
  ad_x = sin(q[0]);
  bd_x = sin(q[0]);
  cd_x = sin(q[0]);
  dd_x = cos(q[0]);
  ed_x = cos(q[0]);
  fd_x = sin(q[0]);
  gd_x = sin(q[0]);
  hd_x = cos(q[0]);
  id_x = cos(q[0]);
  jd_x = sin(q[0]);
  kd_x = sin(q[0]);
  ld_x = cos(q[0]);
  md_x = cos(q[0]);
  nd_x = sin(q[0]);
  od_x = sin(q[0]);
  pd_x = cos(q[0]);
  qd_x = cos(q[0]);
  rd_x = cos(q[0]);
  sd_x = sin(q[0]);
  td_x = sin(q[0]);
  ud_x = sin(q[0]);
  vd_x = cos(q[0]);
  wd_x = cos(q[0]);
  xd_x = sin(q[0]);
  yd_x = sin(q[0]);
  ae_x = cos(q[0]);
  be_x = cos(q[0]);
  qDot[0] = (uDot[0] * (cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * sin(q[0]) *
                        (kC_l5 * kC_l5) - cos(q[1]) * sin(q[0]) * sin(q[1]) *
                        (kC_l3 * kC_l3)) / (((((((((((((((((((rt_powd_snf(kC_l3,
    3.0) * (x * x) * (b_x * b_x) * sin(q[1]) - rt_powd_snf(kC_l5, 3.0) * (c_x *
    c_x) * sin(q[2] + kC_zeta) * (d_x * d_x)) + kC_l2 * (kC_l3 * kC_l3) * cos(q
    [1]) * (e_x * e_x) * sin(q[1])) - kC_l3 * kC_l3 * kC_l7 * cos(q[1]) * (f_x *
    f_x) * sin(q[1])) - kC_l3 * (kC_l5 * kC_l5) * (g_x * g_x) * (h_x * h_x) *
    sin(q[1])) + kC_l3 * kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (i_x * i_x) *
    (j_x * j_x)) - kC_l2 * (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] +
    kC_zeta) * (k_x * k_x)) + kC_l5 * kC_l5 * kC_l7 * cos(q[2] + kC_zeta) * sin
    (q[2] + kC_zeta) * (l_x * l_x)) + kC_l3 * kC_l3 * kC_l4 * cos(q[1]) * (m_x *
    m_x) * sin(q[1]) * cos(kC_zeta)) + kC_l3 * (kC_l5 * kC_l5) * cos(q[2] +
    kC_zeta) * sin(q[2] + kC_zeta) * (n_x * n_x) * cos(q[1])) - kC_l3 * (kC_l5 *
    kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * cos(q[1]) * (o_x * o_x))
    - kC_l3 * kC_l3 * kC_l5 * cos(q[2] + kC_zeta) * (p_x * p_x) * cos(q[1]) *
    sin(q[1])) - kC_l4 * (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] +
    kC_zeta) * (q_x * q_x) * cos(kC_zeta)) + kC_l3 * kC_l3 * kC_l5 * cos(q[2] +
    kC_zeta) * cos(q[1]) * (r_x * r_x) * sin(q[1])) - kC_l2 * kC_l3 * kC_l5 *
    cos(q[2] + kC_zeta) * (s_x * s_x) * sin(q[1])) + kC_l2 * kC_l3 * kC_l5 * sin
                  (q[2] + kC_zeta) * (t_x * t_x) * cos(q[1])) + kC_l3 * kC_l5 *
    kC_l7 * cos(q[2] + kC_zeta) * (u_x * u_x) * sin(q[1])) - kC_l3 * kC_l5 *
    kC_l7 * sin(q[2] + kC_zeta) * (v_x * v_x) * cos(q[1])) - kC_l3 * kC_l4 *
    kC_l5 * cos(q[2] + kC_zeta) * (w_x * w_x) * sin(q[1]) * cos(kC_zeta)) +
              kC_l3 * kC_l4 * kC_l5 * sin(q[2] + kC_zeta) * (x_x * x_x) * cos(q
    [1]) * cos(kC_zeta)) - uDot[2] * (kC_l5 * kC_l5 * (y_x * y_x) * cos(q[0]) *
              sin(q[0]) - kC_l3 * kC_l3 * cos(q[0]) * sin(q[0]) * (ab_x * ab_x))
             / (((((((((((((((((((rt_powd_snf(kC_l3, 3.0) * (bb_x * bb_x) *
    (cb_x * cb_x) * sin(q[1]) - rt_powd_snf(kC_l5, 3.0) * (db_x * db_x) * sin(q
    [2] + kC_zeta) * (eb_x * eb_x)) + kC_l2 * (kC_l3 * kC_l3) * cos(q[1]) *
    (fb_x * fb_x) * sin(q[1])) - kC_l3 * kC_l3 * kC_l7 * cos(q[1]) * (gb_x *
    gb_x) * sin(q[1])) - kC_l3 * (kC_l5 * kC_l5) * (hb_x * hb_x) * (ib_x * ib_x)
    * sin(q[1])) + kC_l3 * kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (jb_x * jb_x) *
                            (kb_x * kb_x)) - kC_l2 * (kC_l5 * kC_l5) * cos(q[2]
    + kC_zeta) * sin(q[2] + kC_zeta) * (lb_x * lb_x)) + kC_l5 * kC_l5 * kC_l7 *
    cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (mb_x * mb_x)) + kC_l3 * kC_l3 *
    kC_l4 * cos(q[1]) * (nb_x * nb_x) * sin(q[1]) * cos(kC_zeta)) + kC_l3 *
    (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (ob_x * ob_x) *
    cos(q[1])) - kC_l3 * (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] +
    kC_zeta) * cos(q[1]) * (pb_x * pb_x)) - kC_l3 * kC_l3 * kC_l5 * cos(q[2] +
    kC_zeta) * (qb_x * qb_x) * cos(q[1]) * sin(q[1])) - kC_l4 * (kC_l5 * kC_l5) *
                       cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (rb_x * rb_x)
                       * cos(kC_zeta)) + kC_l3 * kC_l3 * kC_l5 * cos(q[2] +
    kC_zeta) * cos(q[1]) * (sb_x * sb_x) * sin(q[1])) - kC_l2 * kC_l3 * kC_l5 *
                     cos(q[2] + kC_zeta) * (tb_x * tb_x) * sin(q[1])) + kC_l2 *
                    kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (ub_x * ub_x) * cos(q
    [1])) + kC_l3 * kC_l5 * kC_l7 * cos(q[2] + kC_zeta) * (vb_x * vb_x) * sin(q
    [1])) - kC_l3 * kC_l5 * kC_l7 * sin(q[2] + kC_zeta) * (wb_x * wb_x) * cos(q
    [1])) - kC_l3 * kC_l4 * kC_l5 * cos(q[2] + kC_zeta) * (xb_x * xb_x) * sin(q
    [1]) * cos(kC_zeta)) + kC_l3 * kC_l4 * kC_l5 * sin(q[2] + kC_zeta) * (yb_x *
    yb_x) * cos(q[1]) * cos(kC_zeta))) - uDot[1] * (kC_l3 * kC_l5 * cos(q[2] +
    kC_zeta) * cos(q[0]) * sin(q[1]) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * cos
    (q[0]) * cos(q[1])) / (((((((((((((((((((rt_powd_snf(kC_l3, 3.0) * (ac_x *
    ac_x) * (bc_x * bc_x) * sin(q[1]) - rt_powd_snf(kC_l5, 3.0) * (cc_x * cc_x) *
    sin(q[2] + kC_zeta) * (dc_x * dc_x)) + kC_l2 * (kC_l3 * kC_l3) * cos(q[1]) *
    (ec_x * ec_x) * sin(q[1])) - kC_l3 * kC_l3 * kC_l7 * cos(q[1]) * (fc_x *
    fc_x) * sin(q[1])) - kC_l3 * (kC_l5 * kC_l5) * (gc_x * gc_x) * (hc_x * hc_x)
    * sin(q[1])) + kC_l3 * kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (ic_x * ic_x) *
    (jc_x * jc_x)) - kC_l2 * (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] +
    kC_zeta) * (kc_x * kc_x)) + kC_l5 * kC_l5 * kC_l7 * cos(q[2] + kC_zeta) *
    sin(q[2] + kC_zeta) * (lc_x * lc_x)) + kC_l3 * kC_l3 * kC_l4 * cos(q[1]) *
    (mc_x * mc_x) * sin(q[1]) * cos(kC_zeta)) + kC_l3 * (kC_l5 * kC_l5) * cos(q
    [2] + kC_zeta) * sin(q[2] + kC_zeta) * (nc_x * nc_x) * cos(q[1])) - kC_l3 *
                                    (kC_l5 * kC_l5) * cos(q[2] + kC_zeta) * sin
    (q[2] + kC_zeta) * cos(q[1]) * (oc_x * oc_x)) - kC_l3 * kC_l3 * kC_l5 * cos
    (q[2] + kC_zeta) * (pc_x * pc_x) * cos(q[1]) * sin(q[1])) - kC_l4 * (kC_l5 *
    kC_l5) * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (qc_x * qc_x) * cos
    (kC_zeta)) + kC_l3 * kC_l3 * kC_l5 * cos(q[2] + kC_zeta) * cos(q[1]) * (rc_x
    * rc_x) * sin(q[1])) - kC_l2 * kC_l3 * kC_l5 * cos(q[2] + kC_zeta) * (sc_x *
    sc_x) * sin(q[1])) + kC_l2 * kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (tc_x *
    tc_x) * cos(q[1])) + kC_l3 * kC_l5 * kC_l7 * cos(q[2] + kC_zeta) * (uc_x *
    uc_x) * sin(q[1])) - kC_l3 * kC_l5 * kC_l7 * sin(q[2] + kC_zeta) * (vc_x *
    vc_x) * cos(q[1])) - kC_l3 * kC_l4 * kC_l5 * cos(q[2] + kC_zeta) * (wc_x *
    wc_x) * sin(q[1]) * cos(kC_zeta)) + kC_l3 * kC_l4 * kC_l5 * sin(q[2] +
    kC_zeta) * (xc_x * xc_x) * cos(q[1]) * cos(kC_zeta));
  qDot[1] = (uDot[2] * (kC_l5 * sin(q[2] + kC_zeta) * (yc_x * yc_x) + kC_l3 *
                        sin(q[1]) * (ad_x * ad_x)) / (((kC_l5 * kC_l5 * cos(q[2]
    + kC_zeta) * sin(q[2] + kC_zeta) * (bd_x * bd_x) - kC_l3 * kC_l3 * cos(q[1])
    * (cd_x * cd_x) * sin(q[1])) + kC_l3 * kC_l5 * cos(q[2] + kC_zeta) * (dd_x *
    dd_x) * sin(q[1])) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (ed_x * ed_x) *
              cos(q[1])) - kC_l5 * cos(q[2] + kC_zeta) * cos(q[0]) * uDot[0] /
             (((kC_l5 * kC_l5 * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) *
                (fd_x * fd_x) - kC_l3 * kC_l3 * cos(q[1]) * (gd_x * gd_x) * sin
                (q[1])) + kC_l3 * kC_l5 * cos(q[2] + kC_zeta) * (hd_x * hd_x) *
               sin(q[1])) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (id_x * id_x) *
              cos(q[1]))) - kC_l5 * cos(q[2] + kC_zeta) * sin(q[0]) * uDot[1] /
    (((kC_l5 * kC_l5 * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (jd_x * jd_x)
       - kC_l3 * kC_l3 * cos(q[1]) * (kd_x * kd_x) * sin(q[1])) + kC_l3 * kC_l5 *
      cos(q[2] + kC_zeta) * (ld_x * ld_x) * sin(q[1])) - kC_l3 * kC_l5 * sin(q[2]
      + kC_zeta) * (md_x * md_x) * cos(q[1]));
  qDot[2] = (kC_l3 * cos(q[0]) * cos(q[1]) * uDot[0] / (((kC_l5 * kC_l5 * cos(q
    [2] + kC_zeta) * sin(q[2] + kC_zeta) * (nd_x * nd_x) - kC_l3 * kC_l3 * cos
    (q[1]) * (od_x * od_x) * sin(q[1])) + kC_l3 * kC_l5 * cos(q[2] + kC_zeta) *
               (pd_x * pd_x) * sin(q[1])) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) *
              (qd_x * qd_x) * cos(q[1])) - uDot[2] * (kC_l3 * sin(q[1]) * (rd_x *
    rd_x) + kC_l5 * sin(q[2] + kC_zeta) * (sd_x * sd_x)) / (((kC_l5 * kC_l5 *
    cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (td_x * td_x) - kC_l3 * kC_l3 *
    cos(q[1]) * (ud_x * ud_x) * sin(q[1])) + kC_l3 * kC_l5 * cos(q[2] + kC_zeta)
    * (vd_x * vd_x) * sin(q[1])) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) * (wd_x *
    wd_x) * cos(q[1]))) + kC_l3 * cos(q[1]) * sin(q[0]) * uDot[1] / (((kC_l5 *
    kC_l5 * cos(q[2] + kC_zeta) * sin(q[2] + kC_zeta) * (xd_x * xd_x) - kC_l3 *
    kC_l3 * cos(q[1]) * (yd_x * yd_x) * sin(q[1])) + kC_l3 * kC_l5 * cos(q[2] +
    kC_zeta) * (ae_x * ae_x) * sin(q[1])) - kC_l3 * kC_l5 * sin(q[2] + kC_zeta) *
    (be_x * be_x) * cos(q[1]));
}

//
// File trailer for sherpaTTIKVel.cpp
//
// [EOF]
//
