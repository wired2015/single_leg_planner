//
// File: sherpaTTIKVel.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 12-Feb-2015 09:24:14
//

// Include Files
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "sherpaTTIKVel.h"
#include "extractKinematicConstants.h"
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
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
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
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
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
//                const double kinematicConst[16]
//                double qDot[3]
// Return Type  : void
//
void sherpaTTIKVel(const double uDot[3], const double q[3], const double
                   kinematicConst[16], double qDot[3])
{
  double unusedU9;
  double unusedU8;
  double unusedU7;
  double unusedU6;
  double unusedU5;
  double unusedU4;
  double unusedU3;
  double zeta;
  double unusedU2;
  double L7;
  double unusedU1;
  double L5;
  double L4;
  double L3;
  double L2;
  double unusedU0;
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

  // sherpaTTFKVel.m
  // author: wreid
  // date: 20150122
  extractKinematicConstants(kinematicConst, &unusedU0, &L2, &L3, &L4, &L5,
    &unusedU1, &L7, &unusedU2, &zeta, &unusedU3, &unusedU4, &unusedU5, &unusedU6,
    &unusedU7, &unusedU8, &unusedU9);
  unusedU0 = cos(q[1]);
  unusedU1 = sin(q[0]);
  unusedU2 = cos(q[2] + zeta);
  unusedU3 = sin(q[0]);
  unusedU4 = sin(q[0]);
  unusedU5 = sin(q[0]);
  unusedU6 = cos(q[2] + zeta);
  unusedU7 = cos(q[0]);
  unusedU8 = cos(q[0]);
  unusedU9 = cos(q[1]);
  x = sin(q[0]);
  b_x = sin(q[0]);
  c_x = sin(q[0]);
  d_x = cos(q[0]);
  e_x = sin(q[0]);
  f_x = cos(q[0]);
  g_x = sin(q[0]);
  h_x = sin(q[0]);
  i_x = cos(q[0]);
  j_x = cos(q[0]);
  k_x = cos(q[0]);
  l_x = cos(q[0]);
  m_x = cos(q[0]);
  n_x = cos(q[0]);
  o_x = sin(q[2] + zeta);
  p_x = sin(q[1]);
  q_x = cos(q[1]);
  r_x = sin(q[0]);
  s_x = cos(q[2] + zeta);
  t_x = sin(q[0]);
  u_x = sin(q[0]);
  v_x = sin(q[0]);
  w_x = cos(q[2] + zeta);
  x_x = cos(q[0]);
  y_x = cos(q[0]);
  ab_x = cos(q[1]);
  bb_x = sin(q[0]);
  cb_x = sin(q[0]);
  db_x = sin(q[0]);
  eb_x = cos(q[0]);
  fb_x = sin(q[0]);
  gb_x = cos(q[0]);
  hb_x = sin(q[0]);
  ib_x = sin(q[0]);
  jb_x = cos(q[0]);
  kb_x = cos(q[0]);
  lb_x = cos(q[0]);
  mb_x = cos(q[0]);
  nb_x = cos(q[0]);
  ob_x = cos(q[0]);
  pb_x = cos(q[1]);
  qb_x = sin(q[0]);
  rb_x = cos(q[2] + zeta);
  sb_x = sin(q[0]);
  tb_x = sin(q[0]);
  ub_x = sin(q[0]);
  vb_x = cos(q[2] + zeta);
  wb_x = cos(q[0]);
  xb_x = cos(q[0]);
  yb_x = cos(q[1]);
  ac_x = sin(q[0]);
  bc_x = sin(q[0]);
  cc_x = sin(q[0]);
  dc_x = cos(q[0]);
  ec_x = sin(q[0]);
  fc_x = cos(q[0]);
  gc_x = sin(q[0]);
  hc_x = sin(q[0]);
  ic_x = cos(q[0]);
  jc_x = cos(q[0]);
  kc_x = cos(q[0]);
  lc_x = cos(q[0]);
  mc_x = cos(q[0]);
  nc_x = cos(q[0]);
  oc_x = cos(q[0]);
  pc_x = sin(q[0]);
  qc_x = sin(q[0]);
  rc_x = sin(q[0]);
  sc_x = cos(q[0]);
  tc_x = cos(q[0]);
  uc_x = sin(q[0]);
  vc_x = sin(q[0]);
  wc_x = cos(q[0]);
  xc_x = cos(q[0]);
  yc_x = sin(q[0]);
  ad_x = sin(q[0]);
  bd_x = cos(q[0]);
  cd_x = cos(q[0]);
  dd_x = sin(q[0]);
  ed_x = sin(q[0]);
  fd_x = cos(q[0]);
  gd_x = cos(q[0]);
  hd_x = cos(q[0]);
  id_x = sin(q[0]);
  jd_x = sin(q[0]);
  kd_x = sin(q[0]);
  ld_x = cos(q[0]);
  md_x = cos(q[0]);
  nd_x = sin(q[0]);
  od_x = sin(q[0]);
  pd_x = cos(q[0]);
  qd_x = cos(q[0]);
  qDot[0] = (uDot[0] * (cos(q[2] + zeta) * sin(q[2] + zeta) * sin(q[0]) * (L5 *
    L5) - cos(q[1]) * sin(q[0]) * sin(q[1]) * (L3 * L3)) /
             (((((((((((((((((((rt_powd_snf(L3, 3.0) * (unusedU0 * unusedU0) *
    (unusedU1 * unusedU1) * sin(q[1]) - rt_powd_snf(L5, 3.0) * (unusedU2 *
    unusedU2) * sin(q[2] + zeta) * (unusedU3 * unusedU3)) + L2 * (L3 * L3) * cos
                               (q[1]) * (unusedU4 * unusedU4) * sin(q[1])) - L3 *
    L3 * L7 * cos(q[1]) * (unusedU5 * unusedU5) * sin(q[1])) - L3 * (L5 * L5) *
                             (unusedU6 * unusedU6) * (unusedU7 * unusedU7) * sin
                             (q[1])) + L3 * L3 * L5 * sin(q[2] + zeta) *
    (unusedU8 * unusedU8) * (unusedU9 * unusedU9)) - L2 * (L5 * L5) * cos(q[2] +
    zeta) * sin(q[2] + zeta) * (x * x)) + L5 * L5 * L7 * cos(q[2] + zeta) * sin
    (q[2] + zeta) * (b_x * b_x)) + L3 * L3 * L4 * cos(q[1]) * (c_x * c_x) * sin
    (q[1]) * cos(zeta)) + L3 * (L5 * L5) * cos(q[2] + zeta) * sin(q[2] + zeta) *
                        (d_x * d_x) * cos(q[1])) - L3 * (L5 * L5) * cos(q[2] +
    zeta) * sin(q[2] + zeta) * cos(q[1]) * (e_x * e_x)) - L3 * L3 * L5 * cos(q[2]
    + zeta) * (f_x * f_x) * cos(q[1]) * sin(q[1])) - L4 * (L5 * L5) * cos(q[2] +
    zeta) * sin(q[2] + zeta) * (g_x * g_x) * cos(zeta)) + L3 * L3 * L5 * cos(q[2]
    + zeta) * cos(q[1]) * (h_x * h_x) * sin(q[1])) - L2 * L3 * L5 * cos(q[2] +
    zeta) * (i_x * i_x) * sin(q[1])) + L2 * L3 * L5 * sin(q[2] + zeta) * (j_x *
    j_x) * cos(q[1])) + L3 * L5 * L7 * cos(q[2] + zeta) * (k_x * k_x) * sin(q[1]))
                - L3 * L5 * L7 * sin(q[2] + zeta) * (l_x * l_x) * cos(q[1])) -
               L3 * L4 * L5 * cos(q[2] + zeta) * (m_x * m_x) * sin(q[1]) * cos
               (zeta)) + L3 * L4 * L5 * sin(q[2] + zeta) * (n_x * n_x) * cos(q[1])
              * cos(zeta)) - uDot[2] * (L5 * L5 * (o_x * o_x) * cos(q[0]) * sin
              (q[0]) - L3 * L3 * cos(q[0]) * sin(q[0]) * (p_x * p_x)) /
             (((((((((((((((((((rt_powd_snf(L3, 3.0) * (q_x * q_x) * (r_x * r_x)
    * sin(q[1]) - rt_powd_snf(L5, 3.0) * (s_x * s_x) * sin(q[2] + zeta) * (t_x *
    t_x)) + L2 * (L3 * L3) * cos(q[1]) * (u_x * u_x) * sin(q[1])) - L3 * L3 * L7
    * cos(q[1]) * (v_x * v_x) * sin(q[1])) - L3 * (L5 * L5) * (w_x * w_x) * (x_x
    * x_x) * sin(q[1])) + L3 * L3 * L5 * sin(q[2] + zeta) * (y_x * y_x) * (ab_x *
    ab_x)) - L2 * (L5 * L5) * cos(q[2] + zeta) * sin(q[2] + zeta) * (bb_x * bb_x))
    + L5 * L5 * L7 * cos(q[2] + zeta) * sin(q[2] + zeta) * (cb_x * cb_x)) + L3 *
    L3 * L4 * cos(q[1]) * (db_x * db_x) * sin(q[1]) * cos(zeta)) + L3 * (L5 * L5)
                        * cos(q[2] + zeta) * sin(q[2] + zeta) * (eb_x * eb_x) *
                        cos(q[1])) - L3 * (L5 * L5) * cos(q[2] + zeta) * sin(q[2]
    + zeta) * cos(q[1]) * (fb_x * fb_x)) - L3 * L3 * L5 * cos(q[2] + zeta) *
                      (gb_x * gb_x) * cos(q[1]) * sin(q[1])) - L4 * (L5 * L5) *
                     cos(q[2] + zeta) * sin(q[2] + zeta) * (hb_x * hb_x) * cos
                     (zeta)) + L3 * L3 * L5 * cos(q[2] + zeta) * cos(q[1]) *
                    (ib_x * ib_x) * sin(q[1])) - L2 * L3 * L5 * cos(q[2] + zeta)
                   * (jb_x * jb_x) * sin(q[1])) + L2 * L3 * L5 * sin(q[2] + zeta)
                  * (kb_x * kb_x) * cos(q[1])) + L3 * L5 * L7 * cos(q[2] + zeta)
                 * (lb_x * lb_x) * sin(q[1])) - L3 * L5 * L7 * sin(q[2] + zeta) *
                (mb_x * mb_x) * cos(q[1])) - L3 * L4 * L5 * cos(q[2] + zeta) *
               (nb_x * nb_x) * sin(q[1]) * cos(zeta)) + L3 * L4 * L5 * sin(q[2]
    + zeta) * (ob_x * ob_x) * cos(q[1]) * cos(zeta))) - uDot[1] * (L3 * L5 * cos
    (q[2] + zeta) * cos(q[0]) * sin(q[1]) - L3 * L5 * sin(q[2] + zeta) * cos(q[0])
    * cos(q[1])) / (((((((((((((((((((rt_powd_snf(L3, 3.0) * (pb_x * pb_x) *
    (qb_x * qb_x) * sin(q[1]) - rt_powd_snf(L5, 3.0) * (rb_x * rb_x) * sin(q[2]
    + zeta) * (sb_x * sb_x)) + L2 * (L3 * L3) * cos(q[1]) * (tb_x * tb_x) * sin
    (q[1])) - L3 * L3 * L7 * cos(q[1]) * (ub_x * ub_x) * sin(q[1])) - L3 * (L5 *
    L5) * (vb_x * vb_x) * (wb_x * wb_x) * sin(q[1])) + L3 * L3 * L5 * sin(q[2] +
    zeta) * (xb_x * xb_x) * (yb_x * yb_x)) - L2 * (L5 * L5) * cos(q[2] + zeta) *
    sin(q[2] + zeta) * (ac_x * ac_x)) + L5 * L5 * L7 * cos(q[2] + zeta) * sin(q
    [2] + zeta) * (bc_x * bc_x)) + L3 * L3 * L4 * cos(q[1]) * (cc_x * cc_x) *
    sin(q[1]) * cos(zeta)) + L3 * (L5 * L5) * cos(q[2] + zeta) * sin(q[2] + zeta)
    * (dc_x * dc_x) * cos(q[1])) - L3 * (L5 * L5) * cos(q[2] + zeta) * sin(q[2]
    + zeta) * cos(q[1]) * (ec_x * ec_x)) - L3 * L3 * L5 * cos(q[2] + zeta) *
    (fc_x * fc_x) * cos(q[1]) * sin(q[1])) - L4 * (L5 * L5) * cos(q[2] + zeta) *
    sin(q[2] + zeta) * (gc_x * gc_x) * cos(zeta)) + L3 * L3 * L5 * cos(q[2] +
    zeta) * cos(q[1]) * (hc_x * hc_x) * sin(q[1])) - L2 * L3 * L5 * cos(q[2] +
    zeta) * (ic_x * ic_x) * sin(q[1])) + L2 * L3 * L5 * sin(q[2] + zeta) * (jc_x
    * jc_x) * cos(q[1])) + L3 * L5 * L7 * cos(q[2] + zeta) * (kc_x * kc_x) * sin
                       (q[1])) - L3 * L5 * L7 * sin(q[2] + zeta) * (lc_x * lc_x)
                      * cos(q[1])) - L3 * L4 * L5 * cos(q[2] + zeta) * (mc_x *
    mc_x) * sin(q[1]) * cos(zeta)) + L3 * L4 * L5 * sin(q[2] + zeta) * (nc_x *
    nc_x) * cos(q[1]) * cos(zeta));
  qDot[1] = (uDot[2] * (L5 * sin(q[2] + zeta) * (oc_x * oc_x) + L3 * sin(q[1]) *
                        (pc_x * pc_x)) / (((L5 * L5 * cos(q[2] + zeta) * sin(q[2]
    + zeta) * (qc_x * qc_x) - L3 * L3 * cos(q[1]) * (rc_x * rc_x) * sin(q[1])) +
    L3 * L5 * cos(q[2] + zeta) * (sc_x * sc_x) * sin(q[1])) - L3 * L5 * sin(q[2]
    + zeta) * (tc_x * tc_x) * cos(q[1])) - L5 * cos(q[2] + zeta) * cos(q[0]) *
             uDot[0] / (((L5 * L5 * cos(q[2] + zeta) * sin(q[2] + zeta) * (uc_x *
    uc_x) - L3 * L3 * cos(q[1]) * (vc_x * vc_x) * sin(q[1])) + L3 * L5 * cos(q[2]
    + zeta) * (wc_x * wc_x) * sin(q[1])) - L3 * L5 * sin(q[2] + zeta) * (xc_x *
    xc_x) * cos(q[1]))) - L5 * cos(q[2] + zeta) * sin(q[0]) * uDot[1] / (((L5 *
    L5 * cos(q[2] + zeta) * sin(q[2] + zeta) * (yc_x * yc_x) - L3 * L3 * cos(q[1])
    * (ad_x * ad_x) * sin(q[1])) + L3 * L5 * cos(q[2] + zeta) * (bd_x * bd_x) *
    sin(q[1])) - L3 * L5 * sin(q[2] + zeta) * (cd_x * cd_x) * cos(q[1]));
  qDot[2] = (L3 * cos(q[0]) * cos(q[1]) * uDot[0] / (((L5 * L5 * cos(q[2] + zeta)
    * sin(q[2] + zeta) * (dd_x * dd_x) - L3 * L3 * cos(q[1]) * (ed_x * ed_x) *
    sin(q[1])) + L3 * L5 * cos(q[2] + zeta) * (fd_x * fd_x) * sin(q[1])) - L3 *
              L5 * sin(q[2] + zeta) * (gd_x * gd_x) * cos(q[1])) - uDot[2] * (L3
              * sin(q[1]) * (hd_x * hd_x) + L5 * sin(q[2] + zeta) * (id_x * id_x))
             / (((L5 * L5 * cos(q[2] + zeta) * sin(q[2] + zeta) * (jd_x * jd_x)
                  - L3 * L3 * cos(q[1]) * (kd_x * kd_x) * sin(q[1])) + L3 * L5 *
                 cos(q[2] + zeta) * (ld_x * ld_x) * sin(q[1])) - L3 * L5 * sin
                (q[2] + zeta) * (md_x * md_x) * cos(q[1]))) + L3 * cos(q[1]) *
    sin(q[0]) * uDot[1] / (((L5 * L5 * cos(q[2] + zeta) * sin(q[2] + zeta) *
    (nd_x * nd_x) - L3 * L3 * cos(q[1]) * (od_x * od_x) * sin(q[1])) + L3 * L5 *
    cos(q[2] + zeta) * (pd_x * pd_x) * sin(q[1])) - L3 * L5 * sin(q[2] + zeta) *
                           (qd_x * qd_x) * cos(q[1]));
}

//
// File trailer for sherpaTTIKVel.cpp
//
// [EOF]
//
