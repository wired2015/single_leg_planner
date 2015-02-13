/*
 * sherpaTTIKVel.c
 *
 * Code generation for function 'sherpaTTIKVel'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "sherpaTTIKVel.h"
#include <stdio.h>

/* Function Definitions */
void sherpaTTIKVel(const real_T uDot[3], const real_T q[3], real_T kC_l2, real_T
                   kC_l3, real_T kC_l4, real_T kC_l5, real_T kC_l7, real_T
                   kC_zeta, real_T qDot[3])
{
  real_T x;
  real_T b_x;
  real_T c_x;
  real_T d_x;
  real_T e_x;
  real_T f_x;
  real_T g_x;
  real_T h_x;
  real_T i_x;
  real_T j_x;
  real_T k_x;
  real_T l_x;
  real_T m_x;
  real_T n_x;
  real_T o_x;
  real_T p_x;
  real_T q_x;
  real_T r_x;
  real_T s_x;
  real_T t_x;
  real_T u_x;
  real_T v_x;
  real_T w_x;
  real_T x_x;
  real_T y_x;
  real_T ab_x;
  real_T bb_x;
  real_T cb_x;
  real_T db_x;
  real_T eb_x;
  real_T fb_x;
  real_T gb_x;
  real_T hb_x;
  real_T ib_x;
  real_T jb_x;
  real_T kb_x;
  real_T lb_x;
  real_T mb_x;
  real_T nb_x;
  real_T ob_x;
  real_T pb_x;
  real_T qb_x;
  real_T rb_x;
  real_T sb_x;
  real_T tb_x;
  real_T ub_x;
  real_T vb_x;
  real_T wb_x;
  real_T xb_x;
  real_T yb_x;
  real_T ac_x;
  real_T bc_x;
  real_T cc_x;
  real_T dc_x;
  real_T ec_x;
  real_T fc_x;
  real_T gc_x;
  real_T hc_x;
  real_T ic_x;
  real_T jc_x;
  real_T kc_x;
  real_T lc_x;
  real_T mc_x;
  real_T nc_x;
  real_T oc_x;
  real_T pc_x;
  real_T qc_x;
  real_T rc_x;
  real_T sc_x;
  real_T tc_x;
  real_T uc_x;
  real_T vc_x;
  real_T wc_x;
  real_T xc_x;
  real_T yc_x;
  real_T ad_x;
  real_T bd_x;
  real_T cd_x;
  real_T dd_x;
  real_T ed_x;
  real_T fd_x;
  real_T gd_x;
  real_T hd_x;
  real_T id_x;
  real_T jd_x;
  real_T kd_x;
  real_T ld_x;
  real_T md_x;
  real_T nd_x;
  real_T od_x;
  real_T pd_x;
  real_T qd_x;
  real_T rd_x;
  real_T sd_x;
  real_T td_x;
  real_T ud_x;
  real_T vd_x;
  real_T wd_x;
  real_T xd_x;
  real_T yd_x;
  real_T ae_x;
  real_T be_x;

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  x = muDoubleScalarCos(q[1]);
  b_x = muDoubleScalarSin(q[0]);
  c_x = muDoubleScalarCos(q[2] + kC_zeta);
  d_x = muDoubleScalarSin(q[0]);
  e_x = muDoubleScalarSin(q[0]);
  f_x = muDoubleScalarSin(q[0]);
  g_x = muDoubleScalarCos(q[2] + kC_zeta);
  h_x = muDoubleScalarCos(q[0]);
  i_x = muDoubleScalarCos(q[0]);
  j_x = muDoubleScalarCos(q[1]);
  k_x = muDoubleScalarSin(q[0]);
  l_x = muDoubleScalarSin(q[0]);
  m_x = muDoubleScalarSin(q[0]);
  n_x = muDoubleScalarCos(q[0]);
  o_x = muDoubleScalarSin(q[0]);
  p_x = muDoubleScalarCos(q[0]);
  q_x = muDoubleScalarSin(q[0]);
  r_x = muDoubleScalarSin(q[0]);
  s_x = muDoubleScalarCos(q[0]);
  t_x = muDoubleScalarCos(q[0]);
  u_x = muDoubleScalarCos(q[0]);
  v_x = muDoubleScalarCos(q[0]);
  w_x = muDoubleScalarCos(q[0]);
  x_x = muDoubleScalarCos(q[0]);
  y_x = muDoubleScalarSin(q[2] + kC_zeta);
  ab_x = muDoubleScalarSin(q[1]);
  bb_x = muDoubleScalarCos(q[1]);
  cb_x = muDoubleScalarSin(q[0]);
  db_x = muDoubleScalarCos(q[2] + kC_zeta);
  eb_x = muDoubleScalarSin(q[0]);
  fb_x = muDoubleScalarSin(q[0]);
  gb_x = muDoubleScalarSin(q[0]);
  hb_x = muDoubleScalarCos(q[2] + kC_zeta);
  ib_x = muDoubleScalarCos(q[0]);
  jb_x = muDoubleScalarCos(q[0]);
  kb_x = muDoubleScalarCos(q[1]);
  lb_x = muDoubleScalarSin(q[0]);
  mb_x = muDoubleScalarSin(q[0]);
  nb_x = muDoubleScalarSin(q[0]);
  ob_x = muDoubleScalarCos(q[0]);
  pb_x = muDoubleScalarSin(q[0]);
  qb_x = muDoubleScalarCos(q[0]);
  rb_x = muDoubleScalarSin(q[0]);
  sb_x = muDoubleScalarSin(q[0]);
  tb_x = muDoubleScalarCos(q[0]);
  ub_x = muDoubleScalarCos(q[0]);
  vb_x = muDoubleScalarCos(q[0]);
  wb_x = muDoubleScalarCos(q[0]);
  xb_x = muDoubleScalarCos(q[0]);
  yb_x = muDoubleScalarCos(q[0]);
  ac_x = muDoubleScalarCos(q[1]);
  bc_x = muDoubleScalarSin(q[0]);
  cc_x = muDoubleScalarCos(q[2] + kC_zeta);
  dc_x = muDoubleScalarSin(q[0]);
  ec_x = muDoubleScalarSin(q[0]);
  fc_x = muDoubleScalarSin(q[0]);
  gc_x = muDoubleScalarCos(q[2] + kC_zeta);
  hc_x = muDoubleScalarCos(q[0]);
  ic_x = muDoubleScalarCos(q[0]);
  jc_x = muDoubleScalarCos(q[1]);
  kc_x = muDoubleScalarSin(q[0]);
  lc_x = muDoubleScalarSin(q[0]);
  mc_x = muDoubleScalarSin(q[0]);
  nc_x = muDoubleScalarCos(q[0]);
  oc_x = muDoubleScalarSin(q[0]);
  pc_x = muDoubleScalarCos(q[0]);
  qc_x = muDoubleScalarSin(q[0]);
  rc_x = muDoubleScalarSin(q[0]);
  sc_x = muDoubleScalarCos(q[0]);
  tc_x = muDoubleScalarCos(q[0]);
  uc_x = muDoubleScalarCos(q[0]);
  vc_x = muDoubleScalarCos(q[0]);
  wc_x = muDoubleScalarCos(q[0]);
  xc_x = muDoubleScalarCos(q[0]);
  yc_x = muDoubleScalarCos(q[0]);
  ad_x = muDoubleScalarSin(q[0]);
  bd_x = muDoubleScalarSin(q[0]);
  cd_x = muDoubleScalarSin(q[0]);
  dd_x = muDoubleScalarCos(q[0]);
  ed_x = muDoubleScalarCos(q[0]);
  fd_x = muDoubleScalarSin(q[0]);
  gd_x = muDoubleScalarSin(q[0]);
  hd_x = muDoubleScalarCos(q[0]);
  id_x = muDoubleScalarCos(q[0]);
  jd_x = muDoubleScalarSin(q[0]);
  kd_x = muDoubleScalarSin(q[0]);
  ld_x = muDoubleScalarCos(q[0]);
  md_x = muDoubleScalarCos(q[0]);
  nd_x = muDoubleScalarSin(q[0]);
  od_x = muDoubleScalarSin(q[0]);
  pd_x = muDoubleScalarCos(q[0]);
  qd_x = muDoubleScalarCos(q[0]);
  rd_x = muDoubleScalarCos(q[0]);
  sd_x = muDoubleScalarSin(q[0]);
  td_x = muDoubleScalarSin(q[0]);
  ud_x = muDoubleScalarSin(q[0]);
  vd_x = muDoubleScalarCos(q[0]);
  wd_x = muDoubleScalarCos(q[0]);
  xd_x = muDoubleScalarSin(q[0]);
  yd_x = muDoubleScalarSin(q[0]);
  ae_x = muDoubleScalarCos(q[0]);
  be_x = muDoubleScalarCos(q[0]);
  qDot[0] = (uDot[0] * (muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q
    [2] + kC_zeta) * muDoubleScalarSin(q[0]) * (kC_l5 * kC_l5) -
                        muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[0]) *
                        muDoubleScalarSin(q[1]) * (kC_l3 * kC_l3)) /
             (((((((((((((((((((muDoubleScalarPower(kC_l3, 3.0) * (x * x) * (b_x
    * b_x) * muDoubleScalarSin(q[1]) - muDoubleScalarPower(kC_l5, 3.0) * (c_x *
    c_x) * muDoubleScalarSin(q[2] + kC_zeta) * (d_x * d_x)) + kC_l2 * (kC_l3 *
    kC_l3) * muDoubleScalarCos(q[1]) * (e_x * e_x) * muDoubleScalarSin(q[1])) -
    kC_l3 * kC_l3 * kC_l7 * muDoubleScalarCos(q[1]) * (f_x * f_x) *
    muDoubleScalarSin(q[1])) - kC_l3 * (kC_l5 * kC_l5) * (g_x * g_x) * (h_x *
    h_x) * muDoubleScalarSin(q[1])) + kC_l3 * kC_l3 * kC_l5 * muDoubleScalarSin
    (q[2] + kC_zeta) * (i_x * i_x) * (j_x * j_x)) - kC_l2 * (kC_l5 * kC_l5) *
    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) * (k_x
    * k_x)) + kC_l5 * kC_l5 * kC_l7 * muDoubleScalarCos(q[2] + kC_zeta) *
    muDoubleScalarSin(q[2] + kC_zeta) * (l_x * l_x)) + kC_l3 * kC_l3 * kC_l4 *
    muDoubleScalarCos(q[1]) * (m_x * m_x) * muDoubleScalarSin(q[1]) *
    muDoubleScalarCos(kC_zeta)) + kC_l3 * (kC_l5 * kC_l5) * muDoubleScalarCos(q
    [2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) * (n_x * n_x) *
                        muDoubleScalarCos(q[1])) - kC_l3 * (kC_l5 * kC_l5) *
                       muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2]
    + kC_zeta) * muDoubleScalarCos(q[1]) * (o_x * o_x)) - kC_l3 * kC_l3 * kC_l5 *
                      muDoubleScalarCos(q[2] + kC_zeta) * (p_x * p_x) *
                      muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[1])) - kC_l4
                     * (kC_l5 * kC_l5) * muDoubleScalarCos(q[2] + kC_zeta) *
                     muDoubleScalarSin(q[2] + kC_zeta) * (q_x * q_x) *
                     muDoubleScalarCos(kC_zeta)) + kC_l3 * kC_l3 * kC_l5 *
                    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarCos(q[1]) *
                    (r_x * r_x) * muDoubleScalarSin(q[1])) - kC_l2 * kC_l3 *
                   kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (s_x * s_x) *
                   muDoubleScalarSin(q[1])) + kC_l2 * kC_l3 * kC_l5 *
                  muDoubleScalarSin(q[2] + kC_zeta) * (t_x * t_x) *
                  muDoubleScalarCos(q[1])) + kC_l3 * kC_l5 * kC_l7 *
                 muDoubleScalarCos(q[2] + kC_zeta) * (u_x * u_x) *
                 muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 * kC_l7 *
                muDoubleScalarSin(q[2] + kC_zeta) * (v_x * v_x) *
                muDoubleScalarCos(q[1])) - kC_l3 * kC_l4 * kC_l5 *
               muDoubleScalarCos(q[2] + kC_zeta) * (w_x * w_x) *
               muDoubleScalarSin(q[1]) * muDoubleScalarCos(kC_zeta)) + kC_l3 *
              kC_l4 * kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (x_x * x_x) *
              muDoubleScalarCos(q[1]) * muDoubleScalarCos(kC_zeta)) - uDot[2] *
             (kC_l5 * kC_l5 * (y_x * y_x) * muDoubleScalarCos(q[0]) *
              muDoubleScalarSin(q[0]) - kC_l3 * kC_l3 * muDoubleScalarCos(q[0]) *
              muDoubleScalarSin(q[0]) * (ab_x * ab_x)) /
             (((((((((((((((((((muDoubleScalarPower(kC_l3, 3.0) * (bb_x * bb_x) *
    (cb_x * cb_x) * muDoubleScalarSin(q[1]) - muDoubleScalarPower(kC_l5, 3.0) *
    (db_x * db_x) * muDoubleScalarSin(q[2] + kC_zeta) * (eb_x * eb_x)) + kC_l2 *
                               (kC_l3 * kC_l3) * muDoubleScalarCos(q[1]) * (fb_x
    * fb_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l3 * kC_l7 *
    muDoubleScalarCos(q[1]) * (gb_x * gb_x) * muDoubleScalarSin(q[1])) - kC_l3 *
                             (kC_l5 * kC_l5) * (hb_x * hb_x) * (ib_x * ib_x) *
    muDoubleScalarSin(q[1])) + kC_l3 * kC_l3 * kC_l5 * muDoubleScalarSin(q[2] +
    kC_zeta) * (jb_x * jb_x) * (kb_x * kb_x)) - kC_l2 * (kC_l5 * kC_l5) *
    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) *
    (lb_x * lb_x)) + kC_l5 * kC_l5 * kC_l7 * muDoubleScalarCos(q[2] + kC_zeta) *
    muDoubleScalarSin(q[2] + kC_zeta) * (mb_x * mb_x)) + kC_l3 * kC_l3 * kC_l4 *
    muDoubleScalarCos(q[1]) * (nb_x * nb_x) * muDoubleScalarSin(q[1]) *
    muDoubleScalarCos(kC_zeta)) + kC_l3 * (kC_l5 * kC_l5) * muDoubleScalarCos(q
    [2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) * (ob_x * ob_x) *
                        muDoubleScalarCos(q[1])) - kC_l3 * (kC_l5 * kC_l5) *
                       muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2]
    + kC_zeta) * muDoubleScalarCos(q[1]) * (pb_x * pb_x)) - kC_l3 * kC_l3 *
                      kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (qb_x * qb_x) *
                      muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[1])) - kC_l4
                     * (kC_l5 * kC_l5) * muDoubleScalarCos(q[2] + kC_zeta) *
                     muDoubleScalarSin(q[2] + kC_zeta) * (rb_x * rb_x) *
                     muDoubleScalarCos(kC_zeta)) + kC_l3 * kC_l3 * kC_l5 *
                    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarCos(q[1]) *
                    (sb_x * sb_x) * muDoubleScalarSin(q[1])) - kC_l2 * kC_l3 *
                   kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (tb_x * tb_x) *
                   muDoubleScalarSin(q[1])) + kC_l2 * kC_l3 * kC_l5 *
                  muDoubleScalarSin(q[2] + kC_zeta) * (ub_x * ub_x) *
                  muDoubleScalarCos(q[1])) + kC_l3 * kC_l5 * kC_l7 *
                 muDoubleScalarCos(q[2] + kC_zeta) * (vb_x * vb_x) *
                 muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 * kC_l7 *
                muDoubleScalarSin(q[2] + kC_zeta) * (wb_x * wb_x) *
                muDoubleScalarCos(q[1])) - kC_l3 * kC_l4 * kC_l5 *
               muDoubleScalarCos(q[2] + kC_zeta) * (xb_x * xb_x) *
               muDoubleScalarSin(q[1]) * muDoubleScalarCos(kC_zeta)) + kC_l3 *
              kC_l4 * kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (yb_x * yb_x) *
              muDoubleScalarCos(q[1]) * muDoubleScalarCos(kC_zeta))) - uDot[1] *
    (kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarCos(q[0])
     * muDoubleScalarSin(q[1]) - kC_l3 * kC_l5 * muDoubleScalarSin(q[2] +
      kC_zeta) * muDoubleScalarCos(q[0]) * muDoubleScalarCos(q[1])) /
    (((((((((((((((((((muDoubleScalarPower(kC_l3, 3.0) * (ac_x * ac_x) * (bc_x *
    bc_x) * muDoubleScalarSin(q[1]) - muDoubleScalarPower(kC_l5, 3.0) * (cc_x *
    cc_x) * muDoubleScalarSin(q[2] + kC_zeta) * (dc_x * dc_x)) + kC_l2 * (kC_l3 *
    kC_l3) * muDoubleScalarCos(q[1]) * (ec_x * ec_x) * muDoubleScalarSin(q[1]))
                     - kC_l3 * kC_l3 * kC_l7 * muDoubleScalarCos(q[1]) * (fc_x *
    fc_x) * muDoubleScalarSin(q[1])) - kC_l3 * (kC_l5 * kC_l5) * (gc_x * gc_x) *
                    (hc_x * hc_x) * muDoubleScalarSin(q[1])) + kC_l3 * kC_l3 *
                   kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (ic_x * ic_x) *
                   (jc_x * jc_x)) - kC_l2 * (kC_l5 * kC_l5) * muDoubleScalarCos
                  (q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) * (kc_x *
    kc_x)) + kC_l5 * kC_l5 * kC_l7 * muDoubleScalarCos(q[2] + kC_zeta) *
                 muDoubleScalarSin(q[2] + kC_zeta) * (lc_x * lc_x)) + kC_l3 *
                kC_l3 * kC_l4 * muDoubleScalarCos(q[1]) * (mc_x * mc_x) *
                muDoubleScalarSin(q[1]) * muDoubleScalarCos(kC_zeta)) + kC_l3 *
               (kC_l5 * kC_l5) * muDoubleScalarCos(q[2] + kC_zeta) *
               muDoubleScalarSin(q[2] + kC_zeta) * (nc_x * nc_x) *
               muDoubleScalarCos(q[1])) - kC_l3 * (kC_l5 * kC_l5) *
              muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] +
    kC_zeta) * muDoubleScalarCos(q[1]) * (oc_x * oc_x)) - kC_l3 * kC_l3 * kC_l5 *
             muDoubleScalarCos(q[2] + kC_zeta) * (pc_x * pc_x) *
             muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[1])) - kC_l4 * (kC_l5
             * kC_l5) * muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q
             [2] + kC_zeta) * (qc_x * qc_x) * muDoubleScalarCos(kC_zeta)) +
           kC_l3 * kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) *
           muDoubleScalarCos(q[1]) * (rc_x * rc_x) * muDoubleScalarSin(q[1])) -
          kC_l2 * kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (sc_x *
           sc_x) * muDoubleScalarSin(q[1])) + kC_l2 * kC_l3 * kC_l5 *
         muDoubleScalarSin(q[2] + kC_zeta) * (tc_x * tc_x) * muDoubleScalarCos
         (q[1])) + kC_l3 * kC_l5 * kC_l7 * muDoubleScalarCos(q[2] + kC_zeta) *
        (uc_x * uc_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 * kC_l7 *
       muDoubleScalarSin(q[2] + kC_zeta) * (vc_x * vc_x) * muDoubleScalarCos(q[1]))
      - kC_l3 * kC_l4 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (wc_x * wc_x)
      * muDoubleScalarSin(q[1]) * muDoubleScalarCos(kC_zeta)) + kC_l3 * kC_l4 *
     kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (xc_x * xc_x) *
     muDoubleScalarCos(q[1]) * muDoubleScalarCos(kC_zeta));
  qDot[1] = (uDot[2] * (kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (yc_x * yc_x)
                        + kC_l3 * muDoubleScalarSin(q[1]) * (ad_x * ad_x)) /
             (((kC_l5 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) *
                muDoubleScalarSin(q[2] + kC_zeta) * (bd_x * bd_x) - kC_l3 *
                kC_l3 * muDoubleScalarCos(q[1]) * (cd_x * cd_x) *
                muDoubleScalarSin(q[1])) + kC_l3 * kC_l5 * muDoubleScalarCos(q[2]
    + kC_zeta) * (dd_x * dd_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 *
              muDoubleScalarSin(q[2] + kC_zeta) * (ed_x * ed_x) *
              muDoubleScalarCos(q[1])) - kC_l5 * muDoubleScalarCos(q[2] +
              kC_zeta) * muDoubleScalarCos(q[0]) * uDot[0] / (((kC_l5 * kC_l5 *
    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) *
    (fd_x * fd_x) - kC_l3 * kC_l3 * muDoubleScalarCos(q[1]) * (gd_x * gd_x) *
    muDoubleScalarSin(q[1])) + kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta)
    * (hd_x * hd_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 *
              muDoubleScalarSin(q[2] + kC_zeta) * (id_x * id_x) *
              muDoubleScalarCos(q[1]))) - kC_l5 * muDoubleScalarCos(q[2] +
    kC_zeta) * muDoubleScalarSin(q[0]) * uDot[1] / (((kC_l5 * kC_l5 *
    muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) *
    (jd_x * jd_x) - kC_l3 * kC_l3 * muDoubleScalarCos(q[1]) * (kd_x * kd_x) *
    muDoubleScalarSin(q[1])) + kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta)
    * (ld_x * ld_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 *
    muDoubleScalarSin(q[2] + kC_zeta) * (md_x * md_x) * muDoubleScalarCos(q[1]));
  qDot[2] = (kC_l3 * muDoubleScalarCos(q[0]) * muDoubleScalarCos(q[1]) * uDot[0]
             / (((kC_l5 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) *
                  muDoubleScalarSin(q[2] + kC_zeta) * (nd_x * nd_x) - kC_l3 *
                  kC_l3 * muDoubleScalarCos(q[1]) * (od_x * od_x) *
                  muDoubleScalarSin(q[1])) + kC_l3 * kC_l5 * muDoubleScalarCos
                 (q[2] + kC_zeta) * (pd_x * pd_x) * muDoubleScalarSin(q[1])) -
                kC_l3 * kC_l5 * muDoubleScalarSin(q[2] + kC_zeta) * (qd_x * qd_x)
                * muDoubleScalarCos(q[1])) - uDot[2] * (kC_l3 *
              muDoubleScalarSin(q[1]) * (rd_x * rd_x) + kC_l5 *
              muDoubleScalarSin(q[2] + kC_zeta) * (sd_x * sd_x)) / (((kC_l5 *
    kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta)
    * (td_x * td_x) - kC_l3 * kC_l3 * muDoubleScalarCos(q[1]) * (ud_x * ud_x) *
    muDoubleScalarSin(q[1])) + kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta)
    * (vd_x * vd_x) * muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 *
              muDoubleScalarSin(q[2] + kC_zeta) * (wd_x * wd_x) *
              muDoubleScalarCos(q[1]))) + kC_l3 * muDoubleScalarCos(q[1]) *
    muDoubleScalarSin(q[0]) * uDot[1] / (((kC_l5 * kC_l5 * muDoubleScalarCos(q[2]
    + kC_zeta) * muDoubleScalarSin(q[2] + kC_zeta) * (xd_x * xd_x) - kC_l3 *
    kC_l3 * muDoubleScalarCos(q[1]) * (yd_x * yd_x) * muDoubleScalarSin(q[1])) +
    kC_l3 * kC_l5 * muDoubleScalarCos(q[2] + kC_zeta) * (ae_x * ae_x) *
    muDoubleScalarSin(q[1])) - kC_l3 * kC_l5 * muDoubleScalarSin(q[2] + kC_zeta)
    * (be_x * be_x) * muDoubleScalarCos(q[1]));
}

/* End of code generation (sherpaTTIKVel.c) */
