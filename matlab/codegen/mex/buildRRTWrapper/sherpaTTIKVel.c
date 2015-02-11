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
#include "extractKinematicConstants.h"
#include <stdio.h>

/* Function Definitions */
void sherpaTTIKVel(const real_T uDot[3], const real_T q[3], const real_T
                   kinematicConst[16], real_T qDot[3])
{
  real_T unusedU9;
  real_T unusedU8;
  real_T unusedU7;
  real_T unusedU6;
  real_T unusedU5;
  real_T unusedU4;
  real_T unusedU3;
  real_T zeta;
  real_T unusedU2;
  real_T L7;
  real_T unusedU1;
  real_T L5;
  real_T L4;
  real_T L3;
  real_T L2;
  real_T unusedU0;
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

  /* sherpaTTFKVel.m */
  /* author: wreid */
  /* date: 20150122 */
  /* sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics. */
  extractKinematicConstants(kinematicConst, &unusedU0, &L2, &L3, &L4, &L5,
    &unusedU1, &L7, &unusedU2, &zeta, &unusedU3, &unusedU4, &unusedU5, &unusedU6,
    &unusedU7, &unusedU8, &unusedU9);
  unusedU0 = muDoubleScalarCos(q[1]);
  unusedU1 = muDoubleScalarSin(q[0]);
  unusedU2 = muDoubleScalarCos(q[2] + zeta);
  unusedU3 = muDoubleScalarSin(q[0]);
  unusedU4 = muDoubleScalarSin(q[0]);
  unusedU5 = muDoubleScalarSin(q[0]);
  unusedU6 = muDoubleScalarCos(q[2] + zeta);
  unusedU7 = muDoubleScalarCos(q[0]);
  unusedU8 = muDoubleScalarCos(q[0]);
  unusedU9 = muDoubleScalarCos(q[1]);
  x = muDoubleScalarSin(q[0]);
  b_x = muDoubleScalarSin(q[0]);
  c_x = muDoubleScalarSin(q[0]);
  d_x = muDoubleScalarCos(q[0]);
  e_x = muDoubleScalarSin(q[0]);
  f_x = muDoubleScalarCos(q[0]);
  g_x = muDoubleScalarSin(q[0]);
  h_x = muDoubleScalarSin(q[0]);
  i_x = muDoubleScalarCos(q[0]);
  j_x = muDoubleScalarCos(q[0]);
  k_x = muDoubleScalarCos(q[0]);
  l_x = muDoubleScalarCos(q[0]);
  m_x = muDoubleScalarCos(q[0]);
  n_x = muDoubleScalarCos(q[0]);
  o_x = muDoubleScalarSin(q[2] + zeta);
  p_x = muDoubleScalarSin(q[1]);
  q_x = muDoubleScalarCos(q[1]);
  r_x = muDoubleScalarSin(q[0]);
  s_x = muDoubleScalarCos(q[2] + zeta);
  t_x = muDoubleScalarSin(q[0]);
  u_x = muDoubleScalarSin(q[0]);
  v_x = muDoubleScalarSin(q[0]);
  w_x = muDoubleScalarCos(q[2] + zeta);
  x_x = muDoubleScalarCos(q[0]);
  y_x = muDoubleScalarCos(q[0]);
  ab_x = muDoubleScalarCos(q[1]);
  bb_x = muDoubleScalarSin(q[0]);
  cb_x = muDoubleScalarSin(q[0]);
  db_x = muDoubleScalarSin(q[0]);
  eb_x = muDoubleScalarCos(q[0]);
  fb_x = muDoubleScalarSin(q[0]);
  gb_x = muDoubleScalarCos(q[0]);
  hb_x = muDoubleScalarSin(q[0]);
  ib_x = muDoubleScalarSin(q[0]);
  jb_x = muDoubleScalarCos(q[0]);
  kb_x = muDoubleScalarCos(q[0]);
  lb_x = muDoubleScalarCos(q[0]);
  mb_x = muDoubleScalarCos(q[0]);
  nb_x = muDoubleScalarCos(q[0]);
  ob_x = muDoubleScalarCos(q[0]);
  pb_x = muDoubleScalarCos(q[1]);
  qb_x = muDoubleScalarSin(q[0]);
  rb_x = muDoubleScalarCos(q[2] + zeta);
  sb_x = muDoubleScalarSin(q[0]);
  tb_x = muDoubleScalarSin(q[0]);
  ub_x = muDoubleScalarSin(q[0]);
  vb_x = muDoubleScalarCos(q[2] + zeta);
  wb_x = muDoubleScalarCos(q[0]);
  xb_x = muDoubleScalarCos(q[0]);
  yb_x = muDoubleScalarCos(q[1]);
  ac_x = muDoubleScalarSin(q[0]);
  bc_x = muDoubleScalarSin(q[0]);
  cc_x = muDoubleScalarSin(q[0]);
  dc_x = muDoubleScalarCos(q[0]);
  ec_x = muDoubleScalarSin(q[0]);
  fc_x = muDoubleScalarCos(q[0]);
  gc_x = muDoubleScalarSin(q[0]);
  hc_x = muDoubleScalarSin(q[0]);
  ic_x = muDoubleScalarCos(q[0]);
  jc_x = muDoubleScalarCos(q[0]);
  kc_x = muDoubleScalarCos(q[0]);
  lc_x = muDoubleScalarCos(q[0]);
  mc_x = muDoubleScalarCos(q[0]);
  nc_x = muDoubleScalarCos(q[0]);
  oc_x = muDoubleScalarCos(q[0]);
  pc_x = muDoubleScalarSin(q[0]);
  qc_x = muDoubleScalarSin(q[0]);
  rc_x = muDoubleScalarSin(q[0]);
  sc_x = muDoubleScalarCos(q[0]);
  tc_x = muDoubleScalarCos(q[0]);
  uc_x = muDoubleScalarSin(q[0]);
  vc_x = muDoubleScalarSin(q[0]);
  wc_x = muDoubleScalarCos(q[0]);
  xc_x = muDoubleScalarCos(q[0]);
  yc_x = muDoubleScalarSin(q[0]);
  ad_x = muDoubleScalarSin(q[0]);
  bd_x = muDoubleScalarCos(q[0]);
  cd_x = muDoubleScalarCos(q[0]);
  dd_x = muDoubleScalarSin(q[0]);
  ed_x = muDoubleScalarSin(q[0]);
  fd_x = muDoubleScalarCos(q[0]);
  gd_x = muDoubleScalarCos(q[0]);
  hd_x = muDoubleScalarCos(q[0]);
  id_x = muDoubleScalarSin(q[0]);
  jd_x = muDoubleScalarSin(q[0]);
  kd_x = muDoubleScalarSin(q[0]);
  ld_x = muDoubleScalarCos(q[0]);
  md_x = muDoubleScalarCos(q[0]);
  nd_x = muDoubleScalarSin(q[0]);
  od_x = muDoubleScalarSin(q[0]);
  pd_x = muDoubleScalarCos(q[0]);
  qd_x = muDoubleScalarCos(q[0]);
  qDot[0] = (uDot[0] * (muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2]
    + zeta) * muDoubleScalarSin(q[0]) * (L5 * L5) - muDoubleScalarCos(q[1]) *
                        muDoubleScalarSin(q[0]) * muDoubleScalarSin(q[1]) * (L3 *
    L3)) / (((((((((((((((((((muDoubleScalarPower(L3, 3.0) * (unusedU0 *
    unusedU0) * (unusedU1 * unusedU1) * muDoubleScalarSin(q[1]) -
    muDoubleScalarPower(L5, 3.0) * (unusedU2 * unusedU2) * muDoubleScalarSin(q[2]
    + zeta) * (unusedU3 * unusedU3)) + L2 * (L3 * L3) * muDoubleScalarCos(q[1]) *
                               (unusedU4 * unusedU4) * muDoubleScalarSin(q[1]))
    - L3 * L3 * L7 * muDoubleScalarCos(q[1]) * (unusedU5 * unusedU5) *
    muDoubleScalarSin(q[1])) - L3 * (L5 * L5) * (unusedU6 * unusedU6) *
    (unusedU7 * unusedU7) * muDoubleScalarSin(q[1])) + L3 * L3 * L5 *
    muDoubleScalarSin(q[2] + zeta) * (unusedU8 * unusedU8) * (unusedU9 *
    unusedU9)) - L2 * (L5 * L5) * muDoubleScalarCos(q[2] + zeta) *
    muDoubleScalarSin(q[2] + zeta) * (x * x)) + L5 * L5 * L7 * muDoubleScalarCos
                        (q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (b_x *
    b_x)) + L3 * L3 * L4 * muDoubleScalarCos(q[1]) * (c_x * c_x) *
                       muDoubleScalarSin(q[1]) * muDoubleScalarCos(zeta)) + L3 *
                      (L5 * L5) * muDoubleScalarCos(q[2] + zeta) *
                      muDoubleScalarSin(q[2] + zeta) * (d_x * d_x) *
                      muDoubleScalarCos(q[1])) - L3 * (L5 * L5) *
                     muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] +
    zeta) * muDoubleScalarCos(q[1]) * (e_x * e_x)) - L3 * L3 * L5 *
                    muDoubleScalarCos(q[2] + zeta) * (f_x * f_x) *
                    muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[1])) - L4 *
                   (L5 * L5) * muDoubleScalarCos(q[2] + zeta) *
                   muDoubleScalarSin(q[2] + zeta) * (g_x * g_x) *
                   muDoubleScalarCos(zeta)) + L3 * L3 * L5 * muDoubleScalarCos
                  (q[2] + zeta) * muDoubleScalarCos(q[1]) * (h_x * h_x) *
                  muDoubleScalarSin(q[1])) - L2 * L3 * L5 * muDoubleScalarCos(q
    [2] + zeta) * (i_x * i_x) * muDoubleScalarSin(q[1])) + L2 * L3 * L5 *
                muDoubleScalarSin(q[2] + zeta) * (j_x * j_x) * muDoubleScalarCos
                (q[1])) + L3 * L5 * L7 * muDoubleScalarCos(q[2] + zeta) * (k_x *
    k_x) * muDoubleScalarSin(q[1])) - L3 * L5 * L7 * muDoubleScalarSin(q[2] +
    zeta) * (l_x * l_x) * muDoubleScalarCos(q[1])) - L3 * L4 * L5 *
             muDoubleScalarCos(q[2] + zeta) * (m_x * m_x) * muDoubleScalarSin(q
    [1]) * muDoubleScalarCos(zeta)) + L3 * L4 * L5 * muDoubleScalarSin(q[2] +
    zeta) * (n_x * n_x) * muDoubleScalarCos(q[1]) * muDoubleScalarCos(zeta)) -
             uDot[2] * (L5 * L5 * (o_x * o_x) * muDoubleScalarCos(q[0]) *
                        muDoubleScalarSin(q[0]) - L3 * L3 * muDoubleScalarCos(q
    [0]) * muDoubleScalarSin(q[0]) * (p_x * p_x)) /
             (((((((((((((((((((muDoubleScalarPower(L3, 3.0) * (q_x * q_x) *
    (r_x * r_x) * muDoubleScalarSin(q[1]) - muDoubleScalarPower(L5, 3.0) * (s_x *
    s_x) * muDoubleScalarSin(q[2] + zeta) * (t_x * t_x)) + L2 * (L3 * L3) *
    muDoubleScalarCos(q[1]) * (u_x * u_x) * muDoubleScalarSin(q[1])) - L3 * L3 *
    L7 * muDoubleScalarCos(q[1]) * (v_x * v_x) * muDoubleScalarSin(q[1])) - L3 *
                             (L5 * L5) * (w_x * w_x) * (x_x * x_x) *
    muDoubleScalarSin(q[1])) + L3 * L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
    (y_x * y_x) * (ab_x * ab_x)) - L2 * (L5 * L5) * muDoubleScalarCos(q[2] +
    zeta) * muDoubleScalarSin(q[2] + zeta) * (bb_x * bb_x)) + L5 * L5 * L7 *
    muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (cb_x *
    cb_x)) + L3 * L3 * L4 * muDoubleScalarCos(q[1]) * (db_x * db_x) *
    muDoubleScalarSin(q[1]) * muDoubleScalarCos(zeta)) + L3 * (L5 * L5) *
                        muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2]
    + zeta) * (eb_x * eb_x) * muDoubleScalarCos(q[1])) - L3 * (L5 * L5) *
                       muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] +
    zeta) * muDoubleScalarCos(q[1]) * (fb_x * fb_x)) - L3 * L3 * L5 *
                      muDoubleScalarCos(q[2] + zeta) * (gb_x * gb_x) *
                      muDoubleScalarCos(q[1]) * muDoubleScalarSin(q[1])) - L4 *
                     (L5 * L5) * muDoubleScalarCos(q[2] + zeta) *
                     muDoubleScalarSin(q[2] + zeta) * (hb_x * hb_x) *
                     muDoubleScalarCos(zeta)) + L3 * L3 * L5 * muDoubleScalarCos
                    (q[2] + zeta) * muDoubleScalarCos(q[1]) * (ib_x * ib_x) *
                    muDoubleScalarSin(q[1])) - L2 * L3 * L5 * muDoubleScalarCos
                   (q[2] + zeta) * (jb_x * jb_x) * muDoubleScalarSin(q[1])) + L2
                  * L3 * L5 * muDoubleScalarSin(q[2] + zeta) * (kb_x * kb_x) *
                  muDoubleScalarCos(q[1])) + L3 * L5 * L7 * muDoubleScalarCos(q
    [2] + zeta) * (lb_x * lb_x) * muDoubleScalarSin(q[1])) - L3 * L5 * L7 *
                muDoubleScalarSin(q[2] + zeta) * (mb_x * mb_x) *
                muDoubleScalarCos(q[1])) - L3 * L4 * L5 * muDoubleScalarCos(q[2]
    + zeta) * (nb_x * nb_x) * muDoubleScalarSin(q[1]) * muDoubleScalarCos(zeta))
              + L3 * L4 * L5 * muDoubleScalarSin(q[2] + zeta) * (ob_x * ob_x) *
              muDoubleScalarCos(q[1]) * muDoubleScalarCos(zeta))) - uDot[1] *
    (L3 * L5 * muDoubleScalarCos(q[2] + zeta) * muDoubleScalarCos(q[0]) *
     muDoubleScalarSin(q[1]) - L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
     muDoubleScalarCos(q[0]) * muDoubleScalarCos(q[1])) /
    (((((((((((((((((((muDoubleScalarPower(L3, 3.0) * (pb_x * pb_x) * (qb_x *
    qb_x) * muDoubleScalarSin(q[1]) - muDoubleScalarPower(L5, 3.0) * (rb_x *
    rb_x) * muDoubleScalarSin(q[2] + zeta) * (sb_x * sb_x)) + L2 * (L3 * L3) *
                      muDoubleScalarCos(q[1]) * (tb_x * tb_x) *
                      muDoubleScalarSin(q[1])) - L3 * L3 * L7 *
                     muDoubleScalarCos(q[1]) * (ub_x * ub_x) * muDoubleScalarSin
                     (q[1])) - L3 * (L5 * L5) * (vb_x * vb_x) * (wb_x * wb_x) *
                    muDoubleScalarSin(q[1])) + L3 * L3 * L5 * muDoubleScalarSin
                   (q[2] + zeta) * (xb_x * xb_x) * (yb_x * yb_x)) - L2 * (L5 *
    L5) * muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) *
                  (ac_x * ac_x)) + L5 * L5 * L7 * muDoubleScalarCos(q[2] + zeta)
                 * muDoubleScalarSin(q[2] + zeta) * (bc_x * bc_x)) + L3 * L3 *
                L4 * muDoubleScalarCos(q[1]) * (cc_x * cc_x) * muDoubleScalarSin
                (q[1]) * muDoubleScalarCos(zeta)) + L3 * (L5 * L5) *
               muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) *
               (dc_x * dc_x) * muDoubleScalarCos(q[1])) - L3 * (L5 * L5) *
              muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) *
              muDoubleScalarCos(q[1]) * (ec_x * ec_x)) - L3 * L3 * L5 *
             muDoubleScalarCos(q[2] + zeta) * (fc_x * fc_x) * muDoubleScalarCos
             (q[1]) * muDoubleScalarSin(q[1])) - L4 * (L5 * L5) *
            muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) *
            (gc_x * gc_x) * muDoubleScalarCos(zeta)) + L3 * L3 * L5 *
           muDoubleScalarCos(q[2] + zeta) * muDoubleScalarCos(q[1]) * (hc_x *
            hc_x) * muDoubleScalarSin(q[1])) - L2 * L3 * L5 * muDoubleScalarCos
          (q[2] + zeta) * (ic_x * ic_x) * muDoubleScalarSin(q[1])) + L2 * L3 *
         L5 * muDoubleScalarSin(q[2] + zeta) * (jc_x * jc_x) * muDoubleScalarCos
         (q[1])) + L3 * L5 * L7 * muDoubleScalarCos(q[2] + zeta) * (kc_x * kc_x)
        * muDoubleScalarSin(q[1])) - L3 * L5 * L7 * muDoubleScalarSin(q[2] +
        zeta) * (lc_x * lc_x) * muDoubleScalarCos(q[1])) - L3 * L4 * L5 *
      muDoubleScalarCos(q[2] + zeta) * (mc_x * mc_x) * muDoubleScalarSin(q[1]) *
      muDoubleScalarCos(zeta)) + L3 * L4 * L5 * muDoubleScalarSin(q[2] + zeta) *
     (nc_x * nc_x) * muDoubleScalarCos(q[1]) * muDoubleScalarCos(zeta));
  qDot[1] = (uDot[2] * (L5 * muDoubleScalarSin(q[2] + zeta) * (oc_x * oc_x) + L3
                        * muDoubleScalarSin(q[1]) * (pc_x * pc_x)) / (((L5 * L5 *
    muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (qc_x *
    qc_x) - L3 * L3 * muDoubleScalarCos(q[1]) * (rc_x * rc_x) *
    muDoubleScalarSin(q[1])) + L3 * L5 * muDoubleScalarCos(q[2] + zeta) * (sc_x *
    sc_x) * muDoubleScalarSin(q[1])) - L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
              (tc_x * tc_x) * muDoubleScalarCos(q[1])) - L5 * muDoubleScalarCos
             (q[2] + zeta) * muDoubleScalarCos(q[0]) * uDot[0] / (((L5 * L5 *
    muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (uc_x *
    uc_x) - L3 * L3 * muDoubleScalarCos(q[1]) * (vc_x * vc_x) *
    muDoubleScalarSin(q[1])) + L3 * L5 * muDoubleScalarCos(q[2] + zeta) * (wc_x *
    wc_x) * muDoubleScalarSin(q[1])) - L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
              (xc_x * xc_x) * muDoubleScalarCos(q[1]))) - L5 * muDoubleScalarCos
    (q[2] + zeta) * muDoubleScalarSin(q[0]) * uDot[1] / (((L5 * L5 *
    muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (yc_x *
    yc_x) - L3 * L3 * muDoubleScalarCos(q[1]) * (ad_x * ad_x) *
    muDoubleScalarSin(q[1])) + L3 * L5 * muDoubleScalarCos(q[2] + zeta) * (bd_x *
    bd_x) * muDoubleScalarSin(q[1])) - L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
    (cd_x * cd_x) * muDoubleScalarCos(q[1]));
  qDot[2] = (L3 * muDoubleScalarCos(q[0]) * muDoubleScalarCos(q[1]) * uDot[0] /
             (((L5 * L5 * muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2]
    + zeta) * (dd_x * dd_x) - L3 * L3 * muDoubleScalarCos(q[1]) * (ed_x * ed_x) *
                muDoubleScalarSin(q[1])) + L3 * L5 * muDoubleScalarCos(q[2] +
    zeta) * (fd_x * fd_x) * muDoubleScalarSin(q[1])) - L3 * L5 *
              muDoubleScalarSin(q[2] + zeta) * (gd_x * gd_x) * muDoubleScalarCos
              (q[1])) - uDot[2] * (L3 * muDoubleScalarSin(q[1]) * (hd_x * hd_x)
              + L5 * muDoubleScalarSin(q[2] + zeta) * (id_x * id_x)) / (((L5 *
    L5 * muDoubleScalarCos(q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (jd_x
    * jd_x) - L3 * L3 * muDoubleScalarCos(q[1]) * (kd_x * kd_x) *
    muDoubleScalarSin(q[1])) + L3 * L5 * muDoubleScalarCos(q[2] + zeta) * (ld_x *
    ld_x) * muDoubleScalarSin(q[1])) - L3 * L5 * muDoubleScalarSin(q[2] + zeta) *
              (md_x * md_x) * muDoubleScalarCos(q[1]))) + L3 * muDoubleScalarCos
    (q[1]) * muDoubleScalarSin(q[0]) * uDot[1] / (((L5 * L5 * muDoubleScalarCos
    (q[2] + zeta) * muDoubleScalarSin(q[2] + zeta) * (nd_x * nd_x) - L3 * L3 *
    muDoubleScalarCos(q[1]) * (od_x * od_x) * muDoubleScalarSin(q[1])) + L3 * L5
    * muDoubleScalarCos(q[2] + zeta) * (pd_x * pd_x) * muDoubleScalarSin(q[1]))
    - L3 * L5 * muDoubleScalarSin(q[2] + zeta) * (qd_x * qd_x) *
    muDoubleScalarCos(q[1]));
}

/* End of code generation (sherpaTTIKVel.c) */
