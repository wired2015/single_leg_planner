/*
 * sherpaTTPlanner_mex_data.c
 *
 * Code generation for function 'sherpaTTPlanner_mex_data'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "buildRRTWrapper.h"
#include "randomStateGenerator.h"
#include "sherpaTTPlanner_mex_data.h"
#include <stdio.h>

/* Variable Definitions */
const volatile char_T *emlrtBreakCheckR2012bFlagVar;
emlrtRSInfo i_emlrtRSI = { 37, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo j_emlrtRSI = { 45, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo k_emlrtRSI = { 46, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo l_emlrtRSI = { 48, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo m_emlrtRSI = { 49, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo n_emlrtRSI = { 51, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo o_emlrtRSI = { 52, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo p_emlrtRSI = { 59, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo q_emlrtRSI = { 62, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo r_emlrtRSI = { 68, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo s_emlrtRSI = { 69, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo t_emlrtRSI = { 70, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo gb_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtRSInfo hb_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

emlrtRSInfo ib_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo jb_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo kb_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo lb_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo mb_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

emlrtRSInfo qb_emlrtRSI = { 21, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

emlrtRSInfo rb_emlrtRSI = { 79, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

emlrtMCInfo b_emlrtMCI = { 56, 9, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtMCInfo c_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo d_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRTEInfo c_emlrtRTEI = { 284, 1, "colon",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/ops/colon.m" };

emlrtRTEInfo h_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtBCInfo emlrtBCI = { 1, 4, 18, 17, "kC.legAngleOffset", "trP2B",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/kinematics/trP2B.m",
  0 };

emlrtRTEInfo s_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtBCInfo c_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo m_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

sherpaTTPlanner_mexStackData *c_sherpaTTPlanner_mexStackDataL;
emlrtRSInfo mc_emlrtRSI = { 56, "randomState",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/randomState.m"
};

emlrtRSInfo oc_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo qc_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

/* End of code generation (sherpaTTPlanner_mex_data.c) */
