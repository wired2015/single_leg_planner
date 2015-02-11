/*
 * buildRRTWrapper_data.c
 *
 * Code generation for function 'buildRRTWrapper_data'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_data.h"
#include <stdio.h>

/* Variable Definitions */
const volatile char_T *emlrtBreakCheckR2012bFlagVar;
emlrtRSInfo f_emlrtRSI = { 14, "sqrt",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/sqrt.m" };

emlrtRSInfo t_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtRSInfo u_emlrtRSI = { 26, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtRSInfo v_emlrtRSI = { 41, "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m"
};

emlrtRSInfo w_emlrtRSI = { 49, "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m"
};

emlrtRSInfo x_emlrtRSI = { 62, "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m"
};

emlrtRSInfo y_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

emlrtRSInfo ab_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo bb_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo cb_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo db_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo eb_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

emlrtMCInfo c_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo d_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRTEInfo g_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtBCInfo bb_emlrtBCI = { -1, -1, 32, 17, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtBCInfo cb_emlrtBCI = { -1, -1, 31, 16, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtBCInfo db_emlrtBCI = { -1, -1, 30, 17, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtBCInfo eb_emlrtBCI = { -1, -1, 24, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtBCInfo fb_emlrtBCI = { -1, -1, 23, 13, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtBCInfo gb_emlrtBCI = { -1, -1, 22, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/heuristicSingleLeg.m",
  0 };

emlrtRTEInfo k_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m"
};

emlrtBCInfo jb_emlrtBCI = { -1, -1, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo kb_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo lb_emlrtBCI = { -1, -1, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo mb_emlrtBCI = { -1, -1, 31, 24, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo nb_emlrtBCI = { -1, -1, 32, 34, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtBCInfo ob_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/nearestNeighbour.m",
  0 };

emlrtRSInfo ib_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo lb_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

/* End of code generation (buildRRTWrapper_data.c) */
