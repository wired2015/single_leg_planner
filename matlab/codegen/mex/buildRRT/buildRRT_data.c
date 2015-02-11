/*
 * buildRRT_data.c
 *
 * Code generation for function 'buildRRT_data'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "buildRRT.h"
#include "buildRRT_data.h"

/* Variable Definitions */
const volatile char_T *emlrtBreakCheckR2012bFlagVar;
emlrtRSInfo f_emlrtRSI = { 20, "eml_int_forloop_overflow_check",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"
};

emlrtRSInfo q_emlrtRSI = { 29, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m"
};

emlrtRSInfo r_emlrtRSI = { 26, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m"
};

emlrtRSInfo s_emlrtRSI = { 44, "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m"
};

emlrtRSInfo t_emlrtRSI = { 14, "sqrt",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/elfun/sqrt.m" };

emlrtRSInfo u_emlrtRSI = { 18, "min",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/datafun/min.m" };

emlrtRSInfo v_emlrtRSI = { 15, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo w_emlrtRSI = { 96, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo x_emlrtRSI = { 229, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo y_emlrtRSI = { 202, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo c_emlrtMCI = { 41, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo d_emlrtMCI = { 38, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo e_emlrtMCI = { 82, 9, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtMCInfo f_emlrtMCI = { 81, 19, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRTEInfo j_emlrtRTEI = { 5, 33, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m"
};

emlrtBCInfo g_emlrtBCI = { -1, -1, 14, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

emlrtBCInfo h_emlrtBCI = { -1, -1, 13, 13, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

emlrtBCInfo i_emlrtBCI = { -1, -1, 12, 14, "xB", "heuristicSingleLeg",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/heuristicSingleLeg.m",
  0 };

emlrtRTEInfo o_emlrtRTEI = { 25, 5, "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m"
};

emlrtBCInfo bc_emlrtBCI = { -1, -1, 26, 39, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo cc_emlrtBCI = { -1, -1, 29, 24, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo dc_emlrtBCI = { -1, -1, 31, 15, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo ec_emlrtBCI = { -1, -1, 31, 24, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo fc_emlrtBCI = { -1, -1, 32, 25, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo gc_emlrtBCI = { -1, -1, 32, 34, "T", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtBCInfo hc_emlrtBCI = { -1, -1, 26, 9, "d", "nearestNeighbour",
  "/Users/fuji/Dropbox/phd/matlab/singleLegPlanning/rrtSherpaTT4/nearestNeighbour.m",
  0 };

emlrtRSInfo gb_emlrtRSI = { 81, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo hb_emlrtRSI = { 38, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo kb_emlrtRSI = { 82, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

emlrtRSInfo lb_emlrtRSI = { 41, "eml_min_or_max",
  "/Applications/MATLAB_R2014b.app/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"
};

/* End of code generation (buildRRT_data.c) */
