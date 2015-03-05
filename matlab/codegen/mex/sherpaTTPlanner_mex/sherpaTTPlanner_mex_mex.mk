START_DIR = /Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab

MATLAB_ROOT = /Applications/MATLAB_R2014b.app
MAKEFILE = sherpaTTPlanner_mex_mex.mk

include sherpaTTPlanner_mex_mex.mki


SRC_FILES =  \
	sherpaTTPlanner_mex_mexutil.c \
	sherpaTTPlanner_mex_data.c \
	sherpaTTPlanner_mex_initialize.c \
	sherpaTTPlanner_mex_terminate.c \
	buildBiDirectionalRRTWrapper.c \
	buildRRTWrapper.c \
	randomStateGenerator.c \
	sin.c \
	trInv.c \
	sherpaTTIK.c \
	eml_error.c \
	exp.c \
	log.c \
	asin.c \
	fprintf.c \
	eml_int_forloop_overflow_check.c \
	sherpaTTIKVel.c \
	validJointState.c \
	buildBiDirectionalRRT.c \
	randomState.c \
	getXStar.c \
	nearestNeighbour.c \
	heuristicSingleLeg.c \
	norm.c \
	selectInput.c \
	rk4.c \
	getPhiAndOmega.c \
	angDiff.c \
	flipud.c \
	_coder_sherpaTTPlanner_mex_api.c \
	sherpaTTPlanner_mex_emxutil.c \
	_coder_sherpaTTPlanner_mex_mex.c \
	_coder_sherpaTTPlanner_mex_info.c

MEX_FILE_NAME_WO_EXT = sherpaTTPlanner_mex
MEX_FILE_NAME = $(MEX_FILE_NAME_WO_EXT).mexmaci64
TARGET = $(MEX_FILE_NAME)

SYS_LIBS = 


#
#====================================================================
# gmake makefile fragment for building MEX functions using Unix
# Copyright 2007-2013 The MathWorks, Inc.
#====================================================================
#
OBJEXT = o
.SUFFIXES: .$(OBJEXT)

OBJLISTC = $(SRC_FILES:.c=.$(OBJEXT))
OBJLISTCPP  = $(OBJLISTC:.cpp=.$(OBJEXT))
OBJLIST  = $(OBJLISTCPP:.cu=.$(OBJEXT))

target: $(TARGET)

ML_INCLUDES = -I "$(MATLAB_ROOT)/simulink/include"
ML_INCLUDES+= -I "$(MATLAB_ROOT)/toolbox/shared/simtargets"
SYS_INCLUDE = $(ML_INCLUDES)

# Additional includes

SYS_INCLUDE += -I "$(START_DIR)"
SYS_INCLUDE += -I "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/codegen/mex/sherpaTTPlanner_mex"
SYS_INCLUDE += -I "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/codegen/mex/sherpaTTPlanner_mex/interface"
SYS_INCLUDE += -I "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt"
SYS_INCLUDE += -I "/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT"
SYS_INCLUDE += -I "$(START_DIR)/evaluation"
SYS_INCLUDE += -I "$(MATLAB_ROOT)/extern/include"
SYS_INCLUDE += -I "."

EML_LIBS = -lemlrt -lcovrt -lut -lmwmathutil -lmwblas 
SYS_LIBS += $(CLIBS) $(EML_LIBS)


EXPORTFILE = $(MEX_FILE_NAME_WO_EXT)_mex.map
ifeq ($(Arch),maci)
  EXPORTOPT = -Wl,-exported_symbols_list,$(EXPORTFILE)
  COMP_FLAGS = -c $(CFLAGS) -DMX_COMPAT_32
  CXX_FLAGS = -c $(CXXFLAGS) -DMX_COMPAT_32
  LINK_FLAGS = $(filter-out %mexFunction.map, $(LDFLAGS))
else ifeq ($(Arch),maci64)
  EXPORTOPT = -Wl,-exported_symbols_list,$(EXPORTFILE)
  COMP_FLAGS = -c $(CFLAGS) -DMX_COMPAT_32
  CXX_FLAGS = -c $(CXXFLAGS) -DMX_COMPAT_32
  LINK_FLAGS = $(filter-out %mexFunction.map, $(LDFLAGS))
else
  EXPORTOPT = -Wl,--version-script,$(EXPORTFILE)
  COMP_FLAGS = -c $(CFLAGS) -DMX_COMPAT_32 $(OMPFLAGS)
  CXX_FLAGS = -c $(CXXFLAGS) -DMX_COMPAT_32 $(OMPFLAGS)
  LINK_FLAGS = $(filter-out %mexFunction.map, $(LDFLAGS)) 
endif
LINK_FLAGS += $(OMPLINKFLAGS)
ifeq ($(Arch),maci)
  LINK_FLAGS += -L$(MATLAB_ROOT)/sys/os/maci
endif
ifeq ($(EMC_CONFIG),optim)
  ifeq ($(Arch),mac)
    COMP_FLAGS += $(CDEBUGFLAGS)
    CXX_FLAGS += $(CXXDEBUGFLAGS)
  else
    COMP_FLAGS += $(COPTIMFLAGS)
    CXX_FLAGS += $(CXXOPTIMFLAGS)
  endif
  LINK_FLAGS += $(LDOPTIMFLAGS)
else
  COMP_FLAGS += $(CDEBUGFLAGS)
  CXX_FLAGS += $(CXXDEBUGFLAGS)
  LINK_FLAGS += $(LDDEBUGFLAGS)
endif
LINK_FLAGS += -o $(TARGET)
LINK_FLAGS += 

CCFLAGS =  $(COMP_FLAGS) $(USER_INCLUDE) $(SYS_INCLUDE)
CPPFLAGS =   $(CXX_FLAGS) $(USER_INCLUDE) $(SYS_INCLUDE)

%.$(OBJEXT) : %.c
	$(CC) $(CCFLAGS) "$<"

%.$(OBJEXT) : %.cpp
	$(CXX) $(CPPFLAGS) "$<"

# Additional sources

%.$(OBJEXT) : $(START_DIR)/%.c
	$(CC) $(CCFLAGS) "$<"

%.$(OBJEXT) : /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/codegen/mex/sherpaTTPlanner_mex/%.c
	$(CC) $(CCFLAGS) "$<"

%.$(OBJEXT) : interface/%.c
	$(CC) $(CCFLAGS) "$<"



%.$(OBJEXT) : $(START_DIR)/%.cu
	$(CC) $(CCFLAGS) "$<"

%.$(OBJEXT) : /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/codegen/mex/sherpaTTPlanner_mex/%.cu
	$(CC) $(CCFLAGS) "$<"

%.$(OBJEXT) : interface/%.cu
	$(CC) $(CCFLAGS) "$<"



%.$(OBJEXT) : $(START_DIR)/%.cpp
	$(CXX) $(CPPFLAGS) "$<"

%.$(OBJEXT) : /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/codegen/mex/sherpaTTPlanner_mex/%.cpp
	$(CXX) $(CPPFLAGS) "$<"

%.$(OBJEXT) : interface/%.cpp
	$(CXX) $(CPPFLAGS) "$<"



$(TARGET): $(OBJLIST) $(MAKEFILE)
	$(LD) $(EXPORTOPT) $(LINK_FLAGS) $(OBJLIST) $(SYS_LIBS)

#====================================================================

