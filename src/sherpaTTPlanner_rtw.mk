###########################################################################
## Makefile generated for MATLAB file/project 'sherpaTTPlanner'. 
## 
## Makefile     : sherpaTTPlanner_rtw.mk
## Generated on : Thu Mar 05 15:01:47 2015
## MATLAB Coder version: 2.7 (R2014b)
## 
## Build Info:
## 
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/sherpaTTPlanner.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = sherpaTTPlanner
MAKEFILE                  = sherpaTTPlanner_rtw.mk
COMPUTER                  = MACI64
MATLAB_ROOT               = /Applications/MATLAB_R2014b.app
MATLAB_BIN                = /Applications/MATLAB_R2014b.app/bin
MATLAB_ARCH_BIN           = /Applications/MATLAB_R2014b.app/bin/maci64
MASTER_ANCHOR_DIR         = 
START_DIR                 = /Users/fuji/Dropbox/phd/matlab/singleLegPlanning/single_leg_planner/matlab/evaluation
ARCH                      = maci64
RELATIVE_PATH_TO_ANCHOR   = .

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Clang v3.1 | gmake (64-bit Mac)
# Supported Version(s):    3.1
# ToolchainInfo Version:   R2014b
# Specification Revision:  1.0
# 

#-----------
# MACROS
#-----------

ANSI_OPTS       = -fno-common -fexceptions
CPP_ANSI_OPTS   = -fno-common -fexceptions
ARCHS           = x86_64
XCODE_SDK_VER   = $(shell xcodebuild -showsdks | perl -anle 'BEGIN{@l = "";} push @l, $$F[-1] if /macosx/; END{ sort @l; $$_ = $$l[1]; s/macosx//; print $$_;}')
XCODE_SDK       = MacOSX$(XCODE_SDK_VER).sdk
XCODE_DEVEL_DIR = $(shell xcode-select -print-path)
XCODE_SDK_ROOT  = $(shell find $(XCODE_DEVEL_DIR) -name $(XCODE_SDK))

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Clang C Compiler
CC = xcrun clang

# Linker: Clang Linker
LD = xcrun clang

# C++ Compiler: Clang C++ Compiler
CPP = xcrun clang++

# C++ Linker: Clang C++ Linker
CPP_LD = xcrun clang++

# Archiver: Clang Archiver
AR = xcrun ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_BIN)
MEX = $(MEX_PATH)/mex

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/maci64
MAKE = $(MAKE_PATH)/gmake


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c -isysroot $(XCODE_SDK_ROOT) -arch $(ARCHS) $(ANSI_OPTS) \
                       -O0
CPPFLAGS             = -c -isysroot $(XCODE_SDK_ROOT) -arch $(ARCHS) $(CPP_ANSI_OPTS) \
                       -O0
CPP_LDFLAGS          = -arch $(ARCHS) -isysroot $(XCODE_SDK_ROOT) -L"$(MATLAB_ARCH_BIN)"
CPP_SHAREDLIB_LDFLAGS  = -dynamiclib -isysroot $(XCODE_SDK_ROOT) -L"$(MATLAB_ARCH_BIN)" \
                         -Wl,$(LD_NAMESPACE) $(LD_UNDEFS)
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -arch $(ARCHS) -isysroot $(XCODE_SDK_ROOT) -L"$(MATLAB_ARCH_BIN)"
MEX_CFLAGS           = -MATLAB_ARCH=$(ARCH) $(INCLUDES) \
                         \
                       COPTIMFLAGS="$(ANSI_OPTS)  \
                       -O0 \
                        $(DEFINES)" \
                         \
                       -silent
MEX_LDFLAGS          = LDFLAGS=='$$LDFLAGS'
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -dynamiclib -isysroot $(XCODE_SDK_ROOT) -L"$(MATLAB_ARCH_BIN)" \
                       -Wl,$(LD_NAMESPACE) $(LD_UNDEFS)

#--------------------
# File extensions
#--------------------

H_EXT               = .h
OBJ_EXT             = .o
C_EXT               = .c
EXE_EXT             =
SHAREDLIB_EXT       = .so
HPP_EXT             = .hpp
OBJ_EXT             = .o
CPP_EXT             = .cpp
EXE_EXT             =
SHAREDLIB_EXT       = .so
STATICLIB_EXT       = .a
MEX_EXT             = .mexmaci64
MAKE_EXT            = .mk


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/sherpaTTPlanner.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src -I/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt -I/Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/matlab/rrt/biDirectionalRRT -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_STANDARD = -DMODEL=sherpaTTPlanner -DHAVESTDIO -DUSE_RTMODEL -DUNIX

DEFINES = $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_rtwutil.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_data.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_initialize.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_terminate.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/buildBiDirectionalRRTWrapper.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/buildRRTWrapper.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/randomStateGenerator.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sin.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/trInv.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTIK.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/exp.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/log.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/asin.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/fprintf.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/fileManager.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTIKVel.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/validJointState.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/buildBiDirectionalRRT.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/randomState.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/rand.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/eml_rand_mt19937ar_stateful.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/nearestNeighbour.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/heuristicSingleLeg.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/norm.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/selectInput.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/rk4.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/getPhiAndOmega.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/generateTrMatrices.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/angDiff.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/flipud.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_emxutil.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/sherpaTTPlanner_emxAPI.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/rt_nonfinite.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/rtGetNaN.cpp /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/rtGetInf.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = sherpaTTPlanner_rtwutil.o sherpaTTPlanner_data.o sherpaTTPlanner_initialize.o sherpaTTPlanner_terminate.o buildBiDirectionalRRTWrapper.o buildRRTWrapper.o randomStateGenerator.o sin.o trInv.o sherpaTTIK.o exp.o log.o asin.o fprintf.o fileManager.o sherpaTTIKVel.o validJointState.o buildBiDirectionalRRT.o randomState.o rand.o eml_rand_mt19937ar_stateful.o nearestNeighbour.o heuristicSingleLeg.o norm.o selectInput.o rk4.o getPhiAndOmega.o generateTrMatrices.o angDiff.o flipud.o sherpaTTPlanner_emxutil.o sherpaTTPlanner_emxAPI.o rt_nonfinite.o rtGetNaN.o rtGetInf.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/codegentest/CMakeLists.txt

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : build


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /Users/fuji/Dropbox/PhD/matlab/singleLegPlanning/single_leg_planner/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : $(MAKEFILE) rtw_proj.tmw


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


