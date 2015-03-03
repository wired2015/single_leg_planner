MATLAB="/Applications/MATLAB_R2014b.app"
Arch=maci64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/Users/fuji/.matlab/R2014b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for sherpaTTPlanner_mex" > sherpaTTPlanner_mex_mex.mki
echo "CC=$CC" >> sherpaTTPlanner_mex_mex.mki
echo "CFLAGS=$CFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "CLIBS=$CLIBS" >> sherpaTTPlanner_mex_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "CXX=$CXX" >> sherpaTTPlanner_mex_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "CXXLIBS=$CXXLIBS" >> sherpaTTPlanner_mex_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "LD=$LD" >> sherpaTTPlanner_mex_mex.mki
echo "LDFLAGS=$LDFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> sherpaTTPlanner_mex_mex.mki
echo "Arch=$Arch" >> sherpaTTPlanner_mex_mex.mki
echo OMPFLAGS= >> sherpaTTPlanner_mex_mex.mki
echo OMPLINKFLAGS= >> sherpaTTPlanner_mex_mex.mki
echo "EMC_COMPILER=Xcode with Clang" >> sherpaTTPlanner_mex_mex.mki
echo "EMC_CONFIG=optim" >> sherpaTTPlanner_mex_mex.mki
"/Applications/MATLAB_R2014b.app/bin/maci64/gmake" -B -f sherpaTTPlanner_mex_mex.mk
