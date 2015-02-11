MATLAB="/Applications/MATLAB_R2014b.app"
Arch=maci64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/Users/fuji/.matlab/R2014b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for buildRRT" > buildRRT_mex.mki
echo "CC=$CC" >> buildRRT_mex.mki
echo "CFLAGS=$CFLAGS" >> buildRRT_mex.mki
echo "CLIBS=$CLIBS" >> buildRRT_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> buildRRT_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> buildRRT_mex.mki
echo "CXX=$CXX" >> buildRRT_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> buildRRT_mex.mki
echo "CXXLIBS=$CXXLIBS" >> buildRRT_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> buildRRT_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> buildRRT_mex.mki
echo "LD=$LD" >> buildRRT_mex.mki
echo "LDFLAGS=$LDFLAGS" >> buildRRT_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> buildRRT_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> buildRRT_mex.mki
echo "Arch=$Arch" >> buildRRT_mex.mki
echo OMPFLAGS= >> buildRRT_mex.mki
echo OMPLINKFLAGS= >> buildRRT_mex.mki
echo "EMC_COMPILER=Xcode with Clang" >> buildRRT_mex.mki
echo "EMC_CONFIG=optim" >> buildRRT_mex.mki
"/Applications/MATLAB_R2014b.app/bin/maci64/gmake" -B -f buildRRT_mex.mk
