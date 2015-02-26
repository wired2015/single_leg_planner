MATLAB="/Applications/MATLAB_R2014b.app"
Arch=maci64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/Users/fuji/.matlab/R2014b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for buildRRTWrapper_mex" > buildRRTWrapper_mex_mex.mki
echo "CC=$CC" >> buildRRTWrapper_mex_mex.mki
echo "CFLAGS=$CFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "CLIBS=$CLIBS" >> buildRRTWrapper_mex_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "CXX=$CXX" >> buildRRTWrapper_mex_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "CXXLIBS=$CXXLIBS" >> buildRRTWrapper_mex_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "LD=$LD" >> buildRRTWrapper_mex_mex.mki
echo "LDFLAGS=$LDFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> buildRRTWrapper_mex_mex.mki
echo "Arch=$Arch" >> buildRRTWrapper_mex_mex.mki
echo OMPFLAGS= >> buildRRTWrapper_mex_mex.mki
echo OMPLINKFLAGS= >> buildRRTWrapper_mex_mex.mki
echo "EMC_COMPILER=Xcode with Clang" >> buildRRTWrapper_mex_mex.mki
echo "EMC_CONFIG=optim" >> buildRRTWrapper_mex_mex.mki
"/Applications/MATLAB_R2014b.app/bin/maci64/gmake" -B -f buildRRTWrapper_mex_mex.mk
