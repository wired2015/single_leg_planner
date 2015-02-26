MATLAB="/Applications/MATLAB_R2014b.app"
Arch=maci64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/Users/fuji/.matlab/R2014b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for buildBiDirectionalRRTWrapper" > buildBiDirectionalRRTWrapper_mex.mki
echo "CC=$CC" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CFLAGS=$CFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CLIBS=$CLIBS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CXX=$CXX" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CXXLIBS=$CXXLIBS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "LD=$LD" >> buildBiDirectionalRRTWrapper_mex.mki
echo "LDFLAGS=$LDFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> buildBiDirectionalRRTWrapper_mex.mki
echo "Arch=$Arch" >> buildBiDirectionalRRTWrapper_mex.mki
echo OMPFLAGS= >> buildBiDirectionalRRTWrapper_mex.mki
echo OMPLINKFLAGS= >> buildBiDirectionalRRTWrapper_mex.mki
echo "EMC_COMPILER=Xcode with Clang" >> buildBiDirectionalRRTWrapper_mex.mki
echo "EMC_CONFIG=optim" >> buildBiDirectionalRRTWrapper_mex.mki
"/Applications/MATLAB_R2014b.app/bin/maci64/gmake" -B -f buildBiDirectionalRRTWrapper_mex.mk
