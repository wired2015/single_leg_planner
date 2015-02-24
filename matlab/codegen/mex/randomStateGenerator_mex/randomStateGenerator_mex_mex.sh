MATLAB="/Applications/MATLAB_R2014b.app"
Arch=maci64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/Users/fuji/.matlab/R2014b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for randomStateGenerator_mex" > randomStateGenerator_mex_mex.mki
echo "CC=$CC" >> randomStateGenerator_mex_mex.mki
echo "CFLAGS=$CFLAGS" >> randomStateGenerator_mex_mex.mki
echo "CLIBS=$CLIBS" >> randomStateGenerator_mex_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> randomStateGenerator_mex_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> randomStateGenerator_mex_mex.mki
echo "CXX=$CXX" >> randomStateGenerator_mex_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> randomStateGenerator_mex_mex.mki
echo "CXXLIBS=$CXXLIBS" >> randomStateGenerator_mex_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> randomStateGenerator_mex_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> randomStateGenerator_mex_mex.mki
echo "LD=$LD" >> randomStateGenerator_mex_mex.mki
echo "LDFLAGS=$LDFLAGS" >> randomStateGenerator_mex_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> randomStateGenerator_mex_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> randomStateGenerator_mex_mex.mki
echo "Arch=$Arch" >> randomStateGenerator_mex_mex.mki
echo OMPFLAGS= >> randomStateGenerator_mex_mex.mki
echo OMPLINKFLAGS= >> randomStateGenerator_mex_mex.mki
echo "EMC_COMPILER=Xcode with Clang" >> randomStateGenerator_mex_mex.mki
echo "EMC_CONFIG=optim" >> randomStateGenerator_mex_mex.mki
"/Applications/MATLAB_R2014b.app/bin/maci64/gmake" -B -f randomStateGenerator_mex_mex.mk
