CC="/usr/bin/xcrun -sdk macosx10.9 clang"
CXX="/usr/bin/xcrun -sdk macosx10.9 clang"
CFLAGS="-fno-common -arch x86_64 -mmacosx-version-min=10.9 -fexceptions -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk -DMATLAB_MEX_FILE"
CXXFLAGS="-fno-common -arch x86_64 -mmacosx-version-min=10.9 -fexceptions -isysroot /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk -DMATLAB_MEX_FILE"
COPTIMFLAGS="-O2 -DNDEBUG"
CXXOPTIMFLAGS="-O2 -DNDEBUG"
CDEBUGFLAGS="-g"
CXXDEBUGFLAGS="-g"
LD="/usr/bin/xcrun -sdk macosx10.9 clang"
LDXX="/usr/bin/xcrun -sdk macosx10.9 clang"
LDFLAGS="-Wl,-twolevel_namespace -undefined error -arch x86_64 -mmacosx-version-min=10.9 -Wl,-syslibroot,/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk -bundle  -Wl,-exported_symbols_list,"/Applications/MATLAB_R2014b.app/extern/lib/maci64/mexFunction.map" -L"/Applications/MATLAB_R2014b.app/bin/maci64" -lmx -lmex -lmat -lstdc++ -Wl,-exported_symbols_list,"/Applications/MATLAB_R2014b.app/extern/lib/maci64/mexFunction.map""
LDDEBUGFLAGS="-g"
