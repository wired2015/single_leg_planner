cd src
echo "rock_library(single_leg_planner" > CMakeLists.txt
echo "SOURCES" >> CMakeLists.txt
ls *.cpp >> CMakeLists.txt
echo "HEADERS" >> CMakeLists.txt
ls *.h >> CMakeLists.txt 
echo ")" >> CMakeLists.txt << EOF
cd ..
