cd src
echo "rock_library(single_leg_planner\nSOURCES\n" > ../test.txt
ls *.cpp >> ../test.txt
ls *.h >> ../test.txt 
echo ")" >> ../test.txt << EOF
cd ..
