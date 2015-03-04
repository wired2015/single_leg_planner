#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include "buildRRTWrapper.h"
#include "sherpaTTPlanner_emxAPI.h"
#include "LegPlanner.h"
#include "rand.h"
#include <time.h>

using namespace std;

int main(int argv, char **argc){

    bool success;    

	LegPlanner lp(4);

    if (lp.success){
    /*    double temp = 0;
        for (int i=0; i<3000; i = i+1){
            temp++;
            lp.getState(temp/100);
        }
        cout << "Interpolation Finished!" << endl;
    */
    }else{
        printf("Plan failed\n");
    }

    return 1;
}
