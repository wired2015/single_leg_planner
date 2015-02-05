#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxAPI.h"
#include "LegPlanner.h"

using namespace std;

int main(int argv, char **argc){
    LegPlanner lp;

    int i;
    double temp;
    //lp.getState(1);
    for (i=0; i<300; i++){
        temp = (double) i;
        lp.getState(temp/10+0.05);
        printf("time: %.2f\n",temp/10);
        printf("t = %.2f, x = %.2f, y = %.2f, z=%.2f\n", lp.state[0],lp.state[1],lp.state[2],lp.state[3]);
    }
}
