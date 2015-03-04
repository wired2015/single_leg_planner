#ifndef LegPlanner_H
#define LegPlanner_H

#include <stdlib.h>
#include <math.h>
#include "sherpaTTPlanner_emxAPI.h"
#include "sherpaTTPlanner_types.h"

class LegPlanner
{
    public:
        
        struct0_T kC;
        
        double jointLimits[20];
        double sInitB[6];
        double sGoalB[6];
        double uBDot[6];
        double phiInit;
        double omegaInit;
        
        emxArray_real_T *emxT;
        emxArray_real_T *emxT2;
        emxArray_real_T *emxPathC;
        emxArray_real_T *emxPathJ;
        boolean_T success;
        
        int* pathLength;
        double** path;
        double state[7];

        LegPlanner(int);
        void getState(double);
        bool plan(int);
        void interpolateState(double t, double* s1,double* s2);
};

#endif
