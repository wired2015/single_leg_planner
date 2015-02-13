#ifndef LegPlanner_H
#define LegPlanner_H

#include <math.h>
#include "buildRRTWrapper_emxAPI.h"

class LegPlanner
{
    public:
        
        const double DEG2RAD = 0.01745329252;
        const double L1 = 0.175*cos(35*DEG2RAD); //[m]
        double L2 = 0.045;                 //[m]
        double L3 = 0.400;                 //[m]
        double L4 = 0.2;                   //[m]
        double L5 = 0.4;                   //[m]
        double L6 = 0.159*sin(34.5*DEG2RAD);      //[m]
        double L7 = 0.159*cos(34.5*DEG2RAD);      //[m]
        double L8 = 0.39569+0.378/2;       //[m]
        double zeta = 7*DEG2RAD;           //[rad]
        double r = 0.378/2;                //[m]
        double B2POffset = 0.564;
	    double leg1AngleOffset = 0.7854;
	    double leg2AngleOffset = -0.7854;
	    double leg3AngleOffset = -2.3562;
	    double leg4AngleOffset = 2.3562;
	    const double kinematicConst[15] = {L1, L2, L3, L4, L5, L6, L7, L8, zeta, r, B2POffset, leg1AngleOffset, leg2AngleOffset, leg3AngleOffset, leg4AngleOffset};
        double threshold = 0.04;
        int NODE_SIZE = 11;
        const double jointLimits[12] = {-2.3562, 2.3562, -1.0385, 0.2967, -0.0873, 1.2863, -0.1745, 0.1745, -0.3491, 0.3491, -0.3491, 0.3491};
        const double nInit[6] = {-0.3352,-0.6625,-0.6000,0,0,0};
        const double nGoal[6] = {0.3488,0.5778,-0.6000,0,0,0};
        int U_SIZE = 5;
        double stepAccRatio = 14;
        double dt_planner = 0.7; //[s]
        double dt_integration = 0.1;
        double eta = dt_planner/stepAccRatio;
        const double U[10] = {eta*1, -eta*1, 0, 0, 0, 0, 0, eta*1, -eta*1, 0};
        double K = -0.6;
        int goalSeedFreq = 20;

        emxArray_real_T *emxT = emxCreate_real_T(0,0);
        emxArray_real_T *emxPathC = emxCreate_real_T(0,0);
        emxArray_real_T *emxPathJ = emxCreate_real_T(0,0);
        boolean_T success = 1;
        
        int* pathLength;
        double** path;
        double state[7];

        LegPlanner(int);
        void getState(double);
        bool plan(int);
        void interpolateState(double t, double* s1,double* s2);
};

#endif
