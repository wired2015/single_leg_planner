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
        double bodyW = 0.65;               //[m]
        double bodyH = 0.65;               //[m]
        const double kinematicConst[12] = {L1, L2, L3, L4, L5, L6, L7, L8, zeta, r, bodyW, bodyH};
        int NUM_NODES = 1000;
        bool exhaustive = false;
        double threshold = 0.04;
        double ankleThreshold = M_PI/8;
        int NODE_SIZE = 11;
        double HGAINS[3] = {1, 0, 0.5};
        const double jointLimits[12] = {-2.3562, 2.3562, -1.0385, 0.2967, -0.0873, 1.2863, -0.1745, 0.1745, -0.3491, 0.3491, -0.3491, 0.3491};
        const double nInit[6] = {-0.4122,-0.6819,-0.6000,0,0,0};
        const double nGoal[6] = {-0.1328,0.7245,-0.6000,0,0,0};
        int U_SIZE = 5;
        double stepAccRatio = 14;
        double Dt = 0.7; //[s]
        double dt = 0.1;
        double eta = Dt/stepAccRatio;
        const double U[10] = {eta*1, -eta*1, 0, 0, 0, 0, 0, eta*1, -eta*1, 0};
        double K = -0.6;
        int goalSeedFreq = 20;

        emxArray_real_T *emxT = emxCreate_real_T(0,0);
        emxArray_real_T *emxPath = emxCreate_real_T(0,0);
        emxArray_real_T *emxPathJoint = emxCreate_real_T(0,0);
        //boolean_T *success;
        
        int* pathLength;
        double** path;
        double state[7];

        LegPlanner(void);
        void getState(double);
        void plan(void);
        void interpolateState(double t, double* s1,double* s2);
};

#endif
