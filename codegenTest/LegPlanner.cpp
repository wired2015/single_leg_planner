#include <stdlib.h>
#include <iostream>
#include "buildRRTWrapper.h"
#include "buildBiDirectionalRRTWrapper.h"
#include "sherpaTTPlanner_emxAPI.h"
#include "LegPlanner.h"

using namespace std;

LegPlanner::LegPlanner(int legNum)
{

    this->kC.l1 = 0.01868;
    this->kC.l2 = 0.045;
    this->kC.l3 = 0.4;    
    this->kC.l4 = 0.220414;
    this->kC.l5 = 0.4;
    this->kC.l6 = 0.12287;
    this->kC.l7 = 0.0430203;
    this->kC.l8 = 0.291;
    this->kC.zeta = 0.1305;
    this->kC.r = 0.2;
    this->kC.B2PZOffset = -0.133;
    this->kC.B2PXOffset = 0.564652;
    this->kC.legAngleOffset[0] = 0.7854;
    this->kC.legAngleOffset[1] = -0.7854;
    this->kC.legAngleOffset[2] = 2.3562;
    this->kC.legAngleOffset[3] = -2.3562;
    
    this->jointLimits[0] = -2.3562;
    this->jointLimits[1] = 2.3562;
    this->jointLimits[2] = -1.0385;
    this->jointLimits[3] = 0.2967;
    this->jointLimits[4] = -0.0873;
    this->jointLimits[5] = 1.2863;
    this->jointLimits[6] = -3.1416;
    this->jointLimits[7] = 3.1416;
    this->jointLimits[8] = 0;
    this->jointLimits[9] = 0;
    this->jointLimits[10] = -0.1745;
    this->jointLimits[11] = 0.1745;
    this->jointLimits[12] = -0.1745;
    this->jointLimits[13] = 0.1745;
    this->jointLimits[14] = -0.1745;
    this->jointLimits[15] = 0.1745;
    this->jointLimits[16] = -0.7854;
    this->jointLimits[17] = 0.7854;
    this->jointLimits[18] = -3.14159;
    this->jointLimits[19] = 3.14159;
    
    this->sInitB[0] = -0.996;
    this->sInitB[1] = -0.996;
    //this->sInitB[0] = -0.8;
    //this->sInitB[1] = -0.8;
    this->sInitB[2] = -1;
    this->sInitB[3] = 0;
    this->sInitB[4] = 0;
    this->sInitB[5] = 0;
    
    this->sGoalB[0] = -1.257;
    this->sGoalB[1] = -0.3278;
    this->sGoalB[2] = -1;
    this->sGoalB[3] = 0;
    this->sGoalB[4] = 0;
    this->sGoalB[5] = 0;
    
    this->phiInit = 0;
    this->omegaInit = 0;
    
    this->uBDot[0] = 0;
    this->uBDot[1] = 0;
    this->uBDot[2] = 0;
    this->uBDot[3] = 0;
    this->uBDot[4] = 0;
    this->uBDot[5] = 0;
    
    this->emxT = emxCreate_real_T(0,0);
    this->emxT2 = emxCreate_real_T(0,0);
    this->emxPathC = emxCreate_real_T(0,0);
    this->emxPathJ = emxCreate_real_T(0,0);

    this->plan(legNum);

}

void LegPlanner::getState(double time){
    
    int length = *(this->pathLength);
    int i;

    //cout << "1" <<endl;

    if(time <= this->path[0][0])
    {
        //cout << "2" <<endl;
        for(int j=0;j<13;j++){
            this->state[j] = this->path[0][j];
        }
    }
    else if(time >= this->path[length-1][0])
    {
        //cout << "3" <<endl;
        for(int j=0;j<13;j++){
            this->state[j] = this->path[length-1][j];
        }
    }
    else{
        //cout << "4" <<endl;
        for (i=0; i<length-1; i++)
        {
            if ((this->path[i][0] <= time) && (this->path[i+1][0] >= time))
            {
                interpolateState(time,this->path[i],this->path[i+1]);
                break;
            }
        }
    }
    
}

bool LegPlanner::plan(int legNum){
    
    /*cout << "nInit[0] = " << this->sInitB[0] << endl;
    cout << "nInit[1] = " << this->sInitB[1] << endl;
    cout << "nInit[2] = " << this->sInitB[2] << endl;
    cout << "nInit[3] = " << this->sInitB[3] << endl;
    cout << "nInit[4] = " << this->sInitB[4] << endl;
    cout << "nInit[5] = " << this->sInitB[5] << endl;
    
    cout << "nGoal[0] = " << this->sGoalB[0] << endl;
    cout << "nGoal[1] = " << this->sGoalB[1] << endl;
    cout << "nGoal[2] = " << this->sGoalB[2] << endl;
    cout << "nGoal[3] = " << this->sGoalB[3] << endl;
    cout << "nGoal[4] = " << this->sGoalB[4] << endl;
    cout << "nGoal[5] = " << this->sGoalB[5] << endl;

    cout << "jointLimits[0] = " << this->jointLimits[0] << endl;
    cout << "jointLimits[1] = " << this->jointLimits[1] << endl;
    cout << "jointLimits[2] = " << this->jointLimits[2] << endl;
    cout << "jointLimits[3] = " << this->jointLimits[3] << endl;
    cout << "jointLimits[4] = " << this->jointLimits[4] << endl;
    cout << "jointLimits[5] = " << this->jointLimits[5] << endl;
    cout << "jointLimits[6] = " << this->jointLimits[6] << endl;
    cout << "jointLimits[7] = " << this->jointLimits[7] << endl;
    cout << "jointLimits[8] = " << this->jointLimits[8] << endl;
    cout << "jointLimits[9] = " << this->jointLimits[9] << endl;
    cout << "jointLimits[19] = " << this->jointLimits[19] << endl;

    cout << "kC.l1  = " << this->kC.l1 << endl;
    cout << "kC.l2  = " << this->kC.l2 << endl;
    cout << "kC.l3  = " << this->kC.l3 << endl;
    cout << "kC.l4  = " << this->kC.l4 << endl;
    cout << "kC.l5  = " << this->kC.l5 << endl;
    cout << "kC.l6  = " << this->kC.l6 << endl;
    cout << "kC.l7  = " << this->kC.l7 << endl;
    cout << "kC.l8  = " << this->kC.l8 << endl;
    cout << "kC.r  = " << this->kC.r << endl;
    cout << "kC.zeta  = " << this->kC.zeta << endl;
    cout << "kC.B2PXOffset  = " << this->kC.B2PXOffset << endl;
    cout << "kC.B2PZOffset  = " << this->kC.B2PZOffset << endl;
    cout << "kC.legAngleOffset[0]  = " << this->kC.legAngleOffset[0] << endl;
    cout << "kC.legAngleOffset[1]  = " << this->kC.legAngleOffset[1] << endl;
    cout << "kC.legAngleOffset[2]  = " << this->kC.legAngleOffset[2] << endl;
    cout << "kC.legAngleOffset[3]  = " << this->kC.legAngleOffset[3] << endl;

    cout << "phiInit = " << this->phiInit << endl;
    cout << "omegaInit = " << this->omegaInit << endl;

    cout << "legNum = " << legNum << endl;

    cout << "uBDot[0] = " << this->uBDot[0] << endl;
    cout << "uBDot[1] = " << this->uBDot[1] << endl;
    cout << "uBDot[2] = " << this->uBDot[2]  << endl;
    cout << "uBDot[3] = " << this->uBDot[3]  << endl;
    cout << "uBDot[4] = " << this->uBDot[4]  << endl;
    cout << "uBDot[5] = " << this->uBDot[5]  << endl;
    */

    //buildBiDirectionalRRTWrapper(this->sInitB, this->sGoalB, this->phiInit, this->omegaInit, this->jointLimits, &(this->kC), legNum, this->uBDot, this->emxT, this->emxT2,this->emxPathC,this->emxPathJ,&(this->success)); 
    buildRRTWrapper(this->sInitB, this->sGoalB, this->phiInit, this->omegaInit, this->jointLimits, &(this->kC), legNum, this->uBDot, this->emxT,this->emxPathC,this->emxPathJ,&(this->success)); 

    cout << "hello world !!!" << endl;

    if((bool) this->success){
	    printf("Path num dimensions = %d\n",emxPathC->numDimensions);
	    printf("Path size = [%d] x [%d]\n",emxPathC->size[0],emxPathC->size[1]);
	    this->path = new double*[emxPathC->size[0]];
	    int i,j;
        for (i=0; i<emxPathC->size[0]; i++){
		    this->path[i] = new double[emxPathC->size[1]];
		    printf("[");
            for (j=0; j<emxPathC->size[1]; j++){
		        this->path[i][j] = emxPathC->data[j*emxPathC->size[0]+i];
		        printf("%.4f ", this->path[i][j]);
		    }
		    printf("]\n\n");
	    }
	    this->pathLength = &(emxPathC->size[0]);
	}else{
        printf("Plan Failed\n");
    }

    return (bool) this->success;
}

void LegPlanner::interpolateState(double t, double* s1, double* s2)
{
    double t1, t2, tDiff, tRatio;
    int i;
    
    t1 = s1[0];
    t2 = s2[0];
    
    tDiff = t-t1;
    tRatio = tDiff/(t2-t1);
    
    this->state[0] = t;
    for(i=1;i<7;i++){
        this->state[i] = s1[i] + (s2[i]-s1[i])*tRatio;
    }
}
