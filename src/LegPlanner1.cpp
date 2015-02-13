#include <stdlib.h>
#include <iostream>
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxAPI.h"
#include "LegPlanner.h"

using namespace std;

LegPlanner::LegPlanner(int legNum)
{
    this->plan(legNum);
}

void LegPlanner::getState(double time){
    
    int length = *(this->pathLength);
    int i;

    if(time <= this->path[0][0])
    {
        for(int j=0;j<7;j++){
            this->state[j] = this->path[0][j];
        }
    }
    else if(time >= this->path[length-1][0])
    {
        for(int j=0;j<7;j++){
            this->state[j] = this->path[length-1][j];
        }
    }
    else{
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
    
/*    cout << "nInit[0] = " << nInit[0] << endl;
    cout << "nInit[1] = " << nInit[1] << endl;
    cout << "nInit[2] = " << nInit[2] << endl;
    cout << "nInit[3] = " << nInit[3] << endl;
    cout << "nInit[4] = " << nInit[4] << endl;
    cout << "nInit[5] = " << nInit[5] << endl;
    
    cout << "nGoal[0] = " << nGoal[0] << endl;
    cout << "nGoal[1] = " << nGoal[1] << endl;
    cout << "nGoal[2] = " << nGoal[2] << endl;
    cout << "nGoal[3] = " << nGoal[3] << endl;
    cout << "nGoal[4] = " << nGoal[4] << endl;
    cout << "nGoal[5] = " << nGoal[5] << endl;

    cout << "jointLimits[0] = " << jointLimits[0] << endl;
    cout << "jointLimits[1] = " << jointLimits[1] << endl;
    cout << "jointLimits[2] = " << jointLimits[2] << endl;
    cout << "jointLimits[3] = " << jointLimits[3] << endl;
    cout << "jointLimits[4] = " << jointLimits[4] << endl;
    cout << "jointLimits[5] = " << jointLimits[5] << endl;

    cout << "kinematicConst[0]  = " << kinematicConst[0] << endl;
    cout << "kinematicConst[1]  = " << kinematicConst[1] << endl;
    cout << "kinematicConst[2]  = " << kinematicConst[2] << endl;
    cout << "kinematicConst[3]  = " << kinematicConst[3] << endl;
    cout << "kinematicConst[4]  = " << kinematicConst[4] << endl;
    cout << "kinematicConst[5]  = " << kinematicConst[5] << endl;
    cout << "kinematicConst[6]  = " << kinematicConst[6] << endl;
    cout << "kinematicConst[7]  = " << kinematicConst[7] << endl;
    cout << "kinematicConst[8]  = " << kinematicConst[8] << endl;
    cout << "kinematicConst[9]  = " << kinematicConst[9] << endl;
    cout << "kinematicConst[10] = " << kinematicConst[10] << endl;
    cout << "kinematicConst[11] = " << kinematicConst[11] << endl;
    cout << "kinematicConst[12] = " << kinematicConst[12] << endl;
    cout << "kinematicConst[13] = " << kinematicConst[13] << endl;
    cout << "kinematicConst[14] = " << kinematicConst[14] << endl;

    cout << "this->success = " << (bool) this->success << endl;
*/

    buildRRTWrapper(this->nInit, this->nGoal, this->jointLimits, this->K, this->U, this->dt_integration, this->dt_planner, this->kinematicConst, this->threshold, legNum, this->emxT,this->emxPathC,this->emxPathJ,&(this->success)); 
  
    if((bool) this->success){
	    //printf("Path num dimensions = %d\n",emxPathC->numDimensions);
	    //printf("Path size = [%d] x [%d]\n",emxPathC->size[0],emxPathC->size[1]);

	    //double path[emxPathC->size[0]][emxPathC->size[1]];
	    this->path = new double*[emxPathC->size[0]];
	    int i,j;
	    for (i=0; i<emxPathC->size[0]; i++){
		this->path[i] = new double[emxPathC->size[1]];
		for (j=0; j<emxPathC->size[1]; j++){
		    this->path[i][j] = emxPathC->data[j*emxPathC->size[0]+i];
		    //printf("%.2f ", this->path[i][j]);
		}
		//printf("]\n\n");
	    }
	    this->pathLength = &(emxPathC->size[0]);
	}else{
        //printf("Plan Failed\n");
    }

    //cout << "this->success " << this->success<< endl;

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
