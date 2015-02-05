#include <stdlib.h>
#include <iostream>
#include "buildRRTWrapper.h"
#include "buildRRTWrapper_emxAPI.h"
#include "LegPlanner.h"

using namespace std;

LegPlanner::LegPlanner()
{
    this->plan();
}

void LegPlanner::getState(double time){
    cout << "hello world from get state." << endl;
    
    int length = *(this->pathLength);
    int i;
    
    if(time <= this->path[0][0])
    {
        cout << "BEFORE!" << endl;
        for(int j=0;j<7;j++){
            this->state[j] = this->path[0][j];
        }
    }
    else if(time >= this->path[length-1][0])
    {
        cout << "AFTER!" << endl;
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

void LegPlanner::plan(void){
    
    cout << "hello world from plan." << endl;
    buildRRTWrapper(this->nInit, this->nGoal, this->NUM_NODES, this->jointLimits, this->K, this->HGAINS,
            this->NODE_SIZE, this->U, this->U_SIZE, this->dt, this->Dt, this->kinematicConst, this->ankleThreshold,
            this->exhaustive, this->threshold, this->goalSeedFreq, this->emxT, this->emxPath, this->emxPathJoint);
    
    //printf("Path num dimensions = %d\n",emxPath->numDimensions);
    //printf("Path size = [%d] x [%d]\n",emxPath->size[0],emxPath->size[1]);

    //double path[emxPath->size[0]][emxPath->size[1]];
    this->path = new double*[emxPath->size[0]];
    int i,j;
    for (i=0; i<emxPath->size[0]; i++){
        this->path[i] = new double[emxPath->size[1]];
        for (j=0; j<emxPath->size[1]; j++){
            this->path[i][j] = emxPath->data[j*emxPath->size[0]+i];
            //printf("%.2f ", this->path[i][j]);
        }
        //printf("]\n\n");
    }
    this->pathLength = &(emxPath->size[0]);
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
