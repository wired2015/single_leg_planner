%buildBiDirectionalRRT.m
%author: wreid
%date: 20150107

function [T1,T2,path] = buildBiDirectionalRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum)
%buildRRT Icrementally builds a rapidly exploring random tree.
%   An RRT is build by incrementally selecting a random state from the
%   available state space as defined by the MIN and MAX vectors. The tree is
%   started at xInit and is extended until the number of maximum nodes, K has
%   been reached. A path is selected if the goal region as defined by xGoal
%   has been reached by the RRT.

    %Constant Declaration                                                      
    transitionArrayLength = (round(Dt/dt)+1)*10;    
    
    T1 = zeros(NUM_NODES/2,NODE_SIZE+transitionArrayLength);     
    T2 = zeros(NUM_NODES/2,NODE_SIZE+transitionArrayLength);
    
    T1(1,:) = [nInit zeros(1,transitionArrayLength)];           
    T2(1,:) = [nGoal zeros(1,transitionArrayLength)];
    
    nMid1 = [];
    nMid2 = [];
    
    nodeID1Count = 1;
    nodeID2Count = 1;

    for i = 2:NUM_NODES
        
        [T1,nodeID1Count,nNew] = rrtLoop(T1,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeID1Count,nGoal,goalSeedFreq,uBDot,legNum);
        [nConnect,~,d] = nearestNeighbour(nNew,T2,HGAINS,jointLimits,kC,nodeID2Count,NODE_SIZE);
        if i <= 2
            dMin = d;
            nMid1 = [nNew nMid1];
            nMid2 = [nConnect nMid2];
        elseif d < dMin
            dMin = d;
            nMid1 = [nNew nMid1];
            nMid2 = [nConnect nMid2];
        end
        
        %Swap the trees.
        TTemp = T1;
        T1 = T2;
        T2 = TTemp;
        nodeIDTempCount = nodeID1Count;
        nodeID1Count = nodeID2Count;
        nodeID2Count = nodeIDTempCount;
        nMidTemp = nMid1;
        nMid1 = nMid2;
        nMid2 = nMidTemp;
    
    end
    
    for i = length(nMid1)
    
        pathT1 = traceBranch(T1,nMid1,NODE_SIZE);
        pathT2 = traceBranch(T2,nMid2,NODE_SIZE);
    
        if T1(1,1:NODE_SIZE) == nInit
            path = [pathT1;flipud(pathT2)];
        else
            path = [pathT2;flipud(pathT1)];
            TTemp = T1;
            T1 = T2;
            T2 = TTemp;
        end
       
        pathLength = getPathLength(path);
        
        if i <= 1
            pathLengthMin = pathLength;
        elseif pathLength < pathLengthMin
            pathLengthMin = pathLength;
        end
        
    end
    
    [pathH,~] = size(path);
    t = dt*(1:pathH)';
    path = [t path];
    
end

function getPathLength(path)
end

function path = traceBranch(T,midPoint,NODE_SIZE)
    
    %Assignn the 
    check = midPoint(1);
    next = midPoint;
    path = [];
    transitionArray = T(midPoint(1),(NODE_SIZE+1):end);
    
    %Iterate over the tree until the initial state has been found.
    while check ~= 0 && next(2) ~=0

        transitionPath = [];
        for i = 1:10:length(transitionArray)
            transitionPath = [transitionPath; transitionArray(i:i+9)];
        end
        path = [transitionPath; path];
        
        next = T(next(2),:);
        check = next(2);
        
        transitionArray = next(14:end);
    end
end

function [T,nodeIDCount,xNew] = rrtLoop(T,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq,uBDot,legNum)
    
    xRand = randomState(jointLimits,cartesianLimits,panHeight,nGoal,nodeIDCount,goalSeedFreq,kC);
    [xNear,~,~] = nearestNeighbour(xRand,T,HGAINS,jointLimits,kC,nodeIDCount,NODE_SIZE);
    [xNew,transitionArray]  = selectInput(xNear,xRand,U,dt,Dt,NODE_SIZE,U_SIZE,HGAINS,kC,ankleThreshold,jointLimits,uBDot,legNum);
    nodeIDCount = nodeIDCount + 1;

    xNew(1) = nodeIDCount;                                  %Node ID
    xNew(2) = xNear(1);                                     %Parent ID
    xNew(3) = xNear(3) + heuristicSingleLeg(xNew,xNear,HGAINS,jointLimits,kC);           %Cost

    T(nodeIDCount,:) = [xNew transitionArray];                                %Append the new node to the tree.    
    %if mod(nodeIDCount,100) == 0
        %fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
    %end
end

