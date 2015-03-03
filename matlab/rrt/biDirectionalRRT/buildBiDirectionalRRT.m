%buildBiDirectionalRRT.m
%author: wreid
%date: 20150107

function [T1,T2,pathJ,pathC] = buildBiDirectionalRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum,TP2B)
%buildRRT Icrementally builds a rapidly exploring random tree.
%   An RRT is build by incrementally selecting a random state from the
%   available state space as defined by the MIN and MAX vectors. The tree is
%   started at xInit and is extended until the number of maximum nodes, K has
%   been reached. A path is selected if the goal region as defined by xGoal
%   has been reached by the RRT.

    %Constant Declaration                                                      
    transitionArrayLength = (round(Dt/dt)+1)*10;    
    
    T1 = zeros(round(NUM_NODES/2),NODE_SIZE+transitionArrayLength);     
    T2 = zeros(round(NUM_NODES/2),NODE_SIZE+transitionArrayLength);
    
    T1(1,:) = [nInit zeros(1,transitionArrayLength)];           
    T2(1,:) = [nGoal zeros(1,transitionArrayLength)];
    
    nMid1 = [];
    nMid2 = [];
    dMin = 100;
    
    nodeIDCount1 = 1;
    nodeIDCount2 = 1;
    
    path = [];
    pathCount = 1;
    
    pathLengthMin = 100;
    pathCMin = [];
    pathJ = [];
    pathC = [];

    for i = 3:NUM_NODES
        
        [T1,~,~] = rrtLoop(T1,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount1,nGoal,goalSeedFreq,uBDot,legNum);
        nodeIDCount1 = nodeIDCount1 + 1;

        %Swap the trees.
        TTemp = T1;
        T1 = T2;
        T2 = TTemp;
        
        %Swap the trees.
        nodeIDCountTemp = nodeIDCount1;
        nodeIDCount1 = nodeIDCount2;
        nodeIDCount2 = nodeIDCountTemp;
        
    end
    
    [T1H,~] = size(T1);
    [T2H,~] = size(T2);
    
    for i = 1:T1H
        
        nMid1 = T1(i,1:NODE_SIZE);
        [nMid2,~,d] = nearestNeighbour(nMid1,T2,[0.9 0.1 0],jointLimits,kC,T2H,NODE_SIZE);
        
        if d < 0.04
            pathT1 = traceBranch(T1,nMid1,NODE_SIZE);
            pathT2 = traceBranch(T2,nMid2,NODE_SIZE);
            if T1(1,4) == nInit(1,4) && T1(1,5) == nInit(1,5) && T1(1,6) == nInit(1,6)
                path = [pathT1;flipud(pathT2)];
            else
                path = [pathT2;flipud(pathT1)];
            end
            
            [pathH,~] = size(path);
            t = dt*(1:pathH)';
            path = [t path];

            pathC = transformPath(path,kC,TP2B);
            pathLength = pathC(end,2);

            if (pathLength < pathLengthMin)
                pathLengthMin = pathLength;
                pathCMin = pathC;
                pathJ = path;
            end
        end
    end
    pathC = pathCMin;
end

function path = traceBranch(T,midPoint,NODE_SIZE)
    
    %Assignn the 
    check = midPoint(1);
    next = midPoint;
    path = [];
    transitionArray = [];
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

function pathC = transformPath(pathJ,kC,TP2B)
    [pathH,~] = size(pathJ);
    pathC = zeros(pathH,9);
    dist2Go = 0;
    for i = 1:pathH
        uP = sherpaTTFK(pathJ(i,2:4),kC);
        uPDot = sherpaTTFKVel(pathJ(i,7:9),pathJ(i,2:4),kC);
        uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
        uBDot = TP2B(1:3,1:3)*uPDot;
        if i ~= 1
            dist2Go = dist2Go + norm(uB'-pathC(i-1,3:5));
        end
        pathC(i,:) = [pathJ(i,1) dist2Go uB' uBDot' false];
    end
end

