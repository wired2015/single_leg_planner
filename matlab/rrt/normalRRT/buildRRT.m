%buildRRT.m
%author: wreid
%date: 20150107

function [T,path] = buildRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum)
%buildRRT Icrementally builds a rapidly exploring random tree.
%   An RRT is build by incrementally selecting a random state from the
%   available state space as defined by the MIN and MAX vectors. The tree is
%   started at xInit and is extended until the number of maximum nodes, K has
%   been reached. A path is selected if the goal region as defined by xGoal
%   has been reached by the RRT.

    %Constant Declaration                                                      
    transitionArrayLength = (round(Dt/dt)+1)*10;    
    
    %Variable Initialization
    T = zeros(NUM_NODES,NODE_SIZE+transitionArrayLength);     %Define a zero array that will be used to 
                                            %store data from each tree node.
    T(1,:) = [nInit zeros(1,transitionArrayLength)];                         %Initialize the tree with initial state.
    nodeIDCount = 1;
    
    
    if ~exhaustive
    
        for i = 2:NUM_NODES
            [T,nodeIDCount] = rrtLoop(T,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq,uBDot,legNum);
        end
        
    else
        %TODO: calculate maximum distance given the configuration space.
        dist = 100; %[m]
        %TODO: make the threshold distance
        while dist > threshold && nodeIDCount < NUM_NODES 
            [T,nodeIDCount] = rrtLoop(T,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq,uBDot,legNum);
            [~,~,dist] = nearestNeighbour(nGoal,T,HGAINS,jointLimits,kC,nodeIDCount,NODE_SIZE);
            if mod(nodeIDCount,100) == 0
                %fprintf('PROGRESS STATUS: dist = %.3f\n',dist);
            end
        end   
    end
    
    %Find the closest node in the tree to the goal node.
    [xNearest,transitionArrayNearest,~] = nearestNeighbour(nGoal,T,HGAINS,jointLimits,kC,nodeIDCount,NODE_SIZE);
        
    check = xNearest(1);
    next = [];
    next = xNearest;
    path = [];
    transitionArray = [];
    transitionArray = transitionArrayNearest;
    
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
    
    [pathH,~] = size(path);
    t = dt*(1:pathH)';
    path = [t path];
    
end

function [T,nodeIDCount] = rrtLoop(T,jointLimits,cartesianLimits,kC,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq,uBDot,legNum)
    
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

