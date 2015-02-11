%buildRRT.m
%author: wreid
%date: 20150107

function [T,path] = buildRRT(nInit,nGoal,NUM_NODES,jointLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kinematicConst,ankleThreshold,exhaustive,threshold,goalSeedFreq)
%buildRRT Icrementally builds a rapidly exploring random tree.
%   An RRT is build by incrementally selecting a random state from the
%   available state space as defined by the MIN and MAX vectors. The tree is
%   started at xInit and is extended until the number of maximum nodes, K has
%   been reached. A path is selected if the goal region as defined by xGoal
%   has been reached by the RRT.

    %Constant Declaration                                                      
    
    transitionArrayLength = (round(Dt/dt)+1)*6;    
    
    %Variable Initialization
    T = zeros(NUM_NODES,NODE_SIZE+transitionArrayLength);     %Define a zero array that will be used to 
                                            %store data from each tree node.
    T(1,:) = [nInit zeros(1,transitionArrayLength)];                         %Initialize the tree with initial state.
    nodeIDCount = 1;
    jointRange = jointLimits(2,:) - jointLimits(1,:);
    
    if ~exhaustive
    
        for i = 2:NUM_NODES
            [T,nodeIDCount] = rrtLoop(T,jointRange,jointLimits,kinematicConst,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq);
        end
        
    else
        %TODO: calculate maximum distance given the configuration space.
        dist = 100;
        %TODO: make the threshold distance
        while dist > threshold && nodeIDCount < NUM_NODES 
            [T,nodeIDCount] = rrtLoop(T,jointRange,jointLimits,kinematicConst,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq);
            [~,~,dist] = nearestNeighbour(nGoal,T,HGAINS,jointLimits,kinematicConst,nodeIDCount,NODE_SIZE);
            if mod(nodeIDCount,100) == 0
                %fprintf('PROGRESS STATUS: dist = %.3f\n',dist);
            end
        end   
    end
    
    %Find the closest node in the tree to the goal node.
    [xNearest,transitionArrayNearest,~] = nearestNeighbour(nGoal,T,HGAINS,jointLimits,kinematicConst,nodeIDCount,NODE_SIZE);
        
    check = xNearest(1);
    next = xNearest;
    path = [next transitionArrayNearest];
    while check ~= 0 && next(2) ~= 0
        next = T(next(2),:);
        path = [path; next];
        check = next(2);
    end
    
end

function [T,nodeIDCount] = rrtLoop(T,jointRange,jointLimits,kinematicConst,panHeight,U,Dt,dt,NODE_SIZE,U_SIZE,HGAINS,ankleThreshold,nodeIDCount,nGoal,goalSeedFreq)
    
    xRand = randomState(jointRange,jointLimits(1,:),kinematicConst,panHeight,nGoal,nodeIDCount,goalSeedFreq);
    [xNear,~,~] = nearestNeighbour(xRand,T,HGAINS,jointLimits,kinematicConst,nodeIDCount,NODE_SIZE);
    [xNew,transitionArray]  = selectInput(xNear,xRand,U,dt,Dt,NODE_SIZE,U_SIZE,HGAINS,kinematicConst,ankleThreshold,jointLimits);
    nodeIDCount = nodeIDCount + 1;

    xNew(1) = nodeIDCount;                                  %Node ID
    xNew(2) = xNear(1);                                     %Parent ID
    xNew(3) = heuristicSingleLeg(xNew,xNear,HGAINS,jointLimits,kinematicConst);           %Cost

    T(nodeIDCount,:) = [xNew transitionArray];                                %Append the new node to the tree.    
    if mod(nodeIDCount,100) == 0
        %fprintf('PROGRESS STATUS: %.0f NODES USED\n',nodeIDCount);
    end
end

