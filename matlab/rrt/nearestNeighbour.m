%nearestNeigbour.m
%author: wreid
%date: 20150107

function [xNear,transitionArray,d] = nearestNeighbour(x,T,HGAINS,jointLimits,kinematicConst,nodeIDCount,NODE_SIZE)
%nearestNeigbour Finds the node in the tree closest to x.
%   This function scans each node within the tree and finds the node that
%   is closest to the xRand node. The nearest node is returned by the
%   function. A distance heuristic is  used
%   Inputs:
%       x:  The 1xn state that each node in the tree will be compared to,
%           to find the node with the minimum distance to it. n refers to
%           the number of dimensions within the state space.
%       T:  The nxm tree being searched, m is the number of possible nodes
%           within the tree.
%       HGAINS: The gains applied to the heuristic function.
%   Outputs:
%       xNear:  The node in the tree that is closet to x.

    %Iterate over the entire tree and apply the distance heuristic function
    %to each node.
    d = zeros(1,nodeIDCount);
    
    %parfor i = 1:nodeIDCount
    for i = 1:nodeIDCount
        d(i) = heuristicSingleLeg(x,T(i,:),HGAINS,jointLimits,kinematicConst);
    end
    
    [d,minIndex] = min(d(1:nodeIDCount));
    %[d,minIndex] = min(d(1:nodeIDCount));
    xNear = T(minIndex,1:NODE_SIZE);
    transitionArray = T(minIndex,(NODE_SIZE+1):end);
end

