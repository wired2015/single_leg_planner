%randomState Picks a random state from the state space.
%   A random state is selected from the state space within the boundaries of
%   the state space as defined by the MIN and MAX vectors. The state space has
%   a dimension n.
%   Inputs:
%   -jointLimits: The 1xn vector containing the minimum boundaries for the
%   state space.
%   -cartesianLimits: The 1xn vector containing the maximum boundaries for
%   the state space.
%   Outputs:
%   - xRand:  The 1xn vector describing the selected random state.
%
%randomStateVariableHeight.m
%author: wreid
%date: 20150107

function nRand = randomState(jointLimits,cartesianLimits,kC)
    
    alphaMin = jointLimits(1,1);
    alphaMax = jointLimits(2,1);
    betaMin = jointLimits(1,2);
    betaMax = jointLimits(2,2);
    gammaMin = jointLimits(1,3);
    gammaMax = jointLimits(2,3);
    
    alphaDotMin = jointLimits(1,5);
    alphaDotMax = jointLimits(2,5);
    betaDotMin = jointLimits(1,6);
    betaDotMax = jointLimits(2,6);
    gammaDotMin = jointLimits(1,7);
    gammaDotMax = jointLimits(2,7);

    alphaRand = alphaMin + (alphaMax-alphaMin)*rand;
    
    zPMax = cartesianLimits(1);
    zPMin = cartesianLimits(2);
    zPMid1 = cartesianLimits(3);
    zPMid2 = cartesianLimits(4);
    
    if panHeight <= zPMax && panHeight >= zPMid1
        xMax = getXStar(panHeight,gammaMin,false,kC);
        xMin = getXStar(panHeight,betaMin,true,kC);
    elseif panHeight < zPMid1 && panHeight >= zPMid2
        xMax = getXStar(panHeight,gammaMin,false,kC);
        xMin = getXStar(panHeight,gammaMax,false,kC);
    elseif  panHeight < zPMid2 && panHeight >= zPMin
        xMax = getXStar(panHeight,betaMax,true,kC);
        xMin = getXStar(panHeight,gammaMax,false,kC);
    else
        xMax = 0;
        xMin = 0;
        disp('z is out of range');
    end
    
    xRand = xMin + (xMax-xMin)*rand;
    
    u  = [xRand 0 panHeight];
    q = sherpaTTIK(u,kC,jointLimits);
    betaRand = q(2);
    gammaRand = q(3);
    
    KVel = 0;
    
    alphaDotRand = (alphaDotMax-alphaDotMin)*rand+alphaDotMin;
    betaDotRand = (betaDotMax-betaDotMin)*rand+betaDotMin;
    gammaDotRand = (gammaDotMax-gammaDotMin)*rand+gammaDotMin;
    
    nRand = [zeros(1,3) alphaRand betaRand gammaRand 0 0 alphaDotRand betaDotRand gammaDotRand 0 0];


end

