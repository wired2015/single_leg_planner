%randomState.m
%author: wreid
%date: 20150107

function xRand = randomState(jointLimits,cartesianLimits,panHeight,nGoal,nodeIDCount,goalSeedFreq,kC)
%randomState Picks a random state from the state space.
%   A random state is selected from the state space within the boundaries of
%   the state space as defined by the MIN and MAX vectors. The state space has
%   a dimension n.
%   Inputs:
%       MIN:    The 1xn vector containing the minimum boundaries for the state
%               space.
%       MAX:    The 1xn vector containing the maximum boundaries for the state
%               space.
%   Outputs:
%       xRand:  The 1xn vector describing the selected random state.

    %[~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);
    
    phi = 0;
    success = false;
    
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
    %betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*kC.l2*alphaDotRand - 2.238e31*kC.l6*alphaDotRand - 1.827e47*kC.l6*gammaDotRand + 2.238e31*kC.l3*alphaDotRand*cos(betaRand) + 1.827e47*kC.l3*gammaDotRand*cos(betaRand) - 2.238e31*kC.l2*alphaDotRand*cos(phi) + 2.238e31*kC.l6*alphaDotRand*cos(phi) - 1.37e15*kC.l6*gammaDotRand*cos(phi) + 2.238e31*kC.l4*alphaDotRand*cos(kC.zeta) + 1.827e47*kC.l4*gammaDotRand*cos(kC.zeta) + 2.74e15*kC.l7*alphaDotRand*sin(phi) + 2.74e15*kC.l8*alphaDotRand*sin(phi) + 2.238e31*kC.l7*gammaDotRand*sin(phi) + 2.238e31*kC.l8*gammaDotRand*sin(phi) - 2.237e31*kC.l3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(kC.zeta) + 1.827e47*kC.l5*gammaDotRand*cos(gammaRand)*cos(kC.zeta) - 2.237e31*kC.l4*alphaDotRand*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*kC.l5*alphaDotRand*sin(gammaRand)*sin(kC.zeta) - 1.827e47*kC.l5*gammaDotRand*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*gammaDotRand*sin(phi)*sin(kC.zeta) - 2.237e31*kC.l5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(kC.zeta) + 2.237e31*kC.l5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*gammaDotRand*sin(gammaRand)*cos(kC.zeta)*sin(phi)))/(1.827e47*kC.l4*cos(kC.zeta) - 1.37e15*kC.l6*cos(phi) - 1.827e47*kC.l6 + 2.238e31*kC.l7*sin(phi) + 2.238e31*kC.l8*sin(phi) + 1.827e47*kC.l5*cos(gammaRand)*cos(kC.zeta) - 1.827e47*kC.l5*sin(gammaRand)*sin(kC.zeta) + 2.237e31*kC.l4*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*cos(gammaRand)*sin(phi)*sin(kC.zeta) + 2.237e31*kC.l5*sin(gammaRand)*cos(kC.zeta)*sin(phi));
    
    xRand = [zeros(1,3) alphaRand betaRand gammaRand 0 0 alphaDotRand betaDotRand gammaDotRand 0 0];
    if mod(nodeIDCount,goalSeedFreq) == 0
        xRand = nGoal;
    end

end

