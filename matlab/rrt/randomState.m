%randomState.m
%author: wreid
%date: 20150107

function xRand = randomState(range,MIN,kinematicConst,panHeight,xGoal,nodeIDCount,goalSeedFreq)
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

    [~,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);
    
    phi = 0;

    alphaRand = range(1)*rand+MIN(1);
    gammaRand = range(2)*rand+MIN(2);
    %betaRand = -asin((K-L1+L4*sin(zeta)+L5*sin(gammaRand+zeta)+L6+L8)/L3);
    betaRand = getConstrainedBeta(panHeight,gammaRand,kinematicConst);
    
    KVel = 0;
    alphaDotRand = range(4)*rand+MIN(4);
    gammaDotRand = range(5)*rand+MIN(5);
    betaDotRand = -(1.0*(1.827e47*KVel + 2.238e31*L2*alphaDotRand - 2.238e31*L6*alphaDotRand - 1.827e47*L6*gammaDotRand + 2.238e31*L3*alphaDotRand*cos(betaRand) + 1.827e47*L3*gammaDotRand*cos(betaRand) - 2.238e31*L2*alphaDotRand*cos(phi) + 2.238e31*L6*alphaDotRand*cos(phi) - 1.37e15*L6*gammaDotRand*cos(phi) + 2.238e31*L4*alphaDotRand*cos(zeta) + 1.827e47*L4*gammaDotRand*cos(zeta) + 2.74e15*L7*alphaDotRand*sin(phi) + 2.74e15*L8*alphaDotRand*sin(phi) + 2.238e31*L7*gammaDotRand*sin(phi) + 2.238e31*L8*gammaDotRand*sin(phi) - 2.237e31*L3*alphaDotRand*cos(betaRand)*cos(phi) + 2.238e31*L5*alphaDotRand*cos(gammaRand)*cos(zeta) + 1.827e47*L5*gammaDotRand*cos(gammaRand)*cos(zeta) - 2.237e31*L4*alphaDotRand*cos(phi)*cos(zeta) + 2.237e31*L3*gammaDotRand*sin(betaRand)*sin(phi) - 2.238e31*L5*alphaDotRand*sin(gammaRand)*sin(zeta) - 1.827e47*L5*gammaDotRand*sin(gammaRand)*sin(zeta) + 2.237e31*L4*gammaDotRand*sin(phi)*sin(zeta) - 2.237e31*L5*alphaDotRand*cos(gammaRand)*cos(phi)*cos(zeta) + 2.237e31*L5*alphaDotRand*cos(phi)*sin(gammaRand)*sin(zeta) + 2.237e31*L5*gammaDotRand*cos(gammaRand)*sin(phi)*sin(zeta) + 2.237e31*L5*gammaDotRand*sin(gammaRand)*cos(zeta)*sin(phi)))/(1.827e47*L4*cos(zeta) - 1.37e15*L6*cos(phi) - 1.827e47*L6 + 2.238e31*L7*sin(phi) + 2.238e31*L8*sin(phi) + 1.827e47*L5*cos(gammaRand)*cos(zeta) - 1.827e47*L5*sin(gammaRand)*sin(zeta) + 2.237e31*L4*sin(phi)*sin(zeta) + 2.237e31*L5*cos(gammaRand)*sin(phi)*sin(zeta) + 2.237e31*L5*sin(gammaRand)*cos(zeta)*sin(phi));
    
    xRand = [zeros(1,3) alphaRand gammaRand betaRand alphaDotRand gammaDotRand betaDotRand];
    if mod(nodeIDCount,goalSeedFreq) == 0
        xRand = xGoal;
    end

end

