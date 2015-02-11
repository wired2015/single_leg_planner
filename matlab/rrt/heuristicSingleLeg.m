%heuristicSingleLeg.m
%author: wreid
%date: 20150107

function d = heuristicSingleLeg(xA,xB,HGAINS,jointLimits,kinematicConst)
%heuristic Calculates the distance between states x1 and x2.

    L1 = kinematicConst(1);
    L2 = kinematicConst(2);
    L3 = kinematicConst(3);
    L4 = kinematicConst(4);
    L5 = kinematicConst(5);
    L6 = kinematicConst(6);
    L7 = kinematicConst(7);
    L8 = kinematicConst(8);
    zeta = kinematicConst(9);

    alphaA = xA(4); 
    betaA = xA(5);
    gammaA = xA(6);
    
    alphaB = xB(4);
    betaB = xB(5);
    gammaB = xB(6);
    
    alphaDotA = xA(7);
    betaDotA = xA(8);
    gammaDotA = xA(9);
    
    alphaDotB = xB(7);
    betaDotB = xB(8);
    gammaDotB = xB(9);
    
    %Calculate the distance between angular positions.
    xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),zeta,L2,L3,L4,L5,L7);
    xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),zeta,L2,L3,L4,L5,L7);
    
    dxStarMax = xStarMax-xStarMin;
    dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));
    
    dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax);
    
    xStarA = legRadius(betaA,gammaA,zeta,L2,L3,L4,L5,L7);
    xStarB = legRadius(betaB,gammaB,zeta,L2,L3,L4,L5,L7);
    
    dxStar = xStarB-xStarA;
    dAlpha = angDiff(alphaA,alphaB);
    
    dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2);
    dPosNorm = HGAINS(1)*dPos/dPosMax;
    
    %Calculate the total distance.
    d = dPos; %dPosNorm+dVelNorm 
    
end

function xStar = legRadius(beta,gamma,zeta,L2,L3,L4,L5,L7)
    xStar = L2+L3*cos(beta)+L4*cos(zeta)+L5*cos(zeta+gamma)-L7;
end

function dPos = posMetric(x,dx,dTheta)
    dPos = sqrt(dx^2+x^2*dTheta^2);
end


