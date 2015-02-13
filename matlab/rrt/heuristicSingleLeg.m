%heuristicSingleLeg.m
%author: wreid
%date: 20150107

function d = heuristicSingleLeg(xA,xB,HGAINS,jointLimits,kC)
%heuristic Calculates the distance between states x1 and x2.

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
    xStarMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC);
    xStarMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC);
    
    dxStarMax = xStarMax-xStarMin;
    dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));
    
    dPosMax = posMetric(xStarMin,dxStarMax,dAlphaMax);
    
    xStarA = legRadius(betaA,gammaA,kC);
    xStarB = legRadius(betaB,gammaB,kC);
    
    dxStar = xStarB-xStarA;
    dAlpha = angDiff(alphaA,alphaB);
    
    dPos = sqrt(dxStar^2+xStarA^2*dAlpha^2);
    dPosNorm = HGAINS(1)*dPos/dPosMax;
    
    %Calculate the total distance.
    d = dPos; %dPosNorm+dVelNorm 
    
end

function xStar = legRadius(beta,gamma,kC)
    xStar = kC.l2+kC.l3*cos(beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma)-kC.l7;
end

function dPos = posMetric(x,dx,dTheta)
    dPos = sqrt(dx^2+x^2*dTheta^2);
end


