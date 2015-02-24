%heuristicSingleLeg.m
%author: wreid
%date: 20150107

function d = heuristicSingleLeg(xA,xB,HGAINS,jointLimits,kC)
%heuristic Calculates the distance between states x1 and x2.

    alphaA = xA(4); 
    betaA = xA(5);
    gammaA = xA(6);
    phiA = xA(7);
    epsilonA = xA(8);
    
    alphaB = xB(4);
    betaB = xB(5);
    gammaB = xB(6);
    phiB = xB(7);
    epsilonB = xB(8);
    
    alphaDotA = xA(9);
    betaDotA = xA(10);
    gammaDotA = xA(11);
    phiDotA = xA(12);
    omegaA = xA(13);
    
    alphaDotB = xB(9);
    betaDotB = xB(10);
    gammaDotB = xB(11);
    phiDotB = xB(12);
    omegaB = xB(13);
    
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
    
    dPosNorm = dPos/dPosMax;
    
    uA = sherpaTTFK([alphaA betaA gammaA],kC);
    uB = sherpaTTFK([alphaB betaB gammaB],kC);
    
    d = norm(uB-uA);
    
    
    %dVel = (alphaDotB - alphaDotA) + (betaDotB - betaDotA) + (gammaDotB - gammaDotA);
    %dVelNorm = jointLimits(2,6) - jointLimits(1,6) + jointLimits(2,7) - jointLimits(1,7) + jointLimits(2,8) - jointLimits(1,8);
    
%    uA = sherpaTTFK(xA(4:6),kC);
%    uB = sherpaTTFK(xB(4:6),kC);
    %dPos = norm(uA-uB);
    
    %Calculate the total distance.
    %d = HGAINS(1)*dPosNorm;%+HGAINS(2)*dVelNorm; 
    
end

function xStar = legRadius(beta,gamma,kC)
    xStar = kC.l2+kC.l3*cos(beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma)-kC.l7;
end

function dPos = posMetric(x,dx,dTheta)
    dPos = sqrt(dx^2+x^2*dTheta^2);
end

function dPos = cartMetric(dx,dy)
    dPos = sqrt(dx^2+dy^2);
end


