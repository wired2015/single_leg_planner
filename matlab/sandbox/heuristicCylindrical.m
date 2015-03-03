function [dS,eta] = heuristicCylindrical(qA,qB,kC,jointLimits)
    
    alphaA = qA(1);
    betaA = qA(2);
    gammaA = qA(3);
    
    alphaB = qB(1);
    betaB = qB(2);
    gammaB = qB(3);

    %Calculate the distance between angular positions.
    rMin = legRadius(jointLimits(1,2),jointLimits(1,3),kC);
    rMax = legRadius(jointLimits(2,2),jointLimits(2,3),kC);

    drMax = rMax-rMin;
    dAlphaMax = angDiff(jointLimits(1,1),jointLimits(1,2));

    dPosMax = posMetric(rMin,drMax,dAlphaMax);

    rA = legRadius(betaA,gammaA,kC);
    rB = legRadius(betaB,gammaB,kC);

    dr = rB-rA;
    dAlpha = angDiff(alphaA,alphaB);

    dS = sqrt(dr^2+rA^2*dAlpha^2);
    
    dx = dr*cos(alphaA)-dAlpha*rA*sin(alphaA);
    dy = dr*sin(alphaA)+dAlpha*rA*cos(alphaA);
    
    eta = atan2(dy,dx);

    %dSNorm = dPos/dPosMax;
end



function dPos = posMetric(x,dx,dTheta)
    dPos = sqrt(dx^2+x^2*dTheta^2);
end