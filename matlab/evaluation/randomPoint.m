function [sRandomCartesian] = randomPoint(jointLimits,cartesianLimits,panHeight,kC,legNum)

    randomInitJoint = randomState(jointLimits,cartesianLimits,panHeight,zeros(1,13),1,0,kC);
    
    q = randomInitJoint(4:6);
    uP = sherpaTTFK(q,kC);
    
    TP2B = trP2B(kC,legNum);
    uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
    
    sRandomCartesian = uB';
    
    
end