function gammaDotDot = getConstrainedGammaDotDot(kinematicConst,qDotDot,qDot,q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    [~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);

    betaDotDot = qDotDot(2);
    betaDot = qDot(2);
    gammaDot = qDot(3);
    beta = q(2);
    gamma = q(3);

    gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma));

end

