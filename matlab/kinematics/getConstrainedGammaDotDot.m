function gammaDotDot = getConstrainedGammaDotDot(kC,qDotDot,qDot,q)
%getConstrainedGammaDotDot Summary of this function goes here
%   Detailed explanation goes here

    %[~,~,L3,~,L5,~,~,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);

    betaDotDot = qDotDot(2);
    betaDot = qDot(2);
    gammaDot = qDot(3);
    beta = q(2);
    gamma = q(3);

    gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma));

end

