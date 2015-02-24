%GETCONSTRAINEDGAMMADOTDOT This function calculates the acceleration of
%gamma given a pan height constraint and an independent beta angle.

function gammaDotDot = getConstrainedGammaDotDot(kC,qDotDot,qDot,q)

    betaDotDot = qDotDot(2);
    betaDot = qDot(2);
    gammaDot = qDot(3);
    beta = q(2);
    gamma = q(3);

    gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma));

end

