%GETCONSTRAINEDBETADOTDOT This function calculates the acceleration of
%beta given a pan height constraint and an indpendent gamma angle.

function betaDotDot = getConstrainedBetaDotDot(kC,qDotDot,qDot,q)

    gammaDotDot = qDotDot(3);
    betaDot = qDot(2);
    gammaDot = qDot(3);
    beta = q(2);
    gamma = q(3);

    betaDotDot = (-gammaDotDot*kC.l5*cos(kC.zeta+gamma) + gammaDot^2*kC.l5*sin(kC.zeta+gamma) - betaDot^2*kC.l3*sin(beta))/(kC.l3*cos(beta));
end