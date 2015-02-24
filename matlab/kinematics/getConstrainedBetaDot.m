%GETCONSTRAINEDBETAADOT This function calculates the velocity of
%gamma given a pan height constraint and an independent beta angle.
%
%getConstrainedBetaDot.m
%author: wreid
%date: 20150224

function betaDot = getConstrainedBetaDot(kC,qDot,q)

    gammaDot = qDot(3);
    beta = q(2);
    gamma = q(3);

    betaDot = (-gammaDot*kC.l5*cos(kC.zeta+gamma))/(kC.l3*cos(beta));

end