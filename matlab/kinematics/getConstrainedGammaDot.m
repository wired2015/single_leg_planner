%GETCONSTRAINEDGAMMADOT This function calculates the velocity of
%gamma given a pan height constraint and an independent beta angle.
%
%getConstrainedGammaDot.m
%author: wreid
%date: 20150224

function gammaDot = getConstrainedGammaDot(kC,qDot,q)

    betaDot = qDot(2);
    beta = q(2);
    gamma = q(3);

    gammaDot = -betaDot*kC.l3*cos(beta)/(kC.l5*cos(kC.zeta+gamma));

end

