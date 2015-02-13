%sherpaTTFKVel.m
%author: wreid
%date: 20150122

function uDot = sherpaTTFKVel(qDot,q,kC)
%sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.

    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = 0;
    
    alphaDot = qDot(1);
    betaDot = qDot(2);
    gammaDot = qDot(3);

    xDot =      - alphaDot*sin(alpha)*(kC.l2 - kC.l7 + kC.l5*cos(gamma + kC.zeta) + kC.l3*cos(beta) + kC.l4*cos(kC.zeta))...
                - betaDot*kC.l3*cos(alpha)*sin(beta)...
                - gammaDot*kC.l5*sin(gamma + kC.zeta)*cos(alpha);
    yDot =      alphaDot*cos(alpha)*(kC.l2 - kC.l7 + kC.l5*cos(gamma + kC.zeta) + kC.l3*cos(beta) + kC.l4*cos(kC.zeta))...
                -gammaDot*kC.l5*sin(gamma + kC.zeta)*sin(alpha)...
                -betaDot*kC.l3*sin(alpha)*sin(beta);
    zDot =      -betaDot*kC.l3*cos(beta)...
                -kC.l5*gammaDot*cos(kC.zeta+gamma);

    uDot = [xDot yDot zDot]';

end