%sherpaTTFKVel.m
%author: wreid
%date: 20150122

function [uDot,uDotNew] = sherpaTTFKVel(qDot,q,kinematicConst)
%sherpaTTFKVel Sherpa_TT single leg forward velocity kinematics.

    [~,L2,L3,L4,L5,~,L7,~,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);

    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = 0;
    
    alphaDot = qDot(1);
    betaDot = qDot(2);
    gammaDot = qDot(3);

    xDot =      - alphaDot*sin(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
                - betaDot*L3*cos(alpha)*sin(beta)...
                - gammaDot*L5*sin(gamma + zeta)*cos(alpha);
    yDot =      alphaDot*cos(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
                -gammaDot*L5*sin(gamma + zeta)*sin(alpha)...
                -betaDot*L3*sin(alpha)*sin(beta);
    zDot =      -betaDot*L3*cos(beta)...
                -L5*gammaDot*cos(zeta+gamma);

    uDot = [xDot yDot zDot]';

end