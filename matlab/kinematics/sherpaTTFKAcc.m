%sherpaTTFKAcc.m
%author: wreid
%date: 20150123

function uDotDot = sherpaTTFKAcc(qDotDot,qDot,q,kinematicConst)
%sherpaTTFKAcc Sherpa_TT single leg forward acceleration kinematics.

    L1 = kinematicConst(1);
    L2 = kinematicConst(2);
    L3 = kinematicConst(3);
    L4 = kinematicConst(4);
    L5 = kinematicConst(5);
    L6 = kinematicConst(6);
    L7 = kinematicConst(7);
    L8 = kinematicConst(8);
    zeta = kinematicConst(9);
    
    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = 0;
    
    alphaDot = qDot(1);
    betaDot = qDot(2);
    gammaDot = qDot(3);

    alphaDotDot = qDotDot(1);
    betaDotDot = qDotDot(2);
    gammaDotDot = qDotDot(3);
    
%     J = [                                                                                                                                                              0.5*L3*sin(beta - 1.0*phi) - 0.5*L4*sin(phi + zeta) - 0.5*L3*sin(beta + phi) - 1.225e-16*L7*cos(phi) - 1.225e-16*L8*cos(phi) - 0.5*L4*sin(phi - 1.0*zeta) - 1.0*L2*sin(phi) + 1.0*L6*sin(phi) - 0.5*L5*sin(gamma + phi + zeta) + 0.5*L5*sin(gamma - 1.0*phi + zeta),                                                                                                                           0.5*L4*sin(phi - 1.0*zeta) - 0.5*L4*sin(phi + zeta) - 0.5*L3*sin(beta - 1.0*phi) - 1.0*L7*cos(phi) - 1.0*L8*cos(phi) - 0.5*L3*sin(beta + phi) - 6.123e-17*L6*sin(phi) - 0.5*L5*sin(gamma + phi + zeta) - 0.5*L5*sin(gamma - 1.0*phi + zeta),                                                                                            0.5*L4*sin(phi - 1.0*zeta) - 1.0*L7*cos(phi) - 1.0*L8*cos(phi) - 0.5*L4*sin(phi + zeta) - 6.123e-17*L6*sin(phi) - 0.5*L5*sin(gamma + phi + zeta) - 0.5*L5*sin(gamma - 1.0*phi + zeta);
%                                                        1.5e-32*L2 - 1.5e-32*L6 + 0.5*L3*cos(beta + phi) + 1.5e-32*L5*cos(gamma + zeta) + 0.5*L4*cos(phi + zeta) + 0.5*L3*cos(beta - 1.0*phi) + 1.5e-32*L3*cos(beta) + 0.5*L4*cos(phi - 1.0*zeta) + 1.0*L2*cos(phi) - 1.0*L6*cos(phi) + 1.5e-32*L4*cos(zeta) - 1.225e-16*L7*sin(phi) - 1.225e-16*L8*sin(phi) + 0.5*L5*cos(gamma + phi + zeta) + 0.5*L5*cos(gamma - 1.0*phi + zeta),                         0.5*L3*cos(beta + phi) - 1.225e-16*L6 + 1.225e-16*L5*cos(gamma + zeta) + 0.5*L4*cos(phi + zeta) - 0.5*L3*cos(beta - 1.0*phi) + 1.225e-16*L3*cos(beta) - 0.5*L4*cos(phi - 1.0*zeta) + 6.123e-17*L6*cos(phi) + 1.225e-16*L4*cos(zeta) - 1.0*L7*sin(phi) - 1.0*L8*sin(phi) + 0.5*L5*cos(gamma + phi + zeta) - 0.5*L5*cos(gamma - 1.0*phi + zeta),                   1.225e-16*L5*cos(gamma + zeta) - 1.225e-16*L6 + 0.5*L4*cos(phi + zeta) - 0.5*L4*cos(phi - 1.0*zeta) + 6.123e-17*L6*cos(phi) + 1.225e-16*L4*cos(zeta) - 1.0*L7*sin(phi) - 1.0*L8*sin(phi) + 0.5*L5*cos(gamma + phi + zeta) - 0.5*L5*cos(gamma - 1.0*phi + zeta);
%         1.225e-16*L6 - 1.225e-16*L2 + 6.123e-17*L3*cos(beta + phi) - 1.225e-16*L5*cos(gamma + zeta) + 6.123e-17*L4*cos(phi + zeta) + 6.123e-17*L3*cos(beta - 1.0*phi) - 1.225e-16*L3*cos(beta) + 6.123e-17*L4*cos(phi - 1.0*zeta) + 1.225e-16*L2*cos(phi) - 1.225e-16*L6*cos(phi) - 1.225e-16*L4*cos(zeta) - 1.5e-32*L7*sin(phi) - 1.5e-32*L8*sin(phi) + 6.123e-17*L5*cos(gamma + phi + zeta) + 6.123e-17*L5*cos(gamma - 1.0*phi + zeta), 1.0*L6 + 6.123e-17*L3*cos(beta + phi) - 1.0*L5*cos(gamma + zeta) + 6.123e-17*L4*cos(phi + zeta) - 6.123e-17*L3*cos(beta - 1.0*phi) - 1.0*L3*cos(beta) - 6.123e-17*L4*cos(phi - 1.0*zeta) + 7.499e-33*L6*cos(phi) - 1.0*L4*cos(zeta) - 1.225e-16*L7*sin(phi) - 1.225e-16*L8*sin(phi) + 6.123e-17*L5*cos(gamma + phi + zeta) - 6.123e-17*L5*cos(gamma - 1.0*phi + zeta), 1.0*L6 - 1.0*L5*cos(gamma + zeta) + 6.123e-17*L4*cos(phi + zeta) - 6.123e-17*L4*cos(phi - 1.0*zeta) + 7.499e-33*L6*cos(phi) - 1.0*L4*cos(zeta) - 1.225e-16*L7*sin(phi) - 1.225e-16*L8*sin(phi) + 6.123e-17*L5*cos(gamma + phi + zeta) - 6.123e-17*L5*cos(gamma - 1.0*phi + zeta)];
%     
%     JDot = [ 0,                        -6.163e-33*L3*(8.113e31*cos(beta + phi) + 8.113e31*cos(beta - 1.0*phi)),                              -6.163e-33*L5*(8.113e31*cos(gamma + phi + zeta) + 8.113e31*cos(gamma - 1.0*phi + zeta));
%             0, -1.233e-32*L3*(9.936e15*sin(beta) + 8.113e31*cos(beta)*sin(phi) - 4.968e15*cos(phi)*sin(beta)), -6.163e-33*L5*(8.113e31*sin(gamma + phi + zeta) - 8.113e31*sin(gamma - 1.0*phi + zeta) + 1.987e16*sin(gamma + zeta));
%             0,   3.039e-64*L3*(3.291e63*sin(beta) - 4.03e47*cos(beta)*sin(phi) + 2.468e31*cos(phi)*sin(beta)),    1.519e-64*L5*(4.03e47*sin(gamma - 1.0*phi + zeta) - 4.03e47*sin(gamma + phi + zeta) + 6.582e63*sin(gamma + zeta))];
%     
    %uDotDot = J*qDotDot + JDot*qDot;
    
    xDotDot =   -alphaDotDot*sin(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
            -alphaDot^2*cos(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
            +alphaDot*betaDot*L3*sin(beta)*sin(alpha)...
            +alphaDot*gammaDot*L5*sin(zeta+gamma)*sin(alpha)...
            -betaDotDot*L3*sin(beta)*cos(alpha)...
            -betaDot^2*L3*cos(beta)*cos(alpha)...
            -gammaDotDot*L5*sin(zeta+gamma)*cos(alpha)...
            -gammaDot^2*L5*cos(zeta+gamma)*cos(alpha);
    
    yDotDot =   alphaDotDot*cos(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
            -alphaDot^2*sin(alpha)*(L2 - L7 + L5*cos(gamma + zeta) + L3*cos(beta) + L4*cos(zeta))...
            -alphaDot*betaDot*L3*cos(alpha)*sin(beta)...
            -alphaDot*gammaDot*L5*sin(zeta+gamma)*cos(alpha)...
            -gammaDotDot*L5*sin(zeta+gamma)*sin(alpha)...
            -gammaDot^2*L5*cos(zeta+gamma)*sin(alpha)...
            -betaDotDot*L3*sin(alpha)*sin(beta)...
            -betaDot^2*L3*sin(alpha)*cos(beta);
        
    zDotDot =   -betaDotDot*L3*cos(beta)...
            +betaDot^2*L3*sin(beta)...
            -gammaDotDot*L5*cos(zeta+gamma)...
            +gammaDot^2*L5*sin(zeta+gamma);
        
    uDotDot = [xDotDot; yDotDot; zDotDot];

end