%calcPhi.m

%function [qS,qWDot] = calcPhi(qDot,q,kinematicConst,sS,qWDot,r)
function [phi,qWDot] = calcPhi(qDot,q,kC),uDotB,legNum)

    alphaDot = qDot(1);
    betaDot = qDot(2);
    gammaDot = qDot(3);
    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    
    vB = uDotB(1:3);
    omegaB = uDotB(4:6);
    
    %[v_D,phi_D] = motionDueToBodyRate(vB,omegaB);
     
    vD = [0 0];
    vL = [-kC.l3*betaDot*sin(beta) - gammaDot*kC.l5*sin(kC.zeta+gamma); alphaDot*(kC.l2+kC.l3*cos(beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma))];
    
    v = vL + vD;    

    qWDot = norm(v);
    
    phi = atan(v(1)/v(2));

end