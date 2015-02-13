function [TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices(uG,q,kC,legNum)
%GENERATETRMATRICES Summary of this function goes here
%   Detailed explanation goes here
    
    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = q(4);
        
    TB2G = [eye(3) [uG(1); uG(2); uG(3)]; 0 0 0 1];
    TP2B = generateDHTransMatrix(kC.legAngleOffset(legNum),kC.B2PZOffset,kC.B2PXOffset,0);
    TI2P = generateDHTransMatrix(alpha,kC.l1,kC.l2,-pi/2);
    TJ2I = generateDHTransMatrix(beta,0,kC.l3,0);
    TO2J = generateDHTransMatrix(-beta+kC.zeta,0,kC.l4,0);
    TQ2O = generateDHTransMatrix(gamma,0,kC.l5,0);
    TR2Q = generateDHTransMatrix(-gamma-kC.zeta,0,-kC.l7,-pi/2);
    TS2R = generateDHTransMatrix(phi,kC.l6,0,0);
    TW2S = generateDHTransMatrix(pi/2,kC.l8,0,pi/2);
    TC2W = generateDHTransMatrix(0,0,0,pi/2)*generateDHTransMatrix(pi/2,-kC.r,0,0);

end

