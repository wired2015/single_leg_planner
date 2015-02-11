function [TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices(uG,q,kinematicConst,legNum)
%GENERATETRMATRICES Summary of this function goes here
%   Detailed explanation goes here

    [L1,L2,L3,L4,L5,L6,L7,L8,zeta,r,B2PXOffset,B2PZOffset,leg1AngleOffset,leg2AngleOffset,leg3AngleOffset,leg4AngleOffset] = extractKinematicConstants(kinematicConst);
    
    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = q(4);
    
    switch legNum
    
    case 1
        legAngleOffset = leg1AngleOffset;
    case 2
        legAngleOffset = leg2AngleOffset;
    case 3
        legAngleOffset = leg3AngleOffset;
    otherwise
        legAngleOffset = leg4AngleOffset;
    end
    
    TB2G = [eye(3) [uG(1); uG(2); uG(3)]; 0 0 0 1];
    TP2B = generateDHTransMatrix(legAngleOffset,B2PZOffset,B2PXOffset,0);
    TI2P = generateDHTransMatrix(alpha,L1,L2,-pi/2);
    TJ2I = generateDHTransMatrix(beta,0,L3,0);
    TO2J = generateDHTransMatrix(-beta+zeta,0,L4,0);
    TQ2O = generateDHTransMatrix(gamma,0,L5,0);
    TR2Q = generateDHTransMatrix(-gamma-zeta,0,-L6,-pi/2);
    TS2R = generateDHTransMatrix(phi,L7,0,0);
    TW2S = generateDHTransMatrix(0,L8-r,0,pi);
    TC2W = generateDHTransMatrix(0,-r,0,0);

end

