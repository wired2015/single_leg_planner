function [L1,L2,L3,L4,L5,L6,L7,L8,zeta,r,B2PXOffset,B2PZOffset,leg1AngleOffset,leg2AngleOffset,leg3AngleOffset,leg4AngleOffset] = extractKinematicConstants(kinematicConst)
%EXTRACTKINEMATICCONSTANTS Gets individual kinematic constants.
%   Takes the kinematicConst array and returns each of the individual
%   kinematic constants.

    L1 = kinematicConst(1);
    L2 = kinematicConst(2); 
    L3 = kinematicConst(3); 
    L4 = kinematicConst(4); 
    L5 = kinematicConst(5); 
    L6 = kinematicConst(6); 
    L7 = kinematicConst(7); 
    L8 = kinematicConst(8); 
    zeta = kinematicConst(9); 
    r = kinematicConst(10); 
    B2PXOffset = kinematicConst(11); 
    B2PZOffset = kinematicConst(12); 
    leg1AngleOffset = kinematicConst(13); 
    leg2AngleOffset = kinematicConst(14);
    leg3AngleOffset = kinematicConst(15); 
    leg4AngleOffset = kinematicConst(16);
    
end

