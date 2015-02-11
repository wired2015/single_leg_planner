function TP2B = trP2B(kinematicConst,legNum)
%TRP2B Calculates the homogeneous transformation matrix between the body
%and pan coordinate frames.

[~,~,~,~,~,~,~,~,~,~,B2PXOffset,B2PZOffset,leg1AngleOffset,...
leg2AngleOffset,leg3AngleOffset,leg4AngleOffset] = ...
extractKinematicConstants(kinematicConst);

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

TP2B = generateDHTransMatrix(legAngleOffset,B2PZOffset,B2PXOffset,0);

end

