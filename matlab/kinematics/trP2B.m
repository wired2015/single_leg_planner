function TP2B = trP2B(kC,legNum)
%TRP2B Calculates the homogeneous transformation matrix between the body
%and pan coordinate frames.

TP2B = generateDHTransMatrix(kC.legAngleOffset(legNum),kC.B2PZOffset,kC.B2PXOffset,0);

end

