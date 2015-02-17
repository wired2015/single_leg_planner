%TRP2B Generates the homogeneous transformation matrix between the body
%and pan coordinate frames. kC is a struct containing the kinematic
%constants of the leg. legNum indicates the number of the leg that is being
%considered.
%
%Inputs:
%-kC: Struct of kinematic constants of the Sherpa_TT leg
%-legNum: The leg number identification.
%Outputs:
%-TP2B: The homogeneous transformation matrix that is used to transform
%coordinates from the pan frame to the body frame.
%
%trP2B.m
%author:    wreid
%date:      20150214

function TP2B = trP2B(kC,legNum)
    TP2B = trDH(kC.legAngleOffset(legNum),kC.B2PZOffset,kC.B2PXOffset,0);
end

