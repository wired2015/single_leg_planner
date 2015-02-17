%GENERATETRMATRICES Generates each of the homogeneous transformation
%matrices that describe the kinematic chain between the Sherpa_TT rover's
%body coordinate frame and its wheel contact frame. Denavit-Hartenburg
%parameters are used to express the transformation between each coordinate
%frame in the kinematic chain.
%
%Inputs:
%-uG: A 1x3 vector giving the [x y z] relationship between the body and
%coordinate frame
%-q: A 1x4 vector describing the leg's joint state. This vector includes 
%[alpha beta gamma].
%-kC: A struct containing the kinematic parameters of the Sherpa_TT leg.
%-legNum: The number of the leg that is being considered (1,2,3 or 4).
%
%Outputs:
%TB2G: Transformation from the body to the ground.
%TP2B: Transformation from the pan joint to the body.
%TI2P: Transformation from the inner leg joint to the pan joint.
%TJ2I: Transformation from the inner leg knee joint to the inner leg joint.
%TO2J: Transformation from the outer leg joint to the inner leg knee joint.
%TQ2O: Transformation from the outer leg end joint to the outer leg joint.
%TR2Q: Transformation from the steering base joint to the outer leg end
%joint.
%TS2R: Transformation from the steering joint to the steering base joint.
%TW2S: Transformation from the wheel joint to the steering joint.
%TC2W: Transformation from the wheel contact point to the wheel joint.
%
%generateTrMatrices.m
%author:    wreid
%date:      20140214

function [TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices(uG,q,kC,legNum)    

    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    phi = q(4);
        
    %TODO: Use a 6-DOF relationship between the ground and body frames by
    %including the roll, pitch and yaw of the platform.
    TB2G = [eye(3) [uG(1); uG(2); uG(3)]; 0 0 0 1];
    TP2B = trDH(kC.legAngleOffset(legNum),kC.B2PZOffset,kC.B2PXOffset,0);
    TI2P = trDH(alpha,kC.l1,kC.l2,-pi/2);
    TJ2I = trDH(beta,0,kC.l3,0);
    TO2J = trDH(-beta+kC.zeta,0,kC.l4,0);
    TQ2O = trDH(gamma,0,kC.l5,0);
    TR2Q = trDH(-gamma-kC.zeta,0,-kC.l7,-pi/2);
    TS2R = trDH(phi,kC.l6,0,0);
    TW2S = trDH(pi/2,kC.l8,0,pi/2);
    TC2W = trDH(0,0,0,pi/2)*trDH(pi/2,-kC.r,0,0);

end

