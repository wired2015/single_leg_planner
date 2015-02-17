%sherpaTTJacobianSymbolicAnalysis.m
%author: wreid
%date: 20150121
%This script is used to generate the Jacobian that maps the velocity of a
%Sherpa_TT's leg joint space to the leg's wheel/ground contact point.
%The inverse Jacobian is calculated. This allows the leg's contact point 
%rates to be mapped to the joint rate space. Additionally, the derivative
%of the Jacobian is calculated for the leg's acceleration kinematics.

clear
clc
clf

symbolic = false;

if symbolic
    %Create symbolic variables.
    syms L1 L2 L3 L4 L5 L6 L7 L8 alpha beta gamma zeta phi alphaDot betaDot gammaDot
else
    sherpaTTKinematicConstants
    alpha = deg2rad(0);
    beta = deg2rad(0);
    %zeta = deg2rad(7);
    zeta = 0.1305;
    gamma = deg2rad(0);
    phi = 0;
end

%Generate the homogeneous transformation matrices from the DH parameters.
TP2B = generateDHTransMatrix(pi/4,B2PZOffset,B2PXOffset,0);
TI2P = generateDHTransMatrix(alpha,L1,L2,-pi/2);
TJ2I = generateDHTransMatrix(beta,0,L3,0);
TO2J = generateDHTransMatrix(-beta+zeta,0,0.2204,0);
TQ2O = generateDHTransMatrix(gamma,0,L5,0);
TR2Q = generateDHTransMatrix(-gamma-zeta,0,-L6,-pi/2);
TS2R = generateDHTransMatrix(phi,L7,0,0);
TC2S = generateDHTransMatrix(0,L8,0,pi);

%Find the inverse transformation matrices.
TP2C = inv(TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S);
TI2C = inv(TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S);
TO2C = inv(TO2J*TQ2O*TR2Q*TS2R*TC2S);

TP2S = inv(TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R);
TI2S = inv(TJ2I*TO2J*TQ2O*TR2Q*TS2R);
TO2S = inv(TO2J*TQ2O*TR2Q*TS2R);

%TC2P = TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S;
%TC2I = TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S;
%TC2O = TO2J*TQ2O*TR2Q*TS2R*TC2S;

if symbolic
    %Generate the jacobian.
%     sigma_1 = TP2C(1:3,1:3)*[0 0 1]';
%     a = -TP2C(1:3,4);
%     
%     sigma_2 = TI2C(1:3,1:3)*[0 0 1]';
%     b = -TI2C(1:3,4);
%     
%     sigma_3 = TO2C(1:3,1:3)*[0 0 1]';
%     c = -TO2C(1:3,4);
%     
%     J_CB = simplify([cross(sigma_1,a) cross(sigma_2,b) cross(sigma_3,c)]);
%     %J_CB = (simplify(subs(J_CB,{L1,L2,L3,L4,L5,L6,L7,L8,zeta},[0.175*cosd(35) 0.45 0.400 0.2 0.4 0.159*sind(34.5) 0.159*cosd(34.5) 0.39569+0.378/2 deg2rad(7)])));
%     J_CB_rough = vpa(J_CB,4);
%     
%     J_CB_inv = simplify(inv(J_CB));
%     J_CB_inv_rough = vpa(J_CB_inv,4);
%     
%     J_CB_dot = simplify([diff(J_CB(1,1),alpha) diff(J_CB(1,2),beta) diff(J_CB(1,3),gamma);...
%                 diff(J_CB(2,1),alpha) diff(J_CB(2,2),beta) diff(J_CB(2,3),gamma);...
%                 diff(J_CB(3,1),alpha) diff(J_CB(3,2),beta) diff(J_CB(3,3),gamma)]);
    
    %J_CB_dot_rough = vpa(simplify(subs(J_CB_dot,{L1,L2,L3,L4,L5,L6,L7,L8,zeta},[0.175*cosd(35) 0.45 0.400 0.2 0.4 0.159*sind(34.5) 0.159*cosd(34.5) 0.39569+0.378/2 deg2rad(7)])),4)
    %J_CB_dot_rough = vpa(J_CB_dot,4);        
    
    %######################################################################
    %Ankle Angle Calculation
    %######################################################################        

    sigma_1 = TP2S(1:3,1:3)*[0 0 1]';
    a = -TP2S(1:3,4);
    
    sigma_2 = TI2S(1:3,1:3)*[0 0 1]';
    b = -TI2S(1:3,4);
    
    sigma_3 = TO2S(1:3,1:3)*[0 0 1]';
    c = -TO2S(1:3,4);
    
    J_SP = simplify(vpa([cross(sigma_1,a) cross(sigma_2,b) cross(sigma_3,c)],5));
    
    uDotP = J_SP*[alphaDot;betaDot;gammaDot];
    
    uDotS = simplify(TP2S(1:3,1:3)*uDotP)
    
    uDotSX = filterSymbolicExpression(uDotS(1),1e-4);
    uDotSY = filterSymbolicExpression(uDotS(2),1e-4);
    
    %######################################################################

else
    
    sigma_1 = TP2C(1:3,1:3)*[0 0 1]';
    a = -TP2C(1:3,4);
    
    sigma_2 = TI2C(1:3,1:3)*[0 0 1]';
    b = -TI2C(1:3,4);
    
    sigma_3 = TO2C(1:3,1:3)*[0 0 1]';
    c = -TO2C(1:3,4);
    
    axesLen = 0.25;
    
    %Plot the coordinate frames.
    hold on
    
    trplot(eye(4,4),'color','r','length',axesLen,'frame','P');
    trplot(TP2B,'color','b','length',axesLen,'frame','P');
    trplot(TP2B*TI2P,'length',axesLen,'frame','I');
    trplot(TP2B*TI2P*TJ2I,'length',axesLen,'frame','J');
    trplot(TP2B*TI2P*TJ2I*TO2J,'length',axesLen,'frame','O');
    trplot(TP2B*TI2P*TJ2I*TO2J*TQ2O,'length',axesLen,'frame','Q');
    trplot(TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q,'length',axesLen,'frame','R');
    trplot(TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R,'length',axesLen,'frame','S');
    trplot(TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S,'length',axesLen,'frame','C');
    axis equal
    view(0,0)
    axis([-2 2 -2 2 -2 2]);
    
    uDotTest = [0.1; 0; 0; 0; 0; 0];
    
    %J = tr2jac(TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R);
    
    TB2S = inv(TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R*TC2S);
    
    uDotNew = TB2S(1:3,1:3)*uDotTest(1:3)

end







