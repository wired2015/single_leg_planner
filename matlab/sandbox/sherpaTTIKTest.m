%sherpaTTIKTest.m
%author: wreid
%date: 20150122

clear
clc

%Kinematic Constants.
sherpaTTKinematicConstants

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta];

jointLimits = [deg2rad([-135 -59.5 -5]);    %[rad]
              deg2rad([135 17 73.7])];      %[rad]

%Start with an x,y,z coordinate.
x = -0.4122;
y = -0.6819;
z = -0.8;

%Find the joint angles using the inverse kinematics of the leg.
[alpha,beta,gamma] = sherpaTTIK(x,y,z,kinematicConst,jointLimits);

%Plug the returned joint angles into the forward kinematics to check that
%the solution is correct.
[x,y,z] = sherpaTTFK(alpha,beta,gamma,kinematicConst);

%Print the results.
fprintf('x: %.2f m\n',x);
fprintf('y: %.2f m\n',y);
fprintf('z: %.2f m\n',z);

fprintf('alpha = %.2f deg\n',rad2deg(alpha));
fprintf('beta  = %.2f deg\n',rad2deg(beta));
fprintf('gamma = %.2f deg\n',rad2deg(gamma));

fprintf('x: %.2f m\n',x);
fprintf('y: %.2f m\n',y);
fprintf('z: %.2f m\n',z);

