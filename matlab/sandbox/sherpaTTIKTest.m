%sherpaTTIKTest.m
%author: wreid
%date: 20150122

clear
clc

%Kinematic Constants.
kinematicConstants


jointLimits = [deg2rad([-135 -59.5 -5]);    %[rad]
              deg2rad([135 17 73.7])];      %[rad]

%Start with an x,y,z coordinate.
x = -0.4122;
y = -0.6819;
z = -0.8;

u = [x y z];

%Find the joint angles using the inverse kinematics of the leg.
q = sherpaTTIK(u,kC,jointLimits);

%Plug the returned joint angles into the forward kinematics to check that
%the solution is correct.
u = sherpaTTFK(q,kC);

%Print the results.
fprintf('x: %.2f m\n',x);
fprintf('y: %.2f m\n',y);
fprintf('z: %.2f m\n',z);

fprintf('alpha = %.2f deg\n',rad2deg(q(1)));
fprintf('beta  = %.2f deg\n',rad2deg(q(2)));
fprintf('gamma = %.2f deg\n',rad2deg(q(3)));

fprintf('x: %.2f m\n',u(1));
fprintf('y: %.2f m\n',u(2));
fprintf('z: %.2f m\n',u(3));

