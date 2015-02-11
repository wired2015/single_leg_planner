%sherpaTTVelKinematicsTest.m
%author: wreid
%date: 20150122

clear
clc


sherpaTTKinematicConstants

alphaDot = 0.1;
betaDot = 0;
gammaDot = 0;

alpha = 0;
beta = 0;
gamma = deg2rad(-7);

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta];

[uDot,uDotNew] = sherpaTTFKVel([alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);

%qDot = sherpaTTIKVel(uDot,[alpha beta gamma],kinematicConst);

%Print the results.
fprintf('\nAngular Velocities.\n')


fprintf('alphaDot: \t%.2f rad/s\n',alphaDot);
fprintf('betaDot: \t%.2f rad/s\n',betaDot);
fprintf('gammaDot: \t%.2f rad/s\n\n',gammaDot);

fprintf('Using Jacobian based solution.\n')

fprintf('xDot = \t\t%.2f m/s\n',uDot(1));
fprintf('yDot  = \t%.2f m/s\n',uDot(2));
fprintf('zDot = \t\t%.2f m/s\n\n',uDot(3));

fprintf('Using analytical solution.\n')

fprintf('xDot = \t\t%.2f m/s\n',uDotNew(1));
fprintf('yDot  = \t%.2f m/s\n',uDotNew(2));
fprintf('zDot = \t\t%.2f m/s\n',uDotNew(3));

% fprintf('alphaDot: \t%.2f rad/s\n',qDot(1));
% fprintf('betaDot: \t%.2f rad/s\n',qDot(2));
% fprintf('gammaDot: \t%.2f rad/s\n',qDot(3));

