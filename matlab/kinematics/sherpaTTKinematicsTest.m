%sherpaTTVelKinematicsTest.m
%author: wreid
%date: 20150122

clear
clc


sherpaTTKinematicConstants

alphaDotDot = 0.1;
betaDotDot = 0;
gammaDotDot = 0;

alphaDot = 0.1;
betaDot = 0;
gammaDot = 0;

alpha = 0;
beta = 0;
gamma = deg2rad(-7);

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta];

[uDot,uDotNew] = sherpaTTFKVel([alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);
[uDotDot] = sherpaTTFKAcc([alphaDotDot betaDotDot gammaDotDot]',[alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);

%qDot = sherpaTTIKVel(uDot,[alpha beta gamma],kinematicConst);

%Print the results.
fprintf('\nAngular Velocities:\n')

fprintf('alphaDot: \t%.5f rad/s\n',alphaDot);
fprintf('betaDot: \t%.5f rad/s\n',betaDot);
fprintf('gammaDot: \t%.5f rad/s\n\n',gammaDot);

fprintf('Cartesian Velocities:\n')

fprintf('xDot = \t\t%.5f m/s\n',uDotNew(1));
fprintf('yDot  = \t%.5f m/s\n',uDotNew(2));
fprintf('zDot = \t\t%.5f m/s\n\n',uDotNew(3));

fprintf('Angular Accelerations:\n')

fprintf('alphaDotDot: \t%.5f rad/s^2\n',alphaDotDot);
fprintf('betaDotDot: \t%.5f rad/s^2\n',betaDotDot);
fprintf('gammaDotDot: \t%.5f rad/s^2\n\n',gammaDotDot);

fprintf('Cartesian Accelerations:\n')

fprintf('xDotDot = \t%.5f m/s^2\n',uDotDot(1));
fprintf('yDotDot  = \t%.5f m/s^2\n',uDotDot(2));
fprintf('zDotDot = \t%.5f m/s^2\n',uDotDot(3));

% fprintf('alphaDot: \t%.2f rad/s\n',qDot(1));
% fprintf('betaDot: \t%.2f rad/s\n',qDot(2));
% fprintf('gammaDot: \t%.2f rad/s\n',qDot(3));

