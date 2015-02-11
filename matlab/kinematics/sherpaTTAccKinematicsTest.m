%sherpaTTAccKinematicsTest.m
%author: wreid
%date: 20150123

clear
clc

sherpaTTKinematicConstants

xDot = 0;
yDot = 0;
zDot = 0;

xDotDot = 1;
yDotDot = 0;
zDotDot = 0;

% alphaDotDot = 0.1;
% betaDotDot = 0;
% gammaDotDot = 0;
% 
% alphaDot = 1;
% betaDot = -1;
% gammaDot = 1;

alpha = 0;
beta = deg2rad(-35);
gamma = deg2rad(50);

alphaDot = 0.1;
betaDot = 0;
gammaDot = 0;

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta];

%uDotDot = sherpaTTFKAcc([alphaDotDot betaDotDot gammaDotDot]',[alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);
%qDotDot = sherpaTTIKAcc(uDotDot,[alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);     

qDotDot = sherpaTTIKAcc([xDotDot yDotDot zDotDot]',[alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);
uDotDot = sherpaTTFKAcc(qDotDot,[alphaDot betaDot gammaDot]',[alpha beta gamma]',kinematicConst);

%Print the results.
% fprintf('alphaDotDot: \t%.2f rad/s^2\n',alphaDotDot);
% fprintf('betaDotDot: \t%.2f rad/s^2\n',betaDotDot);
% fprintf('gammaDotDot: \t%.2f rad/s^2\n',gammaDotDot);
% 
% fprintf('xDot = \t\t%.2f m/s^2\n',uDotDot(1));
% fprintf('yDot  = \t%.2f m/s^2\n',uDotDot(2));
% fprintf('zDot = \t\t%.2f m/s^2\n',uDotDot(3));

fprintf('alphaDotDot: \t%.2f rad/s^2\n',qDotDot(1));
fprintf('betaDotDot: \t%.2f rad/s^2\n',qDotDot(2));
fprintf('gammaDotDot: \t%.2f rad/s^2\n',qDotDot(3));

fprintf('xDot = \t\t%.2f m/s^2\n',uDotDot(1));
fprintf('yDot  = \t%.2f m/s^2\n',uDotDot(2));
fprintf('zDot = \t\t%.2f m/s^2\n',uDotDot(3));
             
%xDotDot = J_CB*qDotDot+J_CB_dot*qDot
%qDotDot = J_CB_inv*(xDotDot-J_CB_dot*qDot)
%qDotDot = [alphaDotDot; gammaDotDot; betaDotDot]

