%sherpaTTAnkleKinematics.m
%author: wreid
%date: 20150128

%Kinematic Constants.
L1 = 0.175*cosd(35);        %[m]
L2 = 0.045;                 %[m]
L3 = 0.400;                 %[m]
L4 = 0.2;                   %[m]
L5 = 0.4;                   %[m]
L6 = 0.159*sind(34.5);      %[m]
L7 = 0.159*cosd(34.5);      %[m]
L8 = 0.39569+0.378/2;       %[m]
zeta = deg2rad(7);

alphaDot = 0;
betaDot = 0.1;
gammaDot = 0.1;

alphaDotDot = 0;
betaDotDot = 0;
gammaDotDot = 0;

alpha = 0;
beta = deg2rad(-35);
gamma = deg2rad(50);

xStar = (L2+L3*cos(-beta)+L4*cos(zeta)+L5*cos(zeta+gamma)-L7);

xADot = -betaDot*L3*sin(beta)-gammaDot*L5*sin(zeta+gamma);
yADot = alphaDot*xStar;

qA = rad2deg(atan2(xADot,yADot));

% w = xADot/yADot;
% 
% A = -1/alphaDotDot^2*((-L3*betaDot*sin(beta)-gammaDot*L5*sin(zeta+gamma))/(xStar));
% B = (alphaDot*(-betaDotDot*L3*sin(beta)-L3*betaDot^2*cos(beta))*xStar-alphaDot*betaDot*(betaDot*L3*sin(beta)+gammaDot*L5*sin(zeta+gamma))*(L3*sin(beta)))/(alphaDot^2*xStar^2);
% C = (alphaDot*(-gammaDotDot*L5*sin(zeta+gamma)-gammaDot^2*L5*cos(zeta+gamma))*xStar-alphaDot*gammaDot*(L3*betaDot*sin(beta)+gammaDot*L5*sin(zeta+gamma))*L5*sin(zeta+gamma))/(alphaDot^2*xStar^2);
% 
% dw = A + B + C;
% 
% qADot = 1/(1+w^2)*dw;
