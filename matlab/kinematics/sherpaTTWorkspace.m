%sherpaTTWorkspace.m
%author: wreid
%date: 20150122

%This script is used to visualize the workspace of the Sherpa_TT leg. A
%ground contact constraint is imposed on the leg. This constraint is a
%value along the z-axis, denoted as K.

clear
clc

%Kinematic Constants.
L1 = 0.175*cosd(35);        %[m]
L2 = 0.045;                  %[m]
L3 = 0.400;                 %[m]
L4 = 0.2;                   %[m]
L5 = 0.4;                   %[m]
L6 = 0.159*sind(34.5);      %[m]
L7 = 0.159*cosd(34.5);      %[m]
L8 = 0.39569+0.378/2;       %[m]
zeta = deg2rad(7);

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta];

MIN = [deg2rad([-135 -59.5 -5]) -0.3 -0.7 -0.7];    %[rad, rad/s]
MAX = [deg2rad([135 17 73.7]) 0.3 0.7 0.7];         %[rad, rad/s]

jointLimits = [MIN;MAX];

%Minimum and maximum joint limits.
alphaMAX = deg2rad(135);    %[rad]
alphaMIN = deg2rad(-135);   %[rad]

betaMAX = deg2rad(17);      %[rad]
betaMIN = deg2rad(-59.5);   %[rad]

gammaMAX = deg2rad(73.7);   %[rad]
gammaMIN = deg2rad(-5);     %[rad]

%z-constraint constant.
K = -0.6;

vecLength = 40;

%Generate vectors for alpha, beta and gamma.
alpha = linspace(alphaMIN,alphaMAX,vecLength);
%beta = linspace(betaMIN,betaMAX,vecLength);
gamma = linspace(gammaMIN,gammaMAX,vecLength);
beta = -asin((K-L1+L4*sin(zeta)+L5*sin(gamma+zeta)+L6+L8)/L3);

xVec = zeros(1,vecLength^2);
yVec = zeros(1,vecLength^2);
zVec = zeros(1,vecLength^2);

%For each possible alpha,beta,gamma groups calculate the corresponding x,y,z
%Cartesian position of the contact point.
index = 1;

for i = 1:vecLength
    for j = 1:vecLength
        if (imag(beta(j)) == 0 && beta(j) >= betaMIN && beta(j) <=betaMAX)
            xVec(index) = (L2 + L3*cos(-beta(j))+L4*cos(zeta)+L5*cos(gamma(j)+zeta)-L7).*cos(alpha(i));
            yVec(index) = (L2 + L3*cos(-beta(j))+L4*cos(zeta)+L5*cos(gamma(j)+zeta)-L7).*sin(alpha(i));
            zVec(index) = (L1 + L3*sin(-beta(j))-L4*sin(zeta)-L5*sin(gamma(j)+zeta)-L6-L8);
            index = index + 1;
        end
    end
end


xInit = -0.4122;    %[m]
yInit = -0.6819;    %[m]
zInit = -0.6;       %[m]

xGoal = -0.1328;    %[m]
yGoal = 0.7245;     %[m]
zGoal = -0.6;       %[m]
% xVec = xVec(1:index-1);
% yVec = yVec(1:index-1);
% zVec = zVec(1:index-1);

[alphaInit,betaInit,gammaInit] = sherpaTTIK(xInit,yInit,zInit,kinematicConst,jointLimits);
[alphaGoal,betaGoal,gammaGoal] = sherpaTTIK(xGoal,yGoal,zGoal,kinematicConst,jointLimits);


%Construct a line in joint space between the initial and final
%states.

t = linspace(0,1,100);
line = [alphaInit+t*(alphaGoal-alphaInit); betaInit+t*(betaGoal-betaInit); gammaInit+t*(gammaGoal-gammaInit)];


for i = 1:length(line)
    [xLine(i),yLine(i),zLine(i)] = sherpaTTFK(line(1,i),line(2,i),line(3,i),kinematicConst);
end


%Plot Joint Space
figure(2)
hold on
for i = 1:vecLength
    for j = 1:vecLength
        if (imag(beta(j)) == 0 && beta(j) >= betaMIN && beta(j) <=betaMAX)
            plot3(alpha(i),beta(j),gamma(j),'k.');
        end
    end
end

plot3(alphaInit,betaInit,gammaInit,'g*')
plot3(alphaGoal,betaGoal,gammaGoal,'r*')
axis equal
title('Sherpa\_TT Single Leg Planner Constrained Joint Space')
xlabel('alpha [rad]')
ylabel('beta [rad]')
zlabel('gamma [rad]')
view(-105,12);

plot3(line(1,:),line(2,:),line(3,:),'r.')

%Plot Cartesian Space
figure(1)
hold on
axis equal

%[xInit,yInit,zInit] = sherpaTTFK(alphaInit,betaInit,gammaInit,kinematicConst);
%[xGoal,yGoal,zGoal] = sherpaTTFK(alphaGoal,betaGoal,gammaGoal,kinematicConst);

plot3(0,0,0,'b*');
plot3(xVec,yVec,zVec,'k.');
plot3(xInit,yInit,zInit,'g*');
plot3(xGoal,yGoal,zGoal,'r*');

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Sherpa\_TT Leg Configuration Space');
view(40,32);

plot3(xLine,yLine,zLine,'r.')
% 





    
    
    
    
    
    
    
