clear

kinematicConstants

betaMin = -1.0385;
betaMax = 0.2967;
gammaMin = -0.0873;
gammaMax = 1.2863;

% zp = -0.5;
% 
% for i = 1:100
%     gammaRand = gammaMin + (gammaMax-gammaMin)*rand();
% 
%     A = (-zp+kC.l1-kC.l4*sin(kC.zeta)-kC.l5*sin(gammaRand+kC.zeta)-kC.l6-kC.l8-kC.r)/kC.l3;
%     B = kC.l1-kC.l4*sin(kC.zeta)-kC.l6-kC.l8-kC.r-zp;
% 
%     if A >= 1
%         fprintf('gammaMax: %.2f\n',gammaMax);
%         gammaMax = asin((kC.l3+A)/kC.l5) - kC.zeta;
%         fprintf('gammaMax new: %.2f\n',gammaMax);
%         gammaRand = gammaMin + (gammaMax - gammaMin)*rand();
%         A = (-zp+kC.l1-kC.l4*sin(kC.zeta)-kC.l5*sin(gammaRand+kC.zeta)-kC.l6-kC.l8-kC.r)/kC.l3;
%         beta = asin(A);
%         disp(beta)
%     elseif A <= -1
%         fprintf('gammaMin: %.2f\n',gammaMin);
%         gammaMin = asin((-kC.l3+A)/kC.l5) - kC.zeta;
%         fprintf('gammaMin new: %.2f\n',gammaMin);
%         gammaRand = gammaMin + (gammaMax - gammaMin)*rand(); 
%         A = (-zp+kC.l1-kC.l4*sin(kC.zeta)-kC.l5*sin(gammaRand+kC.zeta)-kC.l6-kC.l8-kC.r)/kC.l3;
%         beta = asin(A);
%         disp(beta)
%     end
% end

betaMin = -1.0385;
betaMax = 0.2967;
gammaMin = -0.0873;
gammaMax = 1.2863;

beta = linspace(betaMin,betaMax,30);
gamma = linspace(gammaMin,gammaMax,30);
figure(1)
axis equal
hold on

%Plot the configuration space isolated to a cylindrical coordinate system
%vertical cross section. -- BLACK
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
        zp = (kC.l1 + kC.l3*sin(-beta(i))-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma(j)+kC.zeta)-kC.l6-(kC.l8+kC.r));
        plot(x,zp,'k.');
    end
end

%Plot the extremum of the configuration space that is attributed to a
%maximum beta joint angle. - Red = betaMin, Green = betaMax
beta = [betaMin betaMax];
gamma = linspace(gammaMin,gammaMax,500);
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
        zp = (kC.l1 + kC.l3*sin(-beta(i))-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma(j)+kC.zeta)-kC.l6-(kC.l8+kC.r));
        if i == 1
            plot(x,zp,'r.');
        else
            plot(x,zp,'g.');
        end
    end
end

%Plot the extremum of the configuration space that is attributed to a
%maximum gamma joint angle. - Blue = gammaMin, Yellow = gammaMax
beta = linspace(betaMin,betaMax,500);
gamma = [gammaMin gammaMax];
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
        zp = (kC.l1 + kC.l3*sin(-beta(i))-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma(j)+kC.zeta)-kC.l6-(kC.l8+kC.r));
        if j == 1
            plot(x,zp,'b.');
        else
            plot(x,zp,'y.');
        end
    end
end

A = kC.l2+kC.l4*cos(kC.zeta)-kC.l7;
B = kC.l1-kC.l4*sin(kC.zeta)-kC.l6-kC.l8-kC.r;

zMax = B-kC.l3*sin(betaMin)-L5*sin(gammaMin+kC.zeta);
zMin = B-kC.l3*sin(betaMax)-L5*sin(gammaMax+kC.zeta);
zMid1 = B-kC.l3*sin(betaMin)-L5*sin(gammaMax+kC.zeta);
zMid2 = B-kC.l3*sin(betaMax)-L5*sin(gammaMin+kC.zeta);

z = -0.5;

if z <= zMax && z >= zMid1
    xMax = A+kC.l3*cos(asin((-z+B-kC.l5*sin(kC.zeta+gammaMin))/kC.l3))+kC.l5*cos(kC.zeta+gammaMin);
    xMin = A+kC.l3*cos(betaMin)+kC.l5*cos(asin((B-kC.l3*sin(betaMin)-z)/kC.l5));
elseif z< zMid1 && z >= zMid2
    xMax = A+kC.l3*cos(asin((-z+B-kC.l5*sin(kC.zeta+gammaMin))/kC.l3))+kC.l5*cos(kC.zeta+gammaMin);
    xMin = A+kC.l3*cos(asin((-z+B-kC.l5*sin(kC.zeta+gammaMax))/kC.l3))+kC.l5*cos(kC.zeta+gammaMax);
elseif  z< zMid2 && z >= zMin
    xMax = A+kC.l3*cos(betaMax)+kC.l5*cos(asin((B-kC.l3*sin(betaMax)-z)/kC.l5));
    xMin = A+kC.l3*cos(asin((-z+B-kC.l5*sin(kC.zeta+gammaMax))/kC.l3))+kC.l5*cos(kC.zeta+gammaMax);
else
    xMax = 0;
    xMin = 0;
    disp('z is out of range');
end

line([xMin xMax],[z z],'Color','c')
ylabel('z_P [m]')
xlabel('r_P [m]')
title('Wheel Contact Point Config Space Study - (\alpha=0 rad)')

% u1 = sherpaTTFK([0 betaMin gammaMin],kC);
% u2 = sherpaTTFK([0 betaMax gammaMax],kC);
% u3 = sherpaTTFK([0 betaMin gammaMax],kC);
% u4 = sherpaTTFK([0 betaMax gammaMin],kC);
% 
% zMax = u1(3);
% zMin = u2(3);
% zMid1 = u3(3);
% zMid2 = u4(3);
% 
% z = -0.9;
% 
% if z <= zMax && z >= zMid1
%     xMax = getXStar(z,gammaMin,false,kC);
%     xMin = getXStar(z,betaMin,true,kC);
% elseif z< zMid1 && z >= zMid2
%     xMax = getXStar(z,gammaMin,false,kC);
%     xMin = getXStar(z,gammaMax,false,kC);
% elseif  z< zMid2 && z >= zMin
%     xMax = getXStar(z,betaMax,true,kC);
%     xMin = getXStar(z,gammaMax,false,kC);
% else
%     xMax = 0;
%     xMin = 0;
%     disp('z is out of range');
% end
% 
% line([xMin xMax],[z z],'Color','r')


