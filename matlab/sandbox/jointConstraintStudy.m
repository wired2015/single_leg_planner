clear

kinematicConstants

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

betaMin = deg2rad(-59.5);
betaMax = deg2rad(17);
gammaMin = deg2rad(-5);
gammaMax = deg2rad(73.7);

N = 40;

beta = linspace(-pi,pi,N);
gamma = linspace(-pi,pi,N);
figure(1)
axis equal
hold on

%Plot the configuration space isolated to a cylindrical coordinate system
%vertical cross section. -- BLACK
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
        zp = (kC.l1 + kC.l3*sin(-beta(i))-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma(j)+kC.zeta)-kC.l6-(kC.l8+kC.r));
        if gamma(j) < gammaMin || gamma(j) > gammaMax || beta(i) < betaMin || beta(i) > betaMax
            plot(x,zp,'k.');
        else
            plot(x,zp,'c.');
        end
    end
end


%Plot the extremum of the configuration space that is attributed to a
%maximum beta joint angle. - Red = betaMin, Green = betaMax
beta = [betaMin betaMax];
gamma = linspace(gammaMin,gammaMax,N);
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
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
beta = linspace(betaMin,betaMax,N);
gamma = [gammaMin gammaMax];
for i = 1:length(beta)
    for j = 1:length(gamma)
        x = kC.l2+kC.l3*cos(-beta(i))+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma(j))-kC.l7;        
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

zMax = B-kC.l3*sin(betaMin)-kC.l5*sin(gammaMin+kC.zeta);
zMin = B-kC.l3*sin(betaMax)-kC.l5*sin(gammaMax+kC.zeta);
zMid1 = B-kC.l3*sin(betaMin)-kC.l5*sin(gammaMax+kC.zeta);
zMid2 = B-kC.l3*sin(betaMax)-kC.l5*sin(gammaMin+kC.zeta);

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

%line([xMin xMax],[z z],'Color','c')
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


alpha = 0.118;
beta = -0.5875;
gamma = 1.3367;
phi = 0;
[TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices([0 0 0],[alpha beta gamma phi],kC,1);

TJ2P = TI2P*TJ2I;
TO2P = TJ2P*TO2J;
TQ2P = TO2P*TQ2O;
TR2P = TQ2P*TR2Q;
TS2P = TR2P*TS2R;
TW2P = TS2P*TW2S;
TC2P = TW2P*TC2W;

%dB2I = TB2P(1:3,4);
dI2P = TI2P(1:3,4);
dJ2P = TJ2P(1:3,4);
dO2P = TO2P(1:3,4);
dQ2P = TQ2P(1:3,4);
dR2P = TR2P(1:3,4);
dS2P = TS2P(1:3,4);
dW2P = TW2P(1:3,4);
dC2P = TC2P(1:3,4);

line([dI2P(1) dJ2P(1)],[dI2P(3) dJ2P(3)],'Color','k');
line([dI2P(1) dJ2P(1)],[dI2P(3) dJ2P(3)],'Color','k');
line([dJ2P(1) dO2P(1)],[dJ2P(3) dO2P(3)],'Color','k');
line([dO2P(1) dQ2P(1)],[dO2P(3) dQ2P(3)],'Color','k');
line([dQ2P(1) dR2P(1)],[dQ2P(3) dR2P(3)],'Color','k');
line([dR2P(1) dS2P(1)],[dR2P(3) dS2P(3)],'Color','k');
line([dS2P(1) dW2P(1)],[dS2P(3) dW2P(3)],'Color','k');
line([dW2P(1) dC2P(1)],[dW2P(3) dC2P(3)],'Color','k');


