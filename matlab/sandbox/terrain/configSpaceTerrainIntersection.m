figure
hold on
view(-35,22)
axis equal

XMIN = -2;
XMAX = 2;
YMIN = -2;
YMAX = 2;

planningConstants

alphaMin = jointLimits(1,1);
alphaMax= jointLimits(2,1);
betaMin = jointLimits(1,2);
betaMax = jointLimits(2,2);
gammaMin = jointLimits(1,3);
gammaMax = jointLimits(2,3);


N= 100;

%% Create the mesh of the Sherpa_TT leg configuration space.

%Create four separate surfaces
%   1. gamma = gamma_max
%   2. gamma = gamma_min
%   3. beta = beta_min
%   4. beta = beta_max

rMIN = kC.l2+kC.l4*cos(kC.zeta)-kC.l7+kC.l3*cos(betaMin)+L5*cos(kC.zeta+gammaMax)-kC.l7;
rMAX = kC.l2+kC.l4*cos(kC.zeta)-kC.l7+kC.l3*cos(betaMax)+L5*cos(kC.zeta+gammaMin)-kC.l7;
thetaMIN = -3*pi/4;
thetaMAX = 3*pi/4;

zPMax = cartesianLimits(1);
zPMin = cartesianLimits(2);
zPMid1 = cartesianLimits(3);
zPMid2 = cartesianLimits(4);

rMid1 = 0.8232;

n = 20;

alpha = linspace(alphaMin,alphaMax,n);

gamma1 = gammaMax*ones(1,n);
beta1 = linspace(betaMin,betaMax,n);

beta2 = betaMin*ones(1,n);
gamma2 = linspace(gammaMin,gammaMax,n);

gamma3 = gammaMin*ones(1,n);
beta3 = linspace(betaMin,betaMax,n);
 
beta4 = betaMax*ones(1,n);
gamma4 = linspace(gammaMax,gammaMin,n);

[x1,y1,z1] = sherpaTTFKSurf(alpha,beta1,gamma1,kC,n);
[x2,y2,z2] = sherpaTTFKSurf(alpha,beta2,gamma2,kC,n);
[x3,y3,z3] = sherpaTTFKSurf(alpha,beta3,gamma3,kC,n);
[x4,y4,z4] = sherpaTTFKSurf(alpha,beta4,gamma4,kC,n);
[TB2G,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices([0 0 1 0 0 0],[0 0 0 0],kC,1);
TP2G = TB2G*TP2B;

xG1 = zeros(n,n);yG1 = zeros(n,n);zG1 = zeros(n,n);
xG2 = zeros(n,n);yG2 = zeros(n,n);zG2 = zeros(n,n);
xG3 = zeros(n,n);yG3 = zeros(n,n);zG3 = zeros(n,n);
xG4 = zeros(n,n);yG4 = zeros(n,n);zG4 = zeros(n,n);

%Transform the x, y and z vectors into the global coordinate frame.
for i = 1:n

    for j = 1:n
        uP1 = [x1(i,j);y1(i,j);z1(i,j)];
        uP2 = [x2(i,j);y2(i,j);z2(i,j)];
        uP3 = [x3(i,j);y3(i,j);z3(i,j)];
        uP4 = [x4(i,j);y4(i,j);z4(i,j)];
        uG1 = TP2G(1:3,1:3)*uP1+TP2G(1:3,4);
        uG2 = TP2G(1:3,1:3)*uP2+TP2G(1:3,4);
        uG3 = TP2G(1:3,1:3)*uP3+TP2G(1:3,4);
        uG4 = TP2G(1:3,1:3)*uP4+TP2G(1:3,4);

        xG1(i,j) = uG1(1);yG1(i,j) = uG1(2);zG1(i,j) = uG1(3);
        xG2(i,j) = uG2(1);yG2(i,j) = uG2(2);zG2(i,j) = uG2(3);
        xG3(i,j) = uG3(1);yG3(i,j) = uG3(2);zG3(i,j) = uG3(3);
        xG4(i,j) = uG4(1);yG4(i,j) = uG4(2);zG4(i,j) = uG4(3);
    end

end

hold on
view(-38,28)
xlabel('x_G [m]')
ylabel('y_G [m]')
zlabel('z_G [m]')
surf(xG1,yG1,zG1,'FaceAlpha',0.5)
surf(xG2,yG2,zG2,'FaceAlpha',0.5)
surf(xG3,yG3,zG3,'FaceAlpha',0.5)
surf(xG4,yG4,zG4,'FaceAlpha',0.5)



%% Create the mesh of the terrain.
xMesh = linspace(XMIN,XMAX,n);
yMesh = linspace(YMIN,YMAX,n);
[X,Y] = meshgrid(xMesh,yMesh);
k = 1;
Z = k*X.*exp(-X.^2 - Y.^2) +...
    k*(X+2).*exp(-(X+2).^2 - (Y+2).^2) +...
    k*(X-2).*exp(-(X-2).^2 - (Y+2).^2) +...
    k*(X+2).*exp(-(X+2).^2 - (Y-2).^2) +...
    k*(X-2).*exp(-(X-2).^2 - (Y-2).^2);
h = mesh(X,Y,Z,'FaceAlpha',1,'LineWidth',1);

%% Find the intersections between the two.


