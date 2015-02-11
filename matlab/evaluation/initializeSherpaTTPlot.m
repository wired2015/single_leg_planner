global fig body l1 l2 l3 l4 l5 l6 l7 l8 wheel
global trB2G trP2G trI2G trJ2G trO2G trQ2G trS2G trC2G
global initPosHandle goalPosHandle

fig = figure('Name','Sherpa_TT Visualization');
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
xlabel('X_G [m]');
ylabel('Y_G [m]');
zlabel('Z_G [m]');
view(145,42);

hold on

%Plot the initial and final position.
initPosHandle = plot3(0,0,0,'g*');
goalPosHandle = plot3(0,0,0,'r*');

body = fill3(0,0,0,'r');
l1 = line([0 0],[0 0],'LineWidth',3);
l2 = line([0 0],[0 0],'LineWidth',3);
l3 = line([0 0],[0 0],'LineWidth',3);
l4 = line([0 0],[0 0],'LineWidth',3);
l5 = line([0 0],[0 0],'LineWidth',3);
l6 = line([0 0],[0 0],'LineWidth',3);
l7 = line([0 0],[0 0],'LineWidth',3);
l8 = line([0 0],[0 0],'LineWidth',3);
wheel = fill3(0,0,0,'k');

axesLen = 0.1;

if showFrames
    trB2G = trplot(eye(4,4),'color','b','length',axesLen,'frame','P');
    trP2G = trplot(eye(4,4),'color','b','length',axesLen,'frame','P');
    trI2G = trplot(eye(4,4),'length',axesLen,'frame','I');
    trJ2G = trplot(eye(4,4),'length',axesLen,'frame','J');
    trO2G = trplot(eye(4,4),'length',axesLen,'frame','O');
    trQ2G = trplot(eye(4,4),'length',axesLen,'frame','Q');
    %trplot(TI2P*TJ2I*TO2J*TQ2O*TR2Q,'length',axesLen,'frame','R');
    trS2G = trplot(eye(4,4),'length',axesLen,'frame','S');
    trC2G = trplot(eye(4,4),'length',axesLen,'frame','C');
end

hold off
