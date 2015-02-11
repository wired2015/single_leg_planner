clear

load('RRTTrial4.mat')

uB = [0.3;0;0.6];
q = deg2rad([0 -35 50]);


time = 0;
init = true;
showFrames = false;
showPath = true;

initializeSherpaTTPlot;

for i = length(path):-1:1
    uB = [0 0 0];
    q = [path(i,4) path(i,5) path(i,6)];
    plotSherpaTT(uB,q,kinematicConst,init,[xInit yInit zInit],[xGoal yGoal zGoal],showFrames,showPath);
    pause(Dt)
    title(['t = ' num2str(time) ' s']);
    time = time+Dt;
    init = false;
end