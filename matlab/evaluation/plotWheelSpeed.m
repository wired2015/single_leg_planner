%plotWheelSpeed.m
%author: wreid
%date: 20150202

function plotWheelSpeed(path,Dt)

    [pathH,~] = size(path);
    t = Dt*(1:pathH);
    plot(t,fliplr(path(:,11)')','b')
    xlabel('Time [s]');
    ylabel('qWDot');
        
end