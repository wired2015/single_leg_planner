function plotAnkle(path,ankleThreshold,dt)

    [pathH,~] = size(path);
    t = dt*(1:pathH);
    subplot(1,2,1);
    plot(t,fliplr(path(:,10)')','b')
    xlabel('Time [s]');
    ylabel('qA');
    subplot(1,2,2);
    hold on
    plot(t(1:end-1),diff(fliplr(path(:,10)')'),'g*');
    plot(t,ankleThreshold*ones(1,pathH),'r--');
    plot(t,-ankleThreshold*ones(1,pathH),'r--');
    hold off

end
