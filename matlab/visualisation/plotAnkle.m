function plotAnkle(pathJ,ankleThreshold,dt)

    [pathH,~] = size(pathJ);
    t = dt*(1:pathH);
    
    subplot(1,3,1);
    
    plot(t,pathJ(:,5),'b')
    xlabel('Time [s]');
    ylabel('\phi [rad]');
    
    subplot(1,3,2);
    hold on
    
    plot(t(1:end-1),diff(pathJ(:,5)),'g*');
    plot(t,ankleThreshold*ones(1,pathH),'r--');
    plot(t,-ankleThreshold*ones(1,pathH),'r--');
    xlabel('Time [s]')
    ylabel('\phi DOT [rad/s]');
    hold off
    
    %Plot the wheel speed.
    subplot(1,3,3);
    hold on
    plot(t,pathJ(:,11),'b.');
    xlabel('Time [s]')
    ylabel('\omega [rad/s]');
    hold off

end
