function plotAnkle(pathJ,ankleThreshold,dt)

    [pathH,~] = size(pathJ);
    t = dt*(1:pathH);
    
    subplot(1,2,1);
    
    plot(t,pathJ(:,8),'b')
    xlabel('Time [s]');
    ylabel('phi [rad]');
    
    subplot(1,2,2);
    hold on
    
    plot(t(1:end-1),diff(pathJ(:,8)),'g*');
    
    plot(t,ankleThreshold*ones(1,pathH),'r--');
    plot(t,-ankleThreshold*ones(1,pathH),'r--');
    
    hold off

end
