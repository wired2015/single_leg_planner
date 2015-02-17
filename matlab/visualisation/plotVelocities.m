function plotVelocities(path,Dt)
%plotVelocities Summary of this function goes here
%   Detailed explanation goes here

    [pathH,~] = size(path);

    hold on
    t = Dt*(1:pathH);
    
    subplot(1,3,1);
    plot(t,fliplr(path(:,7)'),'b');
    xlabel('Time [s]');
    ylabel('alphaDot [rad/s]');
    
    subplot(1,3,2);
    plot(t,fliplr(path(:,8)'),'r');
    xlabel('Time [s]');
    ylabel('betaDot [rad/s]');
  
    subplot(1,3,3);
    plot(t,fliplr(path(:,9)'),'g');
    xlabel('Time [s]');
    ylabel('gammaDot [rad/s]');
    
    hold off

end

