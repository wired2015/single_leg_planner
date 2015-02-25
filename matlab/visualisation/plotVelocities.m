function plotVelocities(path)
%plotVelocities Summary of this function goes here
%   Detailed explanation goes here

    hold on
    
    subplot(1,3,1);
    plot(path(:,1),path(:,6)','b');
    xlabel('Time [s]');
    ylabel('xDot [m/s]');
    
    subplot(1,3,2);
    plot(path(:,1),path(:,7),'r');
    xlabel('Time [s]');
    ylabel('yDot [m/s]');
  
    subplot(1,3,3);
    plot(path(:,1),path(:,8),'g');
    xlabel('Time [s]');
    ylabel('zDot [m/s]');
    
    hold off

end

