function plotJointVelocities(path)
%plotVelocities Summary of this function goes here
%   Detailed explanation goes here

    hold on
    
    subplot(1,3,1);
    plot(path(:,1),path(:,7)','b');
    xlabel('Time [s]');
    ylabel('alphaDot [rad/s]');
    
    subplot(1,3,2);
    plot(path(:,1),path(:,8),'r');
    xlabel('Time [s]');
    ylabel('betaDot [rad/s]');
  
    subplot(1,3,3);
    plot(path(:,1),path(:,9),'g');
    xlabel('Time [s]');
    ylabel('gammaDot [m/s]');
    
    hold off

end

