function plotPath(pathC,kC,nGoalB,K,jointLimits,showConfigSpace,legNum)
    %Plots a path in the Cartesian space, referenced to the rover's body
    %coordinate frame.
    
    axis([-1.5 1.5 -1.5 1.5 -1.5 0.5]);

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
        
    hold on
    
    %Plot the path.
    [pathH,~] = size(pathC);
    for i = 1:pathH
        plot3(pathC(i,2),pathC(i,3),pathC(i,4),'k.');
    end
    %Plot the initial position.
    plot3(pathC(1,2),pathC(1,3),pathC(1,4),'g*');
    %Plot the final position
    plot3(pathC(end,2),pathC(end,3),pathC(end,4),'b*');
    %Plot the goal position.
    plot3(nGoalB(1),nGoalB(2),nGoalB(3),'r*');
    %Plot the leg's configuration space.
    if showConfigSpace
        plotCartesianConfigSpace(kC,jointLimits,K,legNum);
    end
    
    hold off

end