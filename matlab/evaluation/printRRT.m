function printRRT(T,xGoal,MIN,MAX,path,kinematicConst,jointLimits,showConfigSpace,K,nodeCount,NODE_SIZE)
%printRRT Prints an RRT

    h = nodeCount;
    
    subplot(1,2,1);
    hold on
    
    %axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
    view(-42,16);

    xlabel('alpha [rad]');
    ylabel('beta [rad]');
    zlabel('gamma [rad]');
    
    [~,tWid] = size(T);
    
    for i = 2:h
        plot3(T(i,4),T(i,5),T(i,6),'bo');
        j = NODE_SIZE+1;
        while j<=tWid
            if j > NODE_SIZE+1
                line([T(i,j-6) T(i,j)], [T(i,j-5) T(i,j+1)], [T(i,j-4) T(i,j+2)],'Color','k')
            end
            j = j + 6;
        end
        parentID = round(T(i,2));
    end
    
    [pathLength,~] = size(path);
    if pathLength >=2
        for i = 1:pathLength-1
           line([path(i,4) path(i+1,4)], [path(i,5) path(i+1,5)],[path(i,6) path(i+1,6)],'Color','r') 
        end
    end
    
    plot3(T(1,4),T(1,5),T(1,6),'g*');
    %plot3(xGoal(4),xGoal(5),xGoal(6),'r*');
    hold off
    
    subplot(1,2,2);
    hold on
    xlabel('alpha [rad]');
    ylabel('beta [rad]');
    zlabel('gamma [rad]');
    view(-42,16);
    axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
    plot3(T(1,4),T(1,5),T(1,6),'g*');
    plot3(xGoal(4),xGoal(5),xGoal(6),'r*');
    for i = 1:pathLength-1
       line([path(i,4) path(i+1,4)], [path(i,5) path(i+1,5)],[path(i,6) path(i+1,6)],'Color','r') 
    end
    
    if showConfigSpace
        plotJointConfigSpace(kinematicConst,jointLimits,K);
    end
    
    hold off

end

