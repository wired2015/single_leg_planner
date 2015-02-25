function printBiDirRRT(T1,T2,pathJ,kC,jointLimits)%,panHeight,nodeCount)
%printRRT Prints an RRT

    MIN = jointLimits(1,:);
    MAX = jointLimits(2,:);
        
    %subplot(1,2,1);
    hold on
    
    axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
    view(-42,16);

    xlabel('alpha [rad]');
    ylabel('beta [rad]');
    zlabel('gamma [rad]');
    
    [tH,~] = size(T1);
    for i = 2:tH
        plot3(T1(i,4),T1(i,5),T1(i,6),'bo');
        current = T1(i,:);
        parent = T1(current(2),:);
        line([current(4) parent(4)], [current(5) parent(5)], [current(6) parent(6)],'Color','k');
    end
    
    [tH,~] = size(T2);
    for i = 2:tH
        plot3(T2(i,4),T2(i,5),T2(i,6),'go');
        current = T2(i,:);
        if current(2) > 0
            parent = T2(current(2),:);
        else
            parent = T2(1,:);
        end
        line([current(4) parent(4)], [current(5) parent(5)], [current(6) parent(6)],'Color','k');
    end
    
    plot3(T1(1,4),T1(1,5),T1(1,6),'g*');
    plot3(T2(1,4),T2(1,5),T2(1,6),'r*');
    
    [pathLength,~] = size(pathJ);
    if pathLength >=2
        for i = 1:pathLength-1
           line([pathJ(i,2) pathJ(i+1,2)], [pathJ(i,3) pathJ(i+1,3)],[pathJ(i,4) pathJ(i+1,4)],'Color','r') 
        end
    end
    
   
    %plot3(pathJ(end,2),pathJ(end,3),pathJ(end,4),'b*');

    hold off
    
%     subplot(1,2,2);
%     hold on
%     xlabel('alpha [rad]');
%     ylabel('beta [rad]');
%     zlabel('gamma [rad]');
%     view(-42,16);
%     axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
%     plot3(T(1,4),T(1,5),T(1,6),'g*');
%     plot3(sGoalJoint(1),sGoalJoint(2),sGoalJoint(3),'r*');
%     plot3(pathJ(end,2),pathJ(end,3),pathJ(end,4),'b*');
%     for i = 1:pathLength-1
%        line([pathJ(i,2) pathJ(i+1,2)], [pathJ(i,3) pathJ(i+1,3)],[pathJ(i,4) pathJ(i+1,4)],'Color','r') 
%     end
%     
%     plotJointConfigSpace(kC,jointLimits,panHeight);
    
    hold off

end

