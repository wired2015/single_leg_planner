function printRRT(T,sGoalBCartesian,pathJ,kC,jointLimits,panHeight,nodeCount)
%printRRT Prints an RRT

    MIN = jointLimits(1,:);
    MAX = jointLimits(2,:);
    
    [~,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices([0 0 0],[0 0 0 0],kC,1);
    TB2P = trInv(TP2B);
    sGoalPCartesian = tr(sGoalBCartesian',TB2P);
    sGoalJoint = sherpaTTIK(sGoalPCartesian(1:3)',kC,jointLimits);
   
    count = 1;
    for i = 2:nodeCount
        if T(i,1) == 0
            nodeCount = count;
            break;
        else
            count = count + 1;
        end
    end
    
    subplot(1,2,1);
    hold on
    
    axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
    view(-42,16);

    xlabel('alpha [rad]');
    ylabel('beta [rad]');
    zlabel('gamma [rad]');
    
    [~,tWid] = size(T);
    
    for i = 2:nodeCount
        plot3(T(i,4),T(i,5),T(i,6),'bo');
        current = T(i,:);
        parent = T(current(2),:);
        line([current(4) parent(4)], [current(5) parent(5)], [current(6) parent(6)],'Color','k');
%        j = NODE_SIZE+6;
%         while j<=tWid-NODE_SIZE
%             if j > NODE_SIZE+6
%                 line([T(i,j-6) T(i,j)], [T(i,j-5) T(i,j+1)], [T(i,j-4) T(i,j+2)],'Color','k')
%             end
%             j = j + 6;
%         end
        parentID = round(T(i,2));
    end
    
    [pathLength,~] = size(pathJ);
    if pathLength >=2
        for i = 1:pathLength-1
           line([pathJ(i,2) pathJ(i+1,2)], [pathJ(i,3) pathJ(i+1,3)],[pathJ(i,4) pathJ(i+1,4)],'Color','r') 
        end
    end
    
    plot3(T(1,4),T(1,5),T(1,6),'g*');
    plot3(sGoalJoint(1),sGoalJoint(2),sGoalJoint(3),'r*');
    plot3(pathJ(end,2),pathJ(end,3),pathJ(end,4),'b*');

    hold off
    
    subplot(1,2,2);
    hold on
    xlabel('alpha [rad]');
    ylabel('beta [rad]');
    zlabel('gamma [rad]');
    view(-42,16);
    axis([MIN(1) MAX(1) MIN(2) MAX(2) MIN(3) MAX(3)])
    plot3(T(1,4),T(1,5),T(1,6),'g*');
    plot3(sGoalJoint(1),sGoalJoint(2),sGoalJoint(3),'r*');
    plot3(pathJ(end,2),pathJ(end,3),pathJ(end,4),'b*');
    for i = 1:pathLength-1
       line([pathJ(i,2) pathJ(i+1,2)], [pathJ(i,3) pathJ(i+1,3)],[pathJ(i,4) pathJ(i+1,4)],'Color','r') 
    end
    
    plotJointConfigSpace(kC,jointLimits,panHeight);
    
    hold off

end

