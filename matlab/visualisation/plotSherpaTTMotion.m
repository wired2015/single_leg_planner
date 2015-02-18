%PLOTSHERPATTMOTION Given results from the single leg planner in the form
%of a structure of results contained in plannerResults, this function plots
%the robot executing the planned path.
%
%Inputs:
%-plannerResults: A struct containing information about the planned
%trajectory.
%-dt: The sample time at which the planner produced the path.
%
%plotSherpaTTMotion.m
%author: wreid
%date: 20150217

function plotSherpaTTMotion(uG,plannerResults,dt,numLegs,kC)
    
    h = zeros(1,numLegs);
    
    for i = 1:numLegs
        [h(i),~] = size(plannerResults(i).pathJ);
    end
    
    hMax = max(h);

    %For the trajectories that are shorter than the maximum length
    %trajectory append the end of the matrix with the last row.
    for i = 1:numLegs
        if h(i) < hMax
            plannerResults(i).pathJ((h(i)+1):hMax,:) = repmat(plannerResults(i).pathJ(h(i),:),hMax-h(i),1);
        end
    end

    t = 0;
    init = true;
    
    for i = 1:hMax
        plotSherpaTT(uG,[plannerResults(1).pathJ(i,2:4) plannerResults(1).pathJ(i,8); plannerResults(2).pathJ(i,2:4) plannerResults(2).pathJ(i,8); plannerResults(3).pathJ(i,2:4) plannerResults(3).pathJ(i,8); plannerResults(4).pathJ(i,2:4) plannerResults(4).pathJ(i,8)],kC,init);
        pause(dt);
        t = t + dt;
        title(['t = ' num2str(t) 's'])
        init = false;
    end

end