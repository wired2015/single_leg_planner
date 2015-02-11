%rrtEval.m
%author: wreid
%date: 20150107

%This script runs the single planner for a defined number of trials to
%evaluate the performance of the algorithm.

clear
clf
clc

planningConstants

nInit = [xInit yInit zInit xDotInit yDotInit zDotInit];
nGoal = [xGoal yGoal zGoal xDotGoal yDotGoal zDotGoal];

pathLengths = zeros(1,NUM_TRIALS);  %Vector of generated path lengths.
pathTimes = zeros(1,NUM_TRIALS);    %Vector of generated path times.
planningTimes = zeros(1,NUM_TRIALS);    %Vector of generated path times.
errors = zeros(1,NUM_TRIALS);    %Vector of generated path times.
optPathLength = cartDist([xInit yInit zInit],[xGoal yGoal zGoal]);
                                                    %Calculate the optimal 
                                                    %path length from the 
                                                    %goal to initial state
                                                    %with a Euclidean 
                                                    %distance measure.
%Print header for planner evaluation.
fprintf(['\n#### ' name ' Evaluation ####\n\n']);
fprintf('Num Trials: %d\n',NUM_TRIALS);
if exhaustive
    fprintf('Exhaustive Search, Max Number of Nodes: %d\n',NUM_NODES_MAX);
else
    fprintf('Finite Search\nNumber of Nodes: %d\n',NUM_NODES);
end
fprintf('Position Heuristic Gain: %.2f\n',HGAINS(1));
fprintf('Velocity Heuristic Gain: %.2f\n',HGAINS(2));
fprintf('Ankle Heuristic Gain: %.2f\n\n',HGAINS(3));
%fprintf('Optimal Path Distance: %.2f m\n\n',optPathLength);
%***TODO***
%fprintf('ICR in the Body frame: %.2f\n',s);
%fprintf('Maximum Wheel Speed: %.2f\n'qWMax);

%Run the planner mulitple times.
for i=1:NUM_TRIALS

    %Generate the RRT.
    tic
    [path,success] = buildRRTWrapper(nInit,nGoal,NUM_NODES,jointLimits,K,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kinematicConst,ankleThreshold,exhaustive,threshold,goalSeedFreq);
    
    if success
        planningTime = toc;

        %Calculate the path length from the generated RRT.
        [pathH,~] = size(path);
        pathLength = 0;
        for j = 1:pathH-1
            pathLength = pathLength + cartDist([path(i,2) path(i,3) path(i,4)],[path(i+1,2) path(i+1,3) path(i+1,4)]); 
        end

        %Calculate the path time.
        pathTime = dt*pathH;

        %Print the results on the command line.
        fprintf('\nTRIAL %d\n',i);
        fprintf('Path Length: %.2f m\n',pathLength);
        fprintf('Path Time: %.2f sec\n',pathTime);
        xFinal = path(end,4);
        yFinal = path(end,5);
        zFinal = path(end,6);
        error = cartDist([xFinal yFinal zFinal],[xGoal yGoal zGoal]);
        fprintf('Final Cartesian Position Error: %.2f m\n',error);
        fprintf('Planning Time: %.3f s\n',planningTime);

        %Update the pathLength and pathTime vectors.
        pathLengths(i) = pathLength;
        pathTimes(i) = pathTime;
        planningTimes(i) = planningTime;
        errors(i) = error;
    else
        disp('No path found!');
    end
    
end

%If only one tree is being generated, then plot the tree, the trajectory
%generated and the velocities over time.
if NUM_TRIALS == 1 && success
    
    %figure(1)
    %printRRT(T,nGoal,MIN,MAX,path,kinematicConst,jointLimits,true,K,NUM_NODES,NODE_SIZE);

    %figure(2)
    %plotVelocities(path,pathH,Dt);

    %Find the path in an x,y,z representation using a forward kinematic
    %model of the system.
    %figure(3)
    %plotPath(path,kinematicConst,nGoal,K,jointLimits,true);
    
    %figure(4)
    %plotAnkle(path,pathH,ankleThreshold,Dt);
    
    %figure(5)
    %plotWheelSpeed(path,pathH,Dt);
    
    figure(1)
    hold on
    axis([-1.5 1.5 -1.5 1.5 -1.5 0.5]);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    for i = 1:pathH
        plot3(path(i,2),path(i,3),path(i,4),'k.');
    end
    hold off
        
%If there is more than one trial, plot the tree evaluation results for each
%trial.    
elseif success
    figure(1)
    subplot(2,2,1);
    hold on
    plot(1:NUM_TRIALS,pathLengths,'b*');
    %plot(1:NUM_TRIALS,optPathLength*ones(1,NUM_TRIALS),'k*');
    xlabel('Trial Number');
    ylabel('Path Length [m]');
    hold off
    subplot(2,2,2);
    hold on
    plot(1:NUM_TRIALS,pathTimes,'g*');
    xlabel('Trial Number');
    ylabel('Path Time [s]');
    hold off
    subplot(2,2,3);
    plot(1:NUM_TRIALS,planningTimes,'b*');
    xlabel('Trial Number');
    ylabel('Planning Time [s]');
    subplot(2,2,4);
    plot(1:NUM_TRIALS,errors,'k*');
    xlabel('Trial Number');
    ylabel('Final Position Error [m]');
    
    date = datestr(now,30);
    
    save([date '_SherpaRRTTrial'],'pathLengths','pathTimes','planningTimes','errors','NUM_NODES','exhaustive','HGAINS','K','nInit','nGoal','goalSeedFreq','U','threshold','ankleThreshold','NUM_TRIALS');
end


