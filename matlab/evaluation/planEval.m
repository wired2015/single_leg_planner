%PLANEVAL Evaluates the RRT-based single leg planner.
%
% PLANEVAL(useMex) evaluates the RRT-based single leg planner using 
% planning costants defined in planningConstants.m. If useMex is true then
% the planner will be run using the mex function.
%
% Examples::
%       planEval(true)
%
% Notes::
% - If useMex is false the planner will be run using the MATLAB code.
%   Beware not using the mex function will result in the planner running 
%   very slowly.
%
% planEval.m
% author: wreid
% date: 20150107

function planEval(useMex)
    
    clc
    addpath(genpath('~/Dropbox/PhD/matlab/rvctools'))

    planningConstants
    
    numLegs = 4;

    pathLengths = zeros(1,NUM_TRIALS);  %Vector of generated path lengths.
    pathTimes = zeros(1,NUM_TRIALS);    %Vector of generated path times.
    planningTimes = zeros(1,NUM_TRIALS);    %Vector of generated path times.
    errors = zeros(1,NUM_TRIALS);    %Vector of generated path times.
    optPathLength = cartDist(sInitB(1:3),sGoalB(1:3));
                                                        %Calculate the optimal 
                                                        %path length from the 
                                                        %goal to initial state
                                                        %with a Euclidean 
                                                        %distance measure.
    %Print header for planner evaluation.
    fprintf(['\nSingle Leg Planner Evaluation ####\n\n']);
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
    
    %TODO: Incorporate body rates into the planner.
    %fprintf('ICR in the Body frame: %.2f\n',s);
    %fprintf('Maximum Wheel Speed: %.2f\n'qWMax);

    jointPaths = zeros(4,1000,7);
    cartesianPaths = zeros(4,1000,7);

    h = 0;
    %Run the planner mulitple times.
    for i=1:1

        %Generate the RRT.
        tic
        if useMex
            [T,pathC,pathJ,success] = buildRRTWrapper_mex(sInitB(i,:),sGoalB(i,:),jointLimits,bodyHeight,U,dt,Dt,kC,threshold,int32(i),uBDot);
        else
            [T,pathC,pathJ,success] = buildRRTWrapper(sInitB(i,:),sGoalB(i,:),jointLimits,bodyHeight,U,dt,Dt,kC,threshold,i,uBDot);
        end
        planningTime = toc;

        if success
            %Calculate the path length from the generated RRT.
            [pathH,~] = size(pathC);
            pathLength = 0;
            for j = 1:pathH-1
                pathLength = pathLength + cartDist(pathC(j,2:4),pathC(j+1,2:4)); 
            end

            %Calculate the path time.
            pathTime = dt*pathH;

            %Print the results on the command line.
            fprintf('\nTRIAL %d\n',1);
            fprintf('Path Length: %.2f m\n',pathLength);
            fprintf('Path Time: %.2f sec\n',pathTime);
            uFinal = [pathC(end,2) pathC(end,3) pathC(end,4)];
            error = cartDist(uFinal,sGoalB(i,1:3));
            fprintf('Final Cartesian Position Error: %.2f m\n',error);
            fprintf('Planning Time: %.3f s\n',planningTime);

            [h,~] = size(pathJ);
            jointPaths(i,1:h,:) = pathJ;
            cartesianPaths(i,1:h,:) = pathC;

            %Update the pathLength and pathTime vectors.
        %     pathLengths(i) = pathLength;
        %     pathTimes(i) = pathTime;
        %     planningTimes(i) = planningTime;
            %errors(i) = error;

            figure(10)
            printRRT(T,sGoalB,pathJ,kC,jointLimits,panHeight,NUM_NODES);

            %figure(2)
            %plotVelocities(pathJ,Dt);

            %Find the path in an x,y,z representation using a forward kinematic
            %model of the system.
            %figure(3)
            %view(-114,54)
            %plotPath(pathC,kC,nGoalB(i,:),panHeight,jointLimits,true,i);

            %figure(4)
            %plotAnkle(pathJ,ankleThreshold,Dt);

            %figure(5)
            %plotWheelSpeed(pathJ,Dt);

            [hTemp,~] = size(pathC);
            if hTemp > h
                h = hTemp;
            end

        else
            fprintf('Planning Failed\n');
        end
    end

    for i = 1:numLegs
        for j = 2:1000
            if cartesianPaths(i,j,1) == 0
                cartesianPaths(i,j,:) = cartesianPaths(i,j-1,:);
            end
            if jointPaths(i,j,1) == 0
                jointPaths(i,j,:) = jointPaths(i,j-1,:);
            end
        end
    end
    
    %SHERPA TT VISUALISATION

    init = true;
    t = 0;
    for i = 1:h
        plotSherpaTT([0 0 bodyHeight],[jointPaths(1,i,2:7); jointPaths(2,i,2:7); jointPaths(3,i,2:7); jointPaths(4,i,2:7)],kC,init,true,true);
        pause(dt);
        t = t + dt;
        title(['t = ' num2str(t) 's'])
        init = false;
    end

    %If only one tree is being generated, then plot the tree, the trajectory
    %generated and the velocities over time.
    %if NUM_TRIALS == 1



    % %If there is more than one trial, plot the tree evaluation results for each
    % %trial.    
    % else
    %     figure(1)
    %     subplot(2,2,1);
    %     hold on
    %     plot(1:NUM_TRIALS,pathLengths,'b*');
    %     %plot(1:NUM_TRIALS,optPathLength*ones(1,NUM_TRIALS),'k*');
    %     xlabel('Trial Number');
    %     ylabel('Path Length [m]');
    %     hold off
    %     subplot(2,2,2);
    %     hold on
    %     plot(1:NUM_TRIALS,pathTimes,'g*');
    %     xlabel('Trial Number');
    %     ylabel('Path Time [s]');
    %     hold off
    %     subplot(2,2,3);
    %     plot(1:NUM_TRIALS,planningTimes,'b*');
    %     xlabel('Trial Number');
    %     ylabel('Planning Time [s]');
    %     subplot(2,2,4);
    %     plot(1:NUM_TRIALS,errors,'k*');
    %     xlabel('Trial Number');
    %     ylabel('Final Position Error [m]');
    %     
    %     date = datestr(now,30);
    %     
    %     save([date '_SherpaRRTTrial'],'pathLengths','pathTimes','planningTimes','errors','NUM_NODES','exhaustive','HGAINS','K','nInitB','nGoalB','goalSeedFreq','U','threshold','ankleThreshold','NUM_TRIALS');
    % end
end


