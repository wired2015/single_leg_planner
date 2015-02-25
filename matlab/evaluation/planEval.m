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
    
    numLegs = 1;

    %Print header for planner evaluation.
%     fprintf(['\nSingle Leg Planner Evaluation ####\n\n']);
%     fprintf('Num Trials: %d\n',NUM_TRIALS);
%     if exhaustive
%         fprintf('Exhaustive Search, Max Number of Nodes: %d\n',NUM_NODES_MAX);
%     else
%         fprintf('Finite Search\nNumber of Nodes: %d\n',NUM_NODES);
%     end
%     fprintf('Position Heuristic Gain: %.2f\n',gains(1));
%     fprintf('Velocity Heuristic Gain: %.2f\n',gains(2));
%     fprintf('Ankle Heuristic Gain: %.2f\n\n',gains(3));
    
    %Print out the body rates.
    fprintf('Body Velocity: [%.2f %.2f %.2f]\n',uBDot(1), uBDot(2), uBDot(3))
    fprintf('Body Angular Rate: [%.2f %.2f %.2f]\n',uBDot(4), uBDot(5), uBDot(6));

    %Initialize a struct that will hold the results frome each of the
    %generated plans.
    plannerResults = struct('T',{},'pathC',{},'pathJ',{},'success',{},'pathLength',{},'pathTime',{});

    %Run the planner on each of the Sherpa_TT legs.
    for i=1:numLegs
        
        sInitB = [randomPoint(jointLimits,cartesianLimits,panHeight,kC,i) 0 0 0];
        sGoalB = [randomPoint(jointLimits,cartesianLimits,panHeight,kC,i) 0 0 0];

        %Generate the RRT and time how long it takes to be generated.
        tic
        if useMex
            [T,pathC,pathJ,success] = buildRRTWrapper_mex(sInitB(i,:),sGoalB(i,:),0,0,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,int32(i),uBDot);
        else
            [T,pathC,pathJ,success] = buildRRTWrapper(sInitB(i,:),sGoalB(i,:),0,0,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,i,uBDot);
        end
        planningTime = toc;

        %Store the planner results in the plannerResults array of
        %structures.
        plannerResults(i).T = T;
        plannerResults(i).pathC = pathC;
        plannerResults(i).pathJ = pathJ;
        plannerResults(i).success= success;

        if success
            
            %Calculate the path length from the generated RRT.
            [pathH,~] = size(pathC);
            pathLength = pathC(end,2);

            %Calculate the path time.
            pathTime = dt*pathH;

            plannerResults(i).pathLength = pathLength;
            plannerResults(i).pathTime = pathTime;

            %Print the results on the command line.
            fprintf('\nLeg %d\n',i)
            fprintf('Path Length: %.2f m\n',pathLength);
            fprintf('Path Time: %.2f sec\n',pathTime);
            uFinal = [pathC(end,3) pathC(end,4) pathC(end,5)];
            error = cartDist(uFinal,sGoalB(i,1:3));
            fprintf('Final Cartesian Position Error: %.3f m\n',error);
            fprintf('Planning Time: %.3f s\n',planningTime);

            figure((i-1)*numLegs+1)
            printRRT(T,sGoalB(i,1:3),pathJ,kC,jointLimits,panHeight,NUM_NODES);

            figure((i-1)*numLegs+2)
            plotVelocities(pathC);
            
            figure((i-1)*numLegs+3)
            plotJointVelocities(pathJ);

            %Find the path in an x,y,z representation using a forward kinematic
            %model of the system.
            %figure(3)
            %view(-114,54)
            %plotPath(pathC,kC,nGoalB(i,:),panHeight,jointLimits,true,i);

            figure((i-1)*numLegs+4)
            plotAnkle(pathJ,ankleThreshold,Dt);

            %figure(5)
            %plotWheelSpeed(pathJ,Dt);
        else
            fprintf('Planning Failed\n');
        end
    end
    
    %SHERPA TT VISUALISATION
    plotSherpaTTMotion(uG,plannerResults,dt,numLegs,kC);

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


