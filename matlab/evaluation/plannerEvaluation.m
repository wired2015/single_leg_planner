%This script evaluates the planners by running them over a sequence of
%valid initial/goal pairs.

states = load('experimentStates');
[N,~] = size(states.states);

plannerResults = struct('T1',{},'T2',{},'pathC',{},'pathJ',{},'success',{},'pathLength',{},'pathTime',{},'planningTime',{});
biDirResults = plannerResults;
rrtResults = plannerResults;

numTrials = 10;

planningConstants

for i = 1:numTrials
    fprintf('Trial %d/%d\n',i,numTrials)
    for j = 1:N
        fprintf('Path %d/%d\n',j,N);
        
        if j == N
            sInitB = states.states(j,:);
            sGoalB = states.states(1,:);
        else
            sInitB = states.states(j,:);
            sGoalB = states.states(j+1,:);
        end

        tic
        [~,pathC,pathJ,success] = buildRRTWrapper_mex('buildRRTWrapper',sInitB,sGoalB,0,0,jointLimits,bodyHeight,U,dt,Dt,kC,int32(1),uBDot);
        rrtResults(i,j).planningTime = toc;
        
        if success
            rrtResults(i,j).pathC = pathC;
            rrtResults(i,j).pathJ = pathJ;
            rrtResults(i,j).success= success;

            %Calculate the path length from the generated RRT.
            pathLength = pathC(end,2);
            [pathH,~] = size(pathC);
            %Calculate the path time.
            pathTime = dt*pathH;

            rrtResults(i,j).pathLength = pathLength;
            rrtResults(i,j).pathTime = pathTime;

            numAnkleViolations = 0;
            [pathJH,~] = size(pathJ);
            for k = 1:length(pathJH)-1
                if angDiff(pathJ(k,5),rrtResults(i,j).pathJ(k+1,5)) > ankleThreshold
                    numAnkleViolations = numAnkleViolations + 1;
                end
            end
            rrtResults(i,j).numAnkleViolations = numAnkleViolations;
        else
            rrtResults(i,j).pathLength = 1000;
            rrtResults(i,j).pathTime = 1000;
            rrtResults(i,j).numAnkleViolations = 1000;
        end
        
        tic
        [~,~,pathC,pathJ,success] = buildBiDirectionalRRTWrapper_mex(sInitB,sGoalB,0,0,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,int32(1),uBDot);
        biDirResults(i,j).planningTime = toc;
        
        biDirResults(i,j).success= success;

        
        if success
            biDirResults(i,j).pathC = pathC;
            biDirResults(i,j).pathJ = pathJ;

            %Calculate the path length from the generated RRT.
            pathLength = pathC(end,2);
            [pathH,~] = size(pathC);
            %Calculate the path time.
            pathTime = dt*pathH;

            biDirResults(i,j).pathLength = pathLength;
            biDirResults(i,j).pathTime = pathTime;

            numAnkleViolations = 0;
            [pathJH,~] = size(pathJ);
            for k = 1:length(pathJH)-1
                if angDiff(pathJ(k,5),biDirResults(i,j).pathJ(k+1,5)) > ankleThreshold
                    numAnkleViolations = numAnkleViolations + 1;
                end
            end
            biDirResults(i,j).numAnkleViolations = numAnkleViolations;
        else
            fprintf('BiDirRRT Planning Failed\n');
            biDirResults(i,j).pathLength = 1000;
            biDirResults(i,j).pathTime = 1000;
            biDirResults(i,j).numAnkleViolations = 1000;
        end
        
    end
end

save('plannerEvaluationData','rrtResults','biDirResults','N')

%     [~,~,pathC,pathJ,success] = buildRRTWrapper_mex(sInitB(i,:),sGoalB(i,:),0,0,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,int32(i),uBDot);
% 
%     rrtResults(i).pathC = pathC;
%     rrtResults(i).pathJ = pathJ;
%     rrtResults(i).success= success;
%     
%     %Calculate the path length from the generated RRT.
%     pathLength = pathC(end,2);
%     %Calculate the path time.
%     pathTime = dt*pathH;
%     
%     rrtResults(i).pathLength = pathLength;
%     rrtResults(i).pathTime = pathTime;
%     
%     numAnkleViolations = 0;
%     for j = 1:length(pathJ)-1
%         if angDiff(pathJ(j,5),s.pathJ(j+1,5)) > ankleThreshold
%             numAnkleViolations = numAnkleViolations + 1;
%         end
%     end
%     rrtResults(i).numAnkleViolations = numAnkleViolations;




    

    
