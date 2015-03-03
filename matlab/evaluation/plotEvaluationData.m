load('plannerEvaluationData')

hold on
figure(1)
xlabel('Path Number')
ylabel('Path Length [m]')

numTrials = 10;

for j = 1:N
    sumPathLengths = 0;
    for i = 1:numTrials
        sumPathLengths = rrtResults(i,j).pathLength + sumPathLengths;
    end
    meanPathLength = sumPathLengths/numTrials;
    plot(j,meanPathLength,'r*');
end
for j = 1:N
    sumPathLengths = 0;
    for i = 1:numTrials
        sumPathLengths = biDirResults(i,j).pathLength + sumPathLengths;
    end
    meanPathLength = sumPathLengths/numTrials;
    plot(j,meanPathLength,'k*');
end
hold off

figure(2)
hold on
xlabel('Path Number')
ylabel('Path Execution Time [s]')
for j = 1:N
    sumPathTimes = 0;
    for i = 1:numTrials
        sumPathTimes = rrtResults(i,j).pathTime + sumPathTimes;
    end
    meanPathTime = sumPathTimes/numTrials;
    plot(j,meanPathTime,'r*');
end
for j = 1:N
    sumPathTimes = 0;
    for i = 1:numTrials
        sumPathTimes = biDirResults(i,j).pathTime + sumPathTimes;
    end
    meanPathTime = sumPathTimes/numTrials;
    plot(j,meanPathTime,'k*');
end
hold off

figure(3)
hold on
xlabel('Path Number')
ylabel('Num Ankle Violations')
for j = 1:N
    sumAnkleViolations = 0;
    for i = 1:numTrials
        sumAnkleViolations = rrtResults(i,j).numAnkleViolations + sumAnkleViolations;
    end
    meanAnkleViolations = sumAnkleViolations/numTrials;
    plot(j,meanAnkleViolations,'r*');
end
for j = 1:N
    sumAnkleViolations = 0;
    for i = 1:numTrials
        sumAnkleViolations = biDirResults(i,j).numAnkleViolations + sumAnkleViolations;
    end
    meanAnkleViolations = sumAnkleViolations/numTrials;
    plot(j,meanAnkleViolations,'k*');
end
hold off

figure(4)
hold on
xlabel('Path Number')
ylabel('Planning Time [s]')
for j = 1:N
    sumPlanningTime = 0;
    for i = 1:numTrials
        sumPlanningTime = rrtResults(i,j).planningTime + sumPlanningTime;
    end
    meanPlanningTime = sumPlanningTime/numTrials;
    plot(j,meanPlanningTime,'r*');
end
for j = 1:N
    sumPlanningTime = 0;
    for i = 1:numTrials
        sumPlanningTime = biDirResults(i,j).planningTime + sumPlanningTime;
    end
    meanPlanningTime = sumPlanningTime/numTrials;
    plot(j,meanPlanningTime,'k*');
end
hold off