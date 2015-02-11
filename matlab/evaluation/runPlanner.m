%The goal and initial positions.
xInit = -0.4122;    %[m]
yInit = -0.6819;    %[m]
zInit = -0.6;       %[m]

% xInit = 0.4345;    %[m]
% yInit = 0.2627;    %[m]
% zInit = -0.6;       %[m]

xGoal = -0.1328;    %[m]
yGoal = 0.7245;     %[m]
zGoal = -0.6;       %[m]

pe = planExecutor([xInit yInit zInit],[xGoal yGoal zGoal]);