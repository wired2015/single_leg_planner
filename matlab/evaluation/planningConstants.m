%planningConstants.m
%author: wreid
%date: 20150123

%This script stores all of the constants needed to run the rrtEval.m.

%The threshold at which the entire rover body has to stop moving to
%accomodate an ankle moving between time steps.
ankleThreshold = pi/8;      %[rad]

sherpaTTKinematicConstants

%Number of nodes.
NUM_NODES = int32(1000);
NUM_NODES_MAX = int32(1000);

%The relative angles between the body coordinate frame and the pan
%coordinate frames. Leg1 is front left, Leg2 is front right, Leg3 is back
%left and Leg4 is back right.
legNum = int32(4);

%The exhaustive boolean indicates if the planner will search for a
%solution until it has found one, or if it will only use the maximum number
%of nodes.
exhaustive = false;

%TODO: Calculate the threshold based on the kinematics of the system or
%pre-defined performance expectations.
threshold = 0.04;

%The number of data entries in a single node.
NODE_SIZE = int32(11);

%The heuristic gains.
HGAINS = [1 0 0.5];

%The joint angular and rate limits.
MIN = deg2rad([-135 -59.5 -5 -10 -20 -20]);    %[rad, rad/s]
MAX = deg2rad([135 17 73.7 10 20 20]);         %[rad, rad/s]
jointLimits = [MIN;MAX];

%The name that will put on the path.
name = 'Single Leg RRT';

nInitB = [0.996 0.996 -1 0 0 0;
            0.996 -0.996 -1 0 0 0;
            -0.996 0.996 -1 0 0 0;
            -0.9967 -0.996 -1 0 0 0];
      
nGoalB = [1.257 0.3278 -1 0 0 0;
          1.257 -0.3278 -1 0 0 0;
          -1.257 0.3278 -1 0 0 0;
          -1.257 -0.3278 -1 0 0 0];

qInit = zeros(4,6);
qGoal = zeros(4,6);

for i = 1:4  
    [~,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices([0,0,0],[0,0,0,0],kC,i);
    TB2P = invHomoMatrix(TP2B); %inv(TP2B);%
    uInitP = TB2P(1:3,1:3)*nInitB(i,1:3)' + TB2P(1:3,4);
    uDotInitP = TB2P(1:3,1:3)*nInitB(i,4:6)';
    uGoalP = TB2P(1:3,1:3)*nGoalB(i,1:3)' + TB2P(1:3,4);
    uDotGoalP = TB2P(1:3,1:3)*nGoalB(i,4:6)';
    qInit(i,1:3) = sherpaTTIK(uInitP,kC,jointLimits);
    qGoal(i,1:3) = sherpaTTIK(uGoalP,kC,jointLimits);
    qInit(i,4:6) = sherpaTTIKVel(uDotInitP',qInit',kC)';
    qGoal(i,4:6) = sherpaTTIKVel(uDotGoalP',qGoal',kC)';
end

%qInit(:,1:3)
%qGoal(:,1:3)

clear qInit qGoal qDotInit qDotGoal
      
U_SIZE = int32(5);

stepAccRatio = 14;
%The planner time step.
Dt = 0.7; %[s]
%The integration time step.
dt = 0.1;

eta = Dt/stepAccRatio;
U = eta*[1 0;                   % The control input set: 
          -1 0;                 % [alphaDot betaDot gammaDot] [m/s^2].
          0 1;                  % There can only be an acceleration along
          0 -1;                 % the ground plane, or no acceleration.
          0 0]; 

NUM_TRIALS = 1;

bodyHeight = -sum(nInitB(:,3))/4;
panHeight = getPanHeight(bodyHeight,kC);

goalSeedFreq = int32(20);




