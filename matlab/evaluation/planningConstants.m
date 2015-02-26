% PLANNINGCONSTANTS This script stores all of the constants needed to run 
% an evaluation of the planner.
%
% planningConstants.m
% author: wreid
% date: 20150123

%The threshold at which the entire rover body has to stop moving to
%accomodate an ankle moving between time steps.
ankleThreshold = deg2rad(5);      %[rad]

%Run the kinematicConstants script to load all of the constants that
%describe the geometry of the leg being planned for.
kinematicConstants

%Number of nodes to be used by the planner.
NUM_NODES = int32(1000);
%NUM_NODES_MAX = int32(3000);

%The exhaustive boolean indicates if the planner will search for a
%solution until it has found one, or if it will only use the maximum number
%of nodes.
exhaustive = false;

%TODO: Calculate the threshold based on the kinematics of the system or
%pre-defined performance expectations.
%The threshold indicates how far away a state can be from the goal state so
%for a successful plan to be found.
threshold = 0.005;

%TODO: Replace state nodes with a structure. The NODE_SIZE variable will be
%void after this change has taken place.
%The number of data entries in a single node.
NODE_SIZE = int32(13);

%TODO: Make it so that the HGAINS are normalized.
%The heuristic gains.
gains = [1 0 0.1];

%The joint angular and rate limits.
jointLimits = [deg2rad([-135 -59.5 -5 -180 0 -10 -10 -10 -45 -180]);...     %[rad, rad/s]
               deg2rad([135 17 73.7 180 0 10 10 10 45 180])];            %[rad, rad/s]

%The initial state of a leg's wheel contact point relative to the body
%coordinate frame. Each row in the array is structured as 
%[x y z xDot yDot zDot].
sInitB = [0.996 0.996 -1 0 0 0;
            0.996 -0.996 -1 0 0 0;
            -0.996 0.996 -1 0 0 0;
            -0.996 -0.996 -1 0 0 0];

%The goal state of a leg's wheel contact point relative to the body
%coordinate frame. Each row in the array is structured as 
%[x y z xDot yDot zDot].      
sGoalB = [1.257 0.3278 -1 0 0 0;
          1.257 -0.3278 -1 0 0 0;
          -1.257 0.3278 -1 0 0 0;
          -1.257 -0.3278 -1 0 0 0];

%The number of legs that are going to have plans generated.
numLegs = 4;

%TODO: Decide whether or not you want to generate the qInit and qGoal
%matrices here.
%Initialize 4x6 arrays that will contain the initial and goal joint states.      
%qInit = zeros(4,6);
%qGoal = zeros(4,6);
%
%Iterate over each leg and find the initial and goal joint states.
% for i = 1:numLegs  
%     [~,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices([0,0,0],[0,0,0,0],kC,i);
%     TB2P = trInv(TP2B); %inv(TP2B);
%     uInitP = TB2P(1:3,1:3)*nInitB(i,1:3)' + TB2P(1:3,4);
%     uDotInitP = TB2P(1:3,1:3)*nInitB(i,4:6)';
%     uGoalP = TB2P(1:3,1:3)*nGoalB(i,1:3)' + TB2P(1:3,4);
%     uDotGoalP = TB2P(1:3,1:3)*nGoalB(i,4:6)';
%     qInit(i,1:3) = sherpaTTIK(uInitP,kC,jointLimits);
%     qGoal(i,1:3) = sherpaTTIK(uGoalP,kC,jointLimits);
%     qInit(i,4:6) = sherpaTTIKVel(uDotInitP',qInit',kC)';
%     qGoal(i,4:6) = sherpaTTIKVel(uDotGoalP',qGoal',kC)';
% end
% 
% clear qInit qGoal qDotInit qDotGoal
      
%TODO: Investigate appropriate values for the stepAccRatio. Come up with a
%more concrete definition for this.
%A constant that is used to determine the appropriate ratio between joint
%acceleration and planner sample time.
stepAccRatio = 14;
%The planner time step.
Dt = 0.7; %[s]
%The integration time step.
dt = 0.1; %[s]

eta = Dt/stepAccRatio;
U = eta*[1 0;                   % The control input set: 
          -1 0;                 % [alphaDot betaDot gammaDot] [m/s^2].
          0 1;                  % There can only be an acceleration along
          0 -1;                 % the ground plane, or no acceleration.
          0 0]; 
      
U = eta*[1 0;                   % The control input set: 
  -1 0;                 % [alphaDot betaDot gammaDot] [m/s^2].
  0 0.5;                  % There can only be an acceleration along
  0 -0.5;                 % the ground plane, or no acceleration.
  0.5 0.25;                    
  -0.5 0.25;                 
  0.5 -0.25;
  -0.5 -0.25; 
  0 0]; 

%The number of evaluation trials to be performed.
NUM_TRIALS = 1;

%Calculate the height of the body coordinate frame by taking the average of
%each of the wheel contact points initial heights relative to the body.
bodyHeight = -sum(sInitB(:,3))/4;
uG = [0 0 bodyHeight];
panHeight = getPanHeight(bodyHeight,kC);

%The number of steps between which the goal state is sampled in stead of a
%state node being sampled.
goalSeedFreq = int32(20);

uBDot = [0 0 0 0 0 0]';

cartesianLimits = zeros(1,4);
betaMin = jointLimits(1,2);
betaMax = jointLimits(2,2);
gammaMin = jointLimits(1,3);
gammaMax = jointLimits(2,3);
u1 = sherpaTTFK([0 betaMin gammaMin],kC);
u2 = sherpaTTFK([0 betaMax gammaMax],kC);
u3 = sherpaTTFK([0 betaMin gammaMax],kC);
u4 = sherpaTTFK([0 betaMax gammaMin],kC);
cartesianLimits(1) = u1(3);
cartesianLimits(2) = u2(3);
cartesianLimits(3) = u3(3);
cartesianLimits(4) = u4(3);


