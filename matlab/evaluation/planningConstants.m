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

% %The goal and initial positions.
% xInitP = 0.1;    %[m]
% yInitP = 0.1;    %[m]
% zInitP = -0.6;       %[m]
% 
% % xInit = 0.4345;    %[m]
% % yInit = 0.2627;    %[m]
% % zInit = -0.6;       %[m]
% 
% xGoalP = -0.1;    %[m]
% yGoalP = -0.1;     %[m]
% zGoalP = -0.6;       %[m]
% 
% xDotInitP = 0;
% yDotInitP = 0;
% zDotInitP = 0;
% 
% xDotGoalP = 0;
% yDotGoalP = 0;
% zDotGoalP = 0;
% 
% TP2B = generateDHTransMatrix(kinematicConst(11+legNum),0,kinematicConst(10),0);
% TB2P = inv(TP2B);
% 
% uInitB = TP2B(1:3,1:3)*[xInitP;yInitP;zInitP] + TP2B(1:3,4);
% uGoalB = TP2B(1:3,1:3)*[xGoalP;yGoalP;zGoalP] + TP2B(1:3,4);
% uDotInitB = TP2B(1:3,1:3)*[xDotInitP;yDotInitP;zDotInitP];
% uDotGoalB = TP2B(1:3,1:3)*[xDotGoalP;yDotGoalP;zDotGoalP];
% 
% [alphaInit,betaInit,gammaInit] = sherpaTTIK(xInitP,yInitP,zInitP,kinematicConst,jointLimits);
% [alphaGoal,betaGoal,gammaGoal] = sherpaTTIK(xGoalP,yGoalP,zGoalP,kinematicConst,jointLimits);

nInitB = [0.9397 0.9397 -0.6 0 0 0;
            0.9397 -0.9397 -0.6 0 0 0;
            -0.9397 0.9397 -0.6 0 0 0;
            -0.9397 -0.9397 -0.6 0 0 0];
      
nGoalB = [1.126 0.3385 -0.6 0 0 0;
          0.1859 -1.019 -0.6 0 0 0;
          -0.04015 0.9478 -0.6 0 0 0;
          -1.09 -0.3415 -0.6 0 0 0];

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

bodyHeight = 0.6;
panHeight = getPanHeight(bodyHeight,kinematicConst);

goalSeedFreq = int32(20);




