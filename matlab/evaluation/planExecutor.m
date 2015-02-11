%planExecutor.m
%author: wreid
%date: 20150203

classdef planExecutor < handle
    %planExecutor Used to execute a singleLegPlan
    %   Detailed explanation goes here
    
    properties (Constant)
        NUM_NODES = 1000;
        NUM_NODES_MAX = 1000;
        DEG2RAD = 0.0175;
        jointLimits = [DEG2RAD*([-135 -59.5 -5 -10 -20 -20]);...  %[rad, rad/s]
                       DEG2RAD*([135 17 73.7 10 20 20])];         %[rad, rad/s]
        K = -0.6;
        HGAINS = [1 0 0.1];
        NODE_SIZE = 11;
        U_SIZE = 5;
        dt = 0.01;
        Dt = 0.1;
        L1 = 0.175*cosd(35);        %[m]
        L2 = 0.045;                 %[m]
        L3 = 0.400;                 %[m]
        L4 = 0.2;                   %[m]
        L5 = 0.4;                   %[m]
        L6 = 0.159*sind(34.5);      %[m]
        L7 = 0.159*cosd(34.5);      %[m]
        L8 = 0.39569+0.378/2;       %[m]
        zeta = DEG2RAD*7;          %[rad]
        r = 0.378/2;                %[m]
        bodyW = 0.65;               %[m]
        bodyH = 0.65;               %[m]
        ankleThreshold = pi/4;
        exhaustive = true;
        threshold = 0.05;
        goalSeedFreq = 50;
        stepAccRatio = 17;
    end
    
    properties
        nInit
        nGoal
        kinematicConst;
        U;
        eta;
        path;
        T;
        nodeCount;
        currentTime;
    end
    
    methods
        
        function pe = planExecutor(nInit,nGoal,startTime)
            pe.eta = pe.Dt/pe.stepAccRatio;
            pe.U =  pe.eta*[1 0;           % The control input set: 
                     -1 0;                 % [alphaDot betaDot gammaDot] [m/s^2].
                    0 0.5;                 % There can only be an acceleration along
                    0 -0.5;                % the ground plane, or no acceleration.
                    0 0];
            pe.kinematicConst = [pe.L1 pe.L2 pe.L3 pe.L4 pe.L5 pe.L6 pe.L7...
                                 pe.L8 pe.zeta pe.r pe.bodyW pe.bodyH];
            pe.nInit = nInit;
            pe.nGoal = nGoal;
            [alphaInit,betaInit,gammaInit] = sherpaTTIK(pe.nInit(1),pe.nInit(2),pe.nInit(3),pe.kinematicConst,pe.jointLimits);
            [alphaGoal,betaGoal,gammaGoal] = sherpaTTIK(pe.nGoal(1),pe.nGoal(2),pe.nGoal(3),pe.kinematicConst,pe.jointLimits);
            pe.nInit = [1.0000 0 0 alphaInit betaInit gammaInit 0 0 0 0 0];
            pe.nGoal = [1.0000 0 0 alphaGoal betaGoal gammaGoal 0 0 0 0 0];
            [pe.T,pe.path,pe.nodeCount] = buildRRT(pe.nInit,pe.nGoal,pe.NUM_NODES,pe.NUM_NODES_MAX,pe.jointLimits,pe.K,pe.HGAINS,pe.NODE_SIZE,pe.U,pe.U_SIZE,pe.dt,pe.Dt,pe.kinematicConst,pe.ankleThreshold,pe.exhaustive,pe.threshold,pe.goalSeedFreq);
            pe.currentTime = startTime;
        end
        
%         function rePlan(pe,nInitNew,nGoal)
%             
%         end
%         
        function [x,xDot] = getState(pe,t,path)
             
        end
          
    end
    
end

