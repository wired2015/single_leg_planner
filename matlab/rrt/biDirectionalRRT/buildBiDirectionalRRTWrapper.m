%BUILDRRTWRAPPER This function acts as a wrapper for the buildRRT function.
%Code generation for the singleLegPlanner is performed using this function
%as an entry point.
%  
%Inputs:
%-nInitCartesianB: the
%-nGoalCartesianB:
%-jointLimits:
%-bodyHeight:
%-U:
%-dt:
%-Dt:
%-kC:
%-threshold:
%-legNum:
%
%Outputs:
%-T:
%-pathC:
%-pathJ:
%-success:
%
%buildRRTWrapper.m
%author: wreid
%date: 20150502

function [T1,T2,pathC,pathJ,success] = buildBiDirectionalRRTWrapper(nInitCartesianB,nGoalCartesianB,phiInit,omegaInit,jointLimits,bodyHeight,kC,legNum,uBDot)
    
    NODE_SIZE = int32(13);
    U_SIZE = int32(5);
    ankleThreshold = pi/8;
    exhaustive = false;
    goalSeedFreq = int32(20);
    cartesianLimits = [-0.2930   -1.1326   -0.6710   -0.7546];
    dt = 0.1;
    Dt = 0.7;
    HGAINS = [1 0 0.5];
    threshold = 0.005;
    stepAccRatio = 14;
    eta = Dt/stepAccRatio;
    U = eta*[1 0; -1 0; 0 1; 0 -1; 0 0]; 
    NUM_NODES = int32(1500);
    
    panHeight  = getPanHeight(bodyHeight,kC);

    %Transform the nInitCartesianB and nGoalCartesianB variables from the body coordinate frame
    %to the pan coordinate frame.
    TP2B = trP2B(kC,legNum);
    TB2P = trInv(TP2B); %inv(TP2B);%
    uInitP = TB2P(1:3,1:3)*nInitCartesianB(1:3)' + TB2P(1:3,4);
    uDotInitP = TB2P(1:3,1:3)*nInitCartesianB(4:6)';
    uGoalP = TB2P(1:3,1:3)*nGoalCartesianB(1:3)' + TB2P(1:3,4);
    uDotGoalP = TB2P(1:3,1:3)*nGoalCartesianB(4:6)';
    
    %Transform the Cartesian goal and final positions in the pan coordinate
    %frame to the joint space.
    qInit = sherpaTTIK(uInitP,kC,jointLimits);
    qGoal = sherpaTTIK(uGoalP,kC,jointLimits);
    qDotInit = sherpaTTIKVel(uDotInitP',qInit',kC);
    qDotGoal = sherpaTTIKVel(uDotGoalP',qGoal',kC);
    
    nInitJoint = [qInit phiInit 0 qDotInit' 0 omegaInit];
    nGoalJoint = [qGoal 0 0 qDotGoal' 0 0];
        
    %Check that the initial and final positions are valid. If they are not
    %return failure and an empty path.
    if (validJointState(nInitJoint,jointLimits) && validJointState(nGoalJoint,jointLimits))
        success = true;
        %Run buildRRT.
        nInit = [1 0 0 nInitJoint];
        nGoal = [1 0 0 nGoalJoint];
        [T1,T2,pathJ,pathC] = buildBiDirectionalRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum,TP2B);
        %Transform path back to the Cartesian space.
%         for i = 1:length(pathsJ)
%             pathC = transformPath(pathsJ(i).path,kC,TP2B);
%             pathLength = pathC(end,2);
%             if i <= 1
%                 pathLengthMin = pathLength;
%                 pathCMin = pathC;
%                 pathIndexMin = i;
%             elseif pathLength < pathLengthMin
%                 pathLengthMin = pathLength;
%                 pathCMin = pathC;
%                 pathIndexMin = i;
%             end
%         end
%         pathC = pathCMin;
%         pathJ = pathsJ(pathIndexMin).path;
    else
        success = false;
        pathC = [];
        pathJ = [];
        T1 = [];
        T2 = [];
    end
    
    %Linearly interpolate to the goal state from the final state.
%     sFinalC = pathC(end,:);
%     sGoalC = [0 0 nGoalCartesianB true];
%     pathCorrection = linInterp(sFinalC,sGoalC,10);
%     pathC = [pathC; pathCorrection];
%     [h,~] = size(pathCorrection);
%     for i = 1:h
%         uB = TB2P(1:3,1:3)*pathCorrection(i,3:5)' + TB2P(1:3,4);
%         q = sherpaTTIK(uB',kC,jointLimits);
%         pathJ = [pathJ; [pathCorrection(i,1) q 0 0 0 0 0 0 0]];  
%     end

end

% function pathC = transformPath(pathJ,kC,TP2B)
%     [pathH,~] = size(pathJ);
%     pathC = zeros(pathH,9);
%     dist2Go = 0;
%     for i = 1:pathH
%         uP = sherpaTTFK(pathJ(i,2:4),kC);
%         uPDot = sherpaTTFKVel(pathJ(i,7:9),pathJ(i,2:4),kC);
%         uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
%         uBDot = TP2B(1:3,1:3)*uPDot;
%         if i ~= 1
%             dist2Go = dist2Go + norm(uB'-pathC(i-1,3:5));
%         end
%         pathC(i,:) = [pathJ(i,1) dist2Go uB' uBDot' false];
%     end
% end