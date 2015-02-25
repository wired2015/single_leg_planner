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

function [T,pathC,pathJ,success] = buildRRTWrapper(nInitCartesianB,nGoalCartesianB,phiInit,omegaInit,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,legNum,uBDot)

    persistent NUM_NODES 
    persistent NODE_SIZE 
    persistent U_SIZE 
    persistent ankleThreshold 
    persistent exhaustive 
    persistent goalSeedFreq 
    persistent cartesianLimits
    persistent HGAINS
    
    %if isempty(NUM_NODES)
    %    NUM_NODES = int32(1000);
    %end
    if isempty(NODE_SIZE)
        NODE_SIZE = int32(13);
    end
    if isempty(U_SIZE)
        U_SIZE = int32(5);
    end
    if isempty(ankleThreshold)
        ankleThreshold = pi/8;
    end
    if isempty(exhaustive)
        exhaustive = false;
    end
    if isempty(goalSeedFreq)
        goalSeedFreq = int32(20);
    end
    if isempty(cartesianLimits)
        cartesianLimits = [-0.2930   -1.1326   -0.6710   -0.7546];
    end
    if isempty(NUM_NODES)
        NUM_NODES = 1000;
    end
    if isempty(HGAINS)
        HGAINS = [1 0 0.5];
    end
    
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
        nGoal = [0 0 0 nGoalJoint];
        [T,pathJ] = buildRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum);
        %Transform path back to the Cartesian space.
        pathC = transformPath(pathJ,kC,TP2B);
    else
        success = false;
        pathC = [];
        pathJ = [];
        T = [];
    end
    
    %Linearly interpolate to the goal state from the final state.
    sFinalC = pathC(end,:);
    sGoalC = [0 0 nGoalCartesianB true];
    pathCorrection = linInterp(sFinalC,sGoalC,10);
    pathC = [pathC; pathCorrection];
    [h,~] = size(pathCorrection);
    for i = 1:h
        uB = TB2P(1:3,1:3)*pathCorrection(i,3:5)' + TB2P(1:3,4);
        q = sherpaTTIK(uB',kC,jointLimits);
        pathJ = [pathJ; [pathCorrection(i,1) q 0 0 0 0 0 0 0]];  
    end

end

function pathC = transformPath(pathJ,kC,TP2B)
    [pathH,~] = size(pathJ);
    pathC = zeros(pathH,9);
    dist2Go = 0;
    for i = 1:pathH
        uP = sherpaTTFK(pathJ(i,2:4),kC);
        uPDot = sherpaTTFKVel(pathJ(i,7:9),pathJ(i,2:4),kC);
        uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
        uBDot = TP2B(1:3,1:3)*uPDot;
        if i ~= 1
            dist2Go = dist2Go + norm(uB'-pathC(i-1,3:5));
        end
        pathC(i,:) = [pathJ(i,1) dist2Go uB' uBDot' false];
    end
end