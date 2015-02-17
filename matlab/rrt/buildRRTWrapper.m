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

function [T,pathC,pathJ,success] = buildRRTWrapper(nInitCartesianB,nGoalCartesianB,jointLimits,bodyHeight,U,dt,Dt,kC,threshold,legNum,uBDot)

    persistent NUM_NODES 
    persistent HGAINS
    persistent NODE_SIZE 
    persistent U_SIZE 
    persistent ankleThreshold 
    persistent exhaustive 
    persistent goalSeedFreq 
    persistent cartesianLimits
    
    if isempty(NUM_NODES)
        NUM_NODES = int32(1000);
    end
    if isempty(HGAINS)
        HGAINS = [1 0 0.5];
    end
    if isempty(NODE_SIZE)
        NODE_SIZE = int32(11);
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
    
    nInitJoint = [qInit qDotInit'];
    nGoalJoint = [qGoal qDotGoal'];
    
    %Check that the initial and final positions are valid. If they are not
    %return failure and an empty path.
    if (validJointState(nInitJoint,jointLimits) && validJointState(nGoalJoint,jointLimits))
        success = true;
        %Run buildRRT.
        nInit = [0 0 0 nInitJoint 0 0];
        nGoal = [0 0 0 nGoalJoint 0 0];
        [T,pathJ] = buildRRT(nInit,nGoal,NUM_NODES,jointLimits,cartesianLimits,panHeight,HGAINS,NODE_SIZE,U,U_SIZE,dt,Dt,kC,ankleThreshold,exhaustive,threshold,goalSeedFreq,uBDot,legNum);
        %Transform path back to the Cartesian space.
        [pathC,pathJ] = transformPath(pathJ,NODE_SIZE,kC,dt,Dt,TP2B);
        pathC = [0 nInitCartesianB; pathC];
        pathJ = [0 qInit qDotInit'; pathJ];
    else
        success = false;
        pathC = [];
        pathJ = [];
        T = [];
    end 

end

function [pathC,pathJ] = transformPath(pathOld,NODE_SIZE,kC,dt,Dt,TP2B)
    %Take the pathOld array and combine the general nodes and intermediate
    %states into a uniform path. The output path should be a npx6 array
    %that contains the n general nodes and the p intermediate nodes between
    %general nodes. Each row in the path matrix contains 
    %[t,x,y,z,xDot,yDot,zDot] state data.
    
    [pathH,pathW] = size(pathOld);
    pathC = zeros(round(Dt/dt)*pathH,7);
    pathJ = zeros(round(Dt/dt)*pathH,7);
    count = 1;
    time = round(Dt/dt)*pathH*dt;
    for i = 1:pathH
        for j = pathW:-6:NODE_SIZE+7
            uP = sherpaTTFK(pathOld(i,j-5:j-3),kC);
            uB = TP2B(1:3,1:3)*uP' + TP2B(1:3,4);
            uDot = sherpaTTFKVel(pathOld(i,j-2:j)',pathOld(i,j-5:j-3)',kC);
            uBDot = TP2B(1:3,1:3)*uDot;
            pathC(count,:) = [time uB' uBDot'];
            pathJ(count,:) = [time pathOld(i,j-5:j)];
            time = time - dt;
            count = count + 1;
        end
    end
    
    pathC = flipud(pathC(:,1:end));
    pathJ = flipud(pathJ(:,1:end));
    
end