function [xNew,transitionArray] = selectInput(xNear,xRand,U,dt,Dt,NODE_SIZE,U_SIZE,HGAINS,kC,ankleThreshold,jointLimits)
%selectInput Selects the most appropriate control input.
%   A control input is selected from a set of control inputs, U. An input
%   is selected by applying each of the inputs to to state xNear, which
%   results in p candidate states, where p is the size of the input set.
%   The control input corresponding to candidate state that is closest to
%   x1 is returned as u.

    %Initialize arrays to store the candidate new state data and the
    %distances between each candidate state and the xNear state.
    candStates = zeros(U_SIZE,NODE_SIZE);
    candTransArrays = zeros(U_SIZE,(round(Dt/dt)+1)*6);
    distance = zeros(1,U_SIZE);
    
    qANear = xNear(end);
    
    UJoint = zeros(U_SIZE,3);
    
    %Transform the control inputs to joint space.
    for i = 1:U_SIZE
        %gammaDotDot = (-betaDotDot*L3*cos(beta)+betaDot^2*L3*sin(beta)+gammaDot^2*L5*sin(zeta+gamma))/(L5*cos(zeta+gamma));
        gammaDotDot = getConstrainedGammaDotDot(kC,U(i,:),xNear(7:9),xNear(4:6));
        UJoint(i,:) = [U(i,:) gammaDotDot];
    end    

    %Increment over the control vector. Generate a candidate state for each
    %possible control input.
    for i = 1:U_SIZE
        
        %Generate a candidate state using a fourth order Runge-Kutta 
        %integration technique.
        [candStates(i,:),candTransArrays(i,:)] = rk4(UJoint(i,:),dt,Dt,xNear,jointLimits);
        %U_check = sherpaTTFKAcc(U_joint(i,:)',xNear(7:9)',xNear(4:6)',kinematicConst)
        %velCheck = sherpaTTFKVel(candStates(i,7:9)',candStates(i,4:6)',kinematicConst);
       
        %Calculate the distance between the candidate state and the random
        %state.
        hDiff = heuristicSingleLeg(candStates(i,:),xRand,HGAINS,jointLimits,kC);

        %Apply the ankle constraint to penalize any candidate state that
        %requires a change of ankle position greater than the allowed ankle
        %movement in a single time step.
        aGain = HGAINS(3);
        if xNear(1) == 1
            aGain = 0;
        end
        
        %Ss = [1000 1];
        %qWDot = 1;
        %r = 0.378/2;
        
        [candStates(i,10),candStates(i,11)] = calcPhi(candStates(i,7:9),candStates(i,4:6),kC);%,Ss,qWDot,r);
        aDiff = angDiff(qANear,candStates(i,10));
        ankleDiffMax = pi;
        
        if aDiff > ankleThreshold
            aDiff = 1;
        else
            aDiff = abs(aDiff/ankleDiffMax);
            aDiff = 0;
        end
        
        %Calculate a distance metric that includes the heurisitc distance
        %as well as any penalty due to ankle movements.
        distance(i) = (1-aGain)*hDiff + aGain*aDiff;
        %distance(i) = hDiff;
    end
    
    
    [~,minIndex] = min(distance);
    xNew = candStates(minIndex,:);
    transitionArray = candTransArrays(minIndex,:);
    
    %velCheck = sherpaTTFKVel(xNew(7:9)',xNew(4:6)',kinematicConst)

end

