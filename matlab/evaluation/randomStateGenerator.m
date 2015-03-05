%RANDOMSTATEGENERATOR Generates N number of random states that are valid in
%the Sherpa_TT leg's configuration space.
%
%Inputs:
%-N: The number of random states to be generated.
%-jointLimits: A 2x10 array of joint angular limits and angular positions.
%-kC: A structure containing the kinematic constants of the Sherpa TT leg.
%-panHeight: The height of the pan coordinate frame relative to the the
%wheel contact point.
%-legNum: The leg identifier.
%
%Outputs:
%-states: A Nx6 array. Each row of the array describes a single state. A
%state has the structure [x y z xDot yDot zDot]. The state coordinates are
%realtive to the body frame.
%
%randomStateGenerator.m
%author: wreid
%date: 20150305

function states = randomStateGenerator(N,jointLimits,kC,panHeight,legNum)

    legNum = legNum + 1;
    if legNum > 4
        legNum = int32(4);
    elseif legNum < 1
        legNum = int32(1);
    end

    states = zeros(N,6);
    
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

    for i = 1:N
        states(i,:) = [randomPoint(jointLimits,cartesianLimits,panHeight,kC,legNum) 0 0 0];
    end
    
    %save('experimentStates','states')

end
