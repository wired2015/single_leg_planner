function states = randomStateGenerator(NUM_POINTS,jointLimits,kC,panHeight,legNum)

    states = zeros(NUM_POINTS,6);
    
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

    for i = 1:NUM_POINTS
        states(i,:) = [randomPoint(jointLimits,cartesianLimits,panHeight,kC,legNum) 0 0 0];
    end

end
