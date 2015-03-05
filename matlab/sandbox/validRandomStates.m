planningConstants

N = 10;
states = randomStateGenerator(N,jointLimits,kC,-0.617,1);
TB2P = inv(trP2B(kC,1));
statesP = zeros(N,3);
statesJoint = zeros(N,3);
for i = 1:10    
    statesP(i,:) = tr(states(i,1:3)',TB2P);
    statesJoint(i,:) = sherpaTTIK(statesP(i,:)',kC,jointLimits);
end

