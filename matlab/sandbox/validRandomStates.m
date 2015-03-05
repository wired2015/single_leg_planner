planningConstants

N = 100;
states = randomStateGenerator(N,jointLimits,kC,-0.8,1);
%states = [0.770305 0.86415 -0.933 0 0 0];

TB2P = inv(trP2B(kC,1));
statesP = zeros(N,3);
statesJoint = zeros(N,3);
for i = 1:N    
    statesP(i,:) = tr(states(i,1:3)',TB2P);
    statesJoint(i,:) = sherpaTTIK(statesP(i,:)',kC,jointLimits);
end

