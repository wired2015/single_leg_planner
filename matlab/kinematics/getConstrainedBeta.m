function beta = getConstrainedBeta(panHeight,gamma,kinematicConst)
%GETCONSTRAINEDBETA Calculates the beta joint anglgiven a constrained body
%height.

    [L1,~,L3,L4,L5,L6,~,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);
    
    beta = -asin((panHeight-L1+L4*sin(zeta)+L5*sin(gamma+zeta)+L6+L8)/L3);
    
end