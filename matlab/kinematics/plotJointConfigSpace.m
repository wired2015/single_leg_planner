function plotJointConfigSpace(kinematicConst,jointLimits,K)

    L1 = kinematicConst(1);
    L2 = kinematicConst(2);
    L3 = kinematicConst(3);
    L4 = kinematicConst(4);
    L5 = kinematicConst(5);
    L6 = kinematicConst(6);
    L7 = kinematicConst(7);
    L8 = kinematicConst(8);
    zeta = kinematicConst(9);
    
    vecLength = 40;

    %Generate vectors for alpha, beta and gamma.
    alpha = linspace(jointLimits(1,1),jointLimits(2,1),vecLength);
    %beta = linspace(betaMIN,betaMAX,vecLength);
    gamma = linspace(jointLimits(1,3),jointLimits(2,3),vecLength);
    beta = -asin((K-L1+L4*sin(zeta)+L5*sin(gamma+zeta)+L6+L8)/L3);

    xVec = zeros(1,vecLength^2);
    yVec = zeros(1,vecLength^2);
    zVec = zeros(1,vecLength^2);

    %For each possible alpha,beta,gamma groups calculate the corresponding x,y,z
    %Cartesian position of the contact point.
    index = 1;
    
    hold on

    for i = 1:vecLength
        for j = 1:vecLength
            if (imag(beta(j)) == 0 && beta(j) >= jointLimits(1,2) && beta(j) <= jointLimits(2,2))
                index = index + 1;
                plot3(alpha(i),beta(j),gamma(j),'k.');
            end
        end
    end
    
end