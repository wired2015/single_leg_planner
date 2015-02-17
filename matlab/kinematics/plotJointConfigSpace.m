function plotJointConfigSpace(kC,jointLimits,panHeight)
    
    vecLength = 40;

    %Generate vectors for alpha, beta and gamma.
    alpha = linspace(jointLimits(1,1),jointLimits(2,1),vecLength);
    %beta = linspace(betaMIN,betaMAX,vecLength);
    gamma = linspace(jointLimits(1,3),jointLimits(2,3),vecLength);
    B = kC.l1-kC.l4*sin(kC.zeta)-kC.l6-kC.l8-kC.r;
    beta = asin((-panHeight+B-kC.l5*sin(gamma+kC.zeta))/kC.l3);

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