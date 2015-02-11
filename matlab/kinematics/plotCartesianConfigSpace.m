function plotCartesianConfigSpace(kinematicConst,jointLimits,panHeight,legNum)   
    
    vecLength = 20;
    
    %Generate vectors for alpha, beta and gamma.
    alpha = linspace(jointLimits(1,1),jointLimits(2,1),vecLength);
    gamma = linspace(jointLimits(1,3),jointLimits(2,3),vecLength);
    beta = getConstrainedBeta(panHeight,gamma,kinematicConst);

    xVec = zeros(1,vecLength^2);
    yVec = zeros(1,vecLength^2);
    zVec = zeros(1,vecLength^2);

    %For each possible alpha,beta,gamma groups calculate the corresponding x,y,z
    %Cartesian position of the contact point.
    index = 1;
    
    hold on
    
    TP2B = trP2B(kinematicConst,legNum);
    trplot(eye(4,4),'rgb','frame','B','length',0.2);
    trplot(TP2B,'rgb','frame',['P_' num2str(legNum)],'length',0.2);

    for i = 1:vecLength
        for j = 1:vecLength
            if (imag(beta(j)) == 0 && beta(j) >= jointLimits(1,2) && beta(j) <= jointLimits(2,2))
                index = index + 1;
                [xVec(index),yVec(index),zVec(index)] = sherpaTTFK(alpha(i),beta(j),gamma(j),kinematicConst);
                uB = TP2B(1:3,1:3)*[xVec(index);yVec(index);zVec(index)] + TP2B(1:3,4);
                plot3(uB(1),uB(2),uB(3),'Color',[0.1,0.1,0.1],'Marker','.','LineStyle','none');
            end
        end
    end
    
end