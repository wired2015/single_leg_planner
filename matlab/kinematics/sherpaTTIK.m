%sherpaTTIK.m
%author: wreid
%date: 20150122

function [alpha,beta,gamma] = sherpaTTIK(xC,yC,zC,kinematicConst,jointLimits)
%sherpaTTIK Calculates the joint values for a g1iven contact point.
%   Calculates the joint values for a g1iven contact point for the Sherpa TT
%   leg. All coord1inates are in the pan joint coord1inate frame.
    
    [L1,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);


    x = sqrt(xC^2+yC^2);
    z = zC;
    
    alphaMin = jointLimits(1,1);
    alphaMax = jointLimits(2,1);
    betaMin = jointLimits(1,2);
    betaMax = jointLimits(2,2);
    gammaMin = jointLimits(1,3);
    gammaMax = jointLimits(2,3);
    
    gammaRaw = [- zeta - log(-(L1^2*exp(zeta*1i)*1i + L2^2*exp(zeta*1i)*1i - L3^2*exp(zeta*1i)*1i + L5^2*exp(zeta*1i)*1i + L6^2*exp(zeta*1i)*1i + L7^2*exp(zeta*1i)*1i + L8^2*exp(zeta*1i)*1i + x^2*exp(zeta*1i)*1i + z^2*exp(zeta*1i)*1i - (exp(zeta*2*1i)*(L1^2 - 2*sin(zeta)*L1*L4 - 2*L1*L6 - 2*L1*L8 - 2*L1*z + L2^2 + 2*cos(zeta)*L2*L4 - 2*L2*L7 - 2*L2*x - L3^2 + L4^2 + 2*sin(zeta)*L4*L6 - 2*cos(zeta)*L4*L7 + 2*sin(zeta)*L4*L8 - 2*cos(zeta)*L4*x + 2*sin(zeta)*L4*z + L5^2 + L6^2 + 2*L6*L8 + 2*L6*z + L7^2 + 2*L7*x + L8^2 + 2*L8*z + x^2 + z^2)^2 + 4*L5^2*exp(zeta*1i)*(- L4 + x*exp(zeta*1i) + z*exp(zeta*1i)*1i - L1*exp(zeta*1i)*1i - L2*exp(zeta*1i) + L6*exp(zeta*1i)*1i + L7*exp(zeta*1i) + L8*exp(zeta*1i)*1i)*(L2 - L1*1i + L6*1i - L7 + L8*1i - x + z*1i + L4*exp(zeta*1i)))^(1/2)*1i - L4*x*(exp(zeta*2*1i)/2 + 1/2)*2*1i - L1*L6*exp(zeta*1i)*2*1i - L1*L8*exp(zeta*1i)*2*1i - L2*L7*exp(zeta*1i)*2*1i + L6*L8*exp(zeta*1i)*2*1i - L1*L4*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L6*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L8*(cos(2*zeta) + sin(2*zeta)*1i - 1) - L2*x*exp(zeta*1i)*2*1i + L7*x*exp(zeta*1i)*2*1i - L1*z*exp(zeta*1i)*2*1i + L6*z*exp(zeta*1i)*2*1i + L8*z*exp(zeta*1i)*2*1i + L4*z*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4^2*exp(zeta*1i)*cos(zeta)^2*1i + L4^2*exp(zeta*1i)*sin(zeta)^2*1i - L4*L7*(exp(zeta*2*1i)/2 + 1/2)*2*1i + L2*L4*(exp(zeta*2*1i)*1i + 1i))/(2*L5*(L4*1i - x*exp(zeta*1i)*1i + z*exp(zeta*1i) - L1*exp(zeta*1i) + L2*exp(zeta*1i)*1i + L6*exp(zeta*1i) - L7*exp(zeta*1i)*1i + L8*exp(zeta*1i))))*1i;
            - zeta - log(-(L1^2*exp(zeta*1i)*1i + L2^2*exp(zeta*1i)*1i - L3^2*exp(zeta*1i)*1i + L5^2*exp(zeta*1i)*1i + L6^2*exp(zeta*1i)*1i + L7^2*exp(zeta*1i)*1i + L8^2*exp(zeta*1i)*1i + x^2*exp(zeta*1i)*1i + z^2*exp(zeta*1i)*1i + (exp(zeta*2*1i)*(L1^2 - 2*sin(zeta)*L1*L4 - 2*L1*L6 - 2*L1*L8 - 2*L1*z + L2^2 + 2*cos(zeta)*L2*L4 - 2*L2*L7 - 2*L2*x - L3^2 + L4^2 + 2*sin(zeta)*L4*L6 - 2*cos(zeta)*L4*L7 + 2*sin(zeta)*L4*L8 - 2*cos(zeta)*L4*x + 2*sin(zeta)*L4*z + L5^2 + L6^2 + 2*L6*L8 + 2*L6*z + L7^2 + 2*L7*x + L8^2 + 2*L8*z + x^2 + z^2)^2 + 4*L5^2*exp(zeta*1i)*(- L4 + x*exp(zeta*1i) + z*exp(zeta*1i)*1i - L1*exp(zeta*1i)*1i - L2*exp(zeta*1i) + L6*exp(zeta*1i)*1i + L7*exp(zeta*1i) + L8*exp(zeta*1i)*1i)*(L2 - L1*1i + L6*1i - L7 + L8*1i - x + z*1i + L4*exp(zeta*1i)))^(1/2)*1i - L4*x*(exp(zeta*2*1i)/2 + 1/2)*2*1i - L1*L6*exp(zeta*1i)*2*1i - L1*L8*exp(zeta*1i)*2*1i - L2*L7*exp(zeta*1i)*2*1i + L6*L8*exp(zeta*1i)*2*1i - L1*L4*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L6*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L8*(cos(2*zeta) + sin(2*zeta)*1i - 1) - L2*x*exp(zeta*1i)*2*1i + L7*x*exp(zeta*1i)*2*1i - L1*z*exp(zeta*1i)*2*1i + L6*z*exp(zeta*1i)*2*1i + L8*z*exp(zeta*1i)*2*1i + L4*z*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4^2*exp(zeta*1i)*cos(zeta)^2*1i + L4^2*exp(zeta*1i)*sin(zeta)^2*1i - L4*L7*(exp(zeta*2*1i)/2 + 1/2)*2*1i + L2*L4*(exp(zeta*2*1i)*1i + 1i))/(2*L5*(L4*1i - x*exp(zeta*1i)*1i + z*exp(zeta*1i) - L1*exp(zeta*1i) + L2*exp(zeta*1i)*1i + L6*exp(zeta*1i) - L7*exp(zeta*1i)*1i + L8*exp(zeta*1i))))*1i];
 
    betaRaw = -[asin((L6 - L1 + L8 + z + L4*sin(zeta) - L5*sin(log(-(L1^2*exp(zeta*1i)*1i + L2^2*exp(zeta*1i)*1i - L3^2*exp(zeta*1i)*1i + L5^2*exp(zeta*1i)*1i + L6^2*exp(zeta*1i)*1i + L7^2*exp(zeta*1i)*1i + L8^2*exp(zeta*1i)*1i + x^2*exp(zeta*1i)*1i + z^2*exp(zeta*1i)*1i - (exp(zeta*2*1i)*(L1^2 - 2*sin(zeta)*L1*L4 - 2*L1*L6 - 2*L1*L8 - 2*L1*z + L2^2 + 2*cos(zeta)*L2*L4 - 2*L2*L7 - 2*L2*x - L3^2 + L4^2 + 2*sin(zeta)*L4*L6 - 2*cos(zeta)*L4*L7 + 2*sin(zeta)*L4*L8 - 2*cos(zeta)*L4*x + 2*sin(zeta)*L4*z + L5^2 + L6^2 + 2*L6*L8 + 2*L6*z + L7^2 + 2*L7*x + L8^2 + 2*L8*z + x^2 + z^2)^2 + 4*L5^2*exp(zeta*1i)*(- L4 + x*exp(zeta*1i) + z*exp(zeta*1i)*1i - L1*exp(zeta*1i)*1i - L2*exp(zeta*1i) + L6*exp(zeta*1i)*1i + L7*exp(zeta*1i) + L8*exp(zeta*1i)*1i)*(L2 - L1*1i + L6*1i - L7 + L8*1i - x + z*1i + L4*exp(zeta*1i)))^(1/2)*1i - L4*x*(exp(zeta*2*1i)/2 + 1/2)*2*1i - L1*L6*exp(zeta*1i)*2*1i - L1*L8*exp(zeta*1i)*2*1i - L2*L7*exp(zeta*1i)*2*1i + L6*L8*exp(zeta*1i)*2*1i - L1*L4*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L6*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L8*(cos(2*zeta) + sin(2*zeta)*1i - 1) - L2*x*exp(zeta*1i)*2*1i + L7*x*exp(zeta*1i)*2*1i - L1*z*exp(zeta*1i)*2*1i + L6*z*exp(zeta*1i)*2*1i + L8*z*exp(zeta*1i)*2*1i + L4*z*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4^2*exp(zeta*1i)*cos(zeta)^2*1i + L4^2*exp(zeta*1i)*sin(zeta)^2*1i - L4*L7*(exp(zeta*2*1i)/2 + 1/2)*2*1i + L2*L4*(exp(zeta*2*1i)*1i + 1i))/(2*L5*(L4*1i - x*exp(zeta*1i)*1i + z*exp(zeta*1i) - L1*exp(zeta*1i) + L2*exp(zeta*1i)*1i + L6*exp(zeta*1i) - L7*exp(zeta*1i)*1i + L8*exp(zeta*1i))))*1i))/L3);
            asin((L6 - L1 + L8 + z + L4*sin(zeta) - L5*sin(log(-(L1^2*exp(zeta*1i)*1i + L2^2*exp(zeta*1i)*1i - L3^2*exp(zeta*1i)*1i + L5^2*exp(zeta*1i)*1i + L6^2*exp(zeta*1i)*1i + L7^2*exp(zeta*1i)*1i + L8^2*exp(zeta*1i)*1i + x^2*exp(zeta*1i)*1i + z^2*exp(zeta*1i)*1i + (exp(zeta*2*1i)*(L1^2 - 2*sin(zeta)*L1*L4 - 2*L1*L6 - 2*L1*L8 - 2*L1*z + L2^2 + 2*cos(zeta)*L2*L4 - 2*L2*L7 - 2*L2*x - L3^2 + L4^2 + 2*sin(zeta)*L4*L6 - 2*cos(zeta)*L4*L7 + 2*sin(zeta)*L4*L8 - 2*cos(zeta)*L4*x + 2*sin(zeta)*L4*z + L5^2 + L6^2 + 2*L6*L8 + 2*L6*z + L7^2 + 2*L7*x + L8^2 + 2*L8*z + x^2 + z^2)^2 + 4*L5^2*exp(zeta*1i)*(- L4 + x*exp(zeta*1i) + z*exp(zeta*1i)*1i - L1*exp(zeta*1i)*1i - L2*exp(zeta*1i) + L6*exp(zeta*1i)*1i + L7*exp(zeta*1i) + L8*exp(zeta*1i)*1i)*(L2 - L1*1i + L6*1i - L7 + L8*1i - x + z*1i + L4*exp(zeta*1i)))^(1/2)*1i - L4*x*(exp(zeta*2*1i)/2 + 1/2)*2*1i - L1*L6*exp(zeta*1i)*2*1i - L1*L8*exp(zeta*1i)*2*1i - L2*L7*exp(zeta*1i)*2*1i + L6*L8*exp(zeta*1i)*2*1i - L1*L4*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L6*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4*L8*(cos(2*zeta) + sin(2*zeta)*1i - 1) - L2*x*exp(zeta*1i)*2*1i + L7*x*exp(zeta*1i)*2*1i - L1*z*exp(zeta*1i)*2*1i + L6*z*exp(zeta*1i)*2*1i + L8*z*exp(zeta*1i)*2*1i + L4*z*(cos(2*zeta) + sin(2*zeta)*1i - 1) + L4^2*exp(zeta*1i)*cos(zeta)^2*1i + L4^2*exp(zeta*1i)*sin(zeta)^2*1i - L4*L7*(exp(zeta*2*1i)/2 + 1/2)*2*1i + L2*L4*(exp(zeta*2*1i)*1i + 1i))/(2*L5*(L4*1i - x*exp(zeta*1i)*1i + z*exp(zeta*1i) - L1*exp(zeta*1i) + L2*exp(zeta*1i)*1i + L6*exp(zeta*1i) - L7*exp(zeta*1i)*1i + L8*exp(zeta*1i))))*1i))/L3)];
    
    alpha = atan2(yC,xC); 

    if imag(alpha(1)) == 0 || imag(betaRaw(1)) == 0 || imag(gammaRaw(1)) == 0 || imag(betaRaw(2)) == 0 || imag(gammaRaw(2)) == 0
        
        %beta = betaRaw(1);
        %gamma = gammaRaw(1);
        
        if alpha >= alphaMin && alpha <= alphaMax && betaRaw(1) >= betaMin && betaRaw(1) <= betaMax && gammaRaw(1) >= gammaMin && gammaRaw(1) <= gammaMax
            alpha = alpha(1);
            beta = real(betaRaw(1));
            gamma = real(gammaRaw(1));
        %elseif alpha >= alphaMin && alpha <= alphaMax && betaRaw(2) >= betaMin && betaRaw(2) <= betaMax && gammaRaw(2) >= gammaMin && gammaRaw(2) <= gammaMax
        else
            alpha = alpha(1);
            beta = real(betaRaw(2));
            gamma = real(gammaRaw(2));
        end
    else
        fprintf('No Real Solution Found');
        alpha = 0;
        beta = 0;
        gamma = 0;
    end
    
end

