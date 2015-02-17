function [beta,gamma] = getConstrainedBeta(panHeight,gamma,kC,range,min)
%GETCONSTRAINEDBETA Calculates the beta joint angle given a constrained body
%height.

%     check = (panHeight-kC.l1+kC.l4*sin(kC.zeta)+kC.l5*sin(gamma+kC.zeta)+kC.l7+kC.l8+kC.r)/kC.l3;
% 
%     if check < -1 || check > 1
%         gammaMax = asin(1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta;
%         gammaMin = asin(-1-abs(panHeight)+kC.l1-kC.l4*sin(kC.zeta)-kC.l7-kC.l8+kC.r)-kC.zeta;
%         gamma = gammaMin + rand*(gammaMax-gammaMin);
%     end

    check = (-panHeight+kC.l1-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma+kC.zeta)-kC.l6-kC.l8-kC.r)/kC.l3;
    
    count = 1;
    while check < -1 || check > 1
        gamma = range(2)*rand+min;
        check = (-panHeight+kC.l1-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma+kC.zeta)-kC.l6-kC.l8-kC.r)/kC.l3;
        %disp('Imaginary');
        %disp(count);
        count = count + 1;
    end
    
    beta = asin(check);
    
end