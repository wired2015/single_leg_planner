%rk4.m
%author: wreid
%date: 20150107

function [xNew,transitionArray] = rk4(u,dt,Dt,xInit,jointLimits)
%rk4 Summary of this function goes here
%   Detailed explanation goes here

    numIterations = round(Dt/dt);
    xNew = zeros(1,length(xInit));
    xInit = xInit(4:end-2);
    %xInitOrig = xInit;
    transitionArray = zeros(1,(numIterations+1)*6);
    transitionArray(1:6) = xInit;
        
    for i = 1:numIterations
        
        k1 = f(xInit,u); 
        k2 = f(xInit+dt/2*k1,u);    
        k3 = f(xInit+dt/2*k2,u);
        k4 = f(xInit+dt/2*k3,u);
        xNew = xInit + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        
        %Check pan angular position limits
        if xNew(1) > jointLimits(2,1) || xNew(1) < jointLimits(1,1)
            xNew(1) = xInit(1);
            xNew(4) = 0;
            u(1) = 0;
        end
        
         %Check inner and outer leg angular position limits
        if xNew(2) > jointLimits(2,2) || xNew(2) < jointLimits(1,2) || xNew(3) > jointLimits(2,3) || xNew(3) < jointLimits(1,3)
            xNew(2) = xInit(2);
            xNew(3) = xInit(3);
            xNew(5) = 0;
            xNew(6) = 0;
            u(2) = 0;
            u(3) = 0;
        end
        
        %Check pan angular velocity limits
        if xNew(4) > jointLimits(2,4) || xNew(4) < jointLimits(1,4)
            xNew(4) = xInit(4);
            u(1) = 0;
        end
        
        %Check inner and outer leg angular velocity limits
        if xNew(5) > jointLimits(2,5) || xNew(5) < jointLimits(1,5) || xNew(6) > jointLimits(2,6) || xNew(6) < jointLimits(1,6)
            xNew(5) = xInit(5);
            xNew(6) = xInit(6);
            u(2) = 0;
            u(3) = 0;
        end
        
        xInit = xNew;
        transitionArray((6*i)+1:(6*(i+1))) = xNew;
    end
    
    %xInit = [zeros(1,3) xInitOrig 0 0];
    xNew = [zeros(1,3) xNew 0 0];
     
end

function xDot = f(x,u)
    xDot = [x(4:6) u(1:3)];
end


