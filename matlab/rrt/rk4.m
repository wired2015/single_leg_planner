%rk4.m
%author: wreid
%date: 20150107

function [xNew,transitionArray] = rk4(uIn,uBDot,dt,Dt,xInit,jointLimits,kC,legNum)
%rk4 Summary of this function goes here
%   Detailed explanation goes here

    u = uIn;

    numIterations = round(Dt/dt);
    xNew = zeros(1,length(xInit));
    xInit = xInit(4:end);
    xInitOrig = xInit;
    transitionArray = zeros(1,(numIterations+1)*10);
    transitionArray(1:10) = xInit;
        
    for i = 1:numIterations

        xNew = numIntegrate(xInit,u,kC,dt);
        
        phiInit = xInit(4);
        
        alpha = xNew(1);
        beta = xNew(2);
        gamma = xNew(3);
        epsilon = xNew(5);

        alphaDot = xNew(6);
        betaDot = xNew(7);
        gammaDot = xNew(8);
        phiDot = xNew(9);
        omega = xNew(10);
        
        alphaDotDot = u(1);
        betaDotDot = u(2);
        
        alphaMax = jointLimits(2,1);
        alphaMin = jointLimits(1,1);
        betaMax = jointLimits(2,2);
        betaMin = jointLimits(1,2);
        gammaMax = jointLimits(2,3);
        gammaMin = jointLimits(1,3);
        phiMax = jointLimits(2,4);
        phiMin = jointLimits(1,4);
        
        alphaDotMax = jointLimits(2,6);
        alphaDotMin = jointLimits(1,6);
        betaDotMax = jointLimits(2,7);
        betaDotMin = jointLimits(1,7);
        gammaDotMax = jointLimits(2,8);
        gammaDotMin = jointLimits(1,8);
        phiDotMax = jointLimits(2,9);
        phiDotMin = jointLimits(1,9);
        omegaMax = jointLimits(2,10);
        omegaMin = jointLimits(1,10);
        
        %Check pan angular position limits
        if alpha > alphaMax || alpha < alphaMin
            alpha = xInit(1);
            alphaDot = 0;
            alphaDotDot = 0;
        end
        
        %Check inner and outer leg angular position limits
        if beta > betaMax || beta < betaMin || gamma > gammaMax || gamma < gammaMin
            beta = xInit(2);
            gamma = xInit(3);
            betaDot = 0;
            gammaDot = 0;
            betaDotDot = 0;
        end
        
        %Check pan angular velocity limits
        if alphaDot > alphaDotMax || alphaDot < alphaDotMin
            alphaDot = xInit(6);
            alphaDotDot = 0;
        %else
        %    alphaDotDot = uIn(1);
        end
        
        %Check the inner leg velocity limit.
        if betaDot > betaDotMax || betaDot < betaDotMin
            betaDot = xInit(7);
            betaDotDot = 0;
            gammaDot = getConstrainedGammaDot(kC,[alphaDot betaDot gammaDot],[alpha beta gamma]);
            gammaDotDot = getConstrainedGammaDotDot(kC,[alphaDotDot betaDotDot 0],[alphaDot betaDot gammaDot],[alpha beta gamma]);
        end
        
        %Check the outer leg velocity limit.
        if gammaDot > gammaDotMax || gammaDot < gammaDotMin
           gammaDot = xInit(8);
           gammaDotDot = 0;
           betaDot = getConstrainedBetaDot(kC,[alphaDot betaDot gammaDot],[alpha beta gamma]);
           betaDotDot = getConstrainedBetaDotDot(kC,[alphaDotDot 0 gammaDotDot],[alphaDot betaDot gammaDot],[alpha beta gamma]);
           if betaDot > betaDotMax || betaDot < betaDotMin
                betaDot = xInit(7);
                betaDotDot = 0;
           end
        end
        
%         if betaDot > betaDotMax || betaDot < betaDotMin || gammaDot > gammaDotMax || gammaDot < gammaDotMin
%             betaDot = xInit(7);
%             gammaDot = xInit(8);
%             betaDotDot = 0;
%         else
%            betaDotDot = uIn(2);
%         end
        
        %Check the outer leg velocity limit.
        
        [phi,omega] = getPhiAndOmega(uBDot,[alphaDot betaDot gammaDot 0],[alpha beta gamma 0],phiInit,kC,legNum);
        epsilon = epsilon + dt*omega;
        
%         %Check if phi is above threshold, if so then stop the leg and drive
%         %the steering joint until it is in the correct orientation.
%         if abs(phi-phiInit)/dt > phiDotMax
%           if phi > phiInit
%             phiDot = phiDotMax;
%           else
%             phiDot = phiDotMin;  
%           end
%           alphaDotDot = 0;
%           betaDotDot = 0;
%           alphaDot = 0;
%           betaDot = 0;
%           gammaDot = 0;
%           omega = 0;
%           alpha = xInitOrig(1);
%           beta = xInitOrig(2);
%           gamma = xInitOrig(3);
%         end
        
        u = [alphaDotDot betaDotDot];
        
        xNew = [alpha beta gamma phi epsilon alphaDot betaDot gammaDot phiDot omega];
        
        xInit = xNew;
        transitionArray((10*i)+1:(10*(i+1))) = xNew;
    end
    xNew = [zeros(1,3) xNew];
     
end

function xDot = f(x,u,kC)
    
    alpha = x(1);
    beta = x(2);
    gamma = x(3);
    
    alphaDot = x(6);
    betaDot = x(7);
    gammaDot = x(8);
    
    alphaDotDot = u(1);
    betaDotDot = u(2);
    %gammaDotDot = (-betaDotDot*kC.l3*cos(beta)+betaDot^2*kC.l3*sin(beta)+gammaDot^2*kC.l5*sin(kC.zeta+gamma))/(kC.l5*cos(kC.zeta+gamma));
    gammaDotDot = getConstrainedGammaDotDot(kC,[alphaDotDot betaDotDot],[alphaDot betaDot gammaDot],[alpha beta gamma]);
    xDot = [alphaDot betaDot gammaDot 0 0 alphaDotDot betaDotDot gammaDotDot 0 0]; 
    
end

function xNew = numIntegrate(xInit,u,kC,dt)
    k1 = f(xInit,u,kC); 
    k2 = f(xInit+dt/2*k1,u,kC);    
    k3 = f(xInit+dt/2*k2,u,kC);
    k4 = f(xInit+dt/2*k3,u,kC);
    xNew = xInit + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
