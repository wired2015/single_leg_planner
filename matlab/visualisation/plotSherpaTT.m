%PLOTSHERPATT Plots the kinematic skeleton of the Sherpa_TT platform.
%
%Inputs:
%-uG: The Cartesian position vector of the robot relative to the inertial
%frame.
%-q: A 4x4 array containing joint space angular position values for each 
%leg. Each row has the following structure [alpha_i beta_i gamma_i phi_i],
%where is is the legNum index.
%-kC: A structure containing the kinematic constants of the Sherpa_TT
%platform.
%-init: A logical value indicating whether or not it is the initial call
%to this function.
%
%plotSherpa.m
%author: wreid
%date: 20150217

function plotSherpaTT(uG,q,kC,init)
    
    global fig
    
    if init
        initializeSherpaTTPlot(uG);
        %initPosVec = TP2G(1:3,1:3)*[xInit; yInit; zInit] + TP2G(1:3,4);
        %set(initPosHandle,'XData',initPosVec(1),'YData',initPosVec(2),'ZData',initPosVec(3));
        %goalPosVec = TP2G(1:3,1:3)*[xGoal; yGoal; zGoal] + TP2G(1:3,4);
        %set(goalPosHandle,'XData',goalPosVec(1),'YData',goalPosVec(2),'ZData',goalPosVec(3));
    end
    
    figure(fig)
    hold on
    
    plotBody(uG,kC);
    
    for i = 1:4
        plotIndividualLeg(uG,q(i,:),i,kC)
    end
    
    %TODO
    %set(wheel,'XData',[],'YData',[],'ZData',[]);

    hold off
        
end