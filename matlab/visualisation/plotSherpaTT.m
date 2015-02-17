function plotSherpaTT(uG,q,kC,init,showFrames,showPath)
    
    global fig
    
    if init
        initializeSherpaTTPlot(showFrames);
        %initPosVec = TP2G(1:3,1:3)*[xInit; yInit; zInit] + TP2G(1:3,4);
        %set(initPosHandle,'XData',initPosVec(1),'YData',initPosVec(2),'ZData',initPosVec(3));
        %goalPosVec = TP2G(1:3,1:3)*[xGoal; yGoal; zGoal] + TP2G(1:3,4);
        %set(goalPosHandle,'XData',goalPosVec(1),'YData',goalPosVec(2),'ZData',goalPosVec(3));
    end
    
    figure(fig)
    hold on
    
    plotBody(uG,kC);
    
    for i = 1:4
        plotIndividualLeg(uG,q(i,:),i,kC,showPath)
    end
    
    %TODO
    %set(wheel,'XData',[],'YData',[],'ZData',[]);

    hold off
        
end

%getBodyVertices.m