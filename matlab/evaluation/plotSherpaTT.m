function plotSherpaTT(uG,q,kinematicConst,init,showFrames,showPath)
    
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
    
    plotBody(uG,kinematicConst);
    
    for i = 1:4
        plotIndividualLeg(uG,q(i,:),i,kinematicConst)
    end
    
    %TODO
    %set(wheel,'XData',[],'YData',[],'ZData',[]);

    hold off
        
end

function plotBody(uG,kinematicConst)
    
    global body

    uBodyVertices = zeros(4,3);
    
    for i = 1:4
        [TB2G,TP2B,~,~,~,~,~,~,~,~] = generateTrMatrices(uG,[0 0 0 0],kinematicConst,i);
        TP2G = TB2G*TP2B;
        uBodyVertices(i,:)  = TP2G(1:3,4);
    end
    
    set(body,'Vertices',uBodyVertices,'Faces',[1 3 4 2]);
    
end

function plotIndividualLeg(uG,q,legNum,kinematicConst)
    
    global trMats trBody legLinks wheel

    [TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices(uG,q,kinematicConst,legNum);

    %TODO
    %[xWheelVertices,yWheelVertices,zWheelVertices] = getWheelVertices(r);

    TP2G = TB2G*TP2B;
    TI2G = TP2G*TI2P;
    TJ2G = TI2G*TJ2I;
    TO2G = TJ2G*TO2J;
    TQ2G = TO2G*TQ2O;
    TR2G = TQ2G*TR2Q;
    TS2G = TR2G*TS2R;
    TW2G = TS2G*TW2S;
    TC2G = TW2G*TC2W;
    
    set(trBody,'Matrix',TB2G);

    set(trMats(legNum,1),'Matrix',TP2G);
    set(trMats(legNum,2),'Matrix',TI2G);
    set(trMats(legNum,3),'Matrix',TJ2G); 
    set(trMats(legNum,4),'Matrix',TO2G); 
    set(trMats(legNum,5),'Matrix',TQ2G);
    set(trMats(legNum,6),'Matrix',TS2G);
    set(trMats(legNum,7),'Matrix',TW2G);
    set(trMats(legNum,8),'Matrix',TC2G);
    
    
    plot3(TC2G(1,4),TC2G(2,4),TC2G(3,4),'b.') 
   
    set(legLinks(legNum,1),'XData',[TP2G(1,4) TI2G(1,4)],'YData',[TP2G(2,4) TI2G(2,4)],'ZData',[TP2G(3,4) TI2G(3,4)],'Color','k');
    set(legLinks(legNum,2),'XData',[TI2G(1,4) TJ2G(1,4)],'YData',[TI2G(2,4) TJ2G(2,4)],'ZData',[TI2G(3,4) TJ2G(3,4)],'Color','k');
    set(legLinks(legNum,3),'XData',[TJ2G(1,4) TO2G(1,4)],'YData',[TJ2G(2,4) TO2G(2,4)],'ZData',[TJ2G(3,4) TO2G(3,4)],'Color','k');
    set(legLinks(legNum,4),'XData',[TO2G(1,4) TQ2G(1,4)],'YData',[TO2G(2,4) TQ2G(2,4)],'ZData',[TO2G(3,4) TQ2G(3,4)],'Color','k');
    set(legLinks(legNum,5),'XData',[TQ2G(1,4) TR2G(1,4)],'YData',[TQ2G(2,4) TR2G(2,4)],'ZData',[TQ2G(3,4) TR2G(3,4)],'Color','k');
    set(legLinks(legNum,6),'XData',[TR2G(1,4) TS2G(1,4)],'YData',[TR2G(2,4) TS2G(2,4)],'ZData',[TR2G(3,4) TS2G(3,4)],'Color','k');
    set(legLinks(legNum,7),'XData',[TS2G(1,4) TW2G(1,4)],'YData',[TS2G(2,4) TW2G(2,4)],'ZData',[TS2G(3,4) TW2G(3,4)],'Color','k');
    set(legLinks(legNum,8),'XData',[TW2G(1,4) TC2G(1,4)],'YData',[TW2G(2,4) TC2G(2,4)],'ZData',[TW2G(3,4) TC2G(3,4)],'Color','k');

    uG = getWheelVertices(TW2G,kinematicConst);
    set(wheel(legNum),'Vertices',uG,'Faces',1:length(uG));
    
end

%getBodyVertices.m

function initializeSherpaTTPlot(showFrames)

    global body fig wheel
    global initPosHandle goalPosHandle
    global trMats trBody trGround
    global legLinks
    
    trMats = gobjects(4,8);
    legLinks = gobjects(4,8);
    wheel = gobjects(1,4);
    
    fig = figure('Name','Sherpa_TT Visualization','Position',[100 100 700 700]);
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
    xlabel('X_G [m]');
    ylabel('Y_G [m]');
    zlabel('Z_G [m]');
    view(136,40);

    hold on

    %Plot the initial and final position.
    initPosHandle = plot3(0,0,0,'g*');
    goalPosHandle = plot3(0,0,0,'r*');

    body = fill3(0,0,0,'r');

    axesLen = 0.1;

    
    if showFrames
        trGround = trplot(eye(4,4),'length',axesLen,'frame','G','rgb');
        trBody = trplot(eye(4,4),'length',axesLen,'frame','B','rgb');
    end
    
    for i = 1:4
        
        wheel(i) = fill3(0,0,0,'k');
        
        for j = 1:8
            legLinks(i,j) = line([0 0],[0 0],'LineWidth',3,'Color','k');
        end
        
        if showFrames
            trMats(i,1) = trplot(eye(4,4),'length',axesLen,'frame',['P_' num2str(i)],'rgb');
            trMats(i,2) = trplot(eye(4,4),'length',axesLen,'frame',['I_' num2str(i)],'rgb');
            trMats(i,3) = trplot(eye(4,4),'length',axesLen,'frame',['J_' num2str(i)],'rgb');
            trMats(i,4) = trplot(eye(4,4),'length',axesLen,'frame',['O_' num2str(i)],'rgb');
            trMats(i,5) = trplot(eye(4,4),'length',axesLen,'frame',['Q_' num2str(i)],'rgb');
            trMats(i,6) = trplot(eye(4,4),'length',axesLen,'frame',['S_' num2str(i)],'rgb');
            trMats(i,7) = trplot(eye(4,4),'length',axesLen,'frame',['W_' num2str(i)],'rgb');
            trMats(i,8) = trplot(eye(4,4),'length',axesLen,'frame',['C_' num2str(i)],'rgb');
        end
    end

    hold off

end

function uG = getWheelVertices(TW2G,kinematicConst)
    
    [~,~,~,~,~,~,~,~,~,r,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);
    len = 50;
    theta = linspace(0,2*pi,len);
    uG = zeros(3,len);
    xW = zeros(1,len);
    yW = r*cos(theta);
    zW = r*sin(theta);
    for i = 1:len
        uG(1:3,i) = TW2G(1:3,1:3)*[xW(i); yW(i); zW(i)] + TW2G(1:3,4);
    end
    uG = uG';
end