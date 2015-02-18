%INITIALIZESHERPATTPLOT Initializes a figure that will be used to plot the
%Sherpa_TT kinematic skeleton.
%
%plotIndividualLeg.m
%author:    wreid
%date:      20150214

function initializeSherpaTTPlot()

    global body fig wheel
    global initPosHandle goalPosHandle
    global trMats trBody trGround
    global legLinks
    
    trMats = gobjects(4,8);
    legLinks = gobjects(4,8);
    wheel = gobjects(1,4);
    
    fig = figure('Name','Sherpa_TT Visualization','Position',[100 100 700 700]);
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
    axis equal
    xlabel('X_G [m]');
    ylabel('Y_G [m]');
    zlabel('Z_G [m]');
    %view(136,40);
    view(86,-90);

    hold on
    
    %Plot the initial and final position.
    initPosHandle = plot3(0,0,0,'g*');
    goalPosHandle = plot3(0,0,0,'r*');

    body = fill3(0,0,0,'r');

    axesLen = 0.1;

    trGround = trplot(eye(4,4),'length',axesLen,'frame','G','rgb');
    trBody = trplot(eye(4,4),'length',axesLen,'frame','B','rgb');
    
    for i = 1:4
        
        wheel(i) = fill3(0,0,0,'k');
        
        for j = 1:8
            legLinks(i,j) = line([0 0],[0 0],'LineWidth',3,'Color','k');
        end

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

