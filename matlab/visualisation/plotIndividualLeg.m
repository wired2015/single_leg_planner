%PLOTINDIVIDUALLEG Plots the kinematic skeleton of an individual Sherpa_TT
%leg.
%
%Inputs:
%-uG: The 1x3 vector that describes the displacement between the Body and
%Ground coordiante frames, relative to the ground coordiante frame.
%-q: The 1x4 vector containing the joint state of the leg.
%-legNum: The leg number identifier.
%-kC: A struct containing the kinematic constantants of the leg.
%-showPath: A logical value that plots the wheel contact point each time
%this function is called so that the path that the leg follows is shown.
%
%Notes:
%-initializeSherpaTTPlot must be run before this function is called.
%
%plotIndividualLeg.m
%author:    wreid
%date:      20150214

function plotIndividualLeg(uG,q,legNum,kC,showPath)
    
    global trMats trBody legLinks wheel

    [TB2G,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,TW2S,TC2W] = generateTrMatrices(uG,q,kC,legNum);

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
   
    set(legLinks(legNum,1),'XData',[TP2G(1,4) TI2G(1,4)],'YData',[TP2G(2,4) TI2G(2,4)],'ZData',[TP2G(3,4) TI2G(3,4)],'Color','k');
    set(legLinks(legNum,2),'XData',[TI2G(1,4) TJ2G(1,4)],'YData',[TI2G(2,4) TJ2G(2,4)],'ZData',[TI2G(3,4) TJ2G(3,4)],'Color','k');
    set(legLinks(legNum,3),'XData',[TJ2G(1,4) TO2G(1,4)],'YData',[TJ2G(2,4) TO2G(2,4)],'ZData',[TJ2G(3,4) TO2G(3,4)],'Color','k');
    set(legLinks(legNum,4),'XData',[TO2G(1,4) TQ2G(1,4)],'YData',[TO2G(2,4) TQ2G(2,4)],'ZData',[TO2G(3,4) TQ2G(3,4)],'Color','k');
    set(legLinks(legNum,5),'XData',[TQ2G(1,4) TR2G(1,4)],'YData',[TQ2G(2,4) TR2G(2,4)],'ZData',[TQ2G(3,4) TR2G(3,4)],'Color','k');
    set(legLinks(legNum,6),'XData',[TR2G(1,4) TS2G(1,4)],'YData',[TR2G(2,4) TS2G(2,4)],'ZData',[TR2G(3,4) TS2G(3,4)],'Color','k');
    set(legLinks(legNum,7),'XData',[TS2G(1,4) TW2G(1,4)],'YData',[TS2G(2,4) TW2G(2,4)],'ZData',[TS2G(3,4) TW2G(3,4)],'Color','k');
    set(legLinks(legNum,8),'XData',[TW2G(1,4) TC2G(1,4)],'YData',[TW2G(2,4) TC2G(2,4)],'ZData',[TW2G(3,4) TC2G(3,4)],'Color','k');

    uG = getWheelVertices(TW2G,kC);
    set(wheel(legNum),'Vertices',uG,'Faces',1:length(uG));

    if showPath
        plot3(TC2G(1,4),TC2G(2,4),TC2G(3,4),'b.')
    end
    
end

