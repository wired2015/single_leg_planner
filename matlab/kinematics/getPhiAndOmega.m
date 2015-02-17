function [phi,omega] = getPhiAndOmega(uBDot,q,qDot,kC,legNum)

    uG = [0 0 0];

    %Homogeneous transformation matrices.
    [~,TP2B,TI2P,TJ2I,TO2J,TQ2O,TR2Q,TS2R,~,~] = generateTrMatrices(uG,q,kC,legNum);

    TS2B = TP2B*TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R;
    TS2P = TI2P*TJ2I*TO2J*TQ2O*TR2Q*TS2R;
    TS2I = TJ2I*TO2J*TQ2O*TR2Q*TS2R;
    TS2O = TQ2O*TR2Q*TS2R;

    TB2S = trInv(TS2B);
    TP2S = trInv(TS2P);
    TI2S = trInv(TS2I);
    TO2S = trInv(TS2O);

    %Adjunct transformation matrices.
    AdB2S = tr2Adj(TB2S);
    AdP2S = tr2Adj(TP2S);
    AdI2S = tr2Adj(TI2S);
    AdO2S = tr2Adj(TO2S);   

    %Pan joint rate
    alphaDot = qDot(1);         %[rad/s]
    vP = zeros(1,3);            %[m/s]
    omegaP = [0 0  alphaDot];   %[rad/s]
    uPDot = [vP omegaP]';

    %Beta joint rate
    betaDot = qDot(2);          %[rad/s]
    vI = zeros(1,3);            %[m/s]
    omegaI = [0 0  betaDot];    %[rad/s]
    uIDot = [vI omegaI]';

    %Gamma joint rate
    gammaDot = qDot(3);         %[rad/s]
    vO = zeros(1,3);            %[m/s]
    omegaO = [0 0  gammaDot];   %[rad/s]
    uODot = [vO omegaO]';

    %Velocity vector for the ankle frame.
    uSDot = AdB2S*uBDot + AdP2S*uPDot + AdI2S*uIDot + AdO2S*uODot;
    vS = uSDot(1:3);            %[m/s]
    omegaS = uSDot(4:6);        %[rad/s]

    %Calculate the required phi joint angle and the required wheel speed,
    %omega.
    phi = atan2(vS(2),vS(1));   %[rad]
    omega = norm(vS)/kC.r;      %[rad/s]

end
