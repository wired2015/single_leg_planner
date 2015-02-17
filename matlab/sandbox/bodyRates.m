function bodyRates()

    y = 0;
    uGOffset = [1 1];                  %[m]

    TB2G = [cos(y) -sin(y) uGOffset(1);
            sin(y) cos(y)  uGOffset(2);
            0      0       1];
    TG2B = inv(TB2G);

    bodyW = 1;
    bodyL = 1;
    
    verticesB = [0.5 -0.5 -0.5 0.5;0.5 0.5 -0.5 -0.5];
    verticesG = zeros(2,4);

    for i = 1:length(verticesB)
        verticesG(:,i) = tr(verticesB(:,i),TB2G);
    end
    
    hold on
    axis([-15 15 -15 15])
    axis square
    fill(verticesG(1,:),verticesG(2,:),'r')

    gH = trplot2(eye(3,3),'frame','G');
    bH = trplot2(TB2G,'frame','B');
    
    vB = [1 1 0];                          %[m/s]
    omegaB = [0 0 0.1];
    omega = [0 0 0];                        %[rad/s] 
    
    if omegaB == [0 0 0]
        thetaSB = atan2(vB(2),vB(1)) + pi/2;
        sBMag = 1000;
        sB = [sBMag*cos(thetaSB) sBMag*sin(thetaSB) 0];
    elseif vB == [0 0 0]
        sB = [0 0 0];
    else
        sBMag = norm(vB)/norm(omegaB);
        thetaSB = pi/2 + atan2(vB(2),vB(1));
        sB = [sBMag*cos(thetaSB) sBMag*sin(thetaSB) 0];
    end
    
    sBG = tr(sB(1:2)',TB2G);
    
    sH1 = plot(sBG(1),sBG(2),'r*'); 
    sH2 = plot(sBG(1),sBG(2),'bo');
    
    
end

function uNew = tr(u,T)
    uNew = T(1:2,1:2)*u+T(1:2,3);
end