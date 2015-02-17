function uG = getWheelVertices(TW2G,kC)
    
    len = 50;
    theta = linspace(0,2*pi,len);
    uG = zeros(3,len);
    zW = zeros(1,len);
    xW = kC.r*cos(theta);
    yW = kC.r*sin(theta);
    for i = 1:len
        uG(1:3,i) = TW2G(1:3,1:3)*[xW(i); yW(i); zW(i)] + TW2G(1:3,4);
    end
    uG = uG';
    
end

