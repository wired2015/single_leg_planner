%getTB2H.m

function TH2B = getTH2B(bodyW,bodyH)

    d = sqrt((bodyW/2)^2+(bodyH/2)^2);

    TH2B = [cos(pi/4) sin(pi/4) 0 d*cos(pi/4);...
            -sin(pi/4) cos(pi/4) 0 d*sin(pi/4);...
            0 0 1 0;...
            0 0 0 1]; 

end