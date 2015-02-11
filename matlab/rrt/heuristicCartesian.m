%heuristicCartesian.m
%author: wreid
%date: 20150107

function d = heuristicCartesian(x1,x2,HGAINS,range)
%heuristic Calculates the distance between states x1 and x2.
    
    distMAX = [sqrt(range(1)^2+range(2)^2) sqrt(range(3)^2+range(4)^2)];

    d = HGAINS(1)*cartDist(x1(4:5),x2(4:5))/distMAX(1) +...
        HGAINS(2)*cartDist(x1(6:7),x2(6:7))/distMAX(2); 
    
    
    %HGAINS(1)*sqrt((x1(4)-x2(4))^2+(x1(5)-x2(5))^2)/distMAX(1) +...
    %    HGAINS(2)*sqrt((x1(6)-x2(6))^2+(x1(7)-x2(7))^2)/distMAX(2);    
end
