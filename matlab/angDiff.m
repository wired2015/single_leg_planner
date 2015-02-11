function d = angDiff(th1, th2)
%angDiff Finds the angular difference between th1 and th2.

    if nargin < 2
        d = th1;
    else
        d = th1 - th2;
    end
    
    d = abs(mod(d+pi, 2*pi) - pi);

end

