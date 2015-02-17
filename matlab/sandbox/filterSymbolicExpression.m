%filterSymbolicExpression.m

function fNew = filterSymbolicExpression(f,epsilon)

    [c,t] = coeffs(f);
    c = vpa(c,5);

    for i = 1:length(c)
        if abs(c(i)) < epsilon
            c(i) = 0;
        end
    end

    fNew = c*t';
end