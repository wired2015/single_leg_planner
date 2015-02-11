function TInv = invHomoMatrix(T)

    R = T(1:3,1:3);
    d = T(1:3,4);
    
    TInv = [R' -R'*d;
            zeros(1,3) 1];

end