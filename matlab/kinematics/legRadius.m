function xStar = legRadius(beta,gamma,kC)
    xStar = kC.l2+kC.l3*cos(beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(kC.zeta+gamma)-kC.l7;
end