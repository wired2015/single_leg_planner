function [x,y,z] = sherpaTTFKSurf(alpha,beta,gamma,kC,n)
    
    A = kC.l2+kC.l4*cos(kC.zeta)-kC.l7;
    B = kC.l1-kC.l4*sin(kC.zeta)-kC.l6-kC.l8-kC.r;
    x = (kC.l2+kC.l4*cos(kC.zeta)-kC.l7+kC.l3*cos(beta)+kC.l5*cos(kC.zeta+gamma)-kC.l7)'*cos(alpha);
    y = (kC.l2+kC.l4*cos(kC.zeta)-kC.l7+kC.l3*cos(beta)+kC.l5*cos(kC.zeta+gamma)-kC.l7)'*sin(alpha);
    z = (B-kC.l3*sin(beta)-kC.l5*sin(gamma+kC.zeta))'*ones(1,n);

end

