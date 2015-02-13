%sherpaTTFK.m
%author: wreid
%date: 20150122

function u = sherpaTTFK(q,kC)
%sherpaTTFK Sherpa_TT Forward Kinematics
%   Calculates the x,y,z position of the contact point given the alpha,
%   beta and gamma joint values.

    alpha = q(1);
    beta = q(2);
    gamma = q(3);
    
    u = zeros(1,3);

    u(1) = (kC.l2 + kC.l3*cos(-beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(gamma+kC.zeta)-kC.l7).*cos(alpha);
    u(2) = (kC.l2 + kC.l3*cos(-beta)+kC.l4*cos(kC.zeta)+kC.l5*cos(gamma+kC.zeta)-kC.l7).*sin(alpha);
    u(3) = (kC.l1 + kC.l3*sin(-beta)-kC.l4*sin(kC.zeta)-kC.l5*sin(gamma+kC.zeta)-kC.l6-(kC.l8+kC.r));
   
end

