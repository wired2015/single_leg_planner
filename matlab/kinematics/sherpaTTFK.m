%sherpaTTFK.m
%author: wreid
%date: 20150122

function [x,y,z] = sherpaTTFK(alpha,beta,gamma,kinematicConst)
%sherpaTTFK Sherpa_TT Forward Kinematics
%   Calculates the x,y,z position of the contact point given the alpha,
%   beta and gamma joint values.
    
    [L1,L2,L3,L4,L5,L6,L7,L8,zeta,~,~,~,~,~,~,~] = extractKinematicConstants(kinematicConst);

    x = (L2 + L3*cos(-beta)+L4*cos(zeta)+L5*cos(gamma+zeta)-L7).*cos(alpha);
    y = (L2 + L3*cos(-beta)+L4*cos(zeta)+L5*cos(gamma+zeta)-L7).*sin(alpha);
    z = (L1 + L3*sin(-beta)-L4*sin(zeta)-L5*sin(gamma+zeta)-L6-L8);
   
end

