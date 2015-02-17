%TRDH Generates the homogeneous transformation matrix A using the 
%Denavit-Hartenberg parameters theta, d, a and alpha.
%
%trDH.m
%author:    wreid
%date:      20150214

function A = trDH(theta,d,a,alpha)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);...
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
        0 sin(alpha) cos(alpha) d;...
        0 0 0 1];
end

