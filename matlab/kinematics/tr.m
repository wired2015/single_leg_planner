%TR Transforms a position vector, u, to a new coordinate frame based on the
%homoegenous transformation matrix T.
%
%Inputs:
%-u: The position vector to be transformed.
%-T: The homogeneous transformation matrix.
%Outputs:
%uNew:  The new position vector.
%thetaNew: The roll, pitch and yaw angles.

function [uNew] = tr(u,T)
    uNew = T(1:3,1:3)*u+T(1:3,4);
    %thetaNew = tr2rpy(T);
end