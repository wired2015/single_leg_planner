%CALCICR Calculates the ICR relative to the body coordinate frame give the
%body velocity vector, v, and the body angular rate vector, omega.
%
%Inputs:
%-v: A 1x3 vector describing the Cartesian [xDot yDot zDot] velocities
%relative to the inertial frame.
%-omega: A 1x3 vector descring the angular rates of the platform relative
%to the inertial frame.
%
%Outputs:
%-sB: The 1x3 ICR Cartesian vector describing the position of the ICR
%relative to the body frame.
%
%calcICR.m
%author: wreid
%date:   20150217
%
function sB = calcICR(v,omega)
    
    %If the angular rate is zero, then the ICR is located at infinite 
    %distance from the body coordinate frame, perpendicular to the body
    %rate vector direction. A "large" number is used here instead of
    %infinity given that it has the same effect.
    if omega == [0 0 0]
        thetaSB = atan2(v(2),v(1)) + pi/2;
        sBMag = 1e4;
        sB = [sBMag*cos(thetaSB) sBMag*sin(thetaSB) 0];
    %If the Cartesian velocity of the platform is zero the the ICR is
    %located at the body coordinate frame and only the angular velocity
    %contributes to the motion of the platform.
    elseif v == [0 0 0]
        sB = [0 0 0];
    %Otherwise the magnitude of the ICR vector is equal to the quotient of
    %the magnitudes of the velocity vector and the angular rate vector. The
    %orientation of the ICR vector is perpendicular to the the body rate
    %vector.
    else
        sBMag = norm(v)/norm(omega);
        thetaSB = pi/2 + atan2(v(2),v(1));
        sB = [sBMag*cos(thetaSB) sBMag*sin(thetaSB) 0];
    end

end