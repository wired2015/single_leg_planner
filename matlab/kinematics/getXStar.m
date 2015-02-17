%GETXSTAR Returns the xStar value given either the height of the wheel contact
%point relative to the pan coordinate frame and either the beta or gamma
%joint value. It is assumed that the angle input represents the beta joint
%angle if selector = false, and the angle input is the gamma joint angle if
%selector = true
%
%Inputs:
%-z: The height from the pan coordinate frame to the wheel contact point.
%-angle: The angular value that is either beta or gamma, depending on the
%selector input.
%-selector: A logical that indicates that the angle value represents beta
%-kC: A struct containing the kinematic constants of the Sherpa TT leg.
%if selector=true, or gamma if selector=false.
%Outputs:
%-xStar: The radius in a cylindrical coordinate representation that
%connects the pan coordinate frame to the wheel contact coordinate frame.
%
%sherpaTTFK.m
%author: wreid
%date: 20150216

function xStar = getXStar(z,angle,selector,kC)

    A = kC.l2+kC.l4*cos(kC.zeta)-kC.l7;
    B = kC.l1-kC.l4*sin(kC.zeta)-kC.l6-kC.l8-kC.r;

    if ~selector
        xStar = A+kC.l3*cos(asin((-z+B-kC.l5*sin(kC.zeta+angle))/kC.l3))+kC.l5*cos(kC.zeta+angle);
    else
        xStar = A+kC.l3*cos(angle)+kC.l5*cos(asin((B-kC.l3*sin(angle)-z)/kC.l5));
    end

end

