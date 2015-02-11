%sherpaTTKinematicConstants.m

B2PXOffset = 0.564;         %[m]
B2PZOffset = -0.133;        %[m]
L1 = 0.175*cosd(35);        %[m]
L2 = 0.045;                 %[m]
L3 = 0.400;                 %[m]
L4 = 0.2204;                %[m]
L5 = 0.4;                   %[m]
%L6 = 0.159*sind(34.5);     %[m]
L6 = 0.0430203;             %[m]
L7 = 0.10784;               %[m]
%L7 = 0.159*cos(0.1305);    %[m]
r = 0.378/2;                %[m]
L8 = 0.2875+r;              %[m]
zeta = 0.1305;              %[rad]
leg1AngleOffset = pi/4;     %[rad]
leg2AngleOffset = -pi/4;    %[rad]
leg3AngleOffset = 3*pi/4;   %[rad]
leg4AngleOffset = -3*pi/4;  %[rad]

kinematicConst = [L1 L2 L3 L4 L5 L6 L7 L8 zeta r B2PXOffset B2PZOffset leg1AngleOffset leg2AngleOffset leg3AngleOffset leg4AngleOffset];