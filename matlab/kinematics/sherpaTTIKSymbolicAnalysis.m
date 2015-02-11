%sherpaTTIKSymbolicAnalysis
%This file uses the symbolic toolbox to find the inverse kinematics of the
%sherpaTT leg.

clear

syms gamma beta x z L1 L2 L3 L4 L5 L6 L7 L8 zeta
A = L2+L4*cos(zeta)-L7;      %[m]
B = L1-L4*sin(zeta)-L6-L8;   %[m]
Sgamma = solve(x^2-2*x*A+A^2-L3^2+z^2-2*z*B+B^2+L5^2 == cos(zeta+gamma)*(2*x*L5-2*A*L5) + sin(zeta+gamma)*(-2*z*L5+2*B*L5),gamma)
Sbeta = -asin((z-B+L5*sin(zeta+Sgamma))./L3)