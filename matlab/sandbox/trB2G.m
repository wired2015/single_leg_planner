syms r p y
syms xG2B yG2B zG2B
syms xB2P zB2P eta
syms xG yG zG
syms xP yP zP

TB2G = [1 0 0 xG2B;0 1 0 yG2B; 0 0 1 zG2B; 0 0 0 1]*rpy2tr(r,p,y);


%TP2B = trDH(eta,zB2P,xB2P,0);
%TP2G = TB2G*TP2B;
%TG2P = trInv(TP2G);

%uP = TG2P(1:3,1:3)*[xG yG zG]' + TG2P(1:3,4);

