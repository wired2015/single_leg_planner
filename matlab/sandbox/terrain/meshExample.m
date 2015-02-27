[X,Y] = meshgrid(-2:0.1:2,-2:0.1:2);
k = 1;
Z = k*X.*exp(-X.^2 - Y.^2);

figure
hold on
h = mesh(X,Y,Z,'FaceAlpha',1,'LineWidth',1);
axis equal

xq = 0.15;
yq = 0.15;
zq = interp2(X,Y,Z,xq,xq)

%plot3(xq,yq,zq,'r*');

N = 100;
xLine = linspace(-2,2,N);
yLine = linspace(-2,2,N);
zLine = 0.1*ones(1,N);

line([xLine(1) xLine(end)],[yLine(1) yLine(end)],[zLine(1) zLine(end)],'Color','b')

zProf = interp2(X,Y,Z,xLine,yLine);

plot3(xLine,yLine,zProf,'k')

%[xInt,yInt] = intersections()
hold off
