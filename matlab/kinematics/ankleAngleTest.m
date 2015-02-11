%ankleAngleTest.m
%author: wreid
%date: 20150112

qUDot = 0.1;
qHDot = 0.7;
qAOld = 0;

qU = linspace(deg2rad(-20),deg2rad(70),100);
qA = zeros(1,length(qU));

x = zeros(length(qU),7);
x(:,5) = qU;
x(:,7) = qUDot*ones(length(qU),1);
x(:,6) = qHDot*ones(length(qU),1);

for i = 1:length(qU)
    qA(i) = ankleAngle(x(i,:),qAOld,kinematicConst);
    disp(qA(i))
    if qA(i) ~= 0
        qAOld = qA(i);
    else
        disp 'hello';
    end
end

figure(1)

plot(qU,qA,'.b');
xlabel('qU');
ylabel('qA');
