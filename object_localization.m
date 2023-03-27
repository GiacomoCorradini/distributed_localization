%%
clc
clear
close all

%%

x2 = 10;
y2 =  3;
phi2 = to_rad(70);
theta1 = to_rad(30);
theta2 = to_rad(40);
x = 0:20;

rand

figure
hold on
axis equal

% vehicle 1
plot(0,0,'o')
plot(x,tan(theta1).*x)
plot([0 1],[0 0],'->',Color='b')
plot([0 0],[0 1],'->',Color='b')

% vehicle 2
plot(x2,y2,'o')
plot(x,tan(phi2+theta2).*x-tan(phi2+theta2).*x2 + y2)
plot([x2 x2+cos(phi2).*1],[y2 y2+sin(phi2).*1],'->',Color='r')
plot([x2 x2-sin(phi2).*1],[y2 y2+cos(phi2).*1],'->',Color='b')

% object
syms xA yA;


A = [-tan(theta1) 1;
     -tan(theta2+phi2) 1];

B = [0;
     -tan(theta2+phi2).*x2 + y2];R = [0 -1;
     1 0];


A\B

[pointA] = linsolve(A,B)
plot(pointA(1),pointA(2),'o')

function x = to_rad(y)
    x = y*pi/180;
end
