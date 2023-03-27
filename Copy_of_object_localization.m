%% Set-up

clc
clear
close all

%% Initialization

% simulation time definition
Dt = 0.1;
t = 0:Dt:100;

% convert degree to radiant
to_rad = pi/180;

% robot 1 initial position
x1 = 0;
y1 = 0;
theta1 = 0;
s0_1 = [x1; y1; theta1];

% robot 2 initial position
x2 = 10;
y2 =  3;
theta2 = 0;
s0_2 = [x2; y2; theta2];

%% Sensors uncertainty

% camera -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0.1;
sigma_camera = 1e-2;

camera1 = 30*to_rad;
camera2 = 40*to_rad;

% radar -> give the position and orientation of the robot2
mu_radar = 0.1;
sigma_radar = 1e-2;

phi2 = 70*to_rad;

%% Robot dynamics

% Vehicle inputs
u = [sin(t/10)*2;
     sin(t/10)*2;
     sin(t/5).*cos(t/4)*1.5];

sStore_1 = zeros(length(s0_1), length(t));
sStore_2 = zeros(length(s0_2), length(t));

sStore_1(:,1) = s0_1;
sStore_2(:,1) = s0_2;

for cT=1:length(t)-1
    % Unicycle dynamic
    sStore_1(:,cT+1) = RobotDynamic(sStore_1(:,cT), u(:,cT), Dt);
    sStore_2(:,cT+1) = RobotDynamic(sStore_2(:,cT), u(:,cT), Dt);
end

%%

figure
hold on
grid on
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');

for i = 1:length(t)
    plot(sStore_1(1,i),sStore_1(2,i),'*');
    plot(sStore_2(1,i),sStore_2(2,i),'*');
    axis equal
    xlim([-200 200])
    ylim([-200 200])
    drawnow
end

%% Solve

A = [-tan(theta1) 1;
     -tan(theta2+phi2) 1];

B = [0;
     -tan(theta2+phi2).*x2 + y2];

[pointA] = linsolve(A,B);

%% Plot

plot_location(0,0,theta1,x2,y2,theta2,phi2,pointA(1),pointA(2))

