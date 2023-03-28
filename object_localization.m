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
theta1 = 0*to_rad;
s0_1 = [x1; y1; theta1];

% robot 2 initial position
x2 = rand(1)*10;
y2 =  rand(1)*10;
theta2 = randi([-90 90])*to_rad;
s0_2 = [x2; y2; theta2];

% camera initial value
camera1 = rand(1)*90*to_rad;
camera2 = -rand(1)*90*to_rad;

% real value of the object without uncertainty
phi2 = theta2 - theta1;
[xA, yA, x1A, y1A] = object_detection(x1,y1,theta1,x2,y2,phi2,camera1,camera2);
plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)

%% Sensors uncertainty

% camera -> give the angle between the forward axis and the line passing
% through the center of the robot and the object

mu_camera = 0.1;
sigma_camera = 1e-2;

camera1 = camera1*sigma_camera + mu_camera;
camera2 = camera2*sigma_camera + mu_camera;

% radar -> give the position and orientation of the robot2
mu_radar = 0.1;
sigma_radar = 1e-2;

%phi2 = 70*to_rad;

x2 = x2*sigma_radar + mu_radar;
y2 = y2*sigma_radar + mu_radar;
theta2 = theta2*sigma_radar + mu_radar;

%% Robot dynamics

% Vehicle inputs
u = [sin(t/10)*2;
     sin(t/10)*2;
     sin(t/5).*cos(t/4)*1.5];

sStore_1 = zeros(length(s0_1), length(t));
sStore_2 = zeros(length(s0_2), length(t));
pointA = zeros(2,length(t));

sStore_1(:,1) = s0_1;
sStore_2(:,1) = s0_2;

for cT=1:length(t)-1
    % Unicycle dynamic
    sStore_1(:,cT+1) = RobotDynamic(sStore_1(:,cT), u(:,cT), Dt);
    sStore_2(:,cT+1) = RobotDynamic(sStore_2(:,cT), u(:,cT), Dt);
    phi2 = theta1 - theta2;
    [pointA(1), pointA(2)] = object_detection(x1,y1,theta1,x2,y2,theta2,phi2);
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



%% Plot

plot_location(y1,x1,theta1,x2,y2,theta2,phi2,xA,yA)

