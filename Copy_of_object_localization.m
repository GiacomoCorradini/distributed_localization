%% Set-up

clc
clear
close all

% random seed
rng(1)

% simulation time definition
Dt = 0.1;
t = 0:Dt:10;

% convert degree to radiant
to_rad = pi/180;

%% Initialization

% robot 1 initial position
x1 = randi([-50 50]);              % x coordinate
y1 = randi([-50 50]);              % y coordinate
theta1 = randi([-180 180])*to_rad; % theta coordinate
s0_1 = [x1; y1; theta1];           % state of robot 1

% robot 2 initial position
x2 = randi([-50 50]);              % x coordinate
y2 =  randi([-50 50]);             % y coordinate
theta2 = randi([-180 180])*to_rad; % theta coordinate1
s0_2 = [x2; y2; theta2];           % state of robot 2

% camera sensors initial value
camera1 = randi([-180 180])*to_rad;    % measured camera angle robot 1
camera2 = -randi([-180 180])*to_rad;   % measured camera angle robot 2

%% real position of the object without uncertainty

phi2 = theta2 - theta1;
[xA, yA, x1A, y1A] = object_detection(x1,y1,theta1,x2,y2,phi2,camera1,camera2);
%plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)

%% Robot dynamics

% Vehicle inputs
u_velocity = [sin(t/10)*2;
              sin(t/10)*2;
              sin(t/5).*cos(t/4)*1.5];

% initialize array to store the value 
state_r1      = zeros(length(s0_1), length(t));
state_r2      = zeros(length(s0_2), length(t));
obj_g         = zeros(2,length(t));
obj_robot1    = zeros(2,length(t));
camera_store  = zeros(2,length(t));

% store the initial value
state_r1(:,1) = s0_1;
state_r2(:,1) = s0_2;
camera_store(:,1) = [camera1; camera2];

for cT=1:length(t)-1

    phi2 = state_r2(3,cT) - state_r1(3,cT);
    [obj_g(1,cT),obj_g(2,cT),obj_robot1(1,cT),obj_robot1(2,cT)] = ...
        object_detection(state_r1(1,cT),state_r1(2,cT),state_r1(3,cT),state_r2(1,cT),state_r2(2,cT),...
        phi2,camera_store(1,cT),camera_store(2,cT));
    
    % Robot dynamic
    state_r1(:,cT+1) = RobotDynamic(state_r1(:,cT), u_velocity(:,cT), Dt);
    state_r2(:,cT+1) = RobotDynamic(state_r2(:,cT), u_velocity(:,cT), Dt);

    % Camera dynamic
    camera_store(1,cT+1) = atan2(obj_g(2,1)-state_r1(2,cT+1),obj_g(1,1)-state_r1(1,cT+1)) - state_r1(3,cT+1);

    camera_store(2,cT+1) = atan2(obj_g(2,1)-state_r2(2,cT+1),obj_g(1,1)-state_r2(1,cT+1)) - state_r2(3,cT+1);
end

figure('Name','Object A (ground)', 'NumberTitle','off'), hold on, axis equal;
title('Object A w.r.t to ground')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_g(1,cT),obj_g(2,cT),'o')
end

figure('Name','Object A (Robot 1)', 'NumberTitle','off'), hold on, axis equal;
title('Object A w.r.t to robot 1')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_robot1(1,cT),obj_robot1(2,cT),'o')
end


%% Sensors uncertainty

% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object

mu_camera = 0.1;
sigma_camera = 1e-2;

camera1_est = camera1*sigma_camera + mu_camera;
camera2_est = camera2*sigma_camera + mu_camera;

% GPS -> give the position and orientation of the robot2
mu_radar = 0.1;
sigma_radar = 1e-2;

x2 = x2*sigma_radar + mu_radar;
y2 = y2*sigma_radar + mu_radar;
theta1 = theta1 + sigma_radar + mu_radar;

x2 = x2*sigma_radar + mu_radar;
y2 = y2*sigma_radar + mu_radar;
theta2 = theta2*sigma_radar + mu_radar;

%% Robot dynamics

% !! interessante mettere rumore nell'input

% Vehicle inputs
u = [sin(t/10)*2;
     sin(t/10)*2;
     sin(t/5).*cos(t/4)*1.5];

sStore_1 = zeros(length(s0_1), length(t));
sStore_2 = zeros(length(s0_2), length(t));
pointA = zeros(2,length(t));

sStore_1(:,1) = s0_1;
sStore_2(:,1) = s0_2;

% 1) we need to update the value of camera sensors
% 2) need to recalculate pointA
% 3) Kalman filter

for cT=1:length(t)-1
    % Unicycle dynamic
    sStore_1(:,cT+1) = RobotDynamic(sStore_1(:,cT), u(:,cT), Dt);
    sStore_2(:,cT+1) = RobotDynamic(sStore_2(:,cT), u(:,cT), Dt);
    %phi2 = theta1 - theta2;
    %[pointA(1), pointA(2)] = object_detection(x1,y1,theta1,x2,y2,theta2,phi2);
end

%% PLots

figure('Name','Robots positions', 'NumberTitle','off')
hold on
grid on
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');

plot(xA,yA, 'o', 'Color','Red')
for i = 1:length(t)
    plot(state_r1(1,i),state_r1(2,i),'*', 'Color','#0072BD');
    plot(state_r2(1,i),state_r2(2,i),'*','Color',"#D95319");
    axis equal

   % plot_location(state_r1(1,i),state_r1(2,i),state_r1(3,i),state_r2(1,i),state_r2(2,i),phi2 = state_r2(3,cT) - state_r1(3,cT),obj_g(1,i),obj_g(1,i),camera_store(1,i),camera_store(2,i))
    xlim([-200 200])
    ylim([-200 200])
    drawnow
end
