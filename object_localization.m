%% Set-up

clc
clear
close all

% random seed
rng(1)

% simulation time definition
Dt = 0.1;
t = 0:Dt:20;

% convert degree to radiant
to_rad = pi/180;

%% Initialization

% robot 1 initial position
x1 = randi([-20 20]);              % x coordinate
y1 = randi([-20 20]);              % y coordinate
theta1 = randi([-180 180])*to_rad; % theta coordinate
s0_1 = [x1; y1; theta1];           % state of robot 1

% robot 2 initial position
x2 = randi([-20 20]);              % x coordinate
y2 =  randi([-20 20]);             % y coordinate
theta2 = randi([-180 180])*to_rad; % theta coordinate1
s0_2 = [x2; y2; theta2];           % state of robot 2

% camera sensors initial value
camera1 = randi([-180 180])*to_rad;    % measured camera angle robot 1
camera2 = -randi([-180 180])*to_rad;   % measured camera angle robot 2

%% real position of the object without uncertainty

phi2 = theta2 - theta1;
[xA, yA, x1A, y1A] = object_detection(x1,y1,theta1,x2,y2,phi2,camera1,camera2);
 
figure
hold on
axis equal
plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)

%% Robot dynamics without uncertainty

% Vehicle inputs
u_velocity_1 = [sin(t/10)*randi([1,4]);
                sin(t/10)*2.*t;
                sin(t/5).*cos(t/4)*1.5];

u_velocity_2 = [cos(t/10)*randi([1,4]);
                sin(t/10)*2.*tan(t*9);
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
    
    % calculate the position of the object for each time step
    phi2 = state_r2(3,cT) - state_r1(3,cT);
    [obj_g(1,cT),obj_g(2,cT),obj_robot1(1,cT),obj_robot1(2,cT)] = ...
        object_detection(state_r1(1,cT),state_r1(2,cT),state_r1(3,cT),state_r2(1,cT),state_r2(2,cT),...
        phi2,camera_store(1,cT),camera_store(2,cT));
    
    % Robot dynamic update
    state_r1(:,cT+1) = RobotDynamic(state_r1(:,cT), u_velocity_1(:,cT), Dt);
    state_r2(:,cT+1) = RobotDynamic(state_r2(:,cT), u_velocity_2(:,cT), Dt);

    % Camera dynamic update
    camera_store(1,cT+1) = atan2(obj_g(2,cT)-state_r1(2,cT+1),obj_g(1,cT)-state_r1(1,cT+1)) - state_r1(3,cT+1);
    camera_store(2,cT+1) = atan2(obj_g(2,cT)-state_r2(2,cT+1),obj_g(1,cT)-state_r2(1,cT+1)) - state_r2(3,cT+1);

end

% plot position of the object w.r.t ground
figure('Name','Object A (ground)', 'NumberTitle','off'), hold on, axis equal;
title('Object A w.r.t to ground')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_g(1,cT),obj_g(2,cT),'o')
end

% plot position of the object w.r.t robot 1
figure('Name','Object A (Robot 1)', 'NumberTitle','off'), hold on, axis equal;
title('Object A w.r.t to robot 1')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_robot1(1,cT),obj_robot1(2,cT),'o')
end

%% PLots

figure('Name','Robots positions', 'NumberTitle','off')
hold on
grid on
axis equal
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-50 50])
ylim([-50 50])

for i = 1:length(t)

    phi2 = state_r2(3,i) - state_r1(3,i);

    plot_location(state_r1(1,i),state_r1(2,i),state_r1(3,i),...
                  state_r2(1,i),state_r2(2,i),phi2,...
                  obj_g(1,cT),obj_g(2,cT),camera_store(1,i),camera_store(2,i));
 
    drawnow

end

%% Sensors uncertainty

% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0.1;        % mean value
sigma_camera = 1e-2;    % variance

% camera1_est = camera1 + randn(1)*sigma_camera + mu_camera;
% camera2_est = camera2 + randn(1)*sigma_camera + mu_camera;
% camera_store_est = [camera1_est; camera2_est];
camera_store_est = camera_store + randn(1,length(camera_store))*sigma_camera + mu_camera;

% GPS -> give the position and orientation of the robot2
mu_GPS = 0.1;           % mean value
sigma_GPS = 1e-2;       % variance

% x1_est = x1 + randn(1)*sigma_GPS + mu_GPS;
% y1_est = y1 + randn(1)*sigma_GPS + mu_GPS;
% theta1_est = theta1 + randn(1)*sigma_GPS + mu_GPS;
% s0_1est = [x1_est; y1_est; theta1_est];
s0_1est = s0_1 + randn(1,length(s0_1))*sigma_GPS + mu_GPS;

% x2_est = x2 + randn(1)*sigma_GPS + mu_GPS;
% y2_est = y2 + randn(1)*sigma_GPS + mu_GPS;
% theta2_est = theta2 + randn(1)*sigma_GPS + mu_GPS;
% s0_2est = [x2_est; y2_est; theta2_est];
s0_2est = s0_2 + randn(1,length(s0_2))*sigma_GPS + mu_GPS;

%% Logic

% prediction state_1 knowing u_vehicle1 + dynamics
% correction GPS_1 -> state_r1 + uncertainty

% prediction state_2 knowing u_vehicle2 + dynamics
% correction GPS_2 -> state_r2 + uncertainty

% prediction state_camera1,state_camera2 knowing state_1, state_2, xA, yA +
% dynamics
% correction CAMERA -> camera_store + uncertainty

% calculate xA,yA

%% Actuator uncertainty


%% Kalman FIlter


%% PLots

% figure('Name','Robots positions', 'NumberTitle','off')
% hold on
% grid on
% xlabel( 'x [m]' );
% ylabel( 'y [m]' );
% title('Robot position in time');
% 
% plot(xA,yA, 'o', 'Color','Red')
% for i = 1:length(t)
% 
%     state_r2(1,i)
%     plot_location(state_r1(1,i),state_r1(2,i),state_r1(3,i),...
%                   state_r2(1,i),state_r2(2,i),phi2,xA,yA,camera1,camera2)
% 
%     plot(state_r1(1,i),state_r1(2,i),'*', 'Color','#0072BD');
%     plot(state_r2(1,i),state_r2(2,i),'*','Color',"#D95319");
%     axis equal
% 
%    % plot_location(state_r1(1,i),state_r1(2,i),state_r1(3,i),state_r2(1,i),state_r2(2,i),phi2 = state_r2(3,cT) - state_r1(3,cT),obj_g(1,i),obj_g(1,i),camera_store(1,i),camera_store(2,i))
%     xlim([-200 200])
%     ylim([-200 200])
%     drawnow
% end
