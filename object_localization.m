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

%% State initialization

% robot 1 initial position
x1 = randi([-20 20]);                % x coordinate
y1 = randi([-20 20]);                % y coordinate
theta1 = randi([-180 180])*to_rad;   % theta coordinate
s0_1 = [x1; y1; theta1];             % state of robot 1

% robot 2 initial position
x2 = randi([-20 20]);                % x coordinate
y2 =  randi([-20 20]);               % y coordinate
theta2 = randi([-180 180])*to_rad;   % theta coordinate1
s0_2 = [x2; y2; theta2];             % state of robot 2

% camera sensors initial value
camera1 = randi([-180 180])*to_rad;  % measured camera angle robot 1
camera2 = -randi([-180 180])*to_rad; % measured camera angle robot 2
camera_0 = [camera1; camera2];       % state of camera

%% Real position of the object without uncertainty

phi2 = theta2 - theta1;
[xA, yA, x1A, y1A] = object_detection(x1,y1,theta1,x2,y2,phi2,camera1,camera2);
 
figure('Name','Object position'), clf, hold on, axis equal;
plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)

%% Robot dynamics without uncertainty

u_1 = [sin(t/10);
       cos(t/10);
       sin(t/5).*cos(t/4)*1.5];             % velocity of robot 1

u_2 = [cos(t/10);
       sin(t/10);
       sin(t/5).*cos(t/4)*1.5];             % velocity of robot 2

% Initialize array to store the value 
s_r1       = zeros(length(s0_1),length(t));
s_r2       = zeros(length(s0_2),length(t));
obj_ground = zeros(2,length(t));
obj_robot1 = zeros(2,length(t));
s_camera   = zeros(2,length(t));

% Store the initial value
s_r1(:,1) = s0_1;
s_r2(:,1) = s0_2;
s_camera(:,1) = camera_0;

for cT=1:length(t)-1
    
    % calculate the position of the object for each time step
    phi2 = s_r2(3,cT) - s_r1(3,cT);
    [obj_ground(1,cT),obj_ground(2,cT),obj_robot1(1,cT),obj_robot1(2,cT)] = ...
     object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r2(1,cT),s_r2(2,cT),...
     phi2,s_camera(1,cT),s_camera(2,cT));
    
    % Robot dynamic update
    s_r1(:,cT+1) = RobotDynamic(s_r1(:,cT),u_1(:,cT),Dt);
    s_r2(:,cT+1) = RobotDynamic(s_r2(:,cT),u_2(:,cT),Dt);

    % Camera dynamic update
    s_camera(1,cT+1) = atan2(obj_ground(2,cT)-s_r1(2,cT+1),obj_ground(1,cT)-s_r1(1,cT+1))-s_r1(3,cT+1);
    s_camera(2,cT+1) = atan2(obj_ground(2,cT)-s_r2(2,cT+1),obj_ground(1,cT)-s_r2(1,cT+1))-s_r2(3,cT+1);

end

% plot position of the object w.r.t ground
figure('Name','Object A (ground)', 'NumberTitle','off'), clf, hold on, axis equal;
title('Object A w.r.t to ground')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_ground(1,cT),obj_ground(2,cT),'o')
end

% plot position of the object w.r.t robot 1
figure('Name','Object A (Robot 1)', 'NumberTitle','off'), clf, hold on, axis equal;
title('Object A w.r.t to robot 1')
xlim([-200 200])
ylim([-200 200])
for cT=1:length(t)-1
    plot(obj_robot1(1,cT),obj_robot1(2,cT),'o')
end

%% PLots

figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-50 50])
ylim([-50 50])

for i = 1:length(t)-1

    phi2 = s_r2(3,i) - s_r1(3,i);

    plot_location(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                  obj_ground(1,cT),obj_ground(2,cT),s_camera(1,i),s_camera(2,i));
 
    drawnow

end

%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0;        % mean value -> 0 means calibrated
sigma_camera = 1e-2;  % variance

cameraSensor = s_camera + randn(1,length(s_camera))*sigma_camera + mu_camera;

% -------------------------------------------------------------------------
% GPS -> give the position and orientation of the robot2
mu_GPS = 0;           % mean value -> 0 means calibrated
sigma_GPS = 1e-2;     % variance

s_1GPS = s_r1 + randn(1,length(s_r1))*sigma_GPS + mu_GPS;
s_2GPS = s_r2 + randn(1,length(s_r2))*sigma_GPS + mu_GPS;

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = zeros(3,1);        % mean value -> 0 means calibrated
sigma_u = 1e-2;  % covariance

u_1bar = u_1 + randn(1,length(u_1))*sigma_u + mu_u;
u_2bar = u_2 + randn(1,length(u_2))*sigma_u + mu_u;

%% Kalman filter

% CAMERA
s_camera_est = zeros(length(camera_0),length(t));
Pcam = 10^2*eye(length(camera_0));        % our knowledge about the initial position of the robot 1

% GPS
ProbGPS = 0.9;

s_r1_est = zeros(length(s0_1),length(t));
P1 = 10^2*eye(length(s0_1));              % our knowledge about the initial position of the robot 1

s_r2_est = zeros(length(s0_2),length(t));
P2 = 10^2*eye(length(s0_2));              % our knowledge about the initial position of the robot 2

% Logic KF

% prediction state_1 knowing u_vehicle1 + dynamics
% correction GPS_1 -> s_r1 + uncertainty

% prediction state_2 knowing u_vehicle2 + dynamics
% correction GPS_2 -> s_r2 + uncertainty

% prediction state_camera1,state_camera2 knowing state_1, state_2, xA, yA + dynamics
% correction CAMERA -> s_camera + uncertainty

% calculate xA,yA knowing state_1,state_2,cmaera_state

% KF implementation

for i=1:length(t)-1
    
    %% Robot 1
    
    % Prediction step
    % s_k+1 = A*s_k + B*u_k
    A = eye(3);
    B = eye(3)*Dt;
    S1EstPred = A*s_r1_est(:,i) + B*u_1bar(:,i);                     % Dyanmics robot 1
    P1pred = A*P1*A' + B*sigma_u^2*B';                               % Prior predict
    
    % Update step
    pGPS1 = rand(1);                                                 % probability to have GPS information
    if (pGPS1 <= ProbGPS)
        H = eye(3);                                                  % z_k = H*Sk
        InnCov = H*P1pred*H' + sigma_GPS^2;                          % Covariance of Innovation
        W = P1pred*H'*inv(InnCov);                                   % KF gain
        s_r1_est(:,i+1) = S1EstPred + W*(s_1GPS(:,i+1)-H*S1EstPred); % Updated state estimate
        P1 = (eye(length(s0_1))-W*H)*P1pred;                         % Updated covariance matrix
    else
        s_r1_est(:,i+1) = S1EstPred;
        P1 = P1pred;
    end

    %% Robot 2
    
    % Prediction step
    % s_k+1 = A*s_k + B*u_k
    A = eye(3);
    B = eye(3)*Dt;
    S2EstPred = A*s_r2_est(:,i) + B*u_2bar(:,i);                     % Dyanmics robot 1
    P2pred = A*P2*A' + B*sigma_u^2*B';                               % Prior predict
    
    % Update step
    pGPS2 = rand(1);                                                 % probability to have GPS information
    if (pGPS2 <= ProbGPS)
        H = eye(3);                                                  % z_k = H*Sk
        InnCov = H*P2pred*H' + sigma_GPS^2;                          % Covariance of Innovation
        W = P2pred*H'*inv(InnCov);                                   % KF gain
        s_r2_est(:,i+1) = S2EstPred + W*(s_2GPS(:,i+1)-H*S2EstPred); % Updated state estimate
        P2 = (eye(length(s0_2))-W*H)*P2pred;                         % Updated covariance matrix
    else
        s_r2_est(:,i+1) = S2EstPred;
        P2 = P2pred;
    end
    
    %% Object detection
 
%     % Prediction
%     x1p2EstPred = a*x1p2Est(i) + b*u1_bar(i);
%     P1p2pred = a*P1p2*a' + b*sigma_u1^2*b';
%     
%     % Update
%     pRadar = rand(1);
%     if (pGPS1 <= ProbGPS1)
%             H = [1; -1];
%             RadarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar;
%             z = [x1GPS(i+1); RadarMeasurements - x2Est(i+1)];
%             R = [sigma_gps1^2, 0; 0, sigma_radar^2 + P2];
%     else
%          if (pRadar <= ProbRadar)
%             H = -1;
%             RadarMeasurements = x2Store(i+1) - x1Store(i+1) + randn(1)*sigma_radar + mu_radar;
%             z = RadarMeasurements - x2Est(i+1);
%             R = sigma_radar^2 + P2;
%          end
%     end
%        
%     if (pGPS1 <= ProbGPS1) || (pRadar <= ProbRadar)
%         InnCov = H*P1p2pred*H' + R;
%         W = P1p2pred*H'*inv(InnCov);
%         x1p2Est(i+1) = x1p2EstPred + W*(z - H*x1p2EstPred);
%         P1p2 = (eye(length(x1)) - W*H)*P1p2pred;
%     else
%         x1p2Est(i+1) = x1p2EstPred;
%         P1p2 = P1p2pred;
%     end
%     
%     % Store the estimated covariance of the estimation error
%     P1p2Store(i+1) = P1p2;
%     P1p2PredStore(i+1) = P1p2pred;    
    
end

%% PLots

figure('Name','Robot position estimation'), clf, hold on;
plot(s_r1(1,:),s_r1(2,:),'-',Color='r')
plot(s_r1_est(1,:),s_r1_est(2,:),'-',Color='g')
plot(s_r2(1,:),s_r2(2,:),'-',Color='b')
plot(s_r2_est(1,:),s_r2_est(2,:),'-',Color='m')

figure('Name','X Error'), clf, hold on;
plot(t, s_r1_est(1,:) - s_r1(1,:));
plot(t, s_r2_est(1,:) - s_r2(1,:));
title('X Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('x [m]');

figure('Name','Y Error'), clf, hold on;
plot(t, s_r1_est(2,:) - s_r1(2,:));
plot(t, s_r2_est(2,:) - s_r2(2,:));
title('Y Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('y [m]');
