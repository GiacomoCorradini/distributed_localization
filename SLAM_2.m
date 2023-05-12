%% Set-up

clc
clear
close all

% simulation time definition
Dt = 0.1;
t = 0:Dt:20;

% convert degree to radiant
to_rad = pi/180;

% include distributed_localization folder
addpath('functions/')
obj_sol;

FoV = 45*to_rad;

%% State initialization

% robot 1 initial position
x1 = randi([-20 20]);                % x coordinate
y1 = randi([-20 20]);                % y coordinate
theta1 = randi([-180 180])*to_rad;   % theta coordinate
s0_1 = [x1; y1; theta1];             % state of robot 1

% robot 2 initial position
x2 = randi([-20 20]);                % x coordinate
y2 = randi([-20 20]);                % y coordinate
theta2 = randi([-180 180])*to_rad;   % theta coordinate1
s0_2 = [x2; y2; theta2];             % state of robot 2

% objects position
n_obj = 10;                          % number of object
obj = cell(1,n_obj);
for i = 1:length(obj)
    s0_obj = [randi([-20 20]); randi([-20 20])];  % state of robot 2 [x, y]
    obj{i} = s0_obj;
end

figure('Name','Object position'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Object position');
xlim([-50 50])
ylim([-50 50])
for i = 1:length(obj)
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30);
end

%% Create dataset of robot position + camera measurement

u_1 = [3*sin(t/10);
       3*cos(t/10);
       sin(t/5).*cos(t/4)*1.5];             % velocity of robot 1

u_2 = [3*cos(t/10);
       -3*sin(t/10);
       sin(t/5).*cos(t/4)*1.5];             % velocity of robot 2

% Cell array of cameras
camera_cell = cell(1,n_obj);

% Initialize array to store the value 
s_r1       = zeros(length(s0_1),length(t));
s_r2       = zeros(length(s0_2),length(t));

for i = 1:length(obj)
    camera_cell{i} = NaN(2,length(t));
end

% Store the initial value
s_r1(:,1) = s0_1;
s_r2(:,1) = s0_2;

for i = 1:length(obj)
    tmp1 = cam_data(s0_1,obj{i});
    tmp2 = cam_data(s0_2,obj{i});
    if abs(tmp1) < FoV
        camera_cell{i}(1,1) = tmp1;
    elseif abs(tmp2) < FoV
        camera_cell{i}(2,1) = tmp2;
    end
end

for cT=1:length(t)-1
        
    % Robot dynamic update
    s_r1(:,cT+1) = RobotDynamic(s_r1(:,cT),u_1(:,cT),Dt);
    s_r2(:,cT+1) = RobotDynamic(s_r2(:,cT),u_2(:,cT),Dt);

    % Camera dynamic update
    for i = 1:length(obj)
        tmp1 = cam_data(s_r1(:,cT+1),obj{i});
        tmp2 = cam_data(s_r2(:,cT+1),obj{i});
        if abs(tmp1) < FoV
            camera_cell{i}(1,cT+1) = tmp1;
        end
        if abs(tmp2) < FoV
            camera_cell{i}(2,cT+1) = tmp2;
        end
%         camera_cell{i}(:,cT+1) = [cam_data(s_r1(:,cT+1),obj{i}); cam_data(s_r2(:,cT+1),obj{i})];
    end
end

%% Calculate exact position of the robot

obj_ground_cell = cell(1,n_obj);
obj_robot1_cell = cell(1,n_obj);

for i = 1:length(obj)
    obj_ground_cell{i} = zeros(length(s0_obj),length(t));
    obj_robot1_cell{i} = zeros(length(s0_obj),length(t));
end

for i = 1:length(obj)
    for cT=1:length(t)
            
        % calculate the position of the object for each time step
        phi2 = s_r2(3,cT) - s_r1(3,cT);
        [obj_ground_cell{i}(1,cT),obj_ground_cell{i}(2,cT),obj_robot1_cell{i}(1,cT),obj_robot1_cell{i}(2,cT)] = ...
         object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r2(1,cT),s_r2(2,cT),...
         phi2,camera_cell{i}(1,cT),camera_cell{i}(2,cT));
    
    end
end

% PLots real dynamics without uncertainty

figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-50 50])
ylim([-50 50])


for i = 1:length(t)-1
    for j = 1:length(obj)    
        phi2 = s_r2(3,i) - s_r1(3,i);
    
        plot_location(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                      obj_ground_cell{j}(1,cT),obj_ground_cell{j}(2,cT),camera_cell{j}(1,i),camera_cell{j}(2,i),color(j));
     
        drawnow
        disp(['Iter', num2str(i)])
    
    end
end

%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0;        % mean value -> 0 means calibrated
sigma_camera = 0;  % variance

cameraSensor = s_camera + randn(1,length(s_camera))*sigma_camera + mu_camera;

% -------------------------------------------------------------------------
% GPS -> give the position and orientation of the robot2
mu_GPS = 0;           % mean value -> 0 means calibrated
sigma_GPS = 1e-2;     % variance
mu_th_GPS = 0;           % mean value -> 0 means calibrated
sigma_th_GPS = 1e-3;     % variance

s_1GPS(1:2,:) = s_r1(1:2,:) + randn(2,length(s_r1)).*sigma_GPS + mu_GPS.*ones(2,length(s_r1));
s_2GPS(1:2,:) = s_r2(1:2,:) + randn(2,length(s_r2)).*sigma_GPS + mu_GPS.*ones(2,length(s_r2));
s_1GPS(3,:)   = s_r1(3,:)   + randn(1,length(s_r1)).*sigma_th_GPS + mu_th_GPS.*ones(1,length(s_r1));
s_2GPS(3,:)   = s_r2(3,:)   + randn(1,length(s_r2)).*sigma_th_GPS + mu_th_GPS.*ones(1,length(s_r2));

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = zeros(3,1);        % mean value -> 0 means calibrated
sigma_u = 1e-2;  % covariance

u_1bar = u_1 + randn(3,length(u_1)).*sigma_u + mu_u.*ones(3,length(s_r1));
u_2bar = u_2 + randn(3,length(u_2)).*sigma_u + mu_u.*ones(3,length(s_r1));

% figure('Name','Camera noise'), clf, hold on;
% plot(t, cameraSensor(1,:) - s_camera(1,:));
% plot(t, cameraSensor(2,:) - s_camera(2,:));
% title('Camera noise');
% legend('noise camera 1','noise camera 2')
% xlabel('t [s]'); ylabel('noise [m]');

%% Kalman filter

% CAMERA
s_camera_est = zeros(length(camera_0),length(t));
Pcam = 10^2*eye(length(camera_0));        % our knowledge about the initial position of the robot 1

% GPS
ProbGPS = 0.9;

s_r1_est = zeros(length(s0_1),length(t));
s_r1_est(:,1) = s_1GPS(:,1);
P1 = 10^2*eye(length(s0_1));              % our knowledge about the initial position of the robot 1
P1Store = cell(1, length(t));
P1Store{1} = eye(length(s0_1))*sigma_GPS;

s_r2_est = zeros(length(s0_2),length(t));
s_r2_est(:,1) = s_2GPS(:,1);
P2 = 10^2*eye(length(s0_2));              % our knowledge about the initial position of the robot 2
P2Store = cell(1, length(t));
P2Store{1} = eye(length(s0_2))*sigma_GPS;

obj_ground_est = zeros(length(s0_obj),length(t));
obj_robot1_est = zeros(length(s0_obj),length(t));
mu_Err = zeros(length(s0_obj)*2,length(t));
sigma_Err = zeros(length(s0_obj)*2,length(t));

mu_Err2 = zeros(length(s0_obj),length(t));
sigma_Err2 = zeros(length(s0_obj),length(t));

%%
for i=1:length(t)-1
    %% Robot 1
    
    fprintf('Iter %d\n',i)
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
        W = P1pred*H'/InnCov;                                   % KF gain
        s_r1_est(:,i+1) = S1EstPred + W*(s_1GPS(:,i+1)-H*S1EstPred); % Updated state estimate
        P1 = (eye(length(s0_1))-W*H)*P1pred;                         % Updated covariance matrix
    else
        s_r1_est(:,i+1) = S1EstPred;
        P1 = P1pred;
    end

    P1Store{i+1} = P1;

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
        W = P2pred*H'/InnCov;                                   % KF gain
        s_r2_est(:,i+1) = S2EstPred + W*(s_2GPS(:,i+1)-H*S2EstPred); % Updated state estimate
        P2 = (eye(length(s0_2))-W*H)*P2pred;                         % Updated covariance matrix
    else
        s_r2_est(:,i+1) = S2EstPred;
        P2 = P2pred;
    end

    P2Store{i+1} = P2;
 
    %% Object detection robot 1
    
    % calculate the position of the object for each time step
%     index = randi([1 i]); 
%     phi1 = s_r1_est(3,i+1) - s_r1_est(3,index);
%     [obj_ground_est(1,i),obj_ground_est(2,i),obj_robot1_est(1,i),obj_robot1_est(2,i)] = ...
%      object_detection(s_r1_est(1,index),s_r1_est(2,index),s_r1_est(3,index),s_r1_est(1,i+1),s_r1_est(2,i+1),...
%      phi1,s_camera(1,i),s_camera(1,i+1));

    % the error must be propagate

%     for j = 1:i
%         valuelist = [s_r1_est(1,j),s_r1_est(2,j),s_r1_est(3,j),...
%                  cameraSensor(1,j),s_r1_est(1,i+1),s_r1_est(2,i+1),s_r1_est(3,i+1),cameraSensor(1,i+1)];
%         errlist = [sqrt(P1Store{j}(1,1)),sqrt(P1Store{j}(2,2)),sqrt(P1Store{j}(3,3)),sigma_camera,...
%                    sqrt(P1(1,1)),sqrt(P1(2,2)),sqrt(P1(3,3)),sigma_camera];
%         [~,tmp_sigma_Err] = PropError(xA,varlist,valuelist,errlist);
% %         [~,tmp_sigma_Err] = PropError(yA,varlist,valuelist,errlist);
%         if tmp_sigma_Err < tmp || j == 1
%             index = j;
%             tmp = tmp_sigma_Err;
%         end
%     end
    
    if i >= 20
        tmp_sigma_Err = 0;
        while abs(tmp_sigma_Err) < 0.1       
        j = randi([10,i]);
        tmp_sigma_Err = s_r1_est(3,i+1) - s_r1_est(3, j) + cameraSensor(1,i+1) - cameraSensor(1,j);
        end
    else, j = i;
    end

    valuelist = [s_r1_est(1,j),s_r1_est(2,j),s_r1_est(3,j),...
                 cameraSensor(1,j),s_r1_est(1,i+1),s_r1_est(2,i+1),s_r1_est(3,i+1),cameraSensor(1,i+1)];
    errlist = [sqrt(P1Store{j}(1,1)),sqrt(P1Store{j}(2,2)),sqrt(P1Store{j}(3,3)),sigma_camera,...
               sqrt(P1(1,1)),sqrt(P1(2,2)),sqrt(P1(3,3)),sigma_camera];
    [mu_Err(1,i),sigma_Err(1,i)] = PropError(xA,varlist,valuelist,errlist);
    [mu_Err(2,i),sigma_Err(2,i)] = PropError(yA,varlist,valuelist,errlist);

    if i >= 15
        tmp_sigma_Err = 0;
        while abs(tmp_sigma_Err) < 0.1       
        j = randi([7,i]);
        tmp_sigma_Err = s_r2_est(3,i+1) - s_r2_est(3, j) + cameraSensor(2,i+1) - cameraSensor(2,j);
        end
    else, j = i;
    end

    valuelist = [s_r2_est(1,j),s_r2_est(2,j),s_r2_est(3,j),...
                 cameraSensor(2,j),s_r2_est(1,i+1),s_r2_est(2,i+1),s_r2_est(3,i+1),cameraSensor(2,i+1)];
    errlist = [sqrt(P2Store{j}(1,1)),sqrt(P2Store{j}(2,2)),sqrt(P2Store{j}(3,3)),sigma_camera,...
               sqrt(P2(1,1)),sqrt(P2(2,2)),sqrt(P2(3,3)),sigma_camera];
    [mu_Err(3,i),sigma_Err(3,i)] = PropError(xA,varlist,valuelist,errlist);
    [mu_Err(4,i),sigma_Err(4,i)] = PropError(yA,varlist,valuelist,errlist);

%     %% Object detection robot 2
%     
%     % calculate the position of the object for each time step
%     phi2 = s_r2(3,cT) - s_r1(3,cT);
%     [obj_ground(1,cT),obj_ground(2,cT),obj_robot1(1,cT),obj_robot1(2,cT)] = ...
%      object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r2(1,cT),s_r2(2,cT),...
%      phi2,s_camera(1,cT),s_camera(2,cT));
% 
%     % the error must be propagate


%     valuelist = [s_r1_est(1,i+1),s_r1_est(2,i+1),s_r1_est(3,i+1),...
%                  cameraSensor(1,i+1),s_r2_est(1,i+1),s_r2_est(2,i+1),s_r2_est(3,i+1),cameraSensor(2,i+1)];
%     errlist = [sqrt(P1Store{i+1}(1,1)),sqrt(P1Store{i+1}(2,2)),sqrt(P1Store{i+1}(3,3)),sigma_camera,...
%                sqrt(P2Store{i+1}(1,1)),sqrt(P2Store{i+1}(1,1)),sqrt(P2Store{i+1}(1,1)),sigma_camera];
%     [mu_Err2(1,i),sigma_Err2(1,i)] = PropError(xA,varlist,valuelist,errlist);
%     [mu_Err2(2,i),sigma_Err2(2,i)] = PropError(yA,varlist,valuelist,errlist);
end

%% PLots

plots;