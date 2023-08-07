%% Set-up

clc
clear
close all

set(0,'DefaultFigureWindowStyle','docked');

% simulation time definition
Dt = 0.1;
Tf = 20;
t = 0:Dt:Tf;

% convert degree to radiant
to_rad = pi/180;
to_deg = 1/to_rad;

% include functions & plots folder
addpath('functions/')
addpath('plots/')

% import equations to estimate the object position
obj_sol;

% maps limits definitions
lim_min = -20;
lim_max = 20;

%% State initialization

% robot 1 initial position
x1     = randi([lim_min lim_max]);   % x coordinate
y1     = randi([lim_min lim_max]);   % y coordinate
theta1 = randi([-180 180])*to_rad;   % theta coordinate
s0_1   = [x1; y1; theta1];           % state of robot 1

% robot 2 initial position
x2     = randi([lim_min lim_max]);   % x coordinate
y2     = randi([lim_min lim_max]);   % y coordinate
theta2 = randi([-180 180])*to_rad;   % theta coordinate1
s0_2   = [x2; y2; theta2];           % state of robot 2

% object position
obj_x  = randi([lim_min lim_max]);   % x coordinate 
obj_y  = randi([lim_min lim_max]);   % y coordinate
s0_obj = [obj_x; obj_y];             % state of object

figure('Name','Object position'),  hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Object position');
xlim([lim_min*2 lim_max*2])
ylim([lim_min*2 lim_max*2])
plot(obj_x,obj_y,'.','MarkerSize',30);

%% Dataset: robot position + camera measurement

u_1 = [3*sin(t/10);
       3*cos(t/10);
       sin(t/5).*cos(t/4)*0.5];      % velocity of robot 1

u_2 = [3*cos(t/10);
       -3*sin(t/10);
       sin(t/5).*cos(t/4)*0.5];      % velocity of robot 2

% Initialize array to store the value 
s_r1     = zeros(length(s0_1),length(t));
s_r2     = zeros(length(s0_2),length(t));
s_camera = zeros(length(s0_obj),length(t));

% Store the initial value
s_r1(:,1) = s0_1;
s_r2(:,1) = s0_2;
camera_0 = [cam_data(s0_1,s0_obj);cam_data(s0_2,s0_obj)];
s_camera(:,1) = camera_0;

for cT=1:length(t)-1

    % Robot dynamic update
    s_r1(:,cT+1) = RobotDynamic(s_r1(:,cT),u_1(:,cT),Dt);
    s_r2(:,cT+1) = RobotDynamic(s_r2(:,cT),u_2(:,cT),Dt);

    % Camera dynamic update
    s_camera(1,cT+1) = cam_data(s_r1(:,cT+1),s0_obj);
    s_camera(2,cT+1) = cam_data(s_r2(:,cT+1),s0_obj);

end

%% Loop without uncertainty (Calculate position of the object)

obj_ground = zeros(length(s0_obj),length(t));
obj_robot1 = zeros(length(s0_obj),length(t));

for cT=1:length(t)

    % calculate the position of the object for each time step
    phi2 = s_r2(3,cT)-s_r1(3,cT);
    [obj_ground(1,cT),obj_ground(2,cT),obj_robot1(1,cT),obj_robot1(2,cT)] = ...
     object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r2(1,cT),s_r2(2,cT),...
     phi2,s_camera(1,cT),s_camera(2,cT));

end

% PLots real dynamics without uncertainty
figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-50 50])
ylim([-50 50])

for i = 1:3:length(t)-1

    phi2 = s_r2(3,i) - s_r1(3,i);
    plot_location(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                  obj_ground(1,cT),obj_ground(2,cT),s_camera(1,i),s_camera(2,i), color(1), color(11));
    drawnow

end

%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera    = 0;     % mean value -> 0 means calibrated

sigma_camera = 1e-4;  % variance
cameraSensor = s_camera + randn(1,length(s_camera))*sigma_camera + mu_camera;

% % % Uncertainty
% % R_camera = 1e-3*(rand(2,2)-0.5);
% % R_camera = R_camera*R_camera';
% % 
% % cameraSensor = s_camera + mvnrnd([0;0], R_camera)' + mu_camera;

% -------------------------------------------------------------------------
% GPS -> give the position and orientation of the robot2
mu_GPS = 0;           % mean value -> 0 means calibrated

sigma_GPS = 1e-2;     % variance of [x,y]
sigma_th_GPS = 1e-4;  % variance of [theta]
R_GPS = [sigma_GPS,     0 ,         0;
            0,      sigma_GPS,      0;
            0,          0,     sigma_th_GPS];

s_1GPS = s_r1 + (randn(3,length(s_r1))'*R_GPS)' + mu_GPS;
s_2GPS = s_r2 + (randn(3,length(s_r2))'*R_GPS)' + mu_GPS;

% % % Uncertainty
% % R_GPS = 1e-3*(rand(3,3)-0.5);
% % R_GPS = R_GPS*R_GPS';
% % 
% % s_1GPS = s_r1 + mvnrnd([0;0;0], R_GPS)' + mu_GPS;
% % s_2GPS = s_r2 + mvnrnd([0;0;0], R_GPS)' + mu_GPS;

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = 0;    % mean value -> 0 means calibrated

sigma_u = 0.3e-0;       % variance

u_1bar = u_1 + randn(3,length(u_1)).*sigma_u + mu_u.*ones(3,length(s_r1));
u_2bar = u_2 + randn(3,length(u_2)).*sigma_u + mu_u.*ones(3,length(s_r1));

% % % Input uncertainty
% % Qi = 1e-3*(rand(3,3)-0.5);
% % Qi = Qi*Qi';
% % 
% % u_1bar = u_1 + mvnrnd([0;0;0], Qi)' + mu_u;
% % u_2bar = u_2 + mvnrnd([0;0;0], Qi)' + mu_u;

% Plot uncertainty

figure('Name','Camera noise'), clf, hold on;
plot(t, cameraSensor(1,:) - s_camera(1,:));
plot(t, cameraSensor(2,:) - s_camera(2,:),'--');
title('Camera noise');
legend('noise camera 1','noise camera 2')
xlabel('t [s]'); ylabel('noise [m]');

figure('Name','GPS noise robot 1'), clf, hold on;
plot(t, s_1GPS(1,:) - s_r1(1,:));
plot(t, s_1GPS(2,:) - s_r1(2,:));
plot(t, s_1GPS(3,:) - s_r1(3,:));
title('GPS noise robot 1');
legend('noise x GPS','noise y GPS','noise theta GPS')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','GPS noise robot 2'), clf, hold on;
plot(t, s_2GPS(1,:) - s_r2(1,:));
plot(t, s_2GPS(2,:) - s_r2(2,:));
plot(t, s_2GPS(3,:) - s_r2(3,:));
title('GPS noise robot 2');
legend('noise x GPS','noise y GPS','noise theta GPS')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','Input noise robot 1'), clf, hold on;
plot(t, u_1bar(1,:) - u_1(1,:));
plot(t, u_1bar(2,:) - u_1(2,:));
plot(t, u_1bar(3,:) - u_1(3,:));
title('Input noise robot 1');
legend('noise input v_x','noise input v_y','noise input yaw rate')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','Input noise robot 2'), clf, hold on;
plot(t, u_2bar(1,:) - u_2(1,:));
plot(t, u_2bar(2,:) - u_2(2,:));
plot(t, u_2bar(3,:) - u_2(3,:));
title('Input noise robot 2');
legend('noise input v_x','noise input v_y','noise input yaw rate')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

%% Loop with uncertainty (Calculate position of the object)

% CAMERA
s_camera_est = zeros(length(camera_0),length(t));
Pcam = 10^2*eye(length(camera_0));        % our knowledge about the initial position of the robot 1

% GPS
ProbGPS = 0.9;  % probability to have GPS signal

s_r1_est = zeros(length(s0_1),length(t));
s_r1_est(:,1) = s_1GPS(:,1);
P1 = 10^2*eye(length(s0_1));              % our knowledge about the initial position of the robot 1
P1Store = cell(1, length(t));
P1Store{1} = R_GPS;

s_r2_est = zeros(length(s0_2),length(t));
s_r2_est(:,1) = s_2GPS(:,1);
P2 = 10^2*eye(length(s0_2));              % our knowledge about the initial position of the robot 2
P2Store = cell(1, length(t));
P2Store{1} = R_GPS;

obj_ground_est = zeros(length(s0_obj),length(t));
obj_robot1_est = zeros(length(s0_obj),length(t));

% OBJECT
obj_est = cell(2,length(t));
p_est_err = cell(2,length(t));

for i = 1:length(t)
    for j = 1:2
        obj_est{j,i} = zeros(length(s0_obj),1);
        p_est_err{j,i} = zeros(length(s0_obj),1);
    end
end

for i=1:length(t)-1
    
    fprintf('Iter %d\n',i)

    % Robot 1
       
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
        InnCov = H*P1pred*H' + R_GPS^2;                              % Covariance of Innovation
        W = P1pred*H'/InnCov;                                        % KF gain
        s_r1_est(:,i+1) = S1EstPred + W*(s_1GPS(:,i+1)-H*S1EstPred); % Updated state estimate
        P1 = (eye(length(s0_1))-W*H)*P1pred;                         % Updated covariance matrix
    else
        s_r1_est(:,i+1) = S1EstPred;
        P1 = P1pred;
    end

    P1Store{i+1} = P1;

    % Robot 2
    
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
        InnCov = H*P2pred*H' + R_GPS^2;                              % Covariance of Innovation
        W = P2pred*H'/InnCov;                                        % KF gain
        s_r2_est(:,i+1) = S2EstPred + W*(s_2GPS(:,i+1)-H*S2EstPred); % Updated state estimate
        P2 = (eye(length(s0_2))-W*H)*P2pred;                         % Updated covariance matrix
    else
        s_r2_est(:,i+1) = S2EstPred;
        P2 = P2pred;
    end

    P2Store{i+1} = P2;
 
    % Object detection robot 1
    
    if i >= 20
        tmp_sigma_Err = 0;
        while abs(tmp_sigma_Err) < 0.02     
        j = randi([10,i]);
        tmp_sigma_Err = s_r1_est(3,i+1) - s_r1_est(3, j) + cameraSensor(1,i+1) - cameraSensor(1,j);
        end
    else, j = i;
    end

    valuelist = [s_r1_est(1,j),s_r1_est(2,j),s_r1_est(3,j),...
                 cameraSensor(1,j),s_r1_est(1,i+1),s_r1_est(2,i+1),s_r1_est(3,i+1),cameraSensor(1,i+1)];
    errlist = [sqrt(P1Store{j}(1,1)),sqrt(P1Store{j}(2,2)),sqrt(P1Store{j}(3,3)),sigma_camera,...
               sqrt(P1(1,1)),sqrt(P1(2,2)),sqrt(P1(3,3)),sigma_camera];
    [obj_est{1,i}(1),p_est_err{1,i}(1)] = PropError(obj_x_sol,varlist,valuelist,errlist);
    [obj_est{1,i}(2),p_est_err{1,i}(2)] = PropError(obj_y_sol,varlist,valuelist,errlist);

    % Object detection robot 2

    if i >= 20
        tmp_sigma_Err = 0;
        while abs(tmp_sigma_Err) < 0.02       
        j = randi([7,i]);
        tmp_sigma_Err = s_r2_est(3,i+1) - s_r2_est(3, j) + cameraSensor(2,i+1) - cameraSensor(2,j);
        end
    else, j = i;
    end

    valuelist = [s_r2_est(1,j),s_r2_est(2,j),s_r2_est(3,j),...
                 cameraSensor(2,j),s_r2_est(1,i+1),s_r2_est(2,i+1),s_r2_est(3,i+1),cameraSensor(2,i+1)];
    errlist = [sqrt(P2Store{j}(1,1)),sqrt(P2Store{j}(2,2)),sqrt(P2Store{j}(3,3)),sigma_camera,...
               sqrt(P2(1,1)),sqrt(P2(2,2)),sqrt(P2(3,3)),sigma_camera];
    [obj_est{2,i}(1),p_est_err{2,i}(1)] = PropError(obj_x_sol,varlist,valuelist,errlist);
    [obj_est{2,i}(2),p_est_err{2,i}(2)] = PropError(obj_y_sol,varlist,valuelist,errlist);

end

%% Centralised WLS

% initialize
p_hat = zeros(2,length(t));

for j = 1:length(t)-1
    H = [];
    R = [];
    Z = [];
    for i=1:2
        R_new = [p_est_err{i,j}(1).^2,            0;
                        0,              p_est_err{i,j}(2).^2];
        R = blkdiag(R,R_new);
        H = [H; eye(2)];
        Z = [Z; obj_est{i,j}];
    end
    p_hat(:,j) = inv(H'*inv(R)*H)*H'*inv(R)*Z;
end

%% Distributed WLS

% Storing the estimates
n_sens = 2;
p_est_distr = cell(2,length(t));
p_est_distr_MH = cell(2,length(t));

for cT = 1:length(t)-1
    % initialize each sensor
    F = cell(n_sens,1);
    a = cell(n_sens,1);
    F_MH = cell(n_sens,1);
    a_MH = cell(n_sens,1);
    for i=1:n_sens
        Hi = eye(2);
        Ri = [p_est_err{i,cT}(1).^2,          0;
                    0,              p_est_err{i,cT}(2).^2];
        zi = obj_est{i,cT};
        F{i} = Hi'*inv(Ri)*Hi;
        a{i} = Hi'*inv(Ri)*zi;
        F_MH{i} = Hi'*inv(Ri)*Hi;
        a_MH{i} = Hi'*inv(Ri)*zi;
    end

    % Number of consensus protocol msg exchanges
    m = 10;
    
    for k=1:m
        % Topology matrix
        A = zeros(n_sens,n_sens);
        ProbOfConnection = 0.5;
        for i=1:n_sens
            for j=i+1:n_sens
                A(i,j) = round(rand(1)-(0.5-ProbOfConnection));
            end
        end
        A = A + A';
        
        % Degree vector
        D = A*ones(n_sens,1);
    
        % Maximum Degree Waighting
        FStore = F;
        aStore = a;
        for i=1:n_sens
            for j=1:n_sens
                if A(i,j) == 1
                    F{i} = F{i} + 1/(1+max(D))*(FStore{j} - FStore{i});
                    a{i} = a{i} + 1/(1+max(D))*(aStore{j} - aStore{i});
                end
            end
        end
        
        % Metropolis-Hastings
        FStore = F_MH;
        aStore = a_MH;
        for i=1:n_sens
            for j=1:n_sens
                if A(i,j) == 1
                    F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(FStore{j} - FStore{i});
                    a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(aStore{j} - aStore{i});
                end
            end
        end
    
    end
    
    % Estimates
    for i=1:n_sens
        p_est_distr{i,cT} = inv(F{i})*a{i};
    end
    
    % Estimates
    for i=1:n_sens
        p_est_distr_MH{i,cT} = inv(F_MH{i})*a_MH{i};
    end
end

%% PLots

plots_slam1;