%% Set-up

clc
clear
close all

set(0,'DefaultFigureWindowStyle','docked');

% simulation time definition
Tf = 20;
Dt = 0.1;
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

% number of robots
n_robot = 2;

% number of camera sensors (= n° of robots)
n_sensor = n_robot;

% random seed for plot
rng(2);
warning off

%% State initialization

% robot 1 initial position
x1     = randi([lim_min lim_max]);   % x coordinate
y1     = randi([lim_min lim_max]);   % y coordinate
theta1 = randi([-180 180])*to_rad;   % theta coordinate
s0_1   = [x1; y1; theta1];           % state of robot 1

% robot 2 initial position
x2     = randi([-20 20]);            % x coordinate
y2     = randi([-20 20]);            % y coordinate
theta2 = randi([-180 180])*to_rad;   % theta coordinate1
s0_2   = [x2; y2; theta2];           % state of robot 2

% object position
obj_x  = randi([-20 20]);            % x coordinate 
obj_y  = randi([-20 20]);            % y coordinate
s0_obj = [obj_x; obj_y];             % state of object

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
s_camera = zeros(n_sensor,length(t));

% Store the initial value
s_r1(:,1) = s0_1;
s_r2(:,1) = s0_2;
s_camera(:,1) = [cam_data(s0_1,s0_obj);cam_data(s0_2,s0_obj)];

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

%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera    = 0;     % mean value -> 0 means calibrated
sigma_camera = 1e-4;  % variance

s_camera_bar = s_camera + randn(1,length(s_camera))*sigma_camera + mu_camera;

% -------------------------------------------------------------------------
% GPS -> give the position and orientation of the robot
ProbGPS = 0.9;        % probability to have GPS signal
mu_GPS = 0;           % mean value -> 0 means calibrated
sigma_GPS = 1e-2;     % variance of [x,y]
sigma_th_GPS = 1e-4;  % variance of [theta]
R_GPS = [sigma_GPS,     0 ,         0;
            0,      sigma_GPS,      0;
            0,          0,     sigma_th_GPS];

s_1GPS = s_r1 + (randn(3,length(s_r1))'*R_GPS)' + mu_GPS;
s_2GPS = s_r2 + (randn(3,length(s_r2))'*R_GPS)' + mu_GPS;

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = ones(3,1).*0; % mean value -> 0 means calibrated
sigma_u_t = 0.3e-0;          % variance of [x,y]
sigma_u_r = 1e-4;          % variance of [theta]
R_INPUT = [sigma_u_t,     0 ,         0;
              0,      sigma_u_t,      0;
              0,          0,     sigma_u_r];

u_1bar = u_1 + (randn(3,length(u_1))'*R_INPUT)' + mu_u;
u_2bar = u_2 + (randn(3,length(u_2))'*R_INPUT)' + mu_u;

%% Loop with uncertainty (Calculate position of the object)

% Robot estimate initialization
s_r1_est = zeros(length(s0_1),length(t));
s_r1_est(:,1) = s_1GPS(:,1);
P1 = R_GPS;              % our knowledge about the initial position of the robot 1
P1Store = cell(1, length(t));
P1Store{1} = R_GPS;

s_r2_est = zeros(length(s0_2),length(t));
s_r2_est(:,1) = s_2GPS(:,1);
P2 = R_GPS;              % our knowledge about the initial position of the robot 2
P2Store = cell(1, length(t));
P2Store{1} = R_GPS;

% obj_ground_est = zeros(length(s0_obj),length(t));
% obj_robot1_est = zeros(length(s0_obj),length(t));

% Object estimate initialization
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
    P1pred = A*P1*A' + B*R_INPUT^2*B';                               % Prior predict
    
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
    P2pred = A*P2*A' + B*R_INPUT^2*B';                               % Prior predict
    
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
 
    if i >= 15, j = randi([1 i-10]);
    else, j = randi([1 i]);
    end

    % Object detection robot 1
    
    valuelist = [s_r1_est(1,j),s_r1_est(2,j),s_r1_est(3,j),...
                 s_camera_bar(1,j),s_r1_est(1,i+1),s_r1_est(2,i+1),s_r1_est(3,i+1),s_camera_bar(1,i+1)];
    errlist = [sqrt(P1Store{j}(1,1)),sqrt(P1Store{j}(2,2)),sqrt(P1Store{j}(3,3)),sigma_camera,...
               sqrt(P1(1,1)),sqrt(P1(2,2)),sqrt(P1(3,3)),sigma_camera];
    [obj_est{1,i}(1),p_est_err{1,i}(1)] = PropError(obj_x_sol,varlist,valuelist,errlist);
    [obj_est{1,i}(2),p_est_err{1,i}(2)] = PropError(obj_y_sol,varlist,valuelist,errlist);

    % Object detection robot 2

    valuelist = [s_r2_est(1,j),s_r2_est(2,j),s_r2_est(3,j),...
                 s_camera_bar(2,j),s_r2_est(1,i+1),s_r2_est(2,i+1),s_r2_est(3,i+1),s_camera_bar(2,i+1)];
    errlist = [sqrt(P2Store{j}(1,1)),sqrt(P2Store{j}(2,2)),sqrt(P2Store{j}(3,3)),sigma_camera,...
               sqrt(P2(1,1)),sqrt(P2(2,2)),sqrt(P2(3,3)),sigma_camera];
    [obj_est{2,i}(1),p_est_err{2,i}(1)] = PropError(obj_x_sol,varlist,valuelist,errlist);
    [obj_est{2,i}(2),p_est_err{2,i}(2)] = PropError(obj_y_sol,varlist,valuelist,errlist);

end

%% Centralised WLS

% initialize
obj_est_centr = zeros(length(s0_obj),length(t));

for j = 1:length(t)
    H = [];
    R = [];
    Z = [];
    for i=1:n_sensor
        R_new = [p_est_err{i,j}(1).^2,            0;
                        0,              p_est_err{i,j}(2).^2];
        R = blkdiag(R,R_new);
        H = [H; eye(2)];
        Z = [Z; obj_est{i,j}];
    end
    obj_est_centr(:,j) = inv(H'*inv(R)*H)*H'*inv(R)*Z;
end

%% Distributed WLS

% Storing the estimates
obj_est_distr_MD = cell(n_sensor,length(t));
obj_est_distr_MH = cell(n_sensor,length(t));

for cT = 1:length(t)-1
    % initialize each sensor
    F = cell(n_sensor,1);
    a = cell(n_sensor,1);
    F_MH = cell(n_sensor,1);
    a_MH = cell(n_sensor,1);
    for i=1:n_sensor
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
        A = zeros(n_sensor,n_sensor);
        ProbOfConnection = 0.5;
        for i=1:n_sensor
            for j=i+1:n_sensor
                A(i,j) = round(rand(1)-(0.5-ProbOfConnection));
            end
        end
        A = A + A';
        
        % Degree vector
        D = A*ones(n_sensor,1);
    
        % Maximum Degree Waighting
        FStore = F;
        aStore = a;
        for i=1:n_sensor
            for j=1:n_sensor
                if A(i,j) == 1
                    F{i} = F{i} + 1/(1+max(D))*(FStore{j} - FStore{i});
                    a{i} = a{i} + 1/(1+max(D))*(aStore{j} - aStore{i});
                end
            end
        end
        
        % Metropolis-Hastings
        FStore = F_MH;
        aStore = a_MH;
        for i=1:n_sensor
            for j=1:n_sensor
                if A(i,j) == 1
                    F_MH{i} = F_MH{i} + 1/(1+max(D(i), D(j)))*(FStore{j} - FStore{i});
                    a_MH{i} = a_MH{i} + 1/(1+max(D(i), D(j)))*(aStore{j} - aStore{i});
                end
            end
        end
    
    end
    
    % Estimates
    for i=1:n_sensor
        obj_est_distr_MD{i,cT} = inv(F{i})*a{i};
    end
    
    % Estimates
    for i=1:n_sensor
        obj_est_distr_MH{i,cT} = inv(F_MH{i})*a_MH{i};
    end
end

%% PLots

plots_slam1;