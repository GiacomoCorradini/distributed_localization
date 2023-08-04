%% Setup

clc
clear
close all

set(0,'DefaultFigureWindowStyle','docked');

% simulation time definition
Dt = 0.1;
Tf = 25;
t = 0:Dt:Tf;

% object limits definitions
obj_lim_x = 5;
obj_lim_y = 20;

% convert degree to radiant
to_rad = pi/180;
to_deg = 1/to_rad;

% include functions folder
addpath('functions/');

% eqs to find robot positions
obj_sol;

% Camera field of view
FoV = 60*to_rad;

%% State initialization

% robot 1 initial position
x1     = 0;                % x coordinate
y1     = 0;                % y coordinate
theta1 = 45*to_rad;        % theta coordinate
s0_1   = [x1; y1; theta1]; % state of robot 1

% robot 2 initial position
x2     = 30;               % x coordinate
y2     = 30;               % y coordinate
theta2 = -135*to_rad;      % theta coordinate1
s0_2   = [x2; y2; theta2]; % state of robot 2

% Objects position
n_obj = 5;                 % number of objects
obj   = cell(1,n_obj);     % cell array to store obj position

for i = 1:n_obj
    s0_obj = [randi([obj_lim_x obj_lim_y]); randi([obj_lim_x obj_lim_y])];
    obj{i} = s0_obj;
end

figure('Name','Objects position'),  hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Objects position');
xlim([-obj_lim_x*2 obj_lim_y*2])
ylim([-obj_lim_x*2 obj_lim_y*2])
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30,'Color',color(i));
end

%% Dataset: robot position + camera measurement

time_to_switch = floor(length(t)/2); % time to invert the motion of the robot

% velocity of robot 1
u_1 = [2.*ones(1,time_to_switch);
       -6*cos(t(1:time_to_switch)/(Tf/2-Dt)*pi)*(pi/(Tf/2));
       cos(t(1:time_to_switch)/(Tf/2-Dt)*pi/2)*pi^2/2/Tf];
u_1 = [u_1, -flip(u_1(:,1:end),2), [0; pi; 0]];

% velocity of robot 2
u_2 = [+6*cos(t(1:time_to_switch)/(Tf/2-Dt)*pi)*(pi/(Tf/2));
       -2.*ones(1,time_to_switch);
       -cos(t(1:time_to_switch)/(Tf/2-Dt)*pi/2)*pi^2/2/Tf]; 
u_2 = [u_2, -flip(u_2(:,1:end),2), [pi; 0; 0]];

% store robots position 
s_r1 = zeros(length(s0_1),length(t));
s_r2 = zeros(length(s0_2),length(t));

s_r1(:,1) = s0_1; % store robot 1 initial position
s_r2(:,1) = s0_2; % store robot 2 initial position

% store cameras dataset
camera_sensor = cell(1,n_obj);

for i = 1:(n_obj+1)
    camera_sensor{i} = NaN(2,length(t)); % initialize to NaN
end

for i = 1:n_obj
    tmp1 = cam_data(s0_1,obj{i});
    tmp2 = cam_data(s0_2,obj{i});
    if abs(tmp1) < FoV
        camera_sensor{i}(1,1) = tmp1;    % store camera 1 initial value
    end
    if abs(tmp2) < FoV
        camera_sensor{i}(2,1) = tmp2;    % store camera 2 initial value
    end
end

tmp1 = cam_data(s0_1,s0_2);
tmp2 = cam_data(s0_2,s0_1);
if abs(tmp1) < FoV
    camera_sensor{n_obj+1}(1,1) = tmp1;  % store initial camera value of robots 2
end
if abs(tmp2) < FoV
    camera_sensor{n_obj+1}(2,1) = tmp2;  % store initial camera value of robots 1
end

% simulation
for cT = 1:length(t)-1
    
    % Robots dynamic update
    s_r1(:,cT+1) = RobotDynamic(s_r1(:,cT),u_1(:,cT),Dt);
    s_r2(:,cT+1) = RobotDynamic(s_r2(:,cT),u_2(:,cT),Dt);
    
    % Camera dynamic update
    for i = 1:n_obj
        tmp1 = cam_data(s_r1(:,cT+1),obj{i});
        tmp2 = cam_data(s_r2(:,cT+1),obj{i});
        if abs(tmp1) < FoV
            camera_sensor{i}(1,cT+1) = tmp1;
        end
        if abs(tmp2) < FoV
            camera_sensor{i}(2,cT+1) = tmp2;
        end
    end
    
    % store robot position if they see each other
    tmp1 = cam_data(s_r1(:,cT+1),s_r2(:,cT+1));
    tmp2 = cam_data(s_r2(:,cT+1),s_r1(:,cT+1));
    if abs(tmp1) < FoV
       camera_sensor{n_obj+1}(1,cT+1) = tmp1;
    end
    if abs(tmp2) < FoV
       camera_sensor{n_obj+1}(2,cT+1) = tmp2;
    end

end

%% Relative position of robot 2 w.r.t its origin

s_r2_mob = zeros(length(s0_2),length(t));

for i = 1:length(s0_2)
    s_r2_mob(i,:) = s_r2(i,:) - ones(1, length(s_r2(i,:))).*s_r2(i,1);
end

R2 = [cos(-s_r2(3,1)) -sin(-s_r2(3,1));
      sin(-s_r2(3,1))  cos(-s_r2(3,1))];
  
s_r2_mob(1:2,:) = R2*s_r2_mob(1:2,:);

%% Calculate exact position of the object

% store object position w.r.t the robots
obj_robot_cell = cell(2,n_obj);

for i = 1:n_obj
    for j = 1:2
        obj_robot_cell{j,i} = nan(length(s0_obj),length(t));
    end
end

% simulation 
for i = 1:n_obj
    for cT=2:length(t)

        if ~isnan(camera_sensor{i}(1,cT)) && ~isnan(camera_sensor{i}(1,cT-1))
            phi2 = - s_r1(3,cT) + s_r1(3,cT-1);
            [obj_robot_cell{1,i}(1,cT),obj_robot_cell{1,i}(2,cT),~,~] = ...
             object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r1(1,cT-1),s_r1(2,cT-1),...
             phi2,camera_sensor{i}(1,cT),camera_sensor{i}(1,cT-1));
        elseif isnan(camera_sensor{i}(1,cT)) && ~isnan(obj_robot_cell{1,i}(1,cT-1))
            obj_robot_cell{1,i}(1,cT) = obj_robot_cell{1,i}(1,cT-1);
            obj_robot_cell{1,i}(2,cT) = obj_robot_cell{1,i}(2,cT-1);
        end

        if ~isnan(camera_sensor{i}(2,cT)) && ~isnan(camera_sensor{i}(2,cT-1))
            phi2 = - s_r2_mob(3,cT) + s_r2_mob(3,cT-1);
            [obj_robot_cell{2,i}(1,cT),obj_robot_cell{2,i}(2,cT), ~, ~] = ...
             object_detection(s_r2_mob(1,cT),s_r2_mob(2,cT),s_r2_mob(3,cT),s_r2_mob(1,cT-1),s_r2_mob(2,cT-1),...
             phi2,camera_sensor{i}(2,cT),camera_sensor{i}(2,cT-1));
        elseif isnan(camera_sensor{i}(2,cT)) && ~isnan(obj_robot_cell{2,i}(1,cT-1))
            obj_robot_cell{2,i}(1,cT) = obj_robot_cell{2,i}(1,cT-1);
            obj_robot_cell{2,i}(2,cT) = obj_robot_cell{2,i}(2,cT-1);
        end

    end
end

% plot
an_fig1 = figure('Name','Robots positions');
hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-obj_lim_x*2 obj_lim_y*2])
ylim([-obj_lim_x*2 obj_lim_y*2])

for i = 1:4:length(t)-1  
        phi2 = s_r2(3,i) - s_r1(3,i);
        set(0, 'currentfigure', an_fig1);
        for j = 1:n_obj
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                      obj{j}(1),obj{j}(2),camera_sensor{j}(1,i),camera_sensor{j}(2,i),color(j),color(j+10), camera_sensor{n_obj+1}(:,i));
        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),camera_sensor)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),camera_sensor))))])
end

%% Matrix rotation to translate robot 2 RF in robot 1 RF

x = sym('x', [1,3]);

RF = [cos(x(3)) -sin(x(3)) x(1) ;
      sin(x(3))  cos(x(3)) x(2) ;
          0            0      1  ];

obj_to_min = cell(1,n_obj);

%% Compute matrix to translate robot 2 in robot 1 RF

% initialize matrix
matrix_tran = nan(length(t),3);

% find the matrix the minimize the distance
for cT = 1:length(t)-1
    dist = 0;
    if ~isnan(camera_sensor{6}(1,cT))
        for i = 1:n_obj
            obj_to_min{i}(:,cT) = RF*[obj_robot_cell{2,i}(:,cT); 1];
            if ~isnan(obj_robot_cell{2,i}(1,cT)) &&  ~isnan(obj_robot_cell{1,i}(1,cT)) 
            dist = dist + (obj_robot_cell{1,i}(1,cT) - obj_to_min{i}(1,cT))^2 + (obj_robot_cell{1,i}(2,cT) - obj_to_min{i}(2,cT))^2 ;
            end
        end 
        if ~isnan(obj_robot_cell{2,i}(1,cT)) &&  ~isnan(obj_robot_cell{1,i}(1,cT))  %sto if è da fixare
            matrix_tran(cT,:) = fmincon(matlabFunction(dist, 'Vars', {x}), [0,0,0], [0,0,1; 0,0,-1],[2*pi;0]);
        end
    end
end

R2_opt = cell(1,length(t));

for i=2:length(t)

    R2_opt{1,i} = [cos(nanmean(matrix_tran(2:i,3)))      -sin(nanmean(matrix_tran(2:i,3))) nanmean(matrix_tran(2:i,1)) ;
                   sin(nanmean(matrix_tran(2:i,3)))       cos(nanmean(matrix_tran(2:i,3))) nanmean(matrix_tran(2:i,2)) ;
                             0                           0                     1  ];
end

for i = 2:length(t)-1
    for j = 1:n_obj
        obj_robot_cell{3,j}(:,i) = R2_opt{1,i}*[obj_robot_cell{2,j}(:,i); 1];
    end
end

%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0;           % mean value -> 0 means calibrated
sigma_camera = 1e-4;     % variance

% store cameras dataset
camera_sensor_bar = cell(1,n_obj);

for i = 1:length(camera_sensor)
    camera_sensor_bar{i} = camera_sensor{i} + randn(2,length(camera_sensor{i}))*sigma_camera + mu_camera*ones(2,length(camera_sensor{i}));
end

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = ones(3,1).*0.001; % mean value -> 0 means calibrated
sigma_u_t = 0.3e-0;          % variance of [x,y]
sigma_u_r = 1e-4;          % variance of [theta]
R_INPUT = [sigma_u_t,     0 ,         0;
              0,      sigma_u_t,      0;
              0,          0,     sigma_u_r];

u_1bar = u_1 + (randn(3,length(u_1))'*R_INPUT)' + mu_u;
u_2bar = u_2 + (randn(3,length(u_2))'*R_INPUT)' + mu_u;

figure('Name','Camera noise'),  hold on;
for i=1:n_obj
    subplot(n_obj,1,i);
    hold on;
    plot(t, camera_sensor_bar{i}(1,:) - camera_sensor{i}(1,:));
    plot(t, camera_sensor_bar{i}(2,:) - camera_sensor{i}(2,:));
    title(['Camera noise obj ',num2str(i)]);
    legend('noise camera robot 1','noise camera robot 2')
    xlabel('t [s]'); ylabel('noise [m]');
end

figure('Name','Velocity noise robot 1'),  hold on;
plot(t, u_1bar(1,:) - u_1(1,:));
plot(t, u_1bar(2,:) - u_1(2,:));
plot(t, u_1bar(3,:) - u_1(3,:));
title('Velocity noise robot 1');
legend('velocity noise x_1','velocity noise y_1','velocity noise theta_1')
xlabel('t [s]'); ylabel('noise [m]');

figure('Name','Velocity noise robot 2'),  hold on;
plot(t, u_2bar(1,:) - u_2(1,:));
plot(t, u_2bar(2,:) - u_2(2,:));
plot(t, u_2bar(3,:) - u_2(3,:));
title('Velocity noise robot 2');
legend('velocity noise x_2','velocity noise y_2','velocity noise theta_2')
xlabel('t [s]'); ylabel('noise [m]');

%% Loop with uncertainty

% store robots position with uncertainty
s_r1_bar = zeros(length(s0_1),length(t));
s_r2_bar = zeros(length(s0_2),length(t));

% store the initial value
s_r1_bar(:,1) = s0_1;
s_r2_bar(:,1) = s0_2;

% prior
Pstore = cell(2,length(t));

for i = 1:2
    for j = 1:length(t)
        Pstore{i,j} = zeros(3);
    end
end

% simulation with uncertainty
for cT=1:length(t)-1
    
    % Robot dynamic update
    s_r1_bar(:,cT+1) = RobotDynamic(s_r1_bar(:,cT),u_1bar(:,cT),Dt);
    s_r2_bar(:,cT+1) = RobotDynamic(s_r2_bar(:,cT),u_2bar(:,cT),Dt);

    Pstore{1,cT+1} = Pstore{1,cT} + R_INPUT^2.*Dt^2;
    Pstore{2,cT+1} = Pstore{2,cT} + R_INPUT^2.*Dt^2;

end

s_r2_mob_bar = zeros(length(s0_2),length(t));

for i=1:3
    s_r2_mob_bar(i,:) = s_r2_bar(i,:) - ones(1, length(s_r2_bar(i,:))).*s_r2_bar(i,1);
end

R2_bar = [cos(-s_r2_bar(3,1)) -sin(-s_r2_bar(3,1));
          sin(-s_r2_bar(3,1))  cos(-s_r2_bar(3,1))];
  
s_r2_mob_bar(1:2,:) = R2_bar*s_r2_mob_bar(1:2,:);

% ------ RAW LOOP CLOSURE WITH INVERSE DYNAMICS -------
% s_r1_bar_corr = s_r1_bar;
% s_r2_bar_corr = s_r2_bar;
% s_r1_bar_corr(:,end) = s_r1_bar_corr(:,1);
% s_r2_bar_corr(:,end) = s_r2_bar_corr(:,1);
% step = length(t);
% 
% for cT=1:time_to_switch
% 
%     % Robot dynamic update
%     s_r1_bar_corr(:,step - cT) = RobotDynamic(s_r1_bar_corr(:,step-cT+1),-u_1bar(:,step-cT),Dt);
%     s_r2_bar_corr(:,step - cT) = RobotDynamic(s_r2_bar_corr(:,step-cT+1),-u_2bar(:,step-cT),Dt);
% 
%     Pstore{1,cT+1} = Pstore{1,cT} + R_INPUT^2.*Dt^2;
%     Pstore{2,cT+1} = Pstore{2,cT} + R_INPUT^2.*Dt^2;
% 
% end
% 
% s_r2_mob_bar_corr = s_r2_mob_bar;
% 
% for i=1:3
%     s_r2_mob_bar_corr(i,:) = s_r2_bar(i,:) - ones(1, length(s_r2_bar_corr(i,:))).*s_r2_bar_corr(i,1);
% end
% 
% R2_bar_corr = [cos(-s_r2_bar_corr(3,1)) -sin(-s_r2_bar_corr(3,1));
%                sin(-s_r2_bar_corr(3,1))  cos(-s_r2_bar_corr(3,1))];
% 
% s_r2_mob_bar_corr(1:2,:) = R2_bar_corr*s_r2_mob_bar_corr(1:2,:);

figure('Name','Position noise robot 1'),  hold on;
plot(t, s_r1_bar(1,:) - s_r1(1,:));
plot(t, s_r1_bar(2,:) - s_r1(2,:));
plot(t, s_r1_bar(3,:) - s_r1(3,:));
title('Position noise robot 1');
legend('Position noise x_1','Position noise y_1','Position noise theta_1')
xlabel('t [s]'); ylabel('noise [m]');

figure('Name','Position noise robot 2'),  hold on;
plot(t, s_r2_bar(1,:) - s_r2(1,:));
plot(t, s_r2_bar(2,:) - s_r2(2,:));
plot(t, s_r2_bar(3,:) - s_r2(3,:));
title('Position noise robot 2');
legend('velocity noise x_2','velocity noise y_2','velocity noise theta_2')
xlabel('t [s]'); ylabel('noise [m]');

%% Calculate position of the object with uncertainty

obj_robot_cell_bar = cell(2,n_obj);

for i = 1:n_obj
    for j = 1:2
        obj_robot_cell_bar{j,i} = nan(length(s0_obj),length(t));
    end
end

for i = 1:n_obj
    for cT=2:length(t)
        fprintf('Iter (%d,%d)\n',i,cT)
            
        if ~isnan(camera_sensor_bar{i}(1,cT)) && ~isnan(camera_sensor_bar{i}(1,cT-1))
            phi2 = - s_r1_bar(3,cT) + s_r1_bar(3,cT-1);
            [obj_robot_cell_bar{1,i}(1,cT),obj_robot_cell_bar{1,i}(2,cT),~,~] = ...
             object_detection(s_r1_bar(1,cT),s_r1_bar(2,cT),s_r1_bar(3,cT),s_r1_bar(1,cT-1),s_r1_bar(2,cT-1),...
             phi2,camera_sensor_bar{i}(1,cT),camera_sensor_bar{i}(1,cT-1));
        elseif isnan(camera_sensor_bar{i}(1,cT)) && ~isnan(obj_robot_cell_bar{1,i}(1,cT-1))
            obj_robot_cell_bar{1,i}(1,cT) = obj_robot_cell_bar{1,i}(1,cT-1);
            obj_robot_cell_bar{1,i}(2,cT) = obj_robot_cell_bar{1,i}(2,cT-1);
        end

        if ~isnan(camera_sensor_bar{i}(2,cT)) && ~isnan(camera_sensor_bar{i}(2,cT-1))
            phi2 = - s_r2_mob_bar(3,cT) + s_r2_mob_bar(3,cT-1);
            [obj_robot_cell_bar{2,i}(1,cT),obj_robot_cell_bar{2,i}(2,cT), ~, ~] = ...
             object_detection(s_r2_mob_bar(1,cT),s_r2_mob_bar(2,cT),s_r2_mob_bar(3,cT),s_r2_mob_bar(1,cT-1),s_r2_mob_bar(2,cT-1),...
             phi2,camera_sensor_bar{i}(2,cT),camera_sensor_bar{i}(2,cT-1));
        elseif isnan(camera_sensor_bar{i}(2,cT)) && ~isnan(obj_robot_cell_bar{2,i}(1,cT-1))
            obj_robot_cell_bar{2,i}(1,cT) = obj_robot_cell_bar{2,i}(1,cT-1);
            obj_robot_cell_bar{2,i}(2,cT) = obj_robot_cell_bar{2,i}(2,cT-1);
        end

    end
end

% Plots dynamics with uncertainty
an_fig2 = figure('Name','Robots positions with uncertainty');
hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time with uncertainty');
xlim([-40 40])
ylim([-40 40])


for i = 1:4:length(t)-1  
        phi2 = s_r2_bar(3,i) - s_r1_bar(3,i);
        for j = 1:n_obj
        set(0, 'currentfigure', an_fig2);
        % shg;
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1_bar(1,i),s_r1_bar(2,i),s_r1_bar(3,i),s_r2_bar(1,i),s_r2_bar(2,i),phi2,...
                      obj{j}(1),obj{j}(2),camera_sensor_bar{j}(1,i),camera_sensor_bar{j}(2,i),color(j),color(j+10), camera_sensor_bar{n_obj+1}(:,i));
        
        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),camera_sensor_bar)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),camera_sensor_bar))))])
end

% Plots objects position with uncertainty
figure('Name','Obj position noise'), clf;
for i=1:n_obj
    subplot(2,n_obj,i);
    hold on;
    plot(t, obj_robot_cell_bar{1,i}(1,:) - obj_robot_cell{1,i}(1,:));
    plot(t, obj_robot_cell_bar{1,i}(2,:) - obj_robot_cell{1,i}(2,:));
    title('Obj position noise robot 1');
    legend('noise x_1','noise y_1')
    xlabel('t [s]'); ylabel('noise [m]');
    xlim([0, Tf])

    subplot(2,n_obj,i+n_obj);
    hold on;
    plot(t, obj_robot_cell_bar{2,i}(1,:) - obj_robot_cell{2,i}(1,:));
    plot(t, obj_robot_cell_bar{2,i}(2,:) - obj_robot_cell{2,i}(2,:));
    title('Obj position noise robot 2');
    legend('noise x_2','noise y_2')
    xlabel('t [s]'); ylabel('noise [m]');
    xlim([0, Tf])
end

%% Calculate position of the object with uncertainty (distributed)

obj_robot_cell_bar_2 = cell(2,n_obj);
Pstore_obj_2 = cell(2,n_obj);

for i = 1:n_obj
    for j = 1:2
        obj_robot_cell_bar_2{j,i} = nan(length(s0_obj),length(t));
        Pstore_obj_2{j,i} = zeros(2,length(t));
    end
end

for i = 1:n_obj
    for cT=2:length(t)
        fprintf('Iter (%d,%d)\n',i,cT)

        if ~isnan(camera_sensor_bar{i}(1,cT)) && ~isnan(camera_sensor_bar{i}(1,cT-1))
            valuelist = [s_r1_bar(1,cT),s_r1_bar(2,cT),s_r1_bar(3,cT),...
                         camera_sensor_bar{i}(1,cT),s_r1_bar(1,cT-1),s_r1_bar(2,cT-1),s_r1_bar(3,cT-1),camera_sensor_bar{i}(1,cT-1)];
            errorlist = [sqrt(Pstore{1,cT}(1,1)),sqrt(Pstore{1,cT}(2,2)),sqrt(Pstore{1,cT}(3,3)),sigma_camera,...
                       sqrt(Pstore{1,cT-1}(1,1)),sqrt(Pstore{1,cT-1}(2,2)),sqrt(Pstore{1,cT-1}(3,3)),sigma_camera];
            [obj_robot_cell_bar_2{1,i}(1,cT), obj_robot_cell_bar_2{1,i}(2,cT), Pstore_obj_2{1,i}(1,cT),Pstore_obj_2{1,i}(2,cT)] = PropError2(valuelist,errorlist);
            [~,Pstore_obj{1,i}(1,cT)] = PropError(obj_x_sol,varlist,valuelist,errorlist);
            [~,Pstore_obj{1,i}(2,cT)] = PropError(obj_y_sol,varlist,valuelist,errorlist); 
        elseif isnan(camera_sensor_bar{i}(1,cT)) && ~isnan(obj_robot_cell_bar_2{1,i}(1,cT-1))
            obj_robot_cell_bar_2{1,i}(1,cT) = obj_robot_cell_bar_2{1,i}(1,cT-1);
            Pstore_obj_2{1,i}(1,cT) = Pstore_obj_2{1,i}(1,cT-1);
            Pstore_obj{1,i}(1,cT) = Pstore_obj{1,i}(1,cT-1);
            obj_robot_cell_bar_2{1,i}(2,cT) = obj_robot_cell_bar_2{1,i}(2,cT-1);
            Pstore_obj_2{1,i}(2,cT) = Pstore_obj_2{1,i}(2,cT-1);
            Pstore_obj{1,i}(2,cT) = Pstore_obj{1,i}(2,cT-1);
        end


        if ~isnan(camera_sensor_bar{i}(2,cT)) && ~isnan(camera_sensor_bar{i}(2,cT-1))
            valuelist = [s_r2_mob_bar(1,cT),s_r2_mob_bar(2,cT),s_r2_mob_bar(3,cT),...
                         camera_sensor_bar{i}(2,cT),s_r2_mob_bar(1,cT-1),s_r2_mob_bar(2,cT-1),s_r2_mob_bar(3,cT-1),camera_sensor_bar{i}(2,cT-1)];
            errorlist = [sqrt(Pstore{2,cT}(1,1)),sqrt(Pstore{2,cT}(2,2)),sqrt(Pstore{2,cT}(3,3)),sigma_camera,...
                       sqrt(Pstore{2,cT-1}(1,1)),sqrt(Pstore{2,cT-1}(2,2)),sqrt(Pstore{2,cT-1}(3,3)),sigma_camera];
            [obj_robot_cell_bar_2{2,i}(1,cT), obj_robot_cell_bar_2{2,i}(2,cT), Pstore_obj_2{2,i}(1,cT),Pstore_obj_2{2,i}(2,cT)] = PropError2(valuelist,errorlist);
            [~,Pstore_obj{2,i}(1,cT)] = PropError(obj_x_sol,varlist,valuelist,errorlist);
            [~,Pstore_obj{2,i}(2,cT)] = PropError(obj_y_sol,varlist,valuelist,errorlist); 
        elseif isnan(camera_sensor_bar{i}(2,cT)) && ~isnan(obj_robot_cell_bar_2{2,i}(1,cT-1))
            obj_robot_cell_bar_2{2,i}(1,cT) = obj_robot_cell_bar_2{2,i}(1,cT-1);
            Pstore_obj_2{2,i}(1,cT) = Pstore_obj_2{2,i}(1,cT-1);
            Pstore_obj{2,i}(1,cT) = Pstore_obj{2,i}(1,cT-1);
            obj_robot_cell_bar_2{2,i}(2,cT) = obj_robot_cell_bar_2{2,i}(2,cT-1);
            Pstore_obj_2{2,i}(2,cT) = Pstore_obj_2{2,i}(2,cT-1);
            Pstore_obj{2,i}(2,cT) = Pstore_obj{2,i}(2,cT-1);

        end
    end
end

% Plots dynamics with uncertainty
an_fig2 = figure('Name','Robots positions with uncertainty 2');
hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time with uncertainty 2');
xlim([-40 40])
ylim([-40 40])


for i = 1:4:length(t)-1  
        phi2 = s_r2_bar(3,i) - s_r1_bar(3,i);
        for j = 1:n_obj
        set(0, 'currentfigure', an_fig2);
        % shg;
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1_bar(1,i),s_r1_bar(2,i),s_r1_bar(3,i),s_r2_bar(1,i),s_r2_bar(2,i),phi2,...
                      obj{j}(1),obj{j}(2),camera_sensor_bar{j}(1,i),camera_sensor_bar{j}(2,i),color(j),color(j+10), camera_sensor_bar{n_obj+1}(:,i));

        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),camera_sensor_bar)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),camera_sensor_bar))))])
end

% Plots objects position with uncertainty
figure('Name','Obj position noise 2'), clf;
for i=1:n_obj
    subplot(2,n_obj,i);
    hold on;
    plot(t, obj_robot_cell_bar_2{1,i}(1,:) - obj_robot_cell{1,i}(1,:));
    plot(t, obj_robot_cell_bar_2{1,i}(2,:) - obj_robot_cell{1,i}(2,:));
    title('Obj position noise robot 1');
    legend('noise x_1','noise y_1')
    xlabel('t [s]'); ylabel('noise [m]');
    xlim([0, Tf])

    subplot(2,n_obj,i+n_obj);
    hold on;
    plot(t, obj_robot_cell_bar_2{2,i}(1,:) - obj_robot_cell{2,i}(1,:));
    plot(t, obj_robot_cell_bar_2{2,i}(2,:) - obj_robot_cell{2,i}(2,:));
    title('Obj position noise robot 2');
    legend('noise x_2','noise y_2')
    xlabel('t [s]'); ylabel('noise [m]');
    xlim([0, Tf])
end

%% Matrix rotation optimum

x_bar = sym('x', [1,3]);

RF_bar = [cos(x_bar(3)) -sin(x_bar(3)) x_bar(1) ;
          sin(x_bar(3))  cos(x_bar(3)) x_bar(2) ;
                0            0      1  ];

obj_to_min_bar = cell(1,n_obj);

for i = 1:n_obj
%     obj_to_min_bar{i} = NaN(3,length(t));
end

%% Compute initial position of robot 2 in robot 1 ref frame

matrix_tran_bar = nan(length(t),3);

for cT = 1:length(t)-1
    dist_bar = 0;
    if ~isnan(camera_sensor_bar{6}(1,cT))
        for i = 1:n_obj
            obj_to_min_bar{i}(:,cT) = RF*[obj_robot_cell_bar{2,i}(:,cT); 1];
            if ~isnan(obj_robot_cell_bar{2,i}(1,cT)) &&  ~isnan(obj_robot_cell_bar{1,i}(1,cT)) 
            dist_bar = dist_bar + (obj_robot_cell_bar{1,i}(1,cT) - obj_to_min_bar{i}(1,cT))^2 + (obj_robot_cell_bar{1,i}(2,cT) - obj_to_min_bar{i}(2,cT))^2 ;
            end
        end 
        if ~isnan(obj_robot_cell_bar{2,i}(1,cT)) &&  ~isnan(obj_robot_cell_bar{1,i}(1,cT))  %sto if è da fixare
            matrix_tran_bar(cT,:) = fmincon(matlabFunction(dist_bar, 'Vars', {x}), [0,0,0], [0,0,1; 0,0,-1],[2*pi;0]);
        end
    end
end

R2_opt_bar = cell(1,length(t));

for i=2:length(t)

    R2_opt_bar{1,i} = [cos(nanmean(matrix_tran_bar(2:i,3))) -sin(nanmean(matrix_tran_bar(2:i,3))) nanmean(matrix_tran_bar(2:i,1)) ;
                       sin(nanmean(matrix_tran_bar(2:i,3)))  cos(nanmean(matrix_tran_bar(2:i,3))) nanmean(matrix_tran_bar(2:i,2)) ;
                             0                           0                     1  ];
end

for i = 2:length(t)-1
    for j = 1:n_obj
        obj_robot_cell_bar{3,j}(:,i) = R2_opt_bar{1,i}*[obj_robot_cell_bar{2,j}(:,i); 1];
        obj_robot_cell_bar_2{3,j}(:,i) = R2_opt_bar{1,i}*[obj_robot_cell_bar_2{2,j}(:,i); 1];
    end
end


%% Plot ideal map vs real map

figure('Name','Maps'), hold on; axis equal;
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30,'Color',color(i));
end
plot(s_r1(1,:),s_r1(2,:),'-');
plot(s_r2(1,:),s_r2(2,:),'-');
plot(s_r1_bar(1,:),s_r1_bar(2,:),'--');
plot(s_r2_bar(1,:),s_r2_bar(2,:),'--');
for j = 1:2:3
    for i = 1:n_obj
        plot(obj_robot_cell{j,i}(1,:),obj_robot_cell{j,i}(2,:),'*','MarkerSize',5,'Color',color(i));
        plot(obj_robot_cell_bar{j,i}(1,:),obj_robot_cell_bar{j,i}(2,:),'*','MarkerSize',5,'Color',color(i));
        plot(obj_robot_cell_bar_2{j,i}(1,:),obj_robot_cell_bar_2{j,i}(2,:),'*','MarkerSize',5,'Color',color(i));
        plot(nanmean(obj_robot_cell_bar{j,i}(1,:)),nanmean(obj_robot_cell_bar{j,i}(2,:)),'o','MarkerSize',50,'Color',color(i));
        plot(nanmean(obj_robot_cell_bar_2{j,i}(1,:)),nanmean(obj_robot_cell_bar_2{j,i}(2,:)),'o','MarkerSize',50,'Color',color(i));
    end
end



%% Distributed WLS

% Storing the estimates
n_sens = 2;
p_est_distr = cell(2,n_obj);
p_est_distr_MH = cell(2,n_obj);

for i = 1:n_obj
    p_est_distr{1,i} = zeros(2,length(t));
    p_est_distr_MH{1,i} = zeros(2,length(t));
end

for cT = 1:length(t)-1
    % initialize each sensor
    F = cell(n_sens,1);
    a = cell(n_sens,1);
    F_MH = cell(n_sens,1);
    a_MH = cell(n_sens,1);
    for i=1:n_sens
        Hi = eye(2);
        Ri = [Pstore_obj_2{i,1}(1,cT).^2,          0;
                    0,              Pstore_obj_2{i,1}(2,cT).^2];
        zi = obj_robot_cell_bar_2{i,1}(:,cT);
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
        p_est_distr{i,1}(:,cT) = inv(F{i})*a{i};
    end
    
    % Estimates
    for i=1:n_sens
        p_est_distr_MH{i,1}(:,cT) = inv(F_MH{i})*a_MH{i};
    end
end

figure
plot(p_est_distr_MH{1,1}(1,:),p_est_distr_MH{1,1}(2,:),'.')