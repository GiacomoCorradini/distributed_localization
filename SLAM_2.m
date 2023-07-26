%% Set-up

clc
clear
close all

% simulation time definition
Dt = 0.1;
Tf = 25;
t = 0:Dt:Tf;

% limits definitions
obj_lim_x = 5;
obj_lim_y = 20;

% convert degree to radiant
to_rad = pi/180;
to_deg = 1/to_rad;

% include distributed_localization folder
addpath('functions/')
obj_sol;

% Camera field of view
FoV = 60*to_rad;

%% State initialization

% robot 1 initial position
x1 = 0;%randi([-20 20]);                % x coordinate
y1 = 0;%randi([-20 20]);                % y coordinate
theta1 = 45*to_rad; %randi([-180 180])*to_rad;  % theta coordinate
s0_1 = [x1; y1; theta1];                % state of robot 1

% robot 2 initial position
x2 = 30;%randi([-20 20]);                % x coordinate
y2 = 30;%randi([-20 20]);                % y coordinate
theta2 = -135*to_rad;%randi([-180 180])*to_rad;   % theta coordinate1
s0_2 = [x2; y2; theta2];                 % state of robot 2

%% Objects position
n_obj = 5;                          % number of objects
obj = cell(1,n_obj);
for i = 1:length(obj)
    s0_obj = [randi([obj_lim_x obj_lim_y]); randi([obj_lim_x obj_lim_y])];  % state of robot 2 [x, y]
    obj{i} = s0_obj;
end

figure('Name','Object position'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Object position');
xlim([-obj_lim_x*2 obj_lim_y*2])
ylim([-obj_lim_x*2 obj_lim_y*2])
for i = 1:length(obj)
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30,'Color',color(i));
end

%% Create dataset of robot position + camera measurement

time_to_switch = floor(length(t)/2);

u_1 = [2.*ones(1,time_to_switch);
       -6*cos(t(1:time_to_switch)/(Tf/2-Dt)*pi)*(pi/(Tf/2));
       cos(t(1:time_to_switch)/(Tf/2-Dt)*pi/2)*pi^2/2/Tf];             % velocity of robot 1

u_1 = [u_1, -flip(u_1(:,1:end),2), [0; pi; 0]];

u_2 = [+6*cos(t(1:time_to_switch)/(Tf/2-Dt)*pi)*(pi/(Tf/2));
       -2.*ones(1,time_to_switch);
       -cos(t(1:time_to_switch)/(Tf/2-Dt)*pi/2)*pi^2/2/Tf]; 

u_2 = [u_2, -flip(u_2(:,1:end),2), [pi; 0; 0]];

% Cell array of cameras
camera_cell = cell(1,n_obj);

% Initialize array to store the value 
s_r1 = zeros(length(s0_1),length(t));
s_r2 = zeros(length(s0_2),length(t));

for i = 1:(length(obj)+1)
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
    end
    if abs(tmp2) < FoV
        camera_cell{i}(2,1) = tmp2;
    end
end

tmp1 = cam_data(s0_1,s0_2);
tmp2 = cam_data(s0_2,s0_1);
if abs(tmp1) < FoV
    camera_cell{n_obj+1}(1,1) = tmp1;
end
if abs(tmp2) < FoV
    camera_cell{n_obj+1}(2,1) = tmp2;
end

for cT=1:length(t)-1
    
    % if cT > 2 && cT < time_to_switch
    %     % if sum(~isnan(cellfun(@(v)v(1,cT),camera_cell))) <  sum(~isnan(cellfun(@(v)v(1,cT-1),camera_cell)))
    %     % u_1(3, cT:time_to_switch) = -u_1(3, cT:time_to_switch);
    %     % u_1(3, time_to_switch+1:end-1) = -flip(u_1(3,1:time_to_switch),2);
    %     % end  
    % 
    %     if sum(~isnan(cellfun(@(v)v(2,cT),camera_cell))) < sum(~isnan(cellfun(@(v)v(2,cT-1),camera_cell)))
    %     u_2(3, time_to_switch+1:end-1) = -flip(u_2(3,1:time_to_switch),2);
    %     end
    % end
    
    
    % Robot dynamic update
    s_r1(:,cT+1) = RobotDynamic(s_r1(:,cT),u_1(:,cT),Dt);
    s_r2(:,cT+1) = RobotDynamic(s_r2(:,cT),u_2(:,cT),Dt);
    
    % Camera dynamic update
    for i = 1:length(obj)
        tmp1 = cam_data(s_r1(:,cT+1),obj{i});
        tmp2 = cam_data(s_r2(:,cT+1),obj{i});
        camera_cell2{i}(2,cT+1) = tmp2;
        if abs(tmp1) < FoV
            camera_cell{i}(1,cT+1) = tmp1;
        end
        if abs(tmp2) < FoV
            camera_cell{i}(2,cT+1) = tmp2;
        end
%         camera_cell{i}(:,cT+1) = [cam_data(s_r1(:,cT+1),obj{i}); cam_data(s_r2(:,cT+1),obj{i})];
    end

    
    tmp1 = cam_data(s_r1(:,cT+1),s_r2(:,cT+1));
    tmp2 = cam_data(s_r2(:,cT+1),s_r1(:,cT+1));
    if abs(tmp1) < FoV
       camera_cell{n_obj+1}(1,cT+1) = tmp1;
    end
    if abs(tmp2) < FoV
       camera_cell{n_obj+1}(2,cT+1) = tmp2;
    end

end

% 
for i=1:3
    s_r2_mob(i,:) = s_r2(i,:) - ones(1, length(s_r2(i,:))).*s_r2(i,1);
end

R2 = [cos(-s_r2(3,1)) -sin(-s_r2(3,1));
      sin(-s_r2(3,1))  cos(-s_r2(3,1))];
  
s_r2_mob(1:2,:) = R2*s_r2_mob(1:2,:);

%% Calculate exact position of the robot

% s_r1(3,:) = [-atan2(u_1(1,1:time_to_switch),u_1(2,1:time_to_switch))+pi.*ones(1,length(time_to_switch)), -atan2(u_1(1,time_to_switch:end-1),u_1(2,time_to_switch:end-1))];

obj_ground_cell = cell(1,n_obj);
obj_robot1_cell = cell(1,n_obj);
obj_robot2_cell = cell(1,n_obj);

for i = 1:length(obj)
    obj_ground_cell{i} = nan(length(s0_obj),length(t));
    obj_robot1_cell{i} = nan(length(s0_obj),length(t));
    obj_robot2_cell{i} = nan(length(s0_obj),length(t));
end

for i = 1:length(obj)
    for cT=2:length(t)
            
        if ~isnan(camera_cell{i}(1,cT)) && ~isnan(camera_cell{i}(1,cT-1))
            % calculate the position of the object for each time step
            phi2 = - s_r1(3,cT) + s_r1(3,cT-1);
            [obj_robot1_cell{i}(1,cT),obj_robot1_cell{i}(2,cT),~,~] = ...
             object_detection(s_r1(1,cT),s_r1(2,cT),s_r1(3,cT),s_r1(1,cT-1),s_r1(2,cT-1),...
             phi2,camera_cell{i}(1,cT),camera_cell{i}(1,cT-1));

        elseif isnan(camera_cell{i}(1,cT)) && ~isnan(obj_robot1_cell{i}(1,cT-1))
            obj_robot1_cell{i}(1,cT) = obj_robot1_cell{i}(1,cT-1);
            obj_robot1_cell{i}(2,cT) = obj_robot1_cell{i}(2,cT-1);
        end

        if ~isnan(camera_cell{i}(2,cT)) && ~isnan(camera_cell{i}(2,cT-1))
            phi2 = - s_r2_mob(3,cT) + s_r2_mob(3,cT-1);
            [obj_robot2_cell{i}(1,cT),obj_robot2_cell{i}(2,cT), ~, ~] = ...
             object_detection(s_r2_mob(1,cT),s_r2_mob(2,cT),s_r2_mob(3,cT),s_r2_mob(1,cT-1),s_r2_mob(2,cT-1),...
             phi2,camera_cell{i}(2,cT),camera_cell{i}(2,cT-1));

        elseif isnan(camera_cell{i}(2,cT)) && ~isnan(obj_robot2_cell{i}(1,cT-1))
            obj_robot2_cell{i}(1,cT) = obj_robot2_cell{i}(1,cT-1);
            obj_robot2_cell{i}(2,cT) = obj_robot2_cell{i}(2,cT-1);
        end
    end
end

% Plots real dynamics without uncertainty

figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-40 40])
ylim([-40 40])


for i = 1:3:length(t)-1  
        phi2 = s_r2(3,i) - s_r1(3,i);
        for j = 1:length(obj)
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                      obj{j}(1),obj{j}(2),camera_cell{j}(1,i),camera_cell{j}(2,i),color(j),color(j+10), camera_cell{n_obj+1}(:,i));
        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),camera_cell)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),camera_cell))))])
end

%% Matrix rotation optimum

x = sym('x', [1,3]);

RF = [cos(x(3)) -sin(x(3)) x(1) ;
      sin(x(3))  cos(x(3)) x(2) ;
          0            0      1  ];

obj_to_min = cell(1,n_obj);

for i = 1:n_obj
%     obj_to_min{i} = NaN(3,length(t));
end

%% Compute initial position of robot 2 in robot 1 ref frame

for cT = 1:length(t)-1
    dist = 0;
    if ~isnan(camera_cell{6}(1,cT))
        for i = 1:n_obj
            obj_to_min{i}(:,cT) = RF*[obj_robot2_cell{i}(:,cT); 1];
            if ~isnan(obj_robot2_cell{i}(1,cT)) &&  ~isnan(obj_robot1_cell{i}(1,cT)) 
            dist = dist + (obj_robot1_cell{i}(1,cT) - obj_to_min{i}(1,cT))^2 + (obj_robot1_cell{i}(2,cT) - obj_to_min{i}(2,cT))^2 ;
            end
        end 
        if ~isnan(obj_robot2_cell{i}(1,cT)) &&  ~isnan(obj_robot1_cell{i}(1,cT))  %sto if è da fixare
        fmincon(matlabFunction(dist, 'Vars', {x}), [0,0,0], [0,0,1; 0,0,-1],[2*pi;0])
        end
    end
end
   

% optimize due strade:
% 1) calcolare le distanze degli oggetti con lo stesso ID e minizzare amma x_RF y_RF;
% 2) calcolare le differenza delle coordinate x,y di ogni oggetto



%
%% Sensors uncertainty

% -------------------------------------------------------------------------
% CAMERA -> give the angle between the forward axis and the line passing
% through the center of the robot and the object
mu_camera = 0;        % mean value -> 0 means calibrated
sigma_camera = 10e-4;  % variance

for i = 1:length(camera_cell)
    cameraSensor{i} = camera_cell{i} + randn(2,length(camera_cell{i}))*sigma_camera + mu_camera*ones(2,length(camera_cell{i}));
end

% -------------------------------------------------------------------------
% INPUT -> input velocity of the robot
mu_u = zeros(3,1);        % mean value -> 0 means calibrated
sigma_u = 1e-2;  % covariance

u_1bar = u_1 + randn(3,length(u_1)).*sigma_u + mu_u.*ones(3,length(s_r1));
u_2bar = u_2 + randn(3,length(u_2)).*sigma_u + mu_u.*ones(3,length(s_r1));

figure('Name','Camera noise'), clf, hold on;
plot(t, cameraSensor{1}(1,:) - camera_cell{1}(1,:));
plot(t, cameraSensor{1}(2,:) - camera_cell{1}(2,:));
title('Camera noise');
legend('noise camera 1','noise camera 2')
xlabel('t [s]'); ylabel('noise [m]');

%% UNCERTAINTY

s_r1_bar = zeros(length(s0_1),length(t));
s_r2_bar = zeros(length(s0_2),length(t));

% Store the initial value
s_r1_bar(:,1) = s0_1;
s_r2_bar(:,1) = s0_2;

for cT=1:length(t)-1
    
    % Robot dynamic update
    s_r1_bar(:,cT+1) = RobotDynamic(s_r1_bar(:,cT),u_1bar(:,cT),Dt);
    s_r2_bar(:,cT+1) = RobotDynamic(s_r2_bar(:,cT),u_2bar(:,cT),Dt);

end

s_r2_mob_bar = zeros(length(s0_2),length(t));

% 
for i=1:3
    s_r2_mob_bar(i,:) = s_r2_bar(i,:) - ones(1, length(s_r2_bar(i,:))).*s_r2_bar(i,1);
end

R2_bar = [cos(-s_r2_bar(3,1)) -sin(-s_r2_bar(3,1));
      sin(-s_r2_bar(3,1))  cos(-s_r2_bar(3,1))];
  
s_r2_mob_bar(1:2,:) = R2_bar*s_r2_mob_bar(1:2,:);

%% Calculate exact position of the robot

obj_ground_cell_mob = cell(1,n_obj);
obj_robot1_cell_mob = cell(1,n_obj);
obj_robot2_cell_mob = cell(1,n_obj);

for i = 1:length(obj)
    obj_ground_cell_mob{i} = nan(length(s0_obj),length(t));
    obj_robot1_cell_mob{i} = nan(length(s0_obj),length(t));
    obj_robot2_cell_mob{i} = nan(length(s0_obj),length(t));
end

for i = 1:length(obj)
    for cT=2:length(t)
            
        if ~isnan(cameraSensor{i}(1,cT)) && ~isnan(cameraSensor{i}(1,cT-1))
            % calculate the position of the object for each time step
            phi2 = - s_r1_bar(3,cT) + s_r1_bar(3,cT-1);
            [obj_robot1_cell_mob{i}(1,cT),obj_robot1_cell_mob{i}(2,cT),~,~] = ...
             object_detection(s_r1_bar(1,cT),s_r1_bar(2,cT),s_r1_bar(3,cT),s_r1_bar(1,cT-1),s_r1_bar(2,cT-1),...
             phi2,cameraSensor{i}(1,cT),cameraSensor{i}(1,cT-1));

        elseif isnan(cameraSensor{i}(1,cT)) && ~isnan(obj_robot1_cell_mob{i}(1,cT-1))
            obj_robot1_cell_mob{i}(1,cT) = obj_robot1_cell_mob{i}(1,cT-1);
            obj_robot1_cell_mob{i}(2,cT) = obj_robot1_cell_mob{i}(2,cT-1);
        end

        if ~isnan(cameraSensor{i}(2,cT)) && ~isnan(cameraSensor{i}(2,cT-1))
            phi2 = - s_r2_mob_bar(3,cT) + s_r2_mob_bar(3,cT-1);
            [obj_robot2_cell_mob{i}(1,cT),obj_robot2_cell_mob{i}(2,cT), ~, ~] = ...
             object_detection(s_r2_mob_bar(1,cT),s_r2_mob_bar(2,cT),s_r2_mob_bar(3,cT),s_r2_mob_bar(1,cT-1),s_r2_mob_bar(2,cT-1),...
             phi2,cameraSensor{i}(2,cT),cameraSensor{i}(2,cT-1));

        elseif isnan(cameraSensor{i}(2,cT)) && ~isnan(obj_robot2_cell_mob{i}(1,cT-1))
            obj_robot2_cell_mob{i}(1,cT) = obj_robot2_cell_mob{i}(1,cT-1);
            obj_robot2_cell_mob{i}(2,cT) = obj_robot2_cell_mob{i}(2,cT-1);
        end
    end
end

% Plots real dynamics without uncertainty

figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-40 40])
ylim([-40 40])


for i = 1:3:length(t)-1  
        phi2 = s_r2_bar(3,i) - s_r1_bar(3,i);
        for j = 1:length(obj)
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1_bar(1,i),s_r1_bar(2,i),s_r1_bar(3,i),s_r2_bar(1,i),s_r2_bar(2,i),phi2,...
                      obj{j}(1),obj{j}(2),cameraSensor{j}(1,i),cameraSensor{j}(2,i),color(j),color(j+10), cameraSensor{n_obj+1}(:,i));
        
        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),cameraSensor)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),cameraSensor))))])
end
