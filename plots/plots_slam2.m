set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesFontSize', 18);

%% Initial conditions

figure('Name','Initial conditions'), hold on, axis equal;
LegS = {};
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Objects position');
xlim([-obj_lim_x*2 obj_lim_y*2])
ylim([-obj_lim_x*2 obj_lim_y*2])
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30,'Color',color(i));
    LegS{end+1} = ['Object ', num2str(i)];
end
legend(LegS, 'Location', 'best');

%% Robot position in time without uncertainty

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

%% Sensor uncertainty

figure('Name','Camera noise'),  hold on;
tiledlayout(n_obj,1);
for i=1:n_obj
    nexttile;
    hold on;
    plot(t, camera_sensor_bar{i}(1,:) - camera_sensor{i}(1,:));
    plot(t, camera_sensor_bar{i}(2,:) - camera_sensor{i}(2,:));
    title(['Camera noise obj ',num2str(i)]);
    xlabel('t [s]'); ylabel('noise [m]');
end
legend('noise camera robot 1','noise camera robot 2', ...
    'Location','southoutside','Orientation','horizontal')

figure('Name','Input noise robot 1'),  hold on;
plot(t, u_1bar(1,:) - u_1(1,:));
plot(t, u_1bar(2,:) - u_1(2,:));
plot(t, u_1bar(3,:) - u_1(3,:));
title('Input noise robot 1');
legend('noise v_x','noise v_y','noise yaw rate','Location','southoutside','Orientation','horizontal')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','Input noise robot 2'),  hold on;
plot(t, u_2bar(1,:) - u_2(1,:));
plot(t, u_2bar(2,:) - u_2(2,:));
plot(t, u_2bar(3,:) - u_2(3,:));
title('Input noise robot 2');
legend('noise v_x','noise v_y','noise yaw rate','Location','southoutside','Orientation','horizontal')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

%% Plot estimation results

% Robot uncertainty

figure('Name','Robot localisation and mapping'), clf, hold on, axis equal;
plot(s_r1(1,:),s_r1(2,:),'-',Color=color(1))
plot(s_r1_est(1,:),s_r1_est(2,:),'-',Color=color(2))
plot(s_r2(1,:),s_r2(2,:),'-',Color=color(3))
plot(s_r2_est(1,:),s_r2_est(2,:),'-',Color=color(4))
for i = 1:n_obj
    plot(obj_robot_cell_bar{1,i}(1,2:end),obj_robot_cell_bar{1,i}(2,2:end),'.','MarkerSize',10,Color=color(i))
    plot(obj_robot_cell_bar{3,i}(1,2:end),obj_robot_cell_bar{3,i}(2,2:end),'.','MarkerSize',10,Color=color(i + 10))
    plot(obj_robot_cell{1,i}(1,2:end),obj_robot_cell{1,i}(2,2:end),'.','MarkerSize',50,Color='r')
    plot(obj_robot_cell{3,i}(1,2:end),obj_robot_cell{3,i}(2,2:end),'.','MarkerSize',50,Color='r')
end
LG = legend('Robot 1 real position','Robot 1 estimated position','Robot 2 real position','Robot 2 estimated position', ...
    '','','Object real position', ...
    'Location','northwest');
title('Robot localisation and mapping')
set(LG,'FontSize',18)
xlabel('x [m]'); ylabel('y [m]');
xlim([-15 40])
ylim([-15 40])

%%

figure('Name','Robot localisation error');
tiledlayout(3,1);

nexttile, hold on;
% figure('Name','Robot localisation X Error'), clf, hold on;
plot(t(2:end), s_r1_est(1,2:end) - s_r1(1,2:end));
plot(t(2:end), s_r2_est(1,2:end) - s_r2(1,2:end));
title('Robot localisation X Error','FontSize',20);
xlabel('t [s]'); ylabel('x [m]');

nexttile, hold on;
% figure('Name','Robot localisation Y Error'), clf, hold on;
plot(t(2:end), s_r1_est(2,2:end) - s_r1(2,2:end));
plot(t(2:end), s_r2_est(2,2:end) - s_r2(2,2:end));
title('Robot localisation Y Error','FontSize',20);
xlabel('t [s]'); ylabel('y [m]');

nexttile, hold on;
% figure('Name','Robot localisation Theta Error'), clf, hold on;
plot(t, (s_r1_est(3,:) - s_r1(3,:))/to_rad);
plot(t, (s_r2_est(3,:) - s_r2(3,:))/to_rad);
title('Robot localisation Theta Error','FontSize',20);
xlabel('t [s]'); ylabel('theta [deg]');

LG = legend('Robot 1','Robot 2','Location','southoutside','Orientation','horizontal');
set(LG,'FontSize',18)

%%
% Robot position in time with uncertainty

an_fig2 = figure('Name','Robots positions with uncertainty 2');
hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time with uncertainty 2');
xlim([-40 40])
ylim([-40 40])


for i = 1:4:length(t)-1  
        phi2 = s_r2_est(3,i) - s_r1_est(3,i);
        for j = 1:n_obj
        set(0, 'currentfigure', an_fig2);
        % shg;
        [p1(j), p2(j), p11, p22] = plot_location2(s_r1_est(1,i),s_r1_est(2,i),s_r1_est(3,i),s_r2_est(1,i),s_r2_est(2,i),phi2,...
                      obj{j}(1),obj{j}(2),camera_sensor_bar{j}(1,i),camera_sensor_bar{j}(2,i),color(j),color(j+10), camera_sensor_bar{n_obj+1}(:,i));

        drawnow
        if ~isempty(p11), delete(p11), end
        if ~isempty(p22), delete(p22), end
        end
        if ~isempty(p1), delete(p1), end
        if ~isempty(p2), delete(p2), end
        disp(['Iter', num2str(i), ' - obj1 = ' num2str(sum(~isnan(cellfun(@(v)v(1,i),camera_sensor_bar)))), ', obj2 = ', num2str(sum(~isnan(cellfun(@(v)v(2,i),camera_sensor_bar))))])
end

%% Object estimate

figure('Name','Objects position Error'), clf;
for i=1:n_obj
    subplot(2,n_obj,i);
    hold on;
    plot(t, obj_robot_cell_bar{1,i}(1,:) - obj_robot_cell{1,i}(1,:));
    plot(t, obj_robot_cell_bar{1,i}(2,:) - obj_robot_cell{1,i}(2,:));
    title('Object position Error robot 1');
    legend('noise x_1','noise y_1')
    xlabel('t [s]'); ylabel('err [m]');
    xlim([0, Tf])

    subplot(2,n_obj,i+n_obj);
    hold on;
    plot(t, obj_robot_cell_bar{2,i}(1,:) - obj_robot_cell{2,i}(1,:));
    plot(t, obj_robot_cell_bar{2,i}(2,:) - obj_robot_cell{2,i}(2,:));
    title('Objects position Error robot 2');
    legend('noise x_2','noise y_2')
    xlabel('t [s]'); ylabel('err [m]');
    xlim([0, Tf])
end

figure('Name','Object position Error'), clf, hold on;
tiledlayout(2,1);

nexttile, hold on;
plot(t(2:end), obj_robot_cell_bar{1,1}(1,2:end) - obj_robot_cell{1,1}(1,2:end));
plot(t(2:end), obj_robot_cell_bar{1,1}(2,2:end) - obj_robot_cell{1,1}(2,2:end));
title('Object position Error robot 1','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

nexttile, hold on;
plot(t(2:end), obj_robot_cell_bar{2,1}(1,2:end) - obj_robot_cell{2,1}(1,2:end));
plot(t(2:end), obj_robot_cell_bar{2,1}(2,2:end) - obj_robot_cell{2,1}(2,2:end));
title('Object position Error robot 2','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

LG = legend('x error','y error','Location','southoutside','Orientation','horizontal');
set(LG,'FontSize',18)

% TO DO: dev std

%% SLAM plot

figure('Name','Maps'), hold on; axis equal;
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',30,'Color',color(i));
end
plot(s_r1(1,:),s_r1(2,:),'-');
plot(s_r2(1,:),s_r2(2,:),'-');
plot(s_r1_est(1,:),s_r1_est(2,:),'--');
plot(s_r2_est(1,:),s_r2_est(2,:),'--');
for j = 1:2:3
    for i = 1:n_obj
        plot(obj_robot_cell{j,i}(1,:),obj_robot_cell{j,i}(2,:),'*','MarkerSize',5,'Color',color(i));
        plot(obj_robot_cell_bar{j,i}(1,:),obj_robot_cell_bar{j,i}(2,:),'*','MarkerSize',5,'Color',color(i));
        plot(nanmean(obj_robot_cell_bar{j,i}(1,:)),nanmean(obj_robot_cell_bar{j,i}(2,:)),'o','MarkerSize',50,'Color',color(i));
    end
end

%% Centralised WLS

figure('Name','Centralised WLS'), hold on, axis equal;
title('Centralised WLS')
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',60,'Color',color(i));
    plot(p_hat{i}(1,:),p_hat{i}(2,:),'o','MarkerSize',5,'Color',color(i));
end

%% Distributed WLS

figure('Name','Distributed WLS'), hold on, axis equal;
title('Distributed WLS')
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );
for i = 1:n_obj
    plot(obj{i}(1),obj{i}(2),'.','MarkerSize',60,'Color',color(i));
    plot(p_est_distr_MH{1,i}(1,:),p_est_distr_MH{1,i}(2,:),'o','MarkerSize',5,'Color',color(i));
    plot(p_est_distr_MH{2,i}(1,:),p_est_distr_MH{2,i}(2,:),'*','MarkerSize',5,'Color',color(i));
end






