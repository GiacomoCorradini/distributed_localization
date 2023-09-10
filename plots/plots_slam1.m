set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesFontSize', 18);

%% Initial conditions

figure('Name','Initial conditions'),  hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Initial conditions');
xlim([-55 55])
ylim([-55 55])
phi2 = theta2 - theta1;
plot_location(x1,y1,theta1,x2,y2,phi2,obj_x,obj_y,s_camera(1,1),s_camera(2,1),color(1),color(11));
legend('','','Robot 1','','','Robot 2','','','Object');

%% Robot position in time without uncertainty

figure('Name','Robots positions'), clf, hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Robot position in time');
xlim([-55 55])
ylim([-55 55])
for i = 1:3:length(t)-1

    phi2 = s_r2(3,i) - s_r1(3,i);
    plot_location(s_r1(1,i),s_r1(2,i),s_r1(3,i),s_r2(1,i),s_r2(2,i),phi2,...
                  obj_ground(1,cT),obj_ground(2,cT),s_camera(1,i),s_camera(2,i), color(1), color(11));
    legend('','','Robot 1','','','Robot 2','','','Object');
    drawnow

end

%% Sensor uncertainty

figure('Name','Camera noise'), clf, hold on;
plot(t, s_camera_bar(1,:) - s_camera(1,:));
plot(t, s_camera_bar(2,:) - s_camera(2,:),'--');
title('Camera noise');
legend('noise camera 1','noise camera 2')
xlabel('t [s]'); ylabel('noise [m]');

figure('Name','GPS noise robot 1'), clf, hold on;
plot(t, s_1GPS(1,:) - s_r1(1,:));
plot(t, s_1GPS(2,:) - s_r1(2,:));
plot(t, s_1GPS(3,:) - s_r1(3,:));
title('GPS noise robot 1');
legend('noise x','noise y','noise theta')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','GPS noise robot 2'), clf, hold on;
plot(t, s_2GPS(1,:) - s_r2(1,:));
plot(t, s_2GPS(2,:) - s_r2(2,:));
plot(t, s_2GPS(3,:) - s_r2(3,:));
title('GPS noise robot 2');
legend('noise x','noise y','noise theta')
xlabel('t [s]'); ylabel('noise [m],[deg]');

figure('Name','Input noise robot 1'), clf, hold on;
plot(t, u_1bar(1,:) - u_1(1,:));
plot(t, u_1bar(2,:) - u_1(2,:));
plot(t, u_1bar(3,:) - u_1(3,:));
title('Input noise robot 1');
legend('noise v_x','noise v_y','noise yaw rate')
xlabel('t [s]'); ylabel('noise [m]/[deg]');

figure('Name','Input noise robot 2'), clf, hold on;
plot(t, u_2bar(1,:) - u_2(1,:));
plot(t, u_2bar(2,:) - u_2(2,:));
plot(t, u_2bar(3,:) - u_2(3,:));
title('Input noise robot 2');
legend('noise v_x','noise v_y','noise yaw rate')
xlabel('t [s]'); ylabel('noise [m],[deg]');

%% Plot estimation results

% Robot uncertainty

figure('Name','Robot localisation and mapping'), clf, hold on, axis equal;
plot(s_r1(1,:),s_r1(2,:),'-',Color=color(1))
plot(s_r1_est(1,:),s_r1_est(2,:),'-',Color=color(2))
plot(s_r2(1,:),s_r2(2,:),'-',Color=color(3))
plot(s_r2_est(1,:),s_r2_est(2,:),'-',Color=color(4))
for i = 1:length(t)
    plot(obj_est{1,i}(1,:),obj_est{1,i}(2,:),'.','MarkerSize',10,Color=color(12))
    plot(obj_est{2,i}(1,:),obj_est{2,i}(2,:),'.','MarkerSize',10,Color=color(14))
    plot(obj_ground(1,i),obj_ground(2,i),'.','MarkerSize',50,Color='r')
end
LG = legend('Robot 1 real position','Robot 1 estimated position','Robot 2 real position','Robot 2 estimated position', ...
    'Object estimated position robot 1','Object estimated position robot 2','Object real position', ...
    'Location','southwest');
title('Robot localisation and mapping')
set(LG,'FontSize',18)
xlabel('x [m]'); ylabel('y [m]');

figure('Name','Robot localisation error');
tiledlayout(3,1);

nexttile, hold on;
plot(t(2:end), s_r1_est(1,2:end) - s_r1(1,2:end));
plot(t(2:end), s_r2_est(1,2:end) - s_r2(1,2:end));
title('Robot localisation X Error','FontSize',20);
xlabel('t [s]'); ylabel('x [m]');

nexttile, hold on;
plot(t(2:end), s_r1_est(2,2:end) - s_r1(2,2:end));
plot(t(2:end), s_r2_est(2,2:end) - s_r2(2,2:end));
title('Robot localisation Y Error','FontSize',20);
xlabel('t [s]'); ylabel('y [m]');

nexttile, hold on;
plot(t, (s_r1_est(3,:) - s_r1(3,:))/to_rad);
plot(t, (s_r2_est(3,:) - s_r2(3,:))/to_rad);
title('Robot localisation Theta Error','FontSize',20);
xlabel('t [s]'); ylabel('theta [deg]');

LG = legend('Robot 1','Robot 2','Location','southoutside','Orientation','horizontal');
set(LG,'FontSize',18)

% Robot position in time with uncertainty
%TO DO 

% Object estimate

figure('Name','Object position Error'), clf, hold on;
tiledlayout(2,1);

nexttile, hold on;
plot(t, cellfun(@(v)v(1),obj_est(1,:)) - obj_ground(1,:));
plot(t, cellfun(@(v)v(2),obj_est(1,:)) - obj_ground(2,:));
title('Object position Error robot 1','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

nexttile, hold on;
plot(t, cellfun(@(v)v(1),obj_est(2,:)) - obj_ground(1,:));
plot(t, cellfun(@(v)v(2),obj_est(2,:)) - obj_ground(2,:));
title('Object position Error robot 2','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

LG = legend('x error','y error','Location','southoutside','Orientation','horizontal');
set(LG,'FontSize',18)

% figure('Name','Object position estimate with interpolation'), clf, hold on, axis equal;
% p1 = plot(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(2),obj_est(1,10:end-1)),'.','Color', "#0072BD",'DisplayName','Est obj robot 1');
% p2 = plot(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(2),obj_est(2,10:end-1)),'.','Color',"#D95319",'DisplayName','Est obj robot 2');
% p3 = plot(s0_obj(1),s0_obj(2),'.','Color','r','MarkerSize',40,'DisplayName','Object real position');
% pp1 = polyfit(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(2),obj_est(1,10:end-1)),1);
% plot(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(1),obj_est(1,10:end-1)).*pp1(1)+pp1(2),'Color', "#0072BD",'DisplayName','Object estimated position robot 1')
% pp2 = polyfit(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(2),obj_est(2,10:end-1)),1);
% plot(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(1),obj_est(2,10:end-1)).*pp2(1)+pp2(2),'Color',"#D95319",'DisplayName','Object estimated position robot 2')
% xest = (pp2(2)-pp1(2))/(pp1(1)-pp2(1));
% yest = pp1(1)*xest + pp1(2);
% p4 = plot(xest,yest,'.','Color',"#77AC30",'MarkerSize',40,'DisplayName','Object position interpolated');
% title('Object position estimate with interpolation');
% xlim([-55 55])
% ylim([-55 55])
% LG = legend([p1,p2,p3,p4],'Location','southwest');
% set(LG,'FontSize',14)
% xlabel('x [m]'); ylabel('y [m]');

%% Centralised WLS

figure('Name','Object Error in position using centralised WLS'), clf, hold on, axis equal;
plot(obj_est_centr(1,:),obj_est_centr(2,:),'.');
plot(obj_ground(1,:),obj_ground(2,:),'.','MarkerSize',30);
title('Object Error in position using centralised WLS');
LG = legend('x error','y error');
set(LG,'FontSize',18)
xlabel('t [s]'); ylabel('err [m]');

%% Object position centr vs distr

figure('Name','Object position Estimation robot 1'), hold on, axis equal;
plot(obj_est_centr(1,1:end-1),obj_est_centr(2,1:end-1),'.')
plot(cellfun(@(v)v(1),obj_est_distr_MD(1,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MD(1,1:end-1)),'o','MarkerSize',5)
plot(cellfun(@(v)v(1),obj_est_distr_MH(1,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MH(1,1:end-1)),'d','MarkerSize',5)
plot(obj_x,obj_y,'.','MarkerSize',40,color='r')
LG = legend('Centralized WLS','Distributed Maximum degree weights','Distributed Metropolis Hastings weight', 'Object real position','Location','southwest');
set(LG,'FontSize',18)
title('Object position Estimation robot')
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );

figure('Name','Object position Estimation Estimation robot 2'), hold on, axis equal;
plot(obj_est_centr(1,1:end-1),obj_est_centr(2,1:end-1),'.')
plot(cellfun(@(v)v(1),obj_est_distr_MD(2,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MD(2,1:end-1)),'o','MarkerSize',5)
plot(cellfun(@(v)v(1),obj_est_distr_MH(2,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MH(2,1:end-1)),'d','MarkerSize',5)
plot(obj_x,obj_y,'.','MarkerSize',40,color='r')
LG = legend('Centralised WLS','Distributed Maximum degree weights','Distributed Metropolis Hastings weights', 'Object real position','Location','southwest');
set(LG,'FontSize',18)
title('Object position Distributed Estimation robot')
xlabel( 'x [m]' ); 
ylabel( 'y [m]' );

%% Error in time centr vs distr

figure('Name','Object position Error in time with different algorithm'), clf, hold on;
tiledlayout(3,1);

nexttile, hold on;
plot(t(1:end-1), obj_est_centr(1,1:end-1) - obj_ground(1,1:end-1));
plot(t(1:end-1), obj_est_centr(2,1:end-1) - obj_ground(2,1:end-1));
title('Object position Error centralised WLS','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

nexttile, hold on;
plot(t(1:end-1), cellfun(@(v)v(1),obj_est_distr_MD(2,1:end-1)) - obj_ground(1,1:end-1));
plot(t(1:end-1), cellfun(@(v)v(2),obj_est_distr_MD(2,1:end-1)) - obj_ground(2,1:end-1));
title('Object position Error Distributed Maximum degree weights','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

nexttile, hold on;
plot(t(1:end-1), cellfun(@(v)v(1),obj_est_distr_MH(2,1:end-1)) - obj_ground(1,1:end-1));
plot(t(1:end-1), cellfun(@(v)v(2),obj_est_distr_MH(2,1:end-1)) - obj_ground(2,1:end-1));
title('Object position Error Distributed Metropolis Hastings weights','FontSize',20);
xlabel('t [s]'); ylabel('err [m]');

LG = legend('x error','y error','Location','southoutside','Orientation','horizontal');
set(LG,'FontSize',18)
