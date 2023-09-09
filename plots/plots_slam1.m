%% Initial conditions
figure('Name','Initial conditions'),  hold on, axis equal;
xlabel( 'x [m]' );
ylabel( 'y [m]' );
title('Initial conditions');
xlim([lim_min*2 lim_max*2])
ylim([lim_min*2 lim_max*2])
phi2 = theta2 - theta1;
plot_location(x1,y1,theta1,x2,y2,phi2,obj_x,obj_y,s_camera(1,1),s_camera(2,1),color(1),color(11));
legend('','','Robot 1','','','Robot 2','','','Object');

%% PLots real dynamics without uncertainty
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
    legend('','','Robot 1','','','Robot 2','','','Object');
    drawnow

end

%% Plot uncertainty
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
figure('Name','Robot localisation'), clf, hold on;
plot(s_r1(1,:),s_r1(2,:),'-',Color='r')
plot(s_r1_est(1,:),s_r1_est(2,:),'-',Color='g')
plot(s_r2(1,:),s_r2(2,:),'-',Color='b')
plot(s_r2_est(1,:),s_r2_est(2,:),'-',Color='m')
legend('Robot 1 real','Robot 1 est','Robot 2 real','Robot 2 est')
title('Robot position estimation')
xlabel('x [m]'); ylabel('y [m]');

figure('Name','Robot localisation X Error'), clf, hold on;
plot(t(2:end), s_r1_est(1,2:end) - s_r1(1,2:end));
plot(t(2:end), s_r2_est(1,2:end) - s_r2(1,2:end));
title('Robot localisation X Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('x [m]');

figure('Name','Robot localisation Y Error'), clf, hold on;
plot(t(2:end), s_r1_est(2,2:end) - s_r1(2,2:end));
plot(t(2:end), s_r2_est(2,2:end) - s_r2(2,2:end));
title('Robot localisation Y Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('y [m]');

figure('Name','Robot localisation Theta Error'), clf, hold on;
plot(t, (s_r1_est(3,:) - s_r1(3,:))/to_rad);
plot(t, (s_r2_est(3,:) - s_r2(3,:))/to_rad);
title('Robot localisation Theta Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('theta [deg]');

figure('Name','Object position estinate'), clf, hold on;
plot(obj_ground(1,:),obj_ground(2,:),'.','MarkerSize',100);
for i = 1:length(t)
    plot(obj_est{1,i}(1),obj_est{1,i}(2),'Color','r','Marker','.','MarkerSize',20);
    plot(obj_est{2,i}(1),obj_est{2,i}(2),'Color','g','Marker','.','MarkerSize',20);
end
title('Object position estinate');
xlim([0, t(end-1)])
legend('Obj real','Obj est robot 1','Obj est robot 2','Location','best')
xlabel('t [s]'); ylabel('err [m]');

figure('Name','Obj position Error'), clf, hold on;
plot(t, cellfun(@(v)v(1),obj_est(1,:)) - obj_ground(1,:));
plot(t, cellfun(@(v)v(2),obj_est(1,:)) - obj_ground(2,:));
plot(t, cellfun(@(v)v(1),obj_est(2,:)) - obj_ground(1,:));
plot(t, cellfun(@(v)v(2),obj_est(2,:)) - obj_ground(2,:));
title('Obj position Error');
xlim([0, t(end-1)])
legend('x error robot 1','y error robot 1','x error robot 2','y error robot 2')
xlabel('t [s]'); ylabel('err [m]');

figure('Name','Dev std'), clf, hold on;
    plot(t, cellfun(@(v)v(1),p_est_err(1,:)));
    plot(t, cellfun(@(v)v(2),p_est_err(1,:)));
    plot(t, cellfun(@(v)v(1),p_est_err(2,:)));
    plot(t, cellfun(@(v)v(2),p_est_err(2,:)));
title('Dev std');
legend('dev std obj_x-R1','dev std obj_y-R1','dev std obj_x-R2','dev std obj_y-R2')
xlabel('t [s]'); ylabel('[m]');

figure('Name','Object position estimate with interpolation'), clf, hold on;
p1 = plot(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(2),obj_est(1,10:end-1)),'.','Color', "#0072BD",'DisplayName','Est obj robot 1');
p2 = plot(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(2),obj_est(2,10:end-1)),'.','Color',"#D95319",'DisplayName','Est obj robot 2');
p3 = plot(s0_obj(1),s0_obj(2),'.','Color','r','MarkerSize',40,'DisplayName','Real obj');
pp1 = polyfit(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(2),obj_est(1,10:end-1)),1);
plot(cellfun(@(v)v(1),obj_est(1,10:end-1)),cellfun(@(v)v(1),obj_est(1,10:end-1)).*pp1(1)+pp1(2),'Color', "#0072BD",'DisplayName','Est obj robot 1')
pp2 = polyfit(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(2),obj_est(2,10:end-1)),1);
plot(cellfun(@(v)v(1),obj_est(2,10:end-1)),cellfun(@(v)v(1),obj_est(2,10:end-1)).*pp2(1)+pp2(2),'Color',"#D95319",'DisplayName','Est obj robot 2')
xest = (pp2(2)-pp1(2))/(pp1(1)-pp2(1));
yest = pp1(1)*xest + pp1(2);
p4 = plot(xest,yest,'.','Color',"#77AC30",'MarkerSize',40,'DisplayName','Interp obj');
title('Est obj');
legend([p1,p2,p3,p4]);
xlabel('x [m]'); ylabel('y [m]');

%% Centralised WLS
figure('Name','Object Error in time using centralised WLS'), clf, hold on;
plot(t, obj_est_centr(1,:) - obj_ground(1,:));
plot(t, obj_est_centr(2,:) - obj_ground(2,:));
title('Object Error in time using centralised WLS');
legend('x error','y error')
xlabel('t [s]'); ylabel('err [m]');

figure('Name','Object Error in position using centralised WLS'), clf, hold on;
plot(obj_ground(1,:),obj_ground(2,:),'o','MarkerSize',30);
plot(obj_est_centr(1,:),obj_est_centr(2,:),'.');
title('Object Error in position using centralised WLS');
legend('x error','y error')
xlabel('t [s]'); ylabel('err [m]');

%% Distributed
figure('Name','Distributed Estimation robot 1'), hold on;
plot(obj_est_centr(1,1:end-1),obj_est_centr(2,1:end-1),'.')
plot(cellfun(@(v)v(1),obj_est_distr_MD(1,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MD(1,1:end-1)),'o','MarkerSize',5)
plot(cellfun(@(v)v(1),obj_est_distr_MH(1,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MH(1,1:end-1)),'d','MarkerSize',5)
plot(obj_x,obj_y,'.','MarkerSize',40)
legend('Centralized','Maximum degree weights','Metropolis Hastings weight', 'Real Obj')
title('Distributed Estimation robot 1')

figure('Name','Distributed Estimation robot 2'), hold on;
plot(obj_est_centr(1,1:end-1),obj_est_centr(2,1:end-1),'.')
plot(cellfun(@(v)v(1),obj_est_distr_MD(2,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MD(2,1:end-1)),'o','MarkerSize',5)
plot(cellfun(@(v)v(1),obj_est_distr_MH(2,1:end-1)),cellfun(@(v)v(2),obj_est_distr_MH(2,1:end-1)),'d','MarkerSize',5)
plot(obj_x,obj_y,'.','MarkerSize',40)
legend('Centralized','Maximum degree weights','Metropolis Hastings weight', 'Real Obj')
title('Distributed Estimation robot 2')























