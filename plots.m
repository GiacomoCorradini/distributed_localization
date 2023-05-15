%% PLots

figure('Name','Robot position estimation'), clf, hold on;
plot(s_r1(1,:),s_r1(2,:),'-',Color='r')
plot(s_r1_est(1,:),s_r1_est(2,:),'-',Color='g')
plot(s_r2(1,:),s_r2(2,:),'-',Color='b')
plot(s_r2_est(1,:),s_r2_est(2,:),'-',Color='m')
legend('Robot 1 real','Robot 1 est','Robot 2 real','Robot 2 est')
xlabel('x [m]'); ylabel('y [m]');

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

figure('Name','Theta Error'), clf, hold on;
plot(t, (s_r1_est(3,:) - s_r1(3,:))/to_rad);
plot(t, (s_r2_est(3,:) - s_r2(3,:))/to_rad);
title('Theta Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('theta [deg]');

figure('Name','Obj Error'), clf, hold on;
for i=1:length(t)
    plot(t, obj_est{1,i}(1) - obj_ground(1,:));
    plot(t, obj_est{1,i}(2) - obj_ground(2,:));
    plot(t, obj_est{2,i}(1) - obj_ground(1,:));
    plot(t, obj_est{2,i}(2) - obj_ground(2,:));
end
title('Obj Error');
xlim([0, t(end-1)])
legend('x error robot 1','y error robot 1','x error robot 2','y error robot 2')
xlabel('t [s]'); ylabel('err [m]');

% figure('Name','Obj Error'), clf, hold on;
% plot(t, obj_ground_est(1,:) - obj_ground(1,:));
% plot(t, obj_ground_est(2,:) - obj_ground(2,:));
% title('Obj Error');
% legend('x error','y error')
% xlabel('t [s]'); ylabel('err [m]');

figure('Name','Dev std'), clf, hold on;
for i=1:length(t)
    plot(t, p_est_err{1,i}(1));
    plot(t, p_est_err{1,i}(2));
    plot(t, p_est_err{2,i}(1));
    plot(t, p_est_err{2,i}(2));
end
title('Dev std');
legend('dev std obj_x-r1','dev std obj_y-r1','dev std obj_x-r2','dev std obj_y-r2')
xlabel('t [s]'); ylabel('[m]');

figure('Name','Est obj'), clf, hold on;
p1 = plot(mu_Err(1,10:end-1),mu_Err(2,10:end-1),'.','Color', "#0072BD",'DisplayName','Est obj robot 1');
p2 = plot(mu_Err(3,10:end-1),mu_Err(4,10:end-1),'.','Color',"#D95319",'DisplayName','Est obj robot 2');
p3 = plot(s0_obj(1),s0_obj(2),'.','Color','r','MarkerSize',40,'DisplayName','Real obj');
pp1 = polyfit(mu_Err(1,10:end-1),mu_Err(2,10:end-1),1);
plot(mu_Err(1,10:end-1),mu_Err(1,10:end-1).*pp1(1)+pp1(2),'Color', "#0072BD",'DisplayName','Est obj robot 1')
pp2 = polyfit(mu_Err(3,10:end-1),mu_Err(4,10:end-1),1);
plot(mu_Err(3,10:end-1),mu_Err(3,10:end-1).*pp2(1)+pp2(2),'Color',"#D95319",'DisplayName','Est obj robot 2')
xest = (pp2(2)-pp1(2))/(pp1(1)-pp2(1));
yest = pp1(1)*xest + pp1(2);
p4 = plot(xest,yest,'.','Color',"#77AC30",'MarkerSize',40,'DisplayName','Interp obj');
title('Est obj');
legend([p1,p2,p3,p4]);
xlabel('x [m]'); ylabel('y [m]');

figure('Name','Error(t)'), clf, hold on;
plot(t,mu_Err(1,:));
plot(t,s0_obj(1).*ones(length(t)));
plot(t,mu_Err(2,:));
plot(t,s0_obj(2).*ones(length(t)));
title('Est obj');
legend('x Est obj','x Real obj','y Est obj','y Real obj')
xlabel('t [s]'); ylabel('[m]');