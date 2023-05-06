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
plot(t, s_r1_est(3,:) - s_r1(3,:));
plot(t, s_r2_est(3,:) - s_r2(3,:));
title('Theta Error');
legend('Robot1','Robot2')
xlabel('t [s]'); ylabel('theta [m]');

figure('Name','Obj Error'), clf, hold on;
plot(t, mu_Err(1,:) - obj_ground(1,:));
plot(t, mu_Err(2,:) - obj_ground(2,:));
title('Obj Error');
legend('x error','y error')
xlabel('t [s]'); ylabel('err [m]');

% figure('Name','Obj Error'), clf, hold on;
% plot(t, obj_ground_est(1,:) - obj_ground(1,:));
% plot(t, obj_ground_est(2,:) - obj_ground(2,:));
% title('Obj Error');
% legend('x error','y error')
% xlabel('t [s]'); ylabel('err [m]');

figure('Name','Dev std'), clf, hold on;
plot(t, sigma_Err(1,:));
plot(t, sigma_Err(2,:));
title('Dev std');
legend('dev std xA','dev std yA')
xlabel('t [s]'); ylabel('[m]');

figure('Name','Est obj'), clf, hold on;
plot(mu_Err(1,2:end-1),mu_Err(2,2:end-1),'.');
plot(s0_obj(1).*ones(length(t)),s0_obj(2).*ones(length(t)),'.','Color','r','MarkerSize',40);
title('Est obj');
legend('Est obj','Real obj')
xlabel('x [m]'); ylabel('y [m]');

figure('Name','Error(t)'), clf, hold on;
plot(t,mu_Err(1,:));
plot(t,s0_obj(1).*ones(length(t)));
plot(t,mu_Err(2,:));
plot(t,s0_obj(2).*ones(length(t)));
title('Est obj');
legend('x Est obj','x Real obj','y Est obj','y Real obj')
xlabel('t [s]'); ylabel('[m]');