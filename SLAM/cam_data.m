function cam_value = cam_data(s_robot,s_obj)

x_robot = s_robot(1);
y_robot = s_robot(2);
theta_robot = s_robot(3);

x_obj = s_obj(1);
y_obj = s_obj(2);

dx = x_obj - x_robot;
dy = y_obj - y_robot;

cam_value = atan2(dy,dx) - theta_robot;

end