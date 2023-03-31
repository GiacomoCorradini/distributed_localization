function s_new = RobotDynamic(s, u, Dt)
    
    % SUMMURY
    % s: actual state (x,y,theta)
    % u: velocity input (x_dot,y_dot,theta_dot)
    % dt: time step

    % Inputs
    x = s(1);
    y = s(2);
    theta = s(3);
    vx = u(1);
    vy = u(2);
    omega = u(3);
    
    % Dynamics
    x_new = x;% + vx*Dt;
    y_new = y;% + vy*Dt;
    theta_new = theta + omega*Dt;
    
    % Output
    s_new = [x_new; y_new; theta_new];

end