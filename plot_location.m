function plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)
    
    % SUMMURY -> plots of the robots Reference Frame and object
    % Input
        % x1,y1,theta1: state of the robot 1
        % x2,y2,phi2: state of the robot 2
        % camera1, camera2: state of the cameras
    % Output
        % None
    
    x_plot1 = (x1-20):(x1+20);
    x_plot2 = (x2-20):(x2+20);
    theta2 = phi2 + theta1;
    
    % vehicle 1 plot
    plot(x1,y1,'o')
    plot(x_plot1,tan(theta1+camera1).*x_plot1-tan(theta1+camera1).*x1 + y1,Color=[0.6510 0.6510 0.6510])
    plot([x1 x1+cos(theta1)],[y1 y1+sin(theta1)],'-',Color='r',LineWidth=2)
    plot([x1 x1-sin(theta1)],[y1 y1+cos(theta1)],'-',Color='b',LineWidth=2)
    
    % vehicle 2 plot
    plot(x2,y2,'o')
    plot(x_plot2,tan(theta2+camera2).*x_plot2-tan(theta2+camera2).*x2 + y2,Color=[0.6510 0.6510 0.6510])
    plot([x2 x2+cos(theta2)],[y2 y2+sin(theta2)],'-',Color='r',LineWidth=2)
    plot([x2 x2-sin(theta2)],[y2 y2+cos(theta2)],'-',Color='b',LineWidth=2)
    
    % object A
    plot(xA,yA,'o')

end