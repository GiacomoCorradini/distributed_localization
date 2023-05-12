function plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2,col_1, col_2)
    
    % SUMMURY -> plots of the robots Reference Frame and object
    % Input
        % x1,y1,theta1: state of the robot 1
        % x2,y2,phi2: state of the robot 2
        % camera1, camera2: state of the cameras
    % Output
        % None

    tmp = 4;
    
    x_plot1 = (x1-50):(x1+50);
    x_plot2 = (x2-50):(x2+50);
    theta2 = phi2 + theta1;
    
    % vehicle 1 plot
    plot(x1,y1,'o')
    if ~isnan(camera1)
        plot(x_plot1,tan(theta1+camera1).*x_plot1-tan(theta1+camera1).*x1 + y1,Color=col_1)
    end
    plot([x1 x1+cos(theta1)*tmp],[y1 y1+sin(theta1)*tmp],'-',Color='r',LineWidth=2)
    plot([x1 x1-sin(theta1)*tmp],[y1 y1+cos(theta1)*tmp],'-',Color='b',LineWidth=2)
    
    % vehicle 2 plot
    plot(x2,y2,'o')
    if ~isnan(camera2)
        plot(x_plot2,tan(theta2+camera2).*x_plot2-tan(theta2+camera2).*x2 + y2,Color=col_2)
    end
    plot([x2 x2+cos(theta2)*tmp],[y2 y2+sin(theta2)*tmp],'-',Color='r',LineWidth=2)
    plot([x2 x2-sin(theta2)*tmp],[y2 y2+cos(theta2)*tmp],'-',Color='b',LineWidth=2)
    
    % object A
    plot(xA,yA,'.','MarkerSize',30,Color=col_1)

end