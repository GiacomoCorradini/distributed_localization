function plot_location(x1,y1,theta1,x2,y2,phi2,xA,yA,camera1,camera2)
    
    % SUMMURY
    % plots of the robot RF and object A
    
    x = 0:20;
    theta2 = phi2 + theta1;
    
    figure
    hold on
    axis equal
    
    % vehicle 1 plot
    plot(x1,y1,'o')
    plot(x,tan(theta1+camera1).*x-tan(theta1+camera1).*x1 + y1)
    plot([x1 x1+cos(theta1)],[y1 y1+sin(theta1)],'-*',Color='r')
    plot([x1 x1-sin(theta1)],[y1 y1+cos(theta1)],'-*',Color='b')
    
    % vehicle 2 plot
    plot(x2,y2,'o')
    plot(x,tan(theta2+camera2).*x-tan(theta2+camera2).*x2 + y2)
    plot([x2 x2+cos(theta2)],[y2 y2+sin(theta2)],'-*',Color='r')
    plot([x2 x2-sin(theta2)],[y2 y2+cos(theta2)],'-*',Color='b')
    
    % object A
    plot(xA,yA,'o')

end