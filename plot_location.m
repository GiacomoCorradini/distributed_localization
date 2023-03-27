function plot_location(x1,y1,theta1,x2,y2,theta2,phi2,xA,yA)
    
    % SUMMURY
    % plots
    
    x = 0:20;
    
    figure
    hold on
    axis equal
    
    % vehicle 1
    plot(x1,y1,'o')
    plot(x,tan(theta1).*x)
    plot([0 1],[0 0],'->',Color='b')
    plot([0 0],[0 1],'->',Color='b')
    
    % vehicle 2
    plot(x2,y2,'o')
    plot(x,tan(phi2+theta2).*x-tan(phi2+theta2).*x2 + y2)
    plot([x2 x2+cos(phi2).*1],[y2 y2+sin(phi2).*1],'->',Color='r')
    plot([x2 x2-sin(phi2).*1],[y2 y2+cos(phi2).*1],'->',Color='b')
    
    % object A
    plot(xA,yA,'o')

end